//////////////////////////////////////////////////////////////////////
// jvs_bus_logger - JVS bus capture for offline replay (simulation)
//
// Decodes both serial lines (115200 8N1) independently of the controller and
// records every byte plus SETUP / LINK changes with a microsecond timestamp.
// The capture starts on `restart` (and at power-up). In normal mode it stops when
// the buffer is full; in ring mode (`ring`, used by the sniffer mode) it keeps the
// most recent entries. `hold` freezes the buffer while it is read through the bridge.
// With `sync_start` (latched on restart, used by the sniffer mode) nothing is recorded
// until the first SYNC byte (E0) is seen on either line: the capture then starts on a
// frame boundary (e.g. adapter armed before the arcade host is powered on).
//
// Buffer layout (32-bit words, big-endian bytes when dumped by the APF bridge):
//   word 0..1 : "JVSD" "UMP2"
//   word 2    : number of entries
//   word 3    : timestamp unit in ns (1000 = microseconds)
//   word 4    : flags: bit 0 = ring mode (sniffer), bit 1 = ring wrapped
//   word 5    : index of the oldest entry (ring mode, entries are stored circularly)
//   word 6..7 : reserved (0)
//   word 8+2n : entry n timestamp (us since capture start)
//   word 9+2n : entry n {type[31:24], byte[23:16], lines[15:8], seq[7:0]}
//     type : 8'h01 TX byte (Pocket -> I/O board), 8'h02 RX byte (I/O board -> Pocket),
//            8'h03 lines change (byte = 0), 8'h04 DIR change (byte = new DIR level)
//     lines: {5'b0, dir, link_n, setup_n} raw levels (setup_n = SNAC IN7, low = SETUP LED
//            lit; link_n = SNAC IO3, low = LINK LED lit; dir = RS485 driver enable)
//////////////////////////////////////////////////////////////////////
`default_nettype none

module jvs_bus_logger #(
    parameter int CLK_FREQ = 48_000_000,
    parameter int WORDS    = 32768               // 128 KB buffer
)(
    input  wire        clk,
    input  wire        enable,                   // JVS mode active
    input  wire        restart,                  // pulse: clear and restart the capture
    input  wire        hold,                     // freeze the buffer (bridge read in progress)
    input  wire        ring,                     // keep the most recent entries (sniffer mode)
    input  wire        sync_start,               // wait for the first E0 before recording
    input  wire        tx_line,                  // UART line Pocket -> RS485 driver
    input  wire        rx_line,                  // UART line RS485 receiver -> Pocket
    input  wire        dir,                      // RS485 driver enable
    input  wire        setup_n,                  // SNAC IN7 raw level
    input  wire        link_n,                   // SNAC IO3 raw level
    output logic [15:0] entry_count,
    output logic       full,                     // buffer full: capture stopped (never in ring mode), registered
    output wire        waiting,                  // armed, waiting for the first E0

    // Bridge read port (word address), registered on rd_clk
    input  wire        rd_clk,
    input  wire [$clog2(WORDS)-1:0] rd_addr,
    output logic [31:0] rd_data
);
    localparam int AW          = $clog2(WORDS);
    localparam int HDR_WORDS   = 8;
    localparam int MAX_ENTRIES = (WORDS - HDR_WORDS) / 2;
    localparam int US_CYCLES   = CLK_FREQ / 1_000_000;
    logic        ring_mode;                      // latched on restart
    logic        sync_wait;                      // latched on restart, cleared by the first E0
    logic        wrapped;
    logic [15:0] oldest;                         // ring mode: index of the oldest entry
    // Registered: full is read from another clock domain (core_top). A combinational compare
    // could glitch while entry_count changes several bits at once (0x37FF -> 0x3800 looked
    // like 0x3FFF for a moment and triggered the auto save of a 14336 entry init capture)
    always @(posedge clk) full <= !ring_mode && (entry_count >= MAX_ENTRIES);

    (* ramstyle = "M10K" *) logic [31:0] mem [WORDS];
    always @(posedge rd_clk) rd_data <= mem[rd_addr];

    // Single write port (registered request from the capture FSM)
    logic          mem_we;
    logic [AW-1:0] mem_wa;
    logic [31:0]   mem_wd;
    always @(posedge clk) if (mem_we) mem[mem_wa] <= mem_wd;

    // ---- Serial decoders --------------------------------------------------
    wire       tx_dv, rx_dv;
    wire [7:0] tx_byte, rx_byte;
    uart_rx #(.CLKS_PER_BIT(CLK_FREQ / 115200)) u_tx_dec (.i_Clock(clk), .i_Rx_Serial(tx_line), .o_Rx_DV(tx_dv), .o_Rx_Byte(tx_byte));
    uart_rx #(.CLKS_PER_BIT(CLK_FREQ / 115200)) u_rx_dec (.i_Clock(clk), .i_Rx_Serial(rx_line), .o_Rx_DV(rx_dv), .o_Rx_Byte(rx_byte));

    // ---- SETUP / LINK change detection (2FF sync + 100 us stability) ------
    logic [1:0] setup_s, link_s, dir_s;
    logic [1:0] lines_stable, lines_cand;
    logic [$clog2(US_CYCLES*100+1)-1:0] lines_cnt;
    logic       lines_evt;
    always @(posedge clk) begin
        setup_s <= {setup_s[0], setup_n};
        link_s  <= {link_s[0],  link_n};
        dir_s   <= {dir_s[0],   dir};
        lines_evt <= 1'b0;
        if ({link_s[1], setup_s[1]} != lines_cand) begin
            lines_cand <= {link_s[1], setup_s[1]};
            lines_cnt  <= '0;
        end else if (lines_cand != lines_stable) begin
            if (lines_cnt == US_CYCLES*100) begin
                lines_stable <= lines_cand;
                lines_evt    <= 1'b1;
            end else
                lines_cnt <= lines_cnt + 1'b1;
        end
    end
    wire [7:0] lines_now = {5'b0, dir_s[1], lines_stable};

    // ---- RS485 DIR change detection (internal FPGA signal: no filtering) ---
    logic dir_prev = 1'b0;
    logic dir_evt;
    always @(posedge clk) begin
        dir_prev <= dir_s[1];
        dir_evt  <= (dir_s[1] != dir_prev);
    end

    // ---- Timestamp ----------------------------------------------------------
    logic [31:0] time_us;
    logic [$clog2(US_CYCLES+1)-1:0] us_div;

    // ---- Event capture --------------------------------------------------------
    // Pending flags (TX/RX bytes are at least one byte time apart, lines events are rare)
    logic       tx_pend, rx_pend, ln_pend, dir_pend;
    logic       dir_pend_lvl;
    logic [31:0] dir_pend_t;
    logic [7:0] tx_pend_b, rx_pend_b;
    logic [31:0] tx_pend_t, rx_pend_t, ln_pend_t;

    typedef enum logic [2:0] {W_IDLE, W_TIME, W_DATA, W_COUNT, W_OLDEST, W_FLAGS, W_CLEAR_HDR} wstate_t;
    wstate_t     wst = W_CLEAR_HDR;
    logic [AW-1:0] waddr;
    logic [31:0] wtime, wdata;
    logic [7:0]  seq;
    logic [2:0]  hdr_idx;
    logic        running;
    assign waiting = running && sync_wait;

    always @(posedge clk) begin
        // microsecond timer
        if (us_div == US_CYCLES - 1) begin us_div <= '0; time_us <= time_us + 1; end
        else us_div <= us_div + 1'b1;

        // latch events
        if (tx_dv)     begin tx_pend <= 1'b1; tx_pend_b <= tx_byte; tx_pend_t <= time_us; end
        if (rx_dv)     begin rx_pend <= 1'b1; rx_pend_b <= rx_byte; rx_pend_t <= time_us; end
        if (lines_evt) begin ln_pend <= 1'b1;                         ln_pend_t <= time_us; end
        if (dir_evt)   begin dir_pend <= 1'b1; dir_pend_lvl <= dir_prev; dir_pend_t <= time_us; end

        mem_we <= 1'b0;

        if (restart) begin
            wst <= W_CLEAR_HDR; hdr_idx <= 3'd0;
            tx_pend <= 1'b0; rx_pend <= 1'b0; ln_pend <= 1'b0; dir_pend <= 1'b0;
        end else if (!hold) begin
            case (wst)
                W_CLEAR_HDR: begin             // write the header, reset counters
                    mem_we <= 1'b1;
                    mem_wa <= AW'(hdr_idx);
                    case (hdr_idx)
                        3'd0:    mem_wd <= "JVSD";
                        3'd1:    mem_wd <= "UMP2";
                        3'd3:    mem_wd <= 32'd1000;
                        3'd4:    mem_wd <= {31'd0, ring};
                        default: mem_wd <= 32'd0;          // count, oldest, reserved
                    endcase
                    hdr_idx <= hdr_idx + 3'd1;
                    if (hdr_idx == 3'd7) begin
                        entry_count <= '0; seq <= '0; time_us <= '0; us_div <= '0;
                        ring_mode <= ring; wrapped <= 1'b0; oldest <= '0; sync_wait <= sync_start;
                        waddr <= AW'(HDR_WORDS); running <= 1'b1;
                        ln_pend <= 1'b1; ln_pend_t <= '0;   // initial SETUP / LINK state
                        wst <= W_IDLE;
                    end
                end
                W_IDLE: if (running && enable && (ring_mode || entry_count < MAX_ENTRIES)) begin
                    // while waiting for the first E0 every event is dropped
                    if (tx_pend) begin
                        tx_pend <= 1'b0;
                        if (!sync_wait || tx_pend_b == 8'hE0) begin
                            sync_wait <= 1'b0; wtime <= tx_pend_t; wdata <= {8'h01, tx_pend_b, lines_now, seq}; wst <= W_TIME;
                        end
                    end else if (rx_pend) begin
                        rx_pend <= 1'b0;
                        if (!sync_wait || rx_pend_b == 8'hE0) begin
                            sync_wait <= 1'b0; wtime <= rx_pend_t; wdata <= {8'h02, rx_pend_b, lines_now, seq}; wst <= W_TIME;
                        end
                    end else if (ln_pend) begin
                        ln_pend <= 1'b0;
                        if (!sync_wait) begin wtime <= ln_pend_t; wdata <= {8'h03, 8'h00, lines_now, seq}; wst <= W_TIME; end
                    end else if (dir_pend) begin
                        dir_pend <= 1'b0;
                        if (!sync_wait) begin wtime <= dir_pend_t; wdata <= {8'h04, 7'h00, dir_pend_lvl, lines_now, seq}; wst <= W_TIME; end
                    end
                end
                W_TIME:  begin mem_we <= 1'b1; mem_wa <= waddr; mem_wd <= wtime; waddr <= waddr + 1'b1; wst <= W_DATA; end
                W_DATA:  begin
                    mem_we <= 1'b1; mem_wa <= waddr; mem_wd <= wdata;
                    seq <= seq + 1'b1;
                    if (waddr == AW'(WORDS - 1)) begin
                        waddr <= AW'(HDR_WORDS);              // ring mode: wrap to the first entry
                        if (ring_mode) wrapped <= 1'b1;
                    end else
                        waddr <= waddr + 1'b1;
                    if (entry_count < MAX_ENTRIES) entry_count <= entry_count + 1'b1;
                    // ring mode, buffer full: the entry just written replaced the oldest one
                    if (ring_mode && entry_count == MAX_ENTRIES)
                        oldest <= (oldest == MAX_ENTRIES - 1) ? 16'd0 : oldest + 16'd1;
                    wst <= W_COUNT;
                end
                W_COUNT:  begin mem_we <= 1'b1; mem_wa <= AW'(2); mem_wd <= {16'd0, entry_count}; wst <= ring_mode ? W_OLDEST : W_IDLE; end
                W_OLDEST: begin mem_we <= 1'b1; mem_wa <= AW'(5); mem_wd <= {16'd0, oldest}; wst <= W_FLAGS; end
                W_FLAGS:  begin mem_we <= 1'b1; mem_wa <= AW'(4); mem_wd <= {30'd0, wrapped, ring_mode}; wst <= W_IDLE; end
                default: wst <= W_IDLE;
            endcase
        end
    end

    initial begin
        full = 1'b0; running = 1'b0; mem_we = 1'b0; ring_mode = 1'b0; sync_wait = 1'b0; wrapped = 1'b0; oldest = '0; entry_count = '0; hdr_idx = '0; time_us = '0; us_div = '0;
        tx_pend = 1'b0; rx_pend = 1'b0; ln_pend = 1'b0; dir_pend = 1'b0; seq = '0; waddr = '0;
        lines_stable = 2'b11; lines_cand = 2'b11; lines_cnt = '0;
    end
endmodule
