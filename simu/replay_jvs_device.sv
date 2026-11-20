//////////////////////////////////////////////////////////////////////
// replay_jvs_device - plays back a real JVS bus capture against the controller
//
// The replay script is generated from a jvs_dump.bin capture (Pocket B button /
// auto save) with:  scripts/jvs_dump.py jvs_dump.bin --replay capture.replay
//
//   L <IN7> <IO3>            SETUP / LINK line levels from now on
//   T <bytes...>             frame the controller must send (compared byte by byte)
//   R <delay_us> <bytes...>  frame sent back <delay_us> after the end of the last T
//   E <key> <values...>      expected decoded values, available through get_expect()
//
// The script is played in order: every T waits for the next frame sent by the
// controller, so the replay follows the controller timing (request driven).
//////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module replay_jvs_device #(
    parameter int    CLK_FREQ    = 1_000_000,
    parameter string REPLAY_FILE = "capture.replay"
)(
    input  logic clk,
    input  logic rst,
    input  logic uart_rx_from_master,    // controller TX line
    output logic uart_tx_to_master,      // controller RX line
    output logic sense_in7,              // SNAC IN7 level (controller i_sense)
    output logic link_io3,               // SNAC IO3 level
    output logic replay_done,            // whole script played
    output int   tx_frames_checked,
    output int   tx_mismatches,
    output int   rx_frames_sent
);
    localparam int CLKS_PER_BIT = CLK_FREQ / 115200;

    //------------------------------------------------------------------
    // Script storage
    //------------------------------------------------------------------
    typedef struct {
        byte  kind;          // "L", "T", "R"
        int   delay_us;
        int   a, b;          // L levels
        byte  data[$];
    } step_t;
    step_t       steps[$];
    string       expect_map[string];

    function automatic string get_expect(input string key);
        return expect_map.exists(key) ? expect_map[key] : "";
    endfunction

    // split a line into space separated tokens
    function automatic void tokenize(input string line, ref string toks[$]);
        string cur = "";
        toks.delete();
        for (int i = 0; i < line.len(); i++) begin
            if (line[i] == " " || line[i] == "\t" || line[i] == "\n" || line[i] == "\r") begin
                if (cur.len() > 0) begin toks.push_back(cur); cur = ""; end
            end else
                cur = {cur, string'(line[i])};
        end
        if (cur.len() > 0) toks.push_back(cur);
    endfunction

    initial begin
        int    fd;
        string line;
        string toks[$];
        fd = $fopen(REPLAY_FILE, "r");
        if (fd == 0) $fatal(1, "[REPLAY] cannot open %s", REPLAY_FILE);
        while (!$feof(fd)) begin
            automatic step_t st;
            if ($fgets(line, fd) == 0) break;
            tokenize(line, toks);
            if (toks.size() == 0 || toks[0][0] == "#") continue;
            st.kind = toks[0][0];
            st.data = {};
            case (toks[0])
                "L": begin st.a = toks[1].atoi(); st.b = toks[2].atoi(); steps.push_back(st); end
                "T": begin for (int i = 1; i < toks.size(); i++) st.data.push_back(byte'(toks[i].atohex())); steps.push_back(st); end
                "R": begin
                         st.delay_us = toks[1].atoi();
                         for (int i = 2; i < toks.size(); i++) st.data.push_back(byte'(toks[i].atohex()));
                         steps.push_back(st);
                     end
                "E": begin
                         automatic string v = "";
                         for (int i = 2; i < toks.size(); i++) v = {v, (i > 2) ? " " : "", toks[i]};
                         expect_map[toks[1]] = v;
                     end
                default: ;
            endcase
        end
        $fclose(fd);
        $display("[REPLAY] %s: %0d steps, %0d expected values", REPLAY_FILE, steps.size(), expect_map.num());
    end

    //------------------------------------------------------------------
    // Frames sent by the controller (UART decode + JVS framing)
    //------------------------------------------------------------------
    wire       m_dv;
    wire [7:0] m_byte;
    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (.i_Clock(clk), .i_Rx_Serial(uart_rx_from_master), .o_Rx_DV(m_dv), .o_Rx_Byte(m_byte));

    byte  m_frame[$];          // raw bytes of the frame being received
    byte  m_done_frames[$][$]; // complete frames, oldest first
    int   m_unesc_cnt;
    logic m_esc;
    byte  m_len;
    always @(posedge clk) begin
        if (m_dv) begin
            if (m_byte == 8'hE0) begin                   // SYNC: new frame
                m_frame = {8'hE0};
                m_unesc_cnt = 0; m_esc = 0;
            end else if (m_frame.size() > 0) begin
                m_frame.push_back(m_byte);
                if (m_byte == 8'hD0) m_esc = 1;
                else begin
                    if (m_unesc_cnt == 1) m_len = m_esc ? m_byte + 1 : m_byte;
                    m_unesc_cnt++;
                    m_esc = 0;
                    if (m_unesc_cnt >= 2 && m_unesc_cnt == 2 + m_len) begin
                        m_done_frames.push_back(m_frame);
                        m_frame = {};
                    end
                end
            end
        end
    end

    //------------------------------------------------------------------
    // Replay engine
    //------------------------------------------------------------------
    task automatic send_byte(input byte b);
        uart_tx_to_master = 1'b0;                              // start bit
        repeat (CLKS_PER_BIT) @(posedge clk);
        for (int i = 0; i < 8; i++) begin
            uart_tx_to_master = b[i];
            repeat (CLKS_PER_BIT) @(posedge clk);
        end
        uart_tx_to_master = 1'b1;                              // stop bit
        repeat (CLKS_PER_BIT) @(posedge clk);
    endtask

    function automatic string hexs(input byte q[$]);
        string s = "";
        foreach (q[i]) s = {s, $sformatf("%02X ", q[i])};
        return s;
    endfunction

    initial begin
        uart_tx_to_master = 1'b1;
        sense_in7 = 1'b1; link_io3 = 1'b1;
        replay_done = 1'b0;
        tx_frames_checked = 0; tx_mismatches = 0; rx_frames_sent = 0;
        @(negedge rst);
        #1;
        foreach (steps[k]) begin
            case (steps[k].kind)
                "L": begin
                         sense_in7 = steps[k].a[0];
                         link_io3  = steps[k].b[0];
                         $display("[REPLAY] %0t lines IN7=%0d IO3=%0d", $time, sense_in7, link_io3);
                     end
                "T": begin
                         automatic byte got[$];
                         wait (m_done_frames.size() > 0);
                         got = m_done_frames.pop_front();
                         tx_frames_checked++;
                         if (got != steps[k].data) begin
                             tx_mismatches++;
                             $display("[REPLAY] %0t TX MISMATCH frame %0d\n           got      %s\n           expected %s",
                                      $time, tx_frames_checked, hexs(got), hexs(steps[k].data));
                         end
                     end
                "R": begin
                         repeat (steps[k].delay_us * (CLK_FREQ / 1_000_000)) @(posedge clk);
                         foreach (steps[k].data[i]) send_byte(steps[k].data[i]);
                         rx_frames_sent++;
                     end
            endcase
        end
        $display("[REPLAY] %0t script done: %0d TX frames checked (%0d mismatches), %0d answers sent",
                 $time, tx_frames_checked, tx_mismatches, rx_frames_sent);
        replay_done = 1'b1;
    end
endmodule
