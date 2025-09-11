//////////////////////////////////////////////////////////////////////
// JVS Communication Module (jvs_com.sv)
// ALPHA Version - JVS Protocol Communication Layer
//
// This module handles the low-level JVS communication protocol including:
// - UART TX/RX with configurable baud rates
// - RS485 transceiver control with timing requirements
// - JVS frame encapsulation and decapsulation
// - Escape sequence handling (D0 DF → E0, D0 CF → D0)
// - Checksum calculation and validation
//
// RS485 Timing Requirements:
// - Setup delay: 10μs before transmission
// - Hold delay: 30μs after transmission for signal integrity
//
// Frame Format: SYNC(0xE0) + NODE + LENGTH + DATA + CHECKSUM
//
// Author: Totaly FuRy - Sebastien DUPONCHEEL (sduponch on GitHub)
// Compatible I/O Boards: NAJV2 (Tekken 7), NAJV (Time Crisis 4), TAITO CORP Ver2.0 (Viewlix)
// Incompatible: "No Brand;NAOMI Converter98701;ver2.0" (frames are ignored)
//////////////////////////////////////////////////////////////////////

module jvs_com
#(
    parameter JVS_BUFFER_SIZE = 256  // Maximum JVS frame data size
)
(
    // Clock and Reset
    input  wire         clk_sys,
    input  wire         reset,
    
    // UART Physical Interface
    input  wire         uart_rx,
    output wire         uart_tx,
    output wire         uart_rts_n,    // RS485 transmit enable (active low)
    
    // Configuration
    input  wire [15:0]  uart_baud_div, // UART baud rate divisor
    
    // High-level Protocol Interface - TX Path
    input  wire [7:0]   tx_data,       // Data byte (CMD + args)
    input  wire         tx_data_push,  // Pulse to push data byte into buffer
    input  wire         tx_cmd_push,   // Pulse to push command byte (stores in FIFO)
    input  wire [7:0]   dst_node,      // Destination node address
    input  wire         commit,        // Pulse to encapsulate and transmit
    output reg          tx_ready,      // Ready to accept new data
    
    // High-level Protocol Interface - RX Path
    output reg [7:0]    rx_byte,       // Current data byte
    input  wire         rx_next,       // Pulse to get next byte
    output reg [7:0]    rx_remaining,  // Bytes remaining (0 = current is last)
    output reg [7:0]    src_node,      // Source node of response
    output reg [7:0]    src_cmd,       // CMD from command FIFO
    input  wire         src_cmd_next,  // Pulse to get next command from FIFO
    output reg [4:0]    src_cmd_count, // Number of commands available in FIFO
    output reg          rx_complete,   // Pulse when frame complete
    output reg          rx_error,       // Checksum or format error
    
    // Debug/Status Interface
    output reg [3:0]    tx_state_debug, // Current TX state for debugging
    output reg [3:0]    rx_state_debug, // Current RX state for debugging
    output reg [7:0]    frames_tx_count, // Transmitted frame counter
    output reg [7:0]    frames_rx_count, // Received frame counter
    output reg [7:0]    checksum_errors_count // Checksum error counter
);

    //////////////////////////////////////////////////////////////////////
    // Local Parameters
    //////////////////////////////////////////////////////////////////////
    
    // RS485 Timing Constants (in clock cycles at 48MHz)
    localparam RS485_SETUP_CYCLES = 480;   // 10μs at 48MHz
    localparam RS485_HOLD_CYCLES  = 1440;  // 30μs at 48MHz
    
    // TX State Machine
    localparam [3:0] TX_IDLE        = 4'h0;
    localparam [3:0] TX_SETUP       = 4'h1;
    localparam [3:0] TX_SYNC        = 4'h2;
    localparam [3:0] TX_NODE        = 4'h3;
    localparam [3:0] TX_LENGTH      = 4'h4;
    localparam [3:0] TX_DATA        = 4'h5;
    localparam [3:0] TX_CHECKSUM    = 4'h6;
    localparam [3:0] TX_HOLD        = 4'h7;
    localparam [3:0] TX_ESCAPE_WAIT = 4'h8;
    
    // RX State Machine
    localparam [3:0] RX_IDLE        = 4'h0;
    localparam [3:0] RX_SYNC        = 4'h1;
    localparam [3:0] RX_NODE        = 4'h2;
    localparam [3:0] RX_LENGTH      = 4'h3;
    localparam [3:0] RX_DATA        = 4'h4;
    localparam [3:0] RX_CHECKSUM    = 4'h5;
    localparam [3:0] RX_VALIDATE    = 4'h6;
    localparam [3:0] RX_ESCAPE_WAIT = 4'h7;
    
    // JVS Protocol Constants
    localparam [7:0] JVS_SYNC       = 8'hE0;
    localparam [7:0] JVS_ESCAPE     = 8'hD0;
    localparam [7:0] JVS_ESC_SYNC   = 8'hDF;  // D0 DF → E0
    localparam [7:0] JVS_ESC_ESC    = 8'hCF;  // D0 CF → D0
    
    //////////////////////////////////////////////////////////////////////
    // Internal Registers and Wires
    //////////////////////////////////////////////////////////////////////
    
    // UART Interface
    wire        uart_tx_ready;
    wire        uart_tx_valid;
    wire [7:0]  uart_tx_data;
    wire        uart_rx_valid;
    wire [7:0]  uart_rx_data;
    
    // TX State Machine
    reg [3:0]   tx_state;
    reg [15:0]  tx_timer;
    reg [7:0]   tx_data_idx;
    reg [7:0]   tx_checksum;
    reg         tx_escape_pending;
    reg [7:0]   tx_escape_byte;
    
    // TX Data Buffer Management
    reg [7:0]   tx_data_buffer [0:255]; // Buffer for pushed data
    reg [7:0]   tx_data_count;          // Number of bytes in buffer
    reg [7:0]   tx_dst_node_latched;    // Latched destination node
    reg         tx_commit_pending;      // Commit pending processing
    
    // Command FIFO for multi-command frame tracking
    reg [7:0]   cmd_fifo [0:15];        // FIFO to store commands (up to 16 commands per frame)
    reg [3:0]   cmd_write_ptr;          // Write pointer for command FIFO
    reg [3:0]   cmd_read_ptr;           // Read pointer for command FIFO  
    reg [4:0]   cmd_count;              // Number of commands in FIFO (5-bit for overflow detection)
    reg         cmd_fifo_init;          // Pulse to initialize command FIFO reading
    
    // RX State Machine  
    reg [3:0]   rx_state;
    reg [7:0]   rx_data_idx;
    reg [7:0]   rx_checksum_calc;
    reg [7:0]   rx_checksum_recv;
    reg         rx_escape_mode;
    reg [7:0]   rx_byte_buffer;
    reg [7:0]   rx_length_internal; // Internal length storage
    reg [7:0]   rx_buffer [0:255];  // Internal RX data buffer
    reg [7:0]   rx_read_idx;        // Current read index for rx_next interface
    
    // Internal frame buffers
    reg [7:0]   tx_frame_node;
    reg [7:0]   tx_frame_length;
    reg [7:0]   tx_frame_data [0:JVS_BUFFER_SIZE-1];
    reg [7:0]   tx_frame_cmd;        // First byte (CMD) for RX echo
    
    //////////////////////////////////////////////////////////////////////
    // Generate block for API buffer to frame data copying
    //////////////////////////////////////////////////////////////////////
    
    genvar i;
    generate
        for (i = 0; i < JVS_BUFFER_SIZE; i++) begin : gen_buffer_copy
            always @(posedge clk_sys) begin
                if (reset) begin
                    tx_frame_data[i] <= 8'h00;
                end else if (tx_commit_pending && tx_state == TX_IDLE) begin
                    if (i < tx_data_count) begin
                        tx_frame_data[i] <= tx_data_buffer[i];
                    end else begin
                        tx_frame_data[i] <= 8'h00;  // Clear unused bytes
                    end
                end
            end
        end
    endgenerate

    //////////////////////////////////////////////////////////////////////
    // UART Instance
    //////////////////////////////////////////////////////////////////////
    
    uart_tx #(
        .CLKS_PER_BIT(uart_baud_div) // UART baud rate divisor
    ) uart_tx_inst (
        .i_Clock(clk_sys),
        .i_Tx_DV(uart_tx_valid),
        .i_Tx_Byte(uart_tx_data),
        .o_Tx_Active(),
        .o_Tx_Serial(uart_tx),
        .o_Tx_Done(uart_tx_ready)
    );
    
    uart_rx #(
        .CLKS_PER_BIT(uart_baud_div) // UART baud rate divisor
    ) uart_rx_inst (
        .i_Clock(clk_sys),
        .i_Rx_Serial(uart_rx),
        .o_Rx_DV(uart_rx_valid),
        .o_Rx_Byte(uart_rx_data)
    );
    
    //////////////////////////////////////////////////////////////////////
    // RS485 Control Logic
    //////////////////////////////////////////////////////////////////////
    
    reg rs485_tx_enable;
    assign uart_rts_n = ~rs485_tx_enable;
    
    //////////////////////////////////////////////////////////////////////
    // Checksum Calculation Function
    //////////////////////////////////////////////////////////////////////
    
    function automatic [7:0] calculate_checksum(
        input [7:0] node,
        input [7:0] length,
        input [7:0] data_array [0:JVS_BUFFER_SIZE-1],
        input [7:0] data_len
    );
        reg [7:0] sum;
        integer i;
        begin
            sum = node + length;
            for (i = 0; i < data_len; i = i + 1) begin
                sum = sum + data_array[i];
            end
            calculate_checksum = sum;
        end
    endfunction
    
    //////////////////////////////////////////////////////////////////////
    // TX State Machine
    //////////////////////////////////////////////////////////////////////
    
    reg uart_tx_start;
    assign uart_tx_valid = uart_tx_start;
    reg [7:0] uart_tx_byte;
    assign uart_tx_data = uart_tx_byte;
    
    //////////////////////////////////////////////////////////////////////
    // TX Data Buffer Management
    //////////////////////////////////////////////////////////////////////
    
    always @(posedge clk_sys) begin
        if (reset) begin
            tx_data_count <= 0;
            tx_commit_pending <= 1'b0;
            tx_ready <= 1'b1;
            
            // Initialize command FIFO
            cmd_write_ptr <= 0;
            cmd_read_ptr <= 0;
            cmd_count <= 0;
        end else begin
            // Handle data and command push
            if ((tx_data_push || tx_cmd_push) && tx_ready) begin
                tx_data_buffer[tx_data_count] <= tx_data;
                tx_data_count <= tx_data_count + 1;
                
                // Store first CMD for backward compatibility
                if (tx_data_count == 0) begin
                    tx_frame_cmd <= tx_data;
                end
            end
            
            // Handle command push - store in FIFO
            if (tx_cmd_push && tx_ready && cmd_count < 16) begin
                cmd_fifo[cmd_write_ptr] <= tx_data;
                cmd_write_ptr <= cmd_write_ptr + 1;
                cmd_count <= cmd_count + 1;
            end
            
            // Handle commit
            if (commit && tx_ready) begin
                tx_dst_node_latched <= dst_node;
                tx_commit_pending <= 1'b1;
                tx_ready <= 1'b0;
            end
            
            // Reset after processing
            if (tx_commit_pending && tx_state == TX_IDLE) begin
                tx_commit_pending <= 1'b0;
                tx_data_count <= 0;
                tx_ready <= 1'b1;
            end
            
            // Handle command FIFO initialization from RX
            if (cmd_fifo_init && cmd_count > 0) begin
                src_cmd <= cmd_fifo[cmd_read_ptr];
            end
            
            // Handle src_cmd_next to advance command FIFO
            if (src_cmd_next && cmd_count > 0) begin
                cmd_read_ptr <= cmd_read_ptr + 1;
                cmd_count <= cmd_count - 1;
                
                // Update src_cmd with next command if available
                if (cmd_count > 1) begin
                    src_cmd <= cmd_fifo[cmd_read_ptr + 1];
                end
            end
        end
    end
    
    //////////////////////////////////////////////////////////////////////
    // TX State Machine
    //////////////////////////////////////////////////////////////////////
    
    always @(posedge clk_sys) begin
        if (reset) begin
            tx_state <= TX_IDLE;
            tx_timer <= 0;
            rs485_tx_enable <= 1'b0;
            uart_tx_start <= 1'b0;
            tx_data_idx <= 0;
            tx_checksum <= 0;
            tx_escape_pending <= 1'b0;
            frames_tx_count <= 0;
            tx_state_debug <= TX_IDLE;
        end else begin
            tx_state_debug <= tx_state;
            uart_tx_start <= 1'b0; // Default, pulse when needed
            
            case (tx_state)
                TX_IDLE: begin
                    rs485_tx_enable <= 1'b0;
                    if (tx_commit_pending) begin
                        // Prepare frame from TX data buffer
                        tx_frame_node <= tx_dst_node_latched;
                        tx_frame_length <= tx_data_count;
                        
                        tx_timer <= 0;
                        tx_state <= TX_SETUP;
                    end
                end
                
                TX_SETUP: begin
                    // RS485 setup delay (10μs)
                    if (tx_timer < RS485_SETUP_CYCLES) begin
                        tx_timer <= tx_timer + 1;
                    end else begin
                        rs485_tx_enable <= 1'b1;
                        tx_timer <= 0;
                        tx_checksum <= 0;
                        tx_data_idx <= 0;
                        tx_state <= TX_SYNC;
                    end
                end
                
                TX_SYNC: begin
                    if (uart_tx_ready) begin
                        uart_tx_byte <= JVS_SYNC;
                        uart_tx_start <= 1'b1;
                        tx_state <= TX_NODE;
                    end
                end
                
                TX_NODE: begin
                    if (uart_tx_ready && !uart_tx_start) begin
                        if (needs_escape(tx_frame_node)) begin
                            if (!tx_escape_pending) begin
                                uart_tx_byte <= JVS_ESCAPE;
                                uart_tx_start <= 1'b1;
                                tx_escape_pending <= 1'b1;
                                tx_escape_byte <= get_escape_byte(tx_frame_node);
                            end else begin
                                uart_tx_byte <= tx_escape_byte;
                                uart_tx_start <= 1'b1;
                                tx_escape_pending <= 1'b0;
                                tx_checksum <= tx_checksum + tx_frame_node;
                                tx_state <= TX_LENGTH;
                            end
                        end else begin
                            uart_tx_byte <= tx_frame_node;
                            uart_tx_start <= 1'b1;
                            tx_checksum <= tx_checksum + tx_frame_node;
                            tx_state <= TX_LENGTH;
                        end
                    end
                end
                
                TX_LENGTH: begin
                    if (uart_tx_ready && !uart_tx_start) begin
                        if (needs_escape(tx_frame_length)) begin
                            if (!tx_escape_pending) begin
                                uart_tx_byte <= JVS_ESCAPE;
                                uart_tx_start <= 1'b1;
                                tx_escape_pending <= 1'b1;
                                tx_escape_byte <= get_escape_byte(tx_frame_length);
                            end else begin
                                uart_tx_byte <= tx_escape_byte;
                                uart_tx_start <= 1'b1;
                                tx_escape_pending <= 1'b0;
                                tx_checksum <= tx_checksum + tx_frame_length;
                                tx_state <= TX_DATA;
                            end
                        end else begin
                            uart_tx_byte <= tx_frame_length;
                            uart_tx_start <= 1'b1;
                            tx_checksum <= tx_checksum + tx_frame_length;
                            tx_state <= TX_DATA;
                        end
                    end
                end
                
                TX_DATA: begin
                    if (uart_tx_ready && !uart_tx_start) begin
                        if (tx_data_idx < tx_frame_length) begin
                            if (needs_escape(tx_frame_data[tx_data_idx])) begin
                                if (!tx_escape_pending) begin
                                    uart_tx_byte <= JVS_ESCAPE;
                                    uart_tx_start <= 1'b1;
                                    tx_escape_pending <= 1'b1;
                                    tx_escape_byte <= get_escape_byte(tx_frame_data[tx_data_idx]);
                                end else begin
                                    uart_tx_byte <= tx_escape_byte;
                                    uart_tx_start <= 1'b1;
                                    tx_escape_pending <= 1'b0;
                                    tx_checksum <= tx_checksum + tx_frame_data[tx_data_idx];
                                    tx_data_idx <= tx_data_idx + 1;
                                end
                            end else begin
                                uart_tx_byte <= tx_frame_data[tx_data_idx];
                                uart_tx_start <= 1'b1;
                                tx_checksum <= tx_checksum + tx_frame_data[tx_data_idx];
                                tx_data_idx <= tx_data_idx + 1;
                            end
                        end else begin
                            tx_state <= TX_CHECKSUM;
                        end
                    end
                end
                
                TX_CHECKSUM: begin
                    if (uart_tx_ready && !uart_tx_start) begin
                        if (needs_escape(tx_checksum)) begin
                            if (!tx_escape_pending) begin
                                uart_tx_byte <= JVS_ESCAPE;
                                uart_tx_start <= 1'b1;
                                tx_escape_pending <= 1'b1;
                                tx_escape_byte <= get_escape_byte(tx_checksum);
                            end else begin
                                uart_tx_byte <= tx_escape_byte;
                                uart_tx_start <= 1'b1;
                                tx_escape_pending <= 1'b0;
                                tx_timer <= 0;
                                tx_state <= TX_HOLD;
                            end
                        end else begin
                            uart_tx_byte <= tx_checksum;
                            uart_tx_start <= 1'b1;
                            tx_timer <= 0;
                            tx_state <= TX_HOLD;
                        end
                    end
                end
                
                TX_HOLD: begin
                    // RS485 hold delay (30μs)
                    if (tx_timer < RS485_HOLD_CYCLES) begin
                        tx_timer <= tx_timer + 1;
                    end else begin
                        rs485_tx_enable <= 1'b0;
                        frames_tx_count <= frames_tx_count + 1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                default: tx_state <= TX_IDLE;
            endcase
        end
    end
    
    //////////////////////////////////////////////////////////////////////
    // RX State Machine
    //////////////////////////////////////////////////////////////////////
    
    always @(posedge clk_sys) begin
        if (reset) begin
            rx_state <= RX_IDLE;
            rx_complete <= 1'b0;
            rx_error <= 1'b0;
            rx_data_idx <= 0;
            rx_checksum_calc <= 0;
            rx_escape_mode <= 1'b0;
            frames_rx_count <= 0;
            checksum_errors_count <= 0;
            rx_state_debug <= RX_IDLE;
            cmd_fifo_init <= 1'b0;
        end else begin
            rx_state_debug <= rx_state;
            rx_complete <= 1'b0; // Default, pulse when frame complete
            rx_error <= 1'b0; // Default
            cmd_fifo_init <= 1'b0; // Default, pulse when initializing command FIFO
            
            if (uart_rx_valid) begin
                case (rx_state)
                    RX_IDLE: begin
                        if (uart_rx_data == JVS_SYNC) begin
                            rx_checksum_calc <= 0;
                            rx_data_idx <= 0;
                            rx_escape_mode <= 1'b0;
                            rx_state <= RX_NODE;
                        end
                    end
                    
                    RX_NODE: begin
                        if (handle_escape_rx(uart_rx_data, rx_escape_mode, rx_byte_buffer)) begin
                            src_node <= rx_byte_buffer;
                            rx_checksum_calc <= rx_checksum_calc + rx_byte_buffer;
                            rx_state <= RX_LENGTH;
                        end
                    end
                    
                    RX_LENGTH: begin
                        if (handle_escape_rx(uart_rx_data, rx_escape_mode, rx_byte_buffer)) begin
                            rx_length_internal <= rx_byte_buffer;
                            rx_checksum_calc <= rx_checksum_calc + rx_byte_buffer;
                            rx_data_idx <= 0;
                            rx_state <= RX_DATA;
                        end
                    end
                    
                    RX_DATA: begin
                        if (handle_escape_rx(uart_rx_data, rx_escape_mode, rx_byte_buffer)) begin
                            if (rx_data_idx < rx_length_internal) begin
                                rx_buffer[rx_data_idx] <= rx_byte_buffer;
                                rx_checksum_calc <= rx_checksum_calc + rx_byte_buffer;
                                rx_data_idx <= rx_data_idx + 1;
                            end else begin
                                rx_checksum_recv <= rx_byte_buffer;
                                rx_state <= RX_VALIDATE;
                            end
                        end
                    end
                    
                    RX_VALIDATE: begin
                        if (rx_checksum_calc == rx_checksum_recv) begin
                            frames_rx_count <= frames_rx_count + 1;
                            
                            // Initialize command FIFO reading - start with first command
                            cmd_fifo_init <= 1'b1;
                            
                            // Setup sequential read interface
                            rx_read_idx <= 0;
                            rx_byte <= rx_buffer[0];  // First byte ready
                            rx_remaining <= rx_length_internal - 1;
                            rx_complete <= 1'b1;
                        end else begin
                            checksum_errors_count <= checksum_errors_count + 1;
                            rx_error <= 1'b1;
                        end
                        rx_state <= RX_IDLE;
                    end
                    
                    default: rx_state <= RX_IDLE;
                endcase
            end
        end
        
        // Handle rx_next sequential read interface
        if (rx_next && rx_remaining > 0) begin
            rx_read_idx <= rx_read_idx + 1;
            rx_byte <= rx_buffer[rx_read_idx + 1];
            rx_remaining <= rx_remaining - 1;
        end
        
        // Always output current command count
        src_cmd_count <= cmd_count;
    end
    
    //////////////////////////////////////////////////////////////////////
    // Helper Functions
    //////////////////////////////////////////////////////////////////////
    
    function automatic needs_escape(input [7:0] data);
        needs_escape = (data == JVS_SYNC || data == JVS_ESCAPE);
    endfunction
    
    function automatic [7:0] get_escape_byte(input [7:0] data);
        if (data == JVS_SYNC)
            get_escape_byte = JVS_ESC_SYNC;
        else if (data == JVS_ESCAPE)
            get_escape_byte = JVS_ESC_ESC;
        else
            get_escape_byte = data;
    endfunction
    
    function automatic handle_escape_rx(
        input [7:0] rx_byte,
        ref logic escape_mode,
        ref [7:0] decoded_byte
    );
        if (escape_mode) begin
            if (rx_byte == JVS_ESC_SYNC) begin
                decoded_byte = JVS_SYNC;
                escape_mode = 1'b0;
                handle_escape_rx = 1'b1;
            end else if (rx_byte == JVS_ESC_ESC) begin
                decoded_byte = JVS_ESCAPE;
                escape_mode = 1'b0;
                handle_escape_rx = 1'b1;
            end else begin
                escape_mode = 1'b0;
                handle_escape_rx = 1'b0; // Invalid escape sequence
            end
        end else if (rx_byte == JVS_ESCAPE) begin
            escape_mode = 1'b1;
            handle_escape_rx = 1'b0;
        end else begin
            decoded_byte = rx_byte;
            handle_escape_rx = 1'b1;
        end
    endfunction

endmodule