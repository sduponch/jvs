//////////////////////////////////////////////////////////////////////
// JVS Controller Module for Analogizer - ALPHA VERSION
// Partial JVS Master implementation optimized for gaming performance
// 
// ⚠️  ALPHA STATUS - INCOMPLETE PROTOCOL IMPLEMENTATION ⚠️
//
// This module implements a simplified JVS (JAMMA Video Standard) Master 
// controller that allows connecting JVS arcade cabinets to the Analogue 
// Pocket through the Analogizer. 
//
// CURRENT STATUS:
// - Core protocol working (Reset, Address assignment, Input polling)
// - Basic button mapping functional (D-PAD, face buttons, START/SELECT)
// - JVS escape sequence support implemented (D0 DF → E0, D0 CF → D0)
// - Optimized for gaming performance (1ms polling, minimal latency)
// - FPGA resource usage optimized with configurable buffer sizes
// - Protocol implementation incomplete (missing capabilities, device info)
// - Button positions may need verification/adjustment
// - Single device support only
//
// ARCHITECTURE:
// - RS485 State Machine: Manages transceiver direction and timing
// - Main State Machine: Handles JVS protocol sequence and commands  
// - RX State Machine: Processes incoming JVS responses with escape sequence decoding
// - Two-buffer system: Raw buffer for incoming data, processed buffer for unescaped data
//
// HARDWARE REQUIREMENTS:
// - External MAX485 or equivalent RS485 transceiver
// - Proper 120Ω termination for reliable communication
// - JVS-compatible arcade cabinet (tested with Namco Noir)
//
// Author: DUPONCHEEL Sébastien (sduponch on GitHub)
// Project: Analogizer JVS Controller
// Status: Alpha - Work in Progress
// Date: 2025
//////////////////////////////////////////////////////////////////////
//Use: set_global_assignment -name VERILOG_MACRO "USE_DUMMY_JVS_DATA=1" 
//in project .qsf to use dummy data for simulation without JVS device

`default_nettype none
`timescale 1ns / 1ps

import jvs_node_info_pkg::*;

module jvs_controller #(parameter MASTER_CLK_FREQ = 50_000_000)
(
    // System clock and control signals
    input logic i_clk,        // System clock (typically 50MHz)
    input logic i_rst,        // Asynchronous reset (active high)
    input logic i_ena,        // Module enable (active high)
    input logic i_stb,        // Strobe signal (not used in final version)
    
    // UART interface signals for RS485 communication
    input logic i_uart_rx,    // Serial data received from JVS device
    output logic o_uart_tx,   // Serial data transmitted to JVS device
    input logic i_sense,      // JVS SENSE line (read-only for master)
    output logic o_rx485_dir, // RS485 transceiver direction control (0=RX, 1=TX)
    
    // Output registers compatible with Analogue Pocket SNAC format
    output logic [15:0] p1_btn_state,   // Player 1 button states
    output logic [31:0] p1_joy_state,   // Player 1 analog stick states
    output logic [15:0] p2_btn_state,   // Player 2 button states
    output logic [31:0] p2_joy_state,   // Player 2 analog stick states
    output logic [15:0] p3_btn_state,   // Player 3 button states (reserved)
    output logic [15:0] p4_btn_state,    // Player 4 button states (reserved)
    
    // Screen position outputs (light gun/touch screen) - raw 16-bit data
    output logic [15:0] screen_pos_x,   // Screen X position (16-bit from JVS)
    output logic [15:0] screen_pos_y,   // Screen Y position (16-bit from JVS)
    output logic has_screen_pos,        // Device supports screen position inputs
    
    // GPIO control from SNAC
    input logic [7:0] gpio_output_value, // GPIO output value from SNAC (0x80=active, 0x00=inactive)

    //JVS node information structure
    output logic jvs_data_ready,
    output jvs_node_info_t jvs_nodes,
    //RAM interface for node names (for debug/display purposes)
    output logic [7:0] node_name_rd_data,
    input logic [6:0] node_name_rd_addr
); 

//==================================================================================
// Show in Quartus Synthesis if dummy data is used for simulation without JVS device
//==================================================================================
`ifdef USE_DUMMY_JVS_DATA
  initial $warning("=== USE_DUMMY_JVS_DATA is defined (=%0d). Using DUMMY data for JVS IO device ===", `USE_DUMMY_JVS_DATA);
`else
  initial $warning("=== USE_DUMMY_JVS_DATA is NOT defined.  Using REAL data for JVS IO device ===");
`endif


    //=========================================================================
    // UART TIMING CONFIGURATION
    //=========================================================================
    // Calculate UART clock divider for 115200 baud rate
    // Formula: UART_CLKS_PER_BIT = System_Clock_Frequency / Baud_Rate
    localparam UART_CLKS_PER_BIT = MASTER_CLK_FREQ / 115200;
    
    //=========================================================================
    // UART TRANSMITTER INSTANCE
    //=========================================================================
    // Control signals for UART transmitter
    logic uart_tx_dv;              // Data valid strobe to start transmission
    logic [7:0] uart_tx_byte;      // Byte to transmit
    logic uart_tx_active;         // High when transmission is in progress
    logic uart_tx_done;           // Pulse when transmission completes
    
    // Instantiate UART transmitter module
    uart_tx #(.CLKS_PER_BIT(UART_CLKS_PER_BIT)) uart_tx_inst (
        .i_Clock(i_clk),
        .i_Tx_DV(uart_tx_dv),
        .i_Tx_Byte(uart_tx_byte),
        .o_Tx_Active(uart_tx_active),
        .o_Tx_Serial(o_uart_tx),
        .o_Tx_Done(uart_tx_done)
    );
    
    //=========================================================================
    // UART RECEIVER INSTANCE
    //=========================================================================
    // Status signals from UART receiver
    wire uart_rx_dv;             // Data valid pulse when byte is received
    wire [7:0] uart_rx_byte;     // Received byte data
    
    // Instantiate UART receiver module
    uart_rx #(.CLKS_PER_BIT(UART_CLKS_PER_BIT)) uart_rx_inst (
        .i_Clock(i_clk),
        .i_Rx_Serial(i_uart_rx),
        .o_Rx_DV(uart_rx_dv),
        .o_Rx_Byte(uart_rx_byte)
    );

    //=========================================================================
    // JVS PROTOCOL CONSTANTS
    //=========================================================================

    // Common delay timings (in clock cycles at MASTER_CLK_FREQ)
    localparam logic [31:0] INIT_DELAY_COUNT = MASTER_CLK_FREQ * 5.4; // 5.4 seconds
    localparam logic [31:0] FIRST_RESET_DELAY_COUNT = MASTER_CLK_FREQ * 2; // 2 seconds
    localparam logic [31:0] SECOND_RESET_DELAY_COUNT = MASTER_CLK_FREQ / 2; // 0.5 seconds
    localparam logic [15:0] TX_SETUP_DELAY_COUNT = MASTER_CLK_FREQ / 100_000; // ~10µs
    localparam logic [15:0] TX_HOLD_DELAY_COUNT = MASTER_CLK_FREQ / 33_333; // ~30µs
    localparam logic [31:0] RX_TIMEOUT_COUNT = MASTER_CLK_FREQ / 100; // 10ms
    localparam logic [31:0] POLL_INTERVAL_COUNT = MASTER_CLK_FREQ / 1_000; // 1ms


    //=========================================================================
    // JVS COMMAND DEFINITIONS (Based on JVS Specification)
    //=========================================================================
    
    // Standard JVS protocol bytes
    localparam JVS_SYNC_BYTE = 8'hE0;        // Frame start synchronization byte
    localparam JVS_BROADCAST_ADDR = 8'hFF;   // Broadcast address for all devices
    localparam JVS_HOST_ADDR = 8'h00;        // Host/Master address
    
    // Global Commands - Work with any device address or broadcast (0xFF)
    localparam CMD_RESET = 8'hF0;            // [F0 D9] Reset all devices on bus
                                             // Args: D9 (fixed argument)
                                             // Response: No response (devices reset)
    localparam CMD_RESET_ARG = 8'hD9;        // Argument byte that must follow CMD_RESET
    
    localparam CMD_SETADDR = 8'hF1;          // [F1 addr] Assign address to device
                                             // Args: addr (1-31, device address)
                                             // Response: [report] - report=01 if success
    
    localparam CMD_COMMCHG = 8'hF2;          // [F2 baudrate] Change communication speed
                                             // Args: baudrate (communication speed code)
                                             // Response: [report] - report=01 if success
    
    // Initialize Commands - Device identification and capability discovery
    localparam CMD_IOIDENT = 8'h10;          // [10] Read device identification string
                                             // Args: none
                                             // Response: [report name_string 00]
                                             //   name_string: ASCII device name (manufacturer;product;version;region,comment)
    
    localparam CMD_CMDREV = 8'h11;           // [11] Read command format revision
                                             // Args: none  
                                             // Response: [report revision]
                                             //   revision: BCD format (e.g. 0x13 for v1.3)
    
    localparam CMD_JVSREV = 8'h12;           // [12] Read JVS specification revision
                                             // Args: none
                                             // Response: [report revision]
                                             //   revision: BCD format (e.g. 0x30 for v3.0)
    
    localparam CMD_COMMVER = 8'h13;          // [13] Read communication version
                                             // Args: none
                                             // Response: [report version]
                                             //   version: BCD format (e.g. 0x10 for v1.0)
    
    localparam CMD_FEATCHK = 8'h14;          // [14] Check device features/capabilities
                                             // Args: none
                                             // Response: [report func_data... 00]
                                             //   func_data: loop of 4-byte blocks [func_code param1 param2 param3] loop end with 00
    
    localparam CMD_MAINID = 8'h15;           // [15] Send main board ID to I/O device
                                             // Args: [main_pcb_id_string 00] - ASCII string up to 100 chars
                                             //   Format: "Maker;Game;Version;Details" separated by semicolons
                                             //   Example: "NAMCO LTD.;TEKKEN2;ver1.6; TEKKEN2 ver B"
                                             // Response: [report] - report=01 if success
    
    // Data I/O Commands - Input reading and data retrieval
    localparam CMD_SWINP = 8'h20;            // [20 players bytes] Read switch inputs (digital buttons)
                                             // Args: players (number of players), bytes (total bytes needed for bits per player)
                                             // Response: [report switch_data...]
                                             //   switch_data: players × bytes of digital input data
    
    localparam CMD_COININP = 8'h21;          // [21 slots] Read coin inputs and counter
                                             // Args: slots (number of coin slots to read)
                                             // Response: [report coin_status...]
                                             //   coin_status: loop of 2 bytes [condition(2 bits) counter_MSB(6 bits) counter_LSB]
    
    // Coin Input Condition Codes (Table 12)
    localparam COIN_CONDITION_NORMAL = 2'b00;        // Normal operation
    localparam COIN_CONDITION_JAM = 2'b01;           // Coin jam detected
    localparam COIN_CONDITION_DISCONNECTED = 2'b10;  // Coin mechanism disconnected  
    localparam COIN_CONDITION_BUSY = 2'b11;          // Coin mechanism busy

    localparam CMD_ANLINP = 8'h22;           // [22 channels] Read analog inputs
                                             // Args: channels (number of analog channels)
                                             // Response: [report analog_data...]
                                             //   analog_data: 2 bytes per channel [data_MSB data_LSB]
    
    localparam CMD_ROTINP = 8'h23;           // [23 channels] Read rotary inputs
                                             // Args: channels (number of rotary channels to read)
                                             // Response: [report rotary_data...]
                                             //   rotary_data: 2 bytes per channel [data_MSB data_LSB]
    
    localparam CMD_KEYINP = 8'h24;           // [24] Read keycode inputs
                                             // Args: none
                                             // Response: [report keycode]
                                             //   keycode: ASCII key code or 00 if no key
    
    localparam CMD_SCRPOSINP = 8'h25;        // [25 channels] Read screen position inputs (light gun/touch)
                                             // Args: channel index to read
                                             // Response: [report pos_data...]
                                             //   pos_data: 4 bytes [x_MSB x_LSB y_MSB y_LSB]
    
    localparam CMD_MISCSWINP = 8'h26;        // [26 bytes] Read miscellaneous switch inputs
                                             // Args: bytes (number of misc input bytes, depends on bits defined in feature check)
                                             // Response: [report misc_data...]
                                             //   misc_data: specified number of misc input bytes

    localparam CMD_PAYCNT = 8'h2E;           // [2E channel_index] Payout coins/tokens aka. redemption
                                             // Args: channel_index
                                             // Response: [report hopper_status remaining_hi remaining_mid remaining_low]

    localparam CMD_RETRANSMIT = 8'h2F;       // [2F] Retransmit previous response
                                             // Args: none
                                             // Response: Previous response is retransmitted

    localparam CMD_COINDEC = 8'h30;          // [30 slots_index amount_msb amount_lsb] Decrease selected coin counter of specified value
                                             // Args: slot_index, amount_msb, amount_lsb
                                             // Response: [report] - report=01 if success

    localparam CMD_PAYINC = 8'h31;           // [31 slots payval...] Increase payout counters
                                             // Args: slots (number of payout slots), payval per slot (increase amount)
                                             // Response: [report] - report=01 if success
    
    localparam CMD_OUTPUT1 = 8'h32;          // [32 bytes data...] General purpose output 1
                                             // Args: bytes (number of output bytes), data per byte
                                             // Response: [report] - report=01 if success
    
    localparam CMD_ANLOUT = 8'h33;           // [33 channels data...] Analog output control
                                             // Args: channels (number of analog outputs), 2 bytes data per channel [MSB LSB]
                                             // Response: [report] - report=01 if success
    
    localparam CMD_CHAROUT = 8'h34;          // [34 line pos string...] Character display output
                                             // Args: line (display line), pos (position), string data
                                             // Response: [report] - report=01 if success
    
    localparam CMD_COININC = 8'h35;          // [35 slots incval...] Increase coin counters
                                             // Args: slots (number of coin slots), incval per slot (increase amount)
                                             // Response: [report] - report=01 if success
    
    localparam CMD_PAYDEC = 8'h36;           // [36 slots decval...] Decrease payout counters
                                             // Args: slots (number of payout slots), decval per slot (decrease amount)
                                             // Response: [report] - report=01 if success
    
    localparam CMD_OUTPUT2 = 8'h37;          // [37 bytes data...] General purpose output 2
                                             // Args: bytes (number of output bytes), data per byte
                                             // Response: [report] - report=01 if success
    
    localparam CMD_OUTPUT3 = 8'h38;          // [38 bytes data...] General purpose output 3
                                             // Args: bytes (number of output bytes), data per byte
                                             // Response: [report] - report=01 if success
    

    // Status Codes - General response status (position 3 in frame)
    localparam STATUS_NORMAL = 8'h01;        // Normal operation status
    localparam STATUS_UNKNOWN_CMD = 8'h02;   // Unknown command received  
    localparam STATUS_SUM_ERROR = 8'h03;     // Checksum error in received data
    localparam STATUS_ACK_OVERFLOW = 8'h04;  // Acknowledgment overflow
    localparam STATUS_BUSY = 8'h05;          // Device busy, cannot process command
    
    // Report Codes - Command-specific status (position 4+ in frame)
    localparam REPORT_NORMAL = 8'h01;        // Normal operation
    localparam REPORT_PARAM_ERROR_COUNT = 8'h02; // Parameter error (incorrect number)
    localparam REPORT_PARAM_ERROR_DATA = 8'h03;  // Parameter error (invalid data)
    localparam REPORT_BUSY = 8'h04;          // Busy (cannot receive more commands)
    

    // JVS Escape sequence constants for data byte escaping
    localparam JVS_ESCAPE_BYTE = 8'hD0;      // Escape marker byte
    localparam JVS_ESCAPED_E0 = 8'hDF;       // E0 becomes D0 DF
    localparam JVS_ESCAPED_D0 = 8'hCF;       // D0 becomes D0 CF
    
    // Function Codes - Used in feature check responses
    localparam FUNC_INPUT_DIGITAL = 8'h01;    // [01 players bytesperplayer unused] Digital inputs
    localparam FUNC_INPUT_COIN = 8'h02;       // [02 slots unused unused] Coin inputs
    localparam FUNC_INPUT_ANALOG = 8'h03;     // [03 channels bits unused] Analog inputs (channels×bits resolution)
    localparam FUNC_INPUT_ROTARY = 8'h04;     // [04 channels unused unused] Rotary encoder inputs
    localparam FUNC_INPUT_KEYCODE = 8'h05;    // [05 unused unused unused] Keycode inputs
    localparam FUNC_INPUT_SCREEN_POS = 8'h06; // [06 channels bits unused] Screen position inputs (channels×bits)
    localparam FUNC_INPUT_MISC_DIGITAL = 8'h07; // [07 bytes unused unused] Miscellaneous digital inputs
    localparam FUNC_OUTPUT_CARD = 8'h10;      // [10 slots unused unused] Card system outputs
    localparam FUNC_OUTPUT_HOPPER = 8'h11;    // [11 slots unused unused] Medal/token hopper outputs  
    localparam FUNC_OUTPUT_DIGITAL = 8'h12;   // [12 bytes unused unused] Digital outputs (lights/solenoids)
    localparam FUNC_OUTPUT_ANALOG = 8'h13;    // [13 channels unused unused] Analog outputs
    localparam FUNC_OUTPUT_CHAR = 8'h14;      // [14 lines columns type] Character display outputs
    localparam FUNC_OUTPUT_BACKUP = 8'h15;    // [15 unused unused unused] Backup data support

    localparam JVS_FUNC_LENGTH = 8'd4;          // Each function block is 4 bytes long
    
    // Character Output Type codes (Table 9)
    localparam JVS_CHAR_TYPE_UNKNOWN = 8'h00;           // Unknown
    localparam JVS_CHAR_TYPE_ASCII_NUMERIC = 8'h01;     // ASCII (numeric)
    localparam JVS_CHAR_TYPE_ASCII_ALPHANUM = 8'h02;    // ASCII (alphanumeric)
    localparam JVS_CHAR_TYPE_ASCII_KATAKANA = 8'h03;    // ASCII (alphanumeric, half-width katakana)
    localparam JVS_CHAR_TYPE_ASCII_KANJI = 8'h04;       // ASCII (kanji support, SHIFT-JIS)
    
    // JVS Frame structure constants for better code readability
    localparam JVS_SYNC_POS = 8'd0;          // Position of sync byte (E0)
    localparam JVS_ADDR_POS = 8'd1;          // Position of address byte
    localparam JVS_LENGTH_POS = 8'd2;        // Position of length byte
    localparam JVS_DATA_START = 8'd4;        // Start position of data bytes (RX) - after status and include report bytes
    localparam JVS_STATUS_POS = 8'd3;        // Position of status byte in response
    localparam JVS_REPORT_POS = 8'd4;        // Position of report byte in response (should be processed)
    localparam JVS_CMD_START = 8'd3;         // Start position of command bytes (TX)
    localparam JVS_OVERHEAD = 8'd2;          // Overhead for length calculation (includes checksum + command byte)
    localparam JVS_CHECKSUM_SIZE = 8'd1;     // Checksum is coded on one byte

    // Buffer size configuration for resource optimization
    localparam RX_BUFFER_SIZE = 128;         // Size of RX buffers (I/O Identify max 106 bytes)
    localparam TX_BUFFER_SIZE = 24;          // Size of TX buffer (max frame ~21 bytes)
    
    // JVS node management constants
    //localparam MAX_JVS_NODES = 2;            // Maximum supported JVS nodes (current implementation)
    //localparam NODE_NAME_SIZE = 100;         // Maximum size for node identification strings (per JVS spec)
    // Defined in jvs_node_info_pkg.sv

    //=========================================================================
    // STATE MACHINE DEFINITIONS
    //=========================================================================
    
    // Main State Machine - Controls overall JVS protocol sequence
    localparam STATE_IDLE = 4'h0;             // Idle state - continuous input polling
    localparam STATE_INIT_DELAY = 4'h1;       // Initial delay for system stabilization
    localparam STATE_FIRST_RESET = 4'h2;      // Send first reset command
    localparam STATE_FIRST_RESET_DELAY = 4'h3; // Delay after first reset
    localparam STATE_SECOND_RESET = 4'h4;     // Send second reset command
    localparam STATE_SECOND_RESET_DELAY = 4'h5; // Delay after second reset
    localparam STATE_SEND_SETADDR = 4'h6;     // Send address assignment command
    localparam STATE_SEND_READID = 4'h7;      // Send device ID request
    localparam STATE_SEND_CMDREV = 4'h8;      // Send command revision request
    localparam STATE_SEND_JVSREV = 4'h9;      // Send JVS revision request
    localparam STATE_SEND_COMMVER = 4'hA;     // Send communications version request
    localparam STATE_SEND_FEATCHK = 4'hB;     // Send feature check request
    localparam STATE_SEND_INPUTS = 4'hC;      // Send input state request (start progressive build)
    localparam STATE_WAIT_TX_SETUP = 4'hD;    // Wait for RS485 setup time
    localparam STATE_TRANSMIT_BYTE = 4'hE;    // Transmit data bytes
    localparam STATE_WAIT_TX_DONE = 4'hF;     // Wait for transmission completion
    localparam STATE_WAIT_TX_HOLD = 5'h10;    // Wait for RS485 hold time
    localparam STATE_WAIT_RX = 5'h11;         // Wait for device response
    
    // INPUT BUILDING SUB-STATES - Progressive frame construction
    localparam STATE_SEND_INPUTS_SWITCH = 5'h12;   // Add switch inputs if available
    localparam STATE_SEND_INPUTS_COIN = 5'h13;     // Add coin inputs if available  
    localparam STATE_SEND_INPUTS_ANALOG = 5'h14;   // Add analog inputs if available
    localparam STATE_SEND_INPUTS_ROTARY = 5'h15;   // Add rotary inputs if available
    localparam STATE_SEND_INPUTS_KEYCODE = 5'h16;  // Add keycode inputs if available
    localparam STATE_SEND_INPUTS_SCREEN = 5'h17;   // Add screen position inputs if available
    localparam STATE_SEND_INPUTS_MISC = 5'h18;     // Add misc inputs if available
    localparam STATE_SEND_OUTPUT_DIGITAL = 5'h19; // Send output digital command for GPIO
    localparam STATE_SEND_FINALIZE = 5'h1A; // Finalize frame and transmit
    
    // RS485 State Machine - Controls transceiver direction with proper timing
    localparam RS485_RECEIVE = 2'b00;         // Receive mode (default)
    localparam RS485_TX_SETUP = 2'b01;        // Setup time before transmission
    localparam RS485_TRANSMIT = 2'b10;        // Active transmission mode
    localparam RS485_TX_HOLD = 2'b11;         // Hold time after transmission

    // RX State Machine - Processes incoming JVS frames byte by byte
    localparam RX_IDLE = 3'h0;                // Waiting for sync byte
    localparam RX_READ_ADDR = 3'h1;           // Reading address byte
    localparam RX_READ_SIZE = 3'h2;           // Reading length byte
    localparam RX_READ_DATA = 3'h3;           // Reading data bytes and checksum
    localparam RX_UNESCAPE = 3'h4;            // Copy from raw buffer to final buffer, processing escapes
    localparam RX_PROCESS = 3'h5;             // Processing complete and unescaped frame
    localparam RX_COPY_NAME = 3'h6;           // Copy node name from response data
    localparam RX_PARSE_FEATURES = 3'h7;      // Parse feature/capability data
    
    // Additional RX states for input parsing (using 5-bit to expand beyond 3'h7)
    localparam RX_PARSE_INPUTS_START = 5'h8;   // Initialize input response parsing
    localparam RX_PARSE_INPUTS_SWITCH = 5'h9;  // Parse switch inputs data
    localparam RX_PARSE_SWINP_PLAYER = 5'hA;   // Parse individual player SWINP data (recursive)
    localparam RX_PARSE_INPUTS_COIN = 5'hB;    // Parse coin inputs data  
    localparam RX_PARSE_INPUTS_ANALOG = 5'hC;  // Parse analog inputs data
    localparam RX_PARSE_INPUTS_ANALOG_DATA = 5'hD; // Parse analog inputs channel ANLINP data (recursive)
    localparam RX_PARSE_INPUTS_ROTARY = 5'hE;  // Parse rotary inputs data
    localparam RX_PARSE_INPUTS_KEYCODE = 5'hF;  // Parse keycode inputs data
    localparam RX_PARSE_INPUTS_SCREEN_POS = 5'h10; // Parse screen position inputs data 
    localparam RX_PARSE_INPUTS_MISC_DIGITAL = 5'h11; // Parse misc digital inputs data
    localparam RX_PARSE_OUTPUT_DIGITAL = 5'h12;  // Parse output digital response
    localparam RX_PARSE_INPUTS_COMPLETE = 5'h13; // Complete parsing and return to idle

    //=========================================================================
    // STATE VARIABLES AND CONTROL REGISTERS
    //=========================================================================
    // Current state for each state machine
    logic [4:0] main_state;        // Main protocol state
    logic [1:0] rs485_state;       // RS485 transceiver state
    logic [4:0] rx_state;          // Receive frame processing state (5-bit to support new parsing states)
    
    // Transmission buffer and control
    logic [7:0] tx_buffer [0:TX_BUFFER_SIZE-1];  // Buffer for outgoing JVS frames
    logic [7:0] tx_length;         // Total length of current transmission
    logic [7:0] tx_counter;        // Current byte position in transmission
    logic [7:0] tx_checksum;       // Running checksum calculation
    logic rs485_tx_request;        // Signal to start RS485 transmission
    
    // Reception buffer and control
    logic [7:0] rx_buffer_raw [0:RX_BUFFER_SIZE-1]; // Buffer for raw incoming JVS frames with escape sequences
    logic [7:0] rx_buffer [0:RX_BUFFER_SIZE-1]; // Buffer for unescaped JVS frames (final processed data)
    logic [7:0] rx_length;         // Length of current incoming frame
    logic [7:0] rx_counter;        // Current byte position in reception
    logic [7:0] rx_checksum;       // Running checksum verification
    // Generic copy variables (used for unescape and name copying)
    logic [7:0] copy_read_idx;      // Read index for copy operations
    
    // Single player analog concatenation registers (24-bit each for MSB+LSB)
    logic [23:0] p1_analog_x_24bit; // X axis: Ch1(MSB 12-bit) + Ch3(LSB 12-bit)
    logic [23:0] p1_analog_y_24bit; // Y axis: Ch2(MSB 12-bit) + Ch4(LSB 12-bit)
    logic [7:0] copy_write_idx;     // Write index for copy operations
    logic [7:0] request_build_idx;  // Index register for TX buffer construction

    logic [3:0] current_player;     // Current player index for SWINP parsing (0, 1, 2...)
    logic [3:0] current_channel;    // Current channel index for ANLINP

    // Timing and protocol control
    logic [31:0] delay_counter;    // Multi-purpose delay counter
    logic [31:0] timeout_counter;  // Timeout counter for waiting states
    logic [31:0] poll_timer;       // Timer for input polling frequency
    logic [7:0] current_device_addr; // Address assigned to JVS device (usually 0x01)
    logic rx_frame_complete;       // Flag indicating frame has been processed and ready for next step
    logic [4:0] last_tx_state;     // Tracks the last command sent for response handling
    
    
    //=========================================================================
    // JVS NODE INFORMATION STRUCTURES
    //=========================================================================
    // Structure to store information about each JVS node
    jvs_node_info_t jvs_nodes_r;

//see comments in JVS_Debugger.qsf under [JVS project settings] 
`ifdef USE_DUMMY_JVS_DATA
	jvs_node_info_t jvs_nodes_r2;
	 
    localparam jvs_node_info_t JVS_INFO_INIT = '{
        node_id: '{8'h01, 8'h02},
        node_cmd_ver: '{8'h13, 8'h11}, 
        node_jvs_ver: '{8'h30, 8'h30},  
        node_com_ver: '{8'h10, 8'h10},
        // Initialize dummy capabilities based on typical JVS device
        node_players: '{4'h2, 4'h1},              // 2 players for first device, 1 for second
        node_buttons: '{8'h0D, 8'h06},            // 13 buttons for P1, 6 for P2
        node_analog_channels: '{4'h2, 4'h0},      // 2 analog channels for first device
        node_analog_bits: '{8'h0A, 8'h08},        // 10-bit analog for first device, 8-bit for second
        node_rotary_channels: '{4'h0, 4'h0},      // No rotary encoders
        node_coin_slots: '{4'h2, 4'h1},           // 2 coin slots for first device, 1 for second
        // Additional input capabilities
        node_has_keycode_input: '{1'b0, 1'b0},    // No keycode input
        node_has_screen_pos: '{1'b0, 1'b0},       // No screen position input
        node_screen_pos_x_bits: '{8'h00, 8'h00},  // No screen X resolution
        node_screen_pos_y_bits: '{8'h00, 8'h00},  // No screen Y resolution  
        node_misc_digital_inputs: '{16'h0000, 16'h0000}, // No misc digital inputs (16-bit)
        // Output capabilities
        node_digital_outputs: '{8'h08, 8'h00},    // 8 digital outputs for first device
        node_analog_output_channels: '{4'h2, 4'h0}, // 2 analog output channels for first device
        node_card_system_slots: '{8'h00, 8'h00},  // No card system slots
        node_medal_hopper_channels: '{8'h00, 8'h00}, // No medal hopper channels
        node_has_char_display: '{1'b0, 1'b0},     // No character display
        node_char_display_width: '{8'h00, 8'h00}, // No character display width
        node_char_display_height: '{8'h00, 8'h00}, // No character display height
        node_char_display_type: '{8'h00, 8'h00},   // No character display type
        node_has_backup: '{1'b0, 1'b0}            // No backup support
    };
    assign jvs_nodes = jvs_nodes_r2;
`else 
    assign jvs_nodes = jvs_nodes_r;
`endif

    //=========================================================================
    // RAM for current node name during reception
    (* ramstyle = "M10K" *) logic [7:0] node_name_ram [0:jvs_node_info_pkg::NODE_NAME_SIZE -1];

////initial content for simulation without JVS device
`ifdef USE_DUMMY_JVS_DATA
    initial begin
        $readmemh("jvs_device_name.mem", node_name_ram); //null terminated string "namco ltd.;NAJV2;Ver1.00;JPN,Multipurpose."
    end
`endif

    //infer simple dual-port RAM for node name reading
    always_ff @(posedge i_clk) begin
        node_name_rd_data <= node_name_ram[node_name_rd_addr];
    end

    //=========================================================================
    // JVS DATA READY SIGNAL
    //=========================================================================
    logic jvs_data_ready_init, jvs_data_ready_joy;
    assign jvs_data_ready = jvs_data_ready_init | jvs_data_ready_joy;
    
    //=========================================================================
    // RS485 DIRECTION CONTROL
    //=========================================================================
    // Control RS485 transceiver direction based on current state
    // High = Transmit mode, Low = Receive mode
    assign o_rx485_dir = (rs485_state == RS485_TX_SETUP || 
                          rs485_state == RS485_TRANSMIT || 
                          rs485_state == RS485_TX_HOLD);

    //=========================================================================
    // RS485 STATE MACHINE
    //=========================================================================
    // Manages RS485 transceiver direction with proper setup and hold timing
    // This is critical for reliable RS485 communication
    
    logic [15:0] rs485_setup_counter; // Counter for timing delays
    
    always @(posedge i_clk) begin
        if (i_rst || !i_ena) begin
            rs485_state <= RS485_RECEIVE;
            rs485_setup_counter <= 16'h0;
        end else begin
            case (rs485_state)
                RS485_RECEIVE: begin
                    rs485_setup_counter <= 16'h0;
                    // Switch to transmit mode when requested
                    if (rs485_tx_request) begin
                        rs485_state <= RS485_TX_SETUP;
                    end
                end
                
                RS485_TX_SETUP: begin
                    // Setup time: ~10µs (500 cycles at 50MHz)
                    // This allows the RS485 transceiver to stabilize before data transmission
                    //if (rs485_setup_counter < 16'd500) begin
                     if (rs485_setup_counter < TX_SETUP_DELAY_COUNT) begin
                        rs485_setup_counter <= rs485_setup_counter + 1;
                    end else begin
                        rs485_setup_counter <= 16'h0;
                        rs485_state <= RS485_TRANSMIT;
                    end
                end
                
                RS485_TRANSMIT: begin
                    // Stay in transmit mode while data is being sent
                    if (!rs485_tx_request) begin
                        rs485_state <= RS485_TX_HOLD;
                    end
                end
                
                RS485_TX_HOLD: begin
                    // Hold TX mode after transmission (~30µs)
                    // This ensures the last bit is fully transmitted before switching to receive
                    //if (rs485_setup_counter < 16'd1500) begin
                    if (rs485_setup_counter < TX_HOLD_DELAY_COUNT) begin
                        rs485_setup_counter <= rs485_setup_counter + 1;
                    end else begin
                        rs485_setup_counter <= 16'h0;
                        rs485_state <= RS485_RECEIVE;
                    end
                end
            endcase
        end
    end

    //=========================================================================
    // MAIN STATE MACHINE - JVS PROTOCOL HANDLER
    //=========================================================================
    // Implements the complete JVS initialization sequence and input polling
    
    always @(posedge i_clk) begin

        jvs_data_ready_init <= 1'b0;

        if (i_rst || !i_ena) begin
            // Initialize all state variables on reset
            main_state <= STATE_INIT_DELAY;
            delay_counter <= 32'h0;
            timeout_counter <= 32'h0;
            poll_timer <= 32'h0;
            current_device_addr <= 8'h01;    // Standard JVS device address
            rs485_tx_request <= 1'b0;
            uart_tx_dv <= 1'b0;
            last_tx_state <= 5'h0;
        end else begin
            case (main_state)
                //-------------------------------------------------------------
                // IDLE STATE - Continuous input polling for responsive gaming
                //-------------------------------------------------------------
                STATE_IDLE: begin
                    rs485_tx_request <= 1'b0; // should be already set by STATE_WAIT_TX_HOLD
                    
                    // Fast polling timer for inputs - 1ms interval
                    // This provides responsive gaming experience with minimal latency
                    //if (poll_timer < 32'h0C350) begin  // 50,000 cycles = 1ms at 50MHz
                    if (poll_timer < POLL_INTERVAL_COUNT) begin  // 1ms
                        poll_timer <= poll_timer + 1;
                    end else begin
                        poll_timer <= 32'h0;
                        main_state <= STATE_SEND_INPUTS;
                    end
                end

                //-------------------------------------------------------------
                // INITIALIZATION DELAY - Wait for system stabilization
                //-------------------------------------------------------------
                STATE_INIT_DELAY: begin
                    rs485_tx_request <= 1'b0;
                    // Initial delay for core I/O initialization - 5.4 seconds
                    // This ensures the FPGA core and external circuits are fully stable
                    //if (delay_counter < 32'h10000000) begin  // 268,435,456 cycles ≈ 5.4s at 50MHz
                    if (delay_counter < INIT_DELAY_COUNT) begin  // 5.4 seconds
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 32'h0;
                        main_state <= STATE_FIRST_RESET;
                    end
                end

                //-------------------------------------------------------------
                // FIRST RESET COMMAND - Begin JVS device initialization
                //-------------------------------------------------------------
                STATE_FIRST_RESET: begin
                    // Prepare first RESET command frame
                    // JVS requires two reset commands for reliable initialization
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0 - Frame start
                    tx_buffer[JVS_ADDR_POS] <= JVS_BROADCAST_ADDR;  // FF - Broadcast to all devices
                    tx_buffer[JVS_CMD_START + 0] <= CMD_RESET;        // Reset command (0xF0)
                    tx_buffer[JVS_CMD_START + 1] <= CMD_RESET_ARG;        // Reset argument (0xD9)
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 1;               // 1 data byte + overhead
                    rs485_tx_request <= 1'b1;           // Request transmission
                    last_tx_state <= STATE_FIRST_RESET; // Remember command for response handling
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // DELAY AFTER FIRST RESET
                //-------------------------------------------------------------
                STATE_FIRST_RESET_DELAY: begin
                    rs485_tx_request <= 1'b0;
                    // 2 second delay after first RESET
                    // Allows JVS devices to complete their reset sequence
                    //if (delay_counter < 32'h6000000) begin  // 100,663,296 cycles = 2s at 50MHz
                    if (delay_counter <FIRST_RESET_DELAY_COUNT) begin  //   2 seconds
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 32'h0;
                        main_state <= STATE_SECOND_RESET;
                    end
                end

                //-------------------------------------------------------------
                // SECOND RESET COMMAND - Ensure complete device reset
                //-------------------------------------------------------------
                STATE_SECOND_RESET: begin
                    // Prepare second RESET command frame (identical to first)
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= JVS_BROADCAST_ADDR;  // FF
                    tx_buffer[JVS_CMD_START + 0] <= CMD_RESET;        // F0
                    tx_buffer[JVS_CMD_START + 1] <= CMD_RESET_ARG;        // D9
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 1;               // 1 data byte + overhead
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SECOND_RESET;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // DELAY AFTER SECOND RESET
                //-------------------------------------------------------------
                STATE_SECOND_RESET_DELAY: begin
                    rs485_tx_request <= 1'b0;
                    // 500ms delay after second RESET
                    // Shorter delay as devices should be ready after two resets
                    //if (delay_counter < 32'h1800000) begin  // 25,165,824 cycles = 500ms at 50MHz
                    if (delay_counter < SECOND_RESET_DELAY_COUNT) begin  // 500ms
                        delay_counter <= delay_counter + 1;
                    end else begin
                        delay_counter <= 32'h0;
                        main_state <= STATE_SEND_SETADDR;
                    end
                end

                //-------------------------------------------------------------
                // SET ADDRESS COMMAND - Assign unique address to device
                //-------------------------------------------------------------
                STATE_SEND_SETADDR: begin
                    // Prepare SET ADDRESS command frame
                    // This assigns a unique address (0x01) to the JVS device
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= JVS_BROADCAST_ADDR;  // FF - Still broadcast for address assignment
                    tx_buffer[JVS_CMD_START + 0] <= CMD_SETADDR;         // Set address command (0xF1)
                    tx_buffer[JVS_CMD_START + 1] <= current_device_addr; // 01 - Address to assign
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 1;               // 1 data byte + overhead
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_SETADDR;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // READ ID COMMAND - Request device identification
                //-------------------------------------------------------------
                STATE_SEND_READID: begin
                    // Prepare READ ID command frame
                    // This requests the device to send its identification string
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= current_device_addr; // 01 - Address specific device
                    tx_buffer[JVS_CMD_START + 0] <= CMD_IOIDENT;          // IO identity command (0x10)
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 0;               // 0 data byte + overhead (just command)
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_READID;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // COMMAND REVISION REQUEST - Get command format revision
                //-------------------------------------------------------------
                STATE_SEND_CMDREV: begin
                    // Prepare CMDREV command frame
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= current_device_addr; // 01
                    tx_buffer[JVS_CMD_START + 0] <= CMD_CMDREV;          // Command revision command (0x11)
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 0;               // 0 data byte + overhead (just command)
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_CMDREV;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // JVS REVISION REQUEST - Get JVS protocol revision
                //-------------------------------------------------------------
                STATE_SEND_JVSREV: begin
                    // Prepare JVSREV command frame
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= current_device_addr; // 01
                    tx_buffer[JVS_CMD_START + 0] <= CMD_JVSREV;          // JVS revision command (0x12)
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 0;               // 0 data byte + overhead (just command)
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_JVSREV;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // COMMUNICATIONS VERSION REQUEST - Get communication version
                //-------------------------------------------------------------
                STATE_SEND_COMMVER: begin
                    // Prepare COMMVER command frame
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= current_device_addr; // 01
                    tx_buffer[JVS_CMD_START + 0] <= CMD_COMMVER;         // Communication version command (0x13)
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 0;               // 0 data byte + overhead (just command)
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_COMMVER;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // FEATURE CHECK REQUEST - Get device capabilities
                //-------------------------------------------------------------
                STATE_SEND_FEATCHK: begin
                    // Prepare FEATCHK command frame
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0
                    tx_buffer[JVS_ADDR_POS] <= current_device_addr; // 01
                    tx_buffer[JVS_CMD_START + 0] <= CMD_FEATCHK;         // Feature check command (0x14)
                    tx_buffer[JVS_LENGTH_POS] <= JVS_OVERHEAD + 0;               // 0 data byte + overhead (just command)
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_FEATCHK;
                    main_state <= STATE_WAIT_TX_SETUP;
                end

                //-------------------------------------------------------------
                // READ INPUTS COMMAND - Request current input states
                //-------------------------------------------------------------
                STATE_SEND_INPUTS: begin
                    // Initialize INPUT PULLING frame progressive construction
                    // Start with basic frame header
                    tx_buffer[JVS_SYNC_POS] <= JVS_SYNC_BYTE;       // E0 - Sync byte
                    tx_buffer[JVS_ADDR_POS] <= current_device_addr; // Device address (01)
                    
                    // Use JVS_LENGTH_POS as data byte counter (starts at 0)
                    tx_buffer[JVS_LENGTH_POS] <= 8'd0;  // Start counting data bytes from 0
                    
                    // Begin progressive state machine - start with switch inputs
                    main_state <= STATE_SEND_INPUTS_SWITCH;
                end

                //-------------------------------------------------------------
                // WAIT FOR RS485 SETUP - Ensure proper transceiver timing
                //-------------------------------------------------------------
                STATE_WAIT_TX_SETUP: begin
                    // Wait for RS485 transceiver to enter transmit mode
                    if (rs485_state == RS485_TRANSMIT) begin
                        tx_counter <= 8'h00;                    // Reset byte counter
                        tx_checksum <= 8'h00;                   // Reset checksum
                        tx_length <= JVS_CMD_START + tx_buffer[JVS_LENGTH_POS];       // Calculate total frame length
                        main_state <= STATE_TRANSMIT_BYTE;
                    end
                end

                //-------------------------------------------------------------
                // TRANSMIT BYTES - Send frame data byte by byte
                //-------------------------------------------------------------
                STATE_TRANSMIT_BYTE: begin
                    if (tx_counter < tx_length) begin
                        // Wait for UART to be ready for next byte
                        if (!uart_tx_active && !uart_tx_dv) begin
                            // Calculate running checksum for data bytes
                            // (Skip sync byte and checksum position)
                            if (tx_counter > 0 && tx_counter < tx_length - 1) begin
                                tx_checksum <= tx_checksum + tx_buffer[tx_counter];
                            end
                            
                            // Send either data byte or calculated checksum
                            if (tx_counter == tx_length - 1) begin
                                uart_tx_byte <= tx_checksum;        // Send checksum as last byte
                            end else begin
                                uart_tx_byte <= tx_buffer[tx_counter]; // Send data byte
                            end
                            
                            uart_tx_dv <= 1'b1;                     // Start UART transmission
                            main_state <= STATE_WAIT_TX_DONE;
                        end
                    end else begin
                        // All bytes transmitted
                        rs485_tx_request <= 1'b0;               // Release RS485 request
                        main_state <= STATE_WAIT_TX_HOLD;
                    end
                end

                //-------------------------------------------------------------
                // WAIT FOR TRANSMISSION COMPLETION
                //-------------------------------------------------------------
                STATE_WAIT_TX_DONE: begin
                    // Clear data valid signal when UART starts transmission
                    if (uart_tx_dv && uart_tx_active) begin
                        uart_tx_dv <= 1'b0;
                    end
                    // Move to next byte when current transmission completes
                    if (uart_tx_done) begin
                        tx_counter <= tx_counter + 1;
                        main_state <= STATE_TRANSMIT_BYTE;
                    end
                end

                //-------------------------------------------------------------
                // WAIT FOR RS485 HOLD TIME
                //-------------------------------------------------------------
                STATE_WAIT_TX_HOLD: begin
                    // Wait for RS485 to return to receive mode
                    if (rs485_state == RS485_RECEIVE) begin
                        timeout_counter <= 32'h0;
                        // Determine next state based on what was just transmitted
                        case (last_tx_state)
                            STATE_FIRST_RESET: main_state <= STATE_FIRST_RESET_DELAY;
                            STATE_SECOND_RESET: main_state <= STATE_SECOND_RESET_DELAY;
                            STATE_SEND_INPUTS: main_state <= STATE_WAIT_RX;
                            default: main_state <= STATE_WAIT_RX;  // Commands expecting response
                        endcase
                    end
                end

                //-------------------------------------------------------------
                // WAIT FOR DEVICE RESPONSE
                //-------------------------------------------------------------
                STATE_WAIT_RX: begin
                    if (rx_frame_complete) begin
                        // Process response based on command sent
                        case (tx_buffer[JVS_CMD_START])
                            CMD_SETADDR: main_state <= STATE_SEND_READID;    // Address set, now read ID
                            CMD_IOIDENT: main_state <= STATE_SEND_CMDREV;     // ID read, get command revision
                            CMD_CMDREV: main_state <= STATE_SEND_JVSREV;     // Command revision read, get JVS revision
                            CMD_JVSREV: main_state <= STATE_SEND_COMMVER;    // JVS revision read, get comm version
                            CMD_COMMVER: main_state <= STATE_SEND_FEATCHK;   // Comm version read, check features
                            CMD_FEATCHK: begin
                                jvs_data_ready_init <= 1'b1;               // Indicate initialization complete
                                main_state <= STATE_IDLE;           // Features checked, start polling
                            end
                            CMD_SWINP: main_state <= STATE_IDLE;
                            default: main_state <= STATE_IDLE;
                        endcase
                    //end else if (timeout_counter < 32'h0C3500) begin  // 10ms timeout - fast for responsive gaming
                    end else if (timeout_counter < RX_TIMEOUT_COUNT) begin  // 10ms timeout - fast for responsive gaming
                        timeout_counter <= timeout_counter + 1;
                    end else begin
                        // Timeout handling - different strategies for different commands
                        case (tx_buffer[JVS_CMD_START])
                            CMD_SETADDR: main_state <= STATE_FIRST_RESET;    // Critical - restart sequence
                            CMD_IOIDENT: main_state <= STATE_SEND_READID;     // Retry ID read
                            CMD_CMDREV: main_state <= STATE_SEND_CMDREV;     // Retry command revision
                            CMD_JVSREV: main_state <= STATE_SEND_JVSREV;     // Retry JVS revision
                            CMD_COMMVER: main_state <= STATE_SEND_COMMVER;   // Retry comm version
                            CMD_FEATCHK: main_state <= STATE_SEND_FEATCHK;   // Retry feature check
                            default: main_state <= STATE_IDLE;               // Continue with polling
                        endcase
                    end
                end

                //-------------------------------------------------------------
                // INPUT BUILDING PROGRESSIVE STATES
                //-------------------------------------------------------------
                
                STATE_SEND_INPUTS_SWITCH: begin
                    if (jvs_nodes.node_players[current_device_addr - 1] > 0) begin
                        request_build_idx = tx_buffer[JVS_LENGTH_POS];
                        // Add SWINP command and parameters using blocking assignments for index calculations
                        tx_buffer[JVS_CMD_START + request_build_idx] <= CMD_SWINP; // SWINP command (0x20)
                        request_build_idx = request_build_idx + 1;
                        tx_buffer[JVS_CMD_START + request_build_idx] <= jvs_nodes.node_players[current_device_addr - 1];  // Number of players
                        request_build_idx = request_build_idx + 1;
                        tx_buffer[JVS_CMD_START + request_build_idx] <= (jvs_nodes.node_buttons[current_device_addr - 1] + 7) / 8; // Compute number of bytes to include all per player bits
                        request_build_idx = request_build_idx + 1;
                        // Update length with final byte count
                        tx_buffer[JVS_LENGTH_POS] <= request_build_idx;
                    end
                    main_state <= STATE_SEND_INPUTS_COIN;
                end
                
                STATE_SEND_INPUTS_COIN: begin
                    if (jvs_nodes.node_coin_slots[current_device_addr - 1] > 0) begin
                        request_build_idx = tx_buffer[JVS_LENGTH_POS];
                        // Add coin input command using blocking assignments for index calculations
                        tx_buffer[JVS_CMD_START + request_build_idx] <= CMD_COININP; // COININP command (0x21)
                        request_build_idx = request_build_idx + 1;
                        tx_buffer[JVS_CMD_START + request_build_idx] <= jvs_nodes.node_coin_slots[current_device_addr - 1]; // Number of coin slots
                        request_build_idx = request_build_idx + 1;
                        // Update length with final byte count
                        tx_buffer[JVS_LENGTH_POS] <= request_build_idx;
                    end
                    main_state <= STATE_SEND_INPUTS_ANALOG;
                end
                
                STATE_SEND_INPUTS_ANALOG: begin
                    if (jvs_nodes.node_analog_channels[current_device_addr - 1] > 0) begin
                        request_build_idx = tx_buffer[JVS_LENGTH_POS];
                        // Add analog input command using blocking assignments for index calculations
                        tx_buffer[JVS_CMD_START + request_build_idx] <= CMD_ANLINP; // ANLINP command (0x22)
                        request_build_idx = request_build_idx + 1;
                        tx_buffer[JVS_CMD_START + request_build_idx] <= jvs_nodes.node_analog_channels[current_device_addr - 1]; // Number of analog channels
                        request_build_idx = request_build_idx + 1;
                        // Update length with final byte count
                        tx_buffer[JVS_LENGTH_POS] <= request_build_idx;
                    end
                    main_state <= STATE_SEND_INPUTS_ROTARY;
                end
                
                STATE_SEND_INPUTS_ROTARY: begin
                    if (jvs_nodes.node_rotary_channels[current_device_addr - 1] > 0) begin
                        request_build_idx = tx_buffer[JVS_LENGTH_POS];
                        // Add rotary input command using blocking assignments for index calculations
                        tx_buffer[JVS_CMD_START + request_build_idx] <= CMD_ROTINP;  // ROTINP command (0x23)
                        request_build_idx = request_build_idx + 1;
                        tx_buffer[JVS_CMD_START + request_build_idx] <= jvs_nodes.node_rotary_channels[current_device_addr - 1]; // Number of rotary channels
                        request_build_idx = request_build_idx + 1;
                        // Update length with final byte count
                        tx_buffer[JVS_LENGTH_POS] <= request_build_idx + 1;
                    end
                    main_state <= STATE_SEND_INPUTS_KEYCODE;
                end
                
                STATE_SEND_INPUTS_KEYCODE: begin
                    if (jvs_nodes.node_has_keycode_input[current_device_addr - 1]) begin
                        request_build_idx = tx_buffer[JVS_LENGTH_POS];
                        // Add keycode input command (no parameters) using blocking assignments for index calculations
                        request_build_idx = request_build_idx + 1;
                        tx_buffer[JVS_CMD_START + request_build_idx] <= CMD_KEYINP;  // KEYINP command (0x24)
                        // Update length with final byte count
                        tx_buffer[JVS_LENGTH_POS] <= request_build_idx;
                    end
                    main_state <= STATE_SEND_INPUTS_SCREEN;
                end
                
                STATE_SEND_INPUTS_SCREEN: begin
                    if (jvs_nodes.node_has_screen_pos[current_device_addr - 1]) begin
                        request_build_idx = tx_buffer[JVS_LENGTH_POS];
                        // Add screen position input command using blocking assignments for index calculations
                        tx_buffer[JVS_CMD_START + request_build_idx] <= CMD_SCRPOSINP;  // SCRPOSINP command (0x25)
                        request_build_idx = request_build_idx + 1;
                        tx_buffer[JVS_CMD_START + request_build_idx] <= 8'h01;  // Channel index
                        request_build_idx = request_build_idx + 1;
                        // Update length with final byte count
                        tx_buffer[JVS_LENGTH_POS] <= request_build_idx;
                    end
                    main_state <= STATE_SEND_INPUTS_MISC;
                end
                
                STATE_SEND_INPUTS_MISC: begin
                    if (jvs_nodes.node_misc_digital_inputs[current_device_addr - 1] > 0) begin
                        request_build_idx = tx_buffer[JVS_LENGTH_POS];
                        // Add misc switch input command using blocking assignments for index calculations
                        tx_buffer[JVS_CMD_START + request_build_idx] <= CMD_MISCSWINP;  // MISCSWINP command (0x26)
                        request_build_idx = request_build_idx + 1;
                        tx_buffer[JVS_CMD_START + request_build_idx] <= (jvs_nodes.node_misc_digital_inputs[current_device_addr - 1] + 7) / 8; // Bytes needed
                        request_build_idx = request_build_idx + 1;
                        // Update length with final byte count
                        tx_buffer[JVS_LENGTH_POS] <= request_build_idx;
                    end
                    main_state <= STATE_SEND_OUTPUT_DIGITAL;
                end
                
                STATE_SEND_OUTPUT_DIGITAL: begin
                    // Check if device has digital outputs
                    if (jvs_nodes.node_digital_outputs[current_device_addr - 1] > 0) begin
                        request_build_idx = tx_buffer[JVS_LENGTH_POS];
                        if (jvs_nodes.node_players[current_device_addr - 1] == 1) begin
                            // Build OUTPUT1 command frame for GPIO control
                            tx_buffer[JVS_CMD_START + request_build_idx] <= CMD_OUTPUT1;         // OUTPUT1 command (0x32)
                            request_build_idx = request_build_idx + 1;
                            tx_buffer[JVS_CMD_START + request_build_idx] <= 8'h03;           // send 3 bytes (from time crisis 4 capture althouth that FEATCHK report 12 channels/bits?)
                            request_build_idx = request_build_idx + 1;
                            tx_buffer[JVS_CMD_START + request_build_idx] <= gpio_output_value;  // Set GPIO1 to current value from SNAC
                            request_build_idx = request_build_idx + 1;
                            tx_buffer[JVS_CMD_START + request_build_idx] <= 8'hA0;              // do not know what A is for, but taken from TC4 capture
                            request_build_idx = request_build_idx + 1;
                            tx_buffer[JVS_CMD_START + request_build_idx] <= 8'h00;
                            request_build_idx = request_build_idx + 1;
                        end
                        // Update length with final byte count
                        tx_buffer[JVS_LENGTH_POS] <= request_build_idx;
                    end
                    main_state <= STATE_SEND_FINALIZE;
                end

                STATE_SEND_FINALIZE: begin
                    // Add checksum size byte count to get total frame length
                    tx_buffer[JVS_LENGTH_POS] <= tx_buffer[JVS_LENGTH_POS] + JVS_CHECKSUM_SIZE;
                    // Start transmission
                    rs485_tx_request <= 1'b1;
                    last_tx_state <= STATE_SEND_INPUTS;
                    main_state <= STATE_WAIT_TX_SETUP;
                end
                
                default: main_state <= STATE_IDLE;
            endcase
        end
    end

    //=========================================================================
    // RX STATE MACHINE - PROCESSES INCOMING JVS RESPONSES
    //=========================================================================
    // Handles byte-by-byte reception of JVS frames with checksum validation
    
    always @(posedge i_clk) begin

    //initial content for simulation without JVS device
    `ifdef USE_DUMMY_JVS_DATA
        jvs_nodes_r2 <= JVS_INFO_INIT;
    `endif
    
        jvs_data_ready_joy <= 1'b0;

        if (i_rst || !i_ena) begin
            // Initialize RX state machine and output registers
            rx_state <= RX_IDLE;
            rx_frame_complete <= 1'b0;
            rx_counter <= 8'h00;
            rx_length <= 8'h00;
            rx_checksum <= 8'h00;
            
            // Initialize JVS node information (single node only)
            jvs_nodes_r.node_id[0] <= 8'h01;
            jvs_nodes_r.node_cmd_ver[0] <= 8'h00;
            jvs_nodes_r.node_jvs_ver[0] <= 8'h00;
            jvs_nodes_r.node_com_ver[0] <= 8'h00;
            jvs_nodes_r.node_players[0] <= 4'h0;
            jvs_nodes_r.node_buttons[0] <= 8'h0;
            jvs_nodes_r.node_analog_channels[0] <= 4'h0;
            jvs_nodes_r.node_rotary_channels[0] <= 4'h0;
            // Additional input capabilities (not yet supported)
            jvs_nodes_r.node_has_keycode_input[0] <= 1'b0;
            jvs_nodes_r.node_has_screen_pos[0] <= 1'b0;
            jvs_nodes_r.node_screen_pos_x_bits[0] <= 8'h0;
            jvs_nodes_r.node_screen_pos_y_bits[0] <= 8'h0;
            // Initialize has_screen_pos output
            has_screen_pos <= 1'b0;
            jvs_nodes_r.node_misc_digital_inputs[0] <= 16'h0;
            // Output capabilities
            jvs_nodes_r.node_digital_outputs[0] <= 8'h0;
            jvs_nodes_r.node_analog_output_channels[0] <= 4'h0;
            jvs_nodes_r.node_card_system_slots[0] <= 8'h0;
            jvs_nodes_r.node_medal_hopper_channels[0] <= 8'h0;
            jvs_nodes_r.node_has_char_display[0] <= 1'b0;
            jvs_nodes_r.node_char_display_width[0] <= 8'h0;
            jvs_nodes_r.node_char_display_height[0] <= 8'h0;
            jvs_nodes_r.node_char_display_type[0] <= 8'h0;
            jvs_nodes_r.node_has_backup[0] <= 1'b0;
            
            // Initialize output button and joystick states
            p1_btn_state <= 16'h0000;           // All buttons released
            p1_joy_state <= 32'h80808080;       // Analog sticks centered (0x80 = center)
            p2_btn_state <= 16'h0000;
            p2_joy_state <= 32'h80808080;
            p3_btn_state <= 16'h0000;
            p4_btn_state <= 16'h0000;
            

            
        end else begin
            // Clear frame complete flag when main state machine processes it
            if (main_state != STATE_WAIT_RX) begin
                rx_frame_complete <= 1'b0;
            end
            
            
            // Process incoming bytes from UART
            if (uart_rx_dv) begin
                case (rx_state)
                    //-----------------------------------------------------
                    // RX_IDLE - Wait for frame start (sync byte)
                    //-----------------------------------------------------
                    RX_IDLE: begin
                        if (uart_rx_byte == JVS_SYNC_BYTE) begin  // E0 detected
                            rx_counter <= 8'h01;                  // Next byte position
                            rx_checksum <= 8'h00;                 // Reset checksum
                            rx_buffer_raw[0] <= uart_rx_byte;         // Store sync byte
                            rx_frame_complete <= 1'b0;
                            rx_state <= RX_READ_ADDR;
                        end else begin
                            rx_counter <= 0;                      // Reset on invalid data
                        end
                    end
                    
                    //-----------------------------------------------------
                    // RX_READ_ADDR - Read address byte (should be 0x00 for master)
                    //-----------------------------------------------------
                    RX_READ_ADDR: begin
                        if (uart_rx_byte == JVS_HOST_ADDR) begin          // Valid master address
                            rx_buffer_raw[rx_counter] <= uart_rx_byte;
                            rx_checksum <= rx_checksum + uart_rx_byte; // Add to checksum
                            rx_counter <= rx_counter + 1;
                            rx_state <= RX_READ_SIZE;
                        end else begin
                            rx_counter <= 0;                      // Invalid address, restart
                            rx_state <= RX_IDLE;
                        end
                    end
                    
                    //-----------------------------------------------------
                    // RX_READ_SIZE - Read frame length byte
                    //-----------------------------------------------------
                    RX_READ_SIZE: begin
                        rx_buffer_raw[rx_counter] <= uart_rx_byte;
                        rx_checksum <= rx_checksum + uart_rx_byte;
                        rx_length <= uart_rx_byte;                // Store frame length
                        rx_counter <= rx_counter + 1;
                        rx_state <= RX_READ_DATA;
                    end
                    
                    //-----------------------------------------------------
                    // RX_READ_DATA - Read data bytes and validate checksum
                    //-----------------------------------------------------
                    RX_READ_DATA: begin
                        rx_buffer_raw[rx_counter] <= uart_rx_byte;
                        
                        if (rx_counter < (JVS_OVERHEAD + rx_length)) begin
                            // Still reading data bytes
                            rx_checksum <= rx_checksum + uart_rx_byte;
                            rx_counter <= rx_counter + 1;
                        end else begin
                            // Last byte (checksum) received
                            if (rx_checksum == uart_rx_byte) begin
                                rx_state <= RX_UNESCAPE;          // Checksum valid, start unescaping
                                copy_read_idx <= 8'd0;            // Start reading from beginning
                                copy_write_idx <= 8'd0;           // Start writing from beginning
                                rx_counter <= 0;
                            end else begin
                                rx_state <= RX_IDLE;              // Checksum invalid, discard frame
                                rx_counter <= 0;
                            end
                        end
                    end
                    
                    default: rx_state <= RX_IDLE;
                endcase
            end
            
            //-------------------------------------------------------------
            // RX_UNESCAPE - Copy from raw buffer to final buffer, processing escape sequences
            //-------------------------------------------------------------
            if (rx_state == RX_UNESCAPE) begin
                if (copy_read_idx <= (JVS_OVERHEAD + rx_length)) begin // Process header + data + checksum
                    if (copy_read_idx < JVS_STATUS_POS) begin
                        // Copy header bytes as-is (sync, addr, length)
                        rx_buffer[copy_write_idx] <= rx_buffer_raw[copy_read_idx];
                        copy_read_idx <= copy_read_idx + 1;
                        copy_write_idx <= copy_write_idx + 1;
                    end else if (copy_read_idx < (JVS_OVERHEAD + rx_length)) begin // In data section
                        // Check for escape sequences in data section
                        if (rx_buffer_raw[copy_read_idx] == JVS_ESCAPE_BYTE && 
                            copy_read_idx + 1 <= (JVS_OVERHEAD + rx_length) &&
                            (rx_buffer_raw[copy_read_idx + 1] == JVS_ESCAPED_E0 || 
                             rx_buffer_raw[copy_read_idx + 1] == JVS_ESCAPED_D0)) begin
                            // Process escape sequence
                            if (rx_buffer_raw[copy_read_idx + 1] == JVS_ESCAPED_E0) begin
                                rx_buffer[copy_write_idx] <= JVS_SYNC_BYTE; // D0 DF -> E0
                            end else begin
                                rx_buffer[copy_write_idx] <= JVS_ESCAPE_BYTE; // D0 CF -> D0
                            end
                            copy_read_idx <= copy_read_idx + 2; // Skip both escape bytes
                            copy_write_idx <= copy_write_idx + 1;
                            // Update length in final buffer (remove 1 byte)
                            rx_buffer[JVS_LENGTH_POS] <= rx_buffer[JVS_LENGTH_POS] - 1;
                        end else begin
                            // Normal byte, copy as-is
                            rx_buffer[copy_write_idx] <= rx_buffer_raw[copy_read_idx];
                            copy_read_idx <= copy_read_idx + 1;
                            copy_write_idx <= copy_write_idx + 1;
                        end
                    end else begin
                        // Copy checksum
                        rx_buffer[copy_write_idx] <= rx_buffer_raw[copy_read_idx];
                        copy_read_idx <= copy_read_idx + 1;
                        copy_write_idx <= copy_write_idx + 1;
                    end
                end else begin
                    // Finished unescaping, move to process
                    rx_state <= RX_PROCESS;
                end
            end

            //-------------------------------------------------------------
            // RX_COPY_NAME - Copy node name from READ ID response
            //-------------------------------------------------------------
            if (rx_state == RX_COPY_NAME) begin
                if (copy_read_idx < (JVS_OVERHEAD + rx_buffer[JVS_LENGTH_POS]) && copy_write_idx < jvs_node_info_pkg::NODE_NAME_SIZE - 1) begin
                    // Check for null terminator
                    if (rx_buffer[copy_read_idx] == 8'h00) begin
                        // Found null terminator, finish copying
                        //jvs_nodes.node_name[current_device_addr - 1][copy_write_idx] <= 8'h00;
                        node_name_ram[copy_write_idx] <= 8'h00; // Also update RAM
                        rx_frame_complete <= 1'b1;     // Signal frame complete to main state machine
                        rx_counter <= 8'h00;           // Reset counter for next frame
                        rx_state <= RX_IDLE;           // Return to idle for next frame
                    end else begin
                        // Copy character and advance indices
                        //jvs_nodes.node_name[current_device_addr - 1][copy_write_idx] <= rx_buffer[copy_read_idx];
                        node_name_ram[copy_write_idx] <= rx_buffer[copy_read_idx]; // Also update RAM
                        copy_read_idx <= copy_read_idx + 1;
                        copy_write_idx <= copy_write_idx + 1;
                    end
                end else begin
                    // Reached end of buffer or max name size, null terminate and finish
                    //jvs_nodes.node_name[current_device_addr - 1][copy_write_idx] <= 8'h00;
                    node_name_ram[copy_write_idx] <= 8'h00; // Also update RAM
                    rx_frame_complete <= 1'b1;     // Signal frame complete to main state machine
                    rx_counter <= 8'h00;           // Reset counter for next frame
                    rx_state <= RX_IDLE;           // Return to idle for next frame
                end
            end

            //-------------------------------------------------------------
            // RX_PARSE_FEATURES - Parse JVS feature/capability data
            //-------------------------------------------------------------
            if (rx_state == RX_PARSE_FEATURES) begin
                // Parse feature data format: [func_code][param1][param2][param3] repeating, then 00
                // copy_read_idx points to current function code position
                if (copy_read_idx < (JVS_OVERHEAD + rx_buffer[JVS_LENGTH_POS]) && 
                    copy_read_idx + JVS_FUNC_LENGTH - 1 < (JVS_OVERHEAD + rx_buffer[JVS_LENGTH_POS])) begin
                    
                    // Check for terminator (00 byte)
                    if (rx_buffer[copy_read_idx] == 8'h00) begin
                        // Feature parsing complete
                        rx_frame_complete <= 1'b1;
                        rx_counter <= 8'h00;
                        rx_state <= RX_IDLE;
                    end else begin
                        // Parse function block [func_code][param1][param2][param3]
                        case (rx_buffer[copy_read_idx])
                            //=========================================================
                            // INPUT FUNCTIONS (0x01 - 0x07)
                            //=========================================================
                            
                            FUNC_INPUT_DIGITAL: begin // 0x01
                                // Digital input: param1=players, param2=buttons config
                                jvs_nodes_r.node_players[current_device_addr - 1] <= rx_buffer[copy_read_idx + 1][3:0];
                                jvs_nodes_r.node_buttons[current_device_addr - 1] <= rx_buffer[copy_read_idx + 2];
                            end
                            
                            FUNC_INPUT_COIN: begin // 0x02
                                // Coin input: param1=number of coin slots
                                jvs_nodes_r.node_coin_slots[current_device_addr - 1] <= rx_buffer[copy_read_idx + 1][3:0];
                            end
                            
                            FUNC_INPUT_ANALOG: begin // 0x03
                                // Analog input: param1=number of channels, param2=bits of precision
                                jvs_nodes_r.node_analog_channels[current_device_addr - 1] <= rx_buffer[copy_read_idx + 1][3:0];
                                jvs_nodes_r.node_analog_bits[current_device_addr - 1] <= rx_buffer[copy_read_idx + 2];
                            end
                            
                            FUNC_INPUT_ROTARY: begin // 0x04
                                // Rotary input: param1=number of channels
                                jvs_nodes_r.node_rotary_channels[current_device_addr - 1] <= rx_buffer[copy_read_idx + 1][3:0];
                            end
                            
                            FUNC_INPUT_KEYCODE: begin // 0x05
                                // Keycode input function - PARSED BUT NOT SUPPORTED YET
                                // Parameters: 0, 0, 0 (no parameters according to JVS spec)
                                jvs_nodes_r.node_has_keycode_input[current_device_addr - 1] <= 1'b1;
                            end
                            
                            FUNC_INPUT_SCREEN_POS: begin // 0x06
                                // Screen position input (touch/lightgun) - PARSED AND SUPPORTED
                                // Parameters: Xbits, Ybits, channels (resolution only, not position)
                                jvs_nodes_r.node_has_screen_pos[current_device_addr - 1] <= 1'b1;
                                jvs_nodes_r.node_screen_pos_x_bits[current_device_addr - 1] <= rx_buffer[copy_read_idx + 1];
                                jvs_nodes_r.node_screen_pos_y_bits[current_device_addr - 1] <= rx_buffer[copy_read_idx + 2];
                                // Set has_screen_pos for any device that supports screen position
                                has_screen_pos <= 1'b1;
                            end
                            
                            FUNC_INPUT_MISC_DIGITAL: begin // 0x07
                                // Miscellaneous digital input - PARSED BUT NOT SUPPORTED YET
                                // Parameters: SW MSB, SW LSB, 0 (16-bit switch count)
                                jvs_nodes_r.node_misc_digital_inputs[current_device_addr - 1] <= {rx_buffer[copy_read_idx + 1], rx_buffer[copy_read_idx + 2]};
                            end
                            
                            //=========================================================
                            // OUTPUT FUNCTIONS (0x10 - 0x15)
                            //=========================================================
                            
                            FUNC_OUTPUT_CARD: begin // 0x10
                                // Card system output - PARSED BUT NOT SUPPORTED YET
                                jvs_nodes_r.node_card_system_slots[current_device_addr - 1] <= rx_buffer[copy_read_idx + 1];
                            end
                            
                            FUNC_OUTPUT_HOPPER: begin // 0x11
                                // Medal hopper output - PARSED BUT NOT SUPPORTED YET
                                jvs_nodes_r.node_medal_hopper_channels[current_device_addr - 1] <= rx_buffer[copy_read_idx + 1];
                            end
                            
                            FUNC_OUTPUT_DIGITAL: begin // 0x12
                                // Digital output: param1=number of outputs
                                jvs_nodes_r.node_digital_outputs[current_device_addr - 1] <= rx_buffer[copy_read_idx + 1];
                            end
                            
                            FUNC_OUTPUT_ANALOG: begin // 0x13
                                // Analog output: param1=number of channels
                                jvs_nodes_r.node_analog_output_channels[current_device_addr - 1] <= rx_buffer[copy_read_idx + 1][3:0];
                            end
                            
                            FUNC_OUTPUT_CHAR: begin // 0x14
                                // Character/text display output - PARSED BUT NOT SUPPORTED YET
                                // Parameters: width, height, type
                                jvs_nodes_r.node_has_char_display[current_device_addr - 1] <= 1'b1;
                                jvs_nodes_r.node_char_display_width[current_device_addr - 1] <= rx_buffer[copy_read_idx + 1];
                                jvs_nodes_r.node_char_display_height[current_device_addr - 1] <= rx_buffer[copy_read_idx + 2];
                                jvs_nodes_r.node_char_display_type[current_device_addr - 1] <= rx_buffer[copy_read_idx + 3];
                            end
                            
                            FUNC_OUTPUT_BACKUP: begin // 0x15
                                // Backup data function - PARSED BUT NOT SUPPORTED YET
                                jvs_nodes_r.node_has_backup[current_device_addr - 1] <= 1'b1;
                            end
                            
                            default: begin
                                // Unknown function code, skip it
                            end
                        endcase
                        
                        // Advance to next function block (JVS_FUNC_LENGTH bytes per function)
                        copy_read_idx <= copy_read_idx + JVS_FUNC_LENGTH;
                    end
                end else begin
                    // Reached end of data, complete parsing
                    rx_frame_complete <= 1'b1;
                    rx_counter <= 8'h00;
                    rx_state <= RX_IDLE;
                end
            end
            
            //-------------------------------------------------------------
            // RX INPUT PARSING STATES - Parse input response data
            //-------------------------------------------------------------
            
            // RX_PARSE_INPUTS_START - Initialize input response parsing
            if (rx_state == RX_PARSE_INPUTS_START) begin
                // Verify the response status byte (01 = success) 
                if (rx_buffer[JVS_STATUS_POS] == STATUS_NORMAL) begin
                    // Status is valid, start parsing from first report byte position
                    copy_read_idx <= JVS_STATUS_POS + 1;  // Skip status byte, point to first report byte
                    rx_state <= RX_PARSE_INPUTS_SWITCH;
                end else begin
                    // Invalid status, return to idle
                    rx_frame_complete <= 1'b1;
                    rx_counter <= 8'h00;
                    rx_state <= RX_IDLE;
                end
            end
            
            // RX_PARSE_INPUTS_SWITCH - Parse switch inputs report and system byte
            if (rx_state == RX_PARSE_INPUTS_SWITCH) begin
                if (jvs_nodes.node_players[current_device_addr - 1] > 0) begin
                    // Check SWINP report byte
                    if (rx_buffer[copy_read_idx] != REPORT_NORMAL) begin
                        // SWINP function failed, skip to next function
                        copy_read_idx <= copy_read_idx + 1; // Skip failed report byte
                        rx_state <= RX_PARSE_INPUTS_COIN;
                    end else begin
                        // SWINP report is normal, advance past report byte and system byte
                        copy_read_idx <= copy_read_idx + 2; // Skip report byte + system byte, now points to first player data
                        // Initialize player parsing
                        current_player <= 4'd0; // Start with player 0
                        rx_state <= RX_PARSE_SWINP_PLAYER; // Go to player parsing state
                    end
                end else begin
                    rx_state <= RX_PARSE_INPUTS_COIN;
                end
            end
            
            // RX_PARSE_SWINP_PLAYER - Parse individual player SWINP data (recursive)
            if (rx_state == RX_PARSE_SWINP_PLAYER) begin
                // Parse current player data
                // @TODO: require chained JVS node support then use jvs_nodes.node_players[current_device_addr - 1] 
                //        and current_device_addr to expand player 3,4 and above to other nodes
                case (current_player)
                    4'd0: begin // Player 1
                        // First player data byte
                        p1_btn_state[15] <= rx_buffer[copy_read_idx][7];  // START  
                        p1_btn_state[14] <= rx_buffer[copy_read_idx][6];  // SELECT/SERVICE
                        p1_btn_state[0]  <= rx_buffer[copy_read_idx][5];  // UP
                        p1_btn_state[1]  <= rx_buffer[copy_read_idx][4];  // DOWN  
                        p1_btn_state[2]  <= rx_buffer[copy_read_idx][3];  // LEFT
                        p1_btn_state[3]  <= rx_buffer[copy_read_idx][2];  // RIGHT
                        p1_btn_state[4]  <= rx_buffer[copy_read_idx][1];  // A (push1)
                        p1_btn_state[5]  <= rx_buffer[copy_read_idx][0];  // B (push2)
                        
                        
                        // Second player data byte if available (additional buttons)
                        if (jvs_nodes.node_buttons[current_device_addr - 1] > 8) begin
                            p1_btn_state[6] <= rx_buffer[copy_read_idx + 1][7];  // X (push3)
                            p1_btn_state[7] <= rx_buffer[copy_read_idx + 1][6];  // Y (push4)
                            p1_btn_state[8] <= rx_buffer[copy_read_idx + 1][5];   // push5 -> L1
                            p1_btn_state[9] <= rx_buffer[copy_read_idx + 1][4];   // push6 -> R1
                            p1_btn_state[10] <= rx_buffer[copy_read_idx + 1][3];  // push7 -> L2  
                            p1_btn_state[11] <= rx_buffer[copy_read_idx + 1][2];  // push8 -> R2
                            p1_btn_state[12] <= rx_buffer[copy_read_idx + 1][1];  // push9 -> L3
                            p1_btn_state[13] <= rx_buffer[copy_read_idx + 1][0];  // push10 -> R3
                        end else begin
                            p1_btn_state[13:6] <= 8'b00000000;
                        end
                    end
                    
                    4'd1: begin // Player 2
                        p2_btn_state[15] <= rx_buffer[copy_read_idx][7];  // P2 START  
                        p2_btn_state[14] <= rx_buffer[copy_read_idx][6];  // P2 SELECT
                        p2_btn_state[0]  <= rx_buffer[copy_read_idx][5];  // P2 UP
                        p2_btn_state[1]  <= rx_buffer[copy_read_idx][4];  // P2 DOWN  
                        p2_btn_state[2]  <= rx_buffer[copy_read_idx][3];  // P2 LEFT
                        p2_btn_state[3]  <= rx_buffer[copy_read_idx][2];  // P2 RIGHT
                        p2_btn_state[4]  <= rx_buffer[copy_read_idx][1];  // P2 A
                        p2_btn_state[5]  <= rx_buffer[copy_read_idx][0];  // P2 B
                        
                        if (jvs_nodes.node_buttons[current_device_addr - 1] > 8) begin
                            p2_btn_state[6] <= rx_buffer[copy_read_idx + 1][6];  // P2 X
                            p2_btn_state[7] <= rx_buffer[copy_read_idx + 1][5];  // P2 Y
                            p2_btn_state[8] <= rx_buffer[copy_read_idx + 1][4];  // P2 L1
                            p2_btn_state[9] <= rx_buffer[copy_read_idx + 1][3];  // P2 R1
                            p2_btn_state[10] <= rx_buffer[copy_read_idx + 1][2]; // P2 L2
                            p2_btn_state[11] <= rx_buffer[copy_read_idx + 1][1]; // P2 R2
                        end else begin
                            p2_btn_state[11:6] <= 6'b000000;
                        end
                        p2_btn_state[13:12] <= 2'b00;
                    end
                    
                    default: begin
                        // Player 3+ not supported yet, clear states  
                    end
                endcase
                
                // Advance to next player
                copy_read_idx <= copy_read_idx + ((jvs_nodes.node_buttons[current_device_addr - 1] + 7) / 8);
                current_player <= current_player + 1;
                
                // Check if done with all players
                if (current_player + 1 >= jvs_nodes.node_players[current_device_addr - 1]) begin
                    rx_state <= RX_PARSE_INPUTS_COIN; // All players processed, go to next function
                end
                // else stay in RX_PARSE_SWINP_PLAYER for next player
            end
            
            // RX_PARSE_INPUTS_COIN - Parse coin inputs data  
            if (rx_state == RX_PARSE_INPUTS_COIN) begin
                if (jvs_nodes.node_coin_slots[current_device_addr - 1] > 0) begin
                    // Check COININP report byte
                    if (rx_buffer[copy_read_idx] != REPORT_NORMAL) begin
                        // COININP function failed, skip to next function
                        copy_read_idx <= copy_read_idx + 1; // Skip failed report byte
                        rx_state <= RX_PARSE_INPUTS_ANALOG;
                    end else begin
                        // COININP report is normal, advance past report byte
                        copy_read_idx <= copy_read_idx + 1 + (jvs_nodes.node_coin_slots[current_device_addr - 1] * 2);; // Skip report byte and coin data
                        
                        // @TODO: ADD sub coin parsing state
                        // @TODO: report COIN INCREASE to a coin output
                        /*
                        // Parse coin data (2 bytes per coin slot)
                        // Format: [condition(2 bits) counter_MSB(6 bits)] [counter_LSB]
                        logic [1:0] coin_condition;
                        logic [5:0] counter_msb;
                        logic [7:0] counter_lsb;
                        logic [13:0] coin_counter; // 14-bit coin counter (6+8 bits)
                        
                        // Parse first coin slot data (can be extended for multiple slots)
                        coin_condition = rx_buffer[copy_read_idx][7:6];        // Top 2 bits = condition
                        counter_msb = rx_buffer[copy_read_idx][5:0];           // Bottom 6 bits = counter MSB
                        counter_lsb = rx_buffer[copy_read_idx + 1];            // Next byte = counter LSB
                        coin_counter = {counter_msb, counter_lsb};             // Combine into 14-bit counter
                        
                        // Check coin condition and handle accordingly
                        case (coin_condition)
                            COIN_CONDITION_NORMAL: begin
                                // Normal operation - coin counter is valid
                                // TODO: Store/use coin_counter value if needed
                            end
                            COIN_CONDITION_JAM: begin
                                // Coin jam detected - counter may not be accurate
                                // TODO: Handle jam condition (display warning, etc.)
                            end  
                            COIN_CONDITION_DISCONNECTED: begin
                                // Coin mechanism disconnected - counter invalid
                                // TODO: Handle disconnection (disable coin features)
                            end
                            COIN_CONDITION_BUSY: begin
                                // Coin mechanism busy - temporary state
                                // TODO: Handle busy state (retry later, etc.)
                            end
                        endcase
                        */
                    end
                end
                rx_state <= RX_PARSE_INPUTS_ANALOG;
            end
            
            // RX_PARSE_INPUTS_ANALOG - Parse analog inputs data // 01
            if (rx_state == RX_PARSE_INPUTS_ANALOG) begin
                if (jvs_nodes.node_analog_channels[current_device_addr - 1] > 0) begin
                    // Check ANLINP report byte
                    if (rx_buffer[copy_read_idx] != REPORT_NORMAL) begin
                        // ANLINP function failed, skip to next function
                        copy_read_idx <= copy_read_idx + 1; // Skip failed report byte
                        rx_state <= RX_PARSE_INPUTS_ROTARY;
                    end else begin // 01
                        // ANLINP report is normal, advance past report byte
                        copy_read_idx <= copy_read_idx + 1; // Skip report byte and point to analog channel values
                        p1_joy_state <= 32'h00000000;
                        p1_analog_x_24bit <= 24'h000000;
                        p1_analog_y_24bit <= 24'h000000;
                        current_channel <= 4'd0;
                        rx_state <= RX_PARSE_INPUTS_ANALOG_DATA;
                    end
                end else begin
                    rx_state <= RX_PARSE_INPUTS_ROTARY;
                end
            end
            
            if (rx_state == RX_PARSE_INPUTS_ANALOG_DATA) begin   // Ch1:4600 Ch2:4540 Ch3:4480 Ch4:44C0 Ch5:4580 Ch6:4440 Ch7:43C0 ch8:4440
                // Parse analog input data (2 bytes per channel in 16-bit format)
                case (current_channel)
                    4'd0: begin // Channel 1
                        if (jvs_nodes.node_players[current_device_addr - 1] == 1) begin
                            p1_joy_state[31:20] <= ~{rx_buffer[copy_read_idx],rx_buffer[copy_read_idx + 1][7:4]};
                        end else begin
                            p1_joy_state[31:24] <= rx_buffer[copy_read_idx];
                            p1_joy_state[23:16] <= rx_buffer[copy_read_idx + 1];
                        end
                    end
                    4'd1: begin // Channel 2
                        if (jvs_nodes.node_players[current_device_addr - 1] == 1) begin
                            p1_joy_state[15:4] <= {rx_buffer[copy_read_idx],rx_buffer[copy_read_idx + 1][7:4]};
                        end else begin
                            p1_joy_state[15:8] <= rx_buffer[copy_read_idx];
                            p1_joy_state[7:0] <= rx_buffer[copy_read_idx + 1];
                        end
                    end
                    4'd2: begin // Channel 3
                        if (jvs_nodes.node_players[current_device_addr - 1] == 1) begin
                            // Channel 3: 12-bit MSBs -> X axis LSB [11:0]
                        end else begin
                            p2_joy_state[31:24] <= rx_buffer[copy_read_idx];
                            p2_joy_state[23:16] <= rx_buffer[copy_read_idx + 1];
                        end
                    end
                    4'd3: begin // Channel 4
                        if (jvs_nodes.node_players[current_device_addr - 1] == 1) begin
                            // Channel 4: 12-bit MSBs -> Y axis LSB [11:0]
                        end else begin
                            p2_joy_state[15:8] <= rx_buffer[copy_read_idx];
                            p2_joy_state[7:0] <= rx_buffer[copy_read_idx + 1];
                        end
                    end
                    4'd4: begin // Channel 5
                    end
                    4'd5: begin // Channel 6
                    end
                    4'd6: begin // Channel 7
                    end
                    4'd7: begin // Channel 8
                    end
                    default: begin
                    end
                endcase
                // Advance read index past all analog data (2 bytes per channel)
                copy_read_idx <= copy_read_idx + 2;
                current_channel <= current_channel + 1;
                if (current_channel + 1 >= jvs_nodes.node_analog_channels[current_device_addr - 1]) begin
                    rx_state <= RX_PARSE_INPUTS_ROTARY; // All channels processed, go to next function
                end
                // else stay in RX_PARSE_INPUTS_ANALOG_DATA for next channel
            end

            // RX_PARSE_INPUTS_ROTARY - Parse rotary inputs data
            if (rx_state == RX_PARSE_INPUTS_ROTARY) begin
                if (jvs_nodes.node_rotary_channels[current_device_addr - 1] > 0) begin
                    // Parse rotary encoder data (2 bytes per channel)
                    // For now, we just advance past the rotary data without processing it
                    // TODO: Implement rotary encoder processing if needed
                    copy_read_idx <= copy_read_idx + (jvs_nodes.node_rotary_channels[current_device_addr - 1] * 2);
                end
                rx_state <= RX_PARSE_INPUTS_KEYCODE;
            end
            
            // RX_PARSE_INPUTS_KEYCODE - Parse keycode inputs data
            if (rx_state == RX_PARSE_INPUTS_KEYCODE) begin
                if (jvs_nodes.node_has_keycode_input[current_device_addr - 1]) begin
                    // Check KEYINP report byte
                    if (rx_buffer[copy_read_idx] != REPORT_NORMAL) begin
                        // KEYINP function failed, skip to next function
                        copy_read_idx <= copy_read_idx + 1; // Skip failed report byte
                        rx_state <= RX_PARSE_INPUTS_SCREEN_POS;
                    end else begin
                        // KEYINP report is normal, advance past report byte
                        copy_read_idx <= copy_read_idx + 1; // Skip report byte
                        
                        if (jvs_nodes.node_has_keycode_input[current_device_addr - 1]) begin
                            // Parse keycode input data
                            // Keycode input format: variable length depending on device
                            // For now, we just advance past the keycode data without processing it
                            // TODO: Implement keycode processing if needed
                            // Assuming 1 byte for simplicity, adjust if needed based on device specs
                            copy_read_idx <= copy_read_idx + 1;
                        end
                    end
                end
                rx_state <= RX_PARSE_INPUTS_SCREEN_POS;
            end
            
            // RX_PARSE_INPUTS_SCREEN_POS - Parse screen position inputs data
            if (rx_state == RX_PARSE_INPUTS_SCREEN_POS) begin
                if (jvs_nodes.node_has_screen_pos[current_device_addr - 1]) begin
                    // Check SCRPOSINP report byte
                    if (rx_buffer[copy_read_idx] != REPORT_NORMAL) begin
                        // SCRPOSINP function failed, skip to next function
                        copy_read_idx <= copy_read_idx + 1; // Skip failed report byte
                        rx_state <= RX_PARSE_INPUTS_MISC_DIGITAL;
                    end else begin
                        // SCRPOSINP report is normal, advance past report byte
                        copy_read_idx <= copy_read_idx + 1; // Skip report byte
                        // Parse screen position data
                        // Screen position format: Always 4 bytes = X_MSB, X_LSB, Y_MSB, Y_LSB (per JVS spec)
                        screen_pos_x <= {rx_buffer[copy_read_idx], rx_buffer[copy_read_idx + 1]};
                        screen_pos_y <= {rx_buffer[copy_read_idx + 2], rx_buffer[copy_read_idx + 3]};
                        copy_read_idx <= copy_read_idx + 4; // Fixed 4 bytes per JVS specification
                    end
                end
                rx_state <= RX_PARSE_INPUTS_MISC_DIGITAL;
            end
            
            // RX_PARSE_INPUTS_MISC_DIGITAL - Parse misc digital inputs data
            if (rx_state == RX_PARSE_INPUTS_MISC_DIGITAL) begin
                if (jvs_nodes.node_misc_digital_inputs[current_device_addr - 1] > 0) begin
                    // Check MISCSWINP report byte
                    if (rx_buffer[copy_read_idx] != REPORT_NORMAL) begin
                        // MISCSWINP function failed, skip to complete
                        copy_read_idx <= copy_read_idx + 1; // Skip failed report byte
                        rx_state <= RX_PARSE_INPUTS_COMPLETE;
                    end else begin
                        // MISCSWINP report is normal, advance past report byte
                        copy_read_idx <= copy_read_idx + 1 + ((jvs_nodes.node_misc_digital_inputs[current_device_addr - 1] + 7) / 8);; // Skip report byte and data
                        // Parse misc digital input data
                        // Calculate bytes needed for misc digital inputs (inline calculation)
                        // For now, we just advance past the misc digital data without processing it
                        // TODO: Implement misc digital input processing if needed with substate
                    end
                end
                rx_state <= RX_PARSE_INPUTS_COMPLETE;
            end
            
            // RX_PARSE_OUTPUT_DIGITAL - Parse misc digital inputs data
            if (rx_state == RX_PARSE_OUTPUT_DIGITAL) begin
                if (jvs_nodes.node_misc_digital_inputs[current_device_addr - 1] > 0) begin
                    if (jvs_nodes.node_players[current_device_addr - 1] == 1) begin
                        // Check MISCSWINP report byte
                        if (rx_buffer[copy_read_idx] != REPORT_NORMAL) begin
                            // MISCSWINP function failed, skip to complete
                            copy_read_idx <= copy_read_idx + 1; // Skip failed report byte
                            rx_state <= RX_PARSE_INPUTS_COMPLETE;
                        end else begin
                            // MISCSWINP report is normal, advance past report byte
                            copy_read_idx <= copy_read_idx + 1 + ((jvs_nodes.node_misc_digital_inputs[current_device_addr - 1] + 7) / 8);; // Skip report byte and data
                            // Parse misc digital input data
                            // Calculate bytes needed for misc digital inputs (inline calculation)
                            // For now, we just advance past the misc digital data without processing it
                            // TODO: Implement misc digital input processing if needed with substate
                        end
                    end
                end
                rx_state <= RX_PARSE_INPUTS_COMPLETE;
            end

            // RX_PARSE_INPUTS_COMPLETE - Complete parsing and return to idle
            if (rx_state == RX_PARSE_INPUTS_COMPLETE) begin
                // Input processing complete, signal completion
                jvs_data_ready_joy <= 1'b1; // Indicate new input data available
                rx_frame_complete <= 1'b1;
                rx_counter <= 8'h00;
                rx_state <= RX_IDLE;
            end

            //-------------------------------------------------------------
            // RX_PROCESS - Process complete valid frames
            //-------------------------------------------------------------
            if (rx_state == RX_PROCESS) begin
                // Process responses based on the last command sent
                case (last_tx_state)
                    STATE_SEND_READID: begin
                        // Process READID response: E0 00 XX 01 01 [ASCII_NAME] 00 checksum
                        // Example: "namco ltd.;NAJV2;Ver1.00;JPN,Multipurpose."
                        if (rx_buffer[JVS_ADDR_POS] == JVS_HOST_ADDR && rx_buffer[JVS_STATUS_POS] == STATUS_NORMAL && rx_buffer[JVS_LENGTH_POS] >= 4) begin
                            // Trigger name copying for current node (current_device_addr - 1 as array index)
                            if (current_device_addr > 0 && current_device_addr <= jvs_node_info_pkg::MAX_JVS_NODES) begin
                                // Setup name copying: rx_buffer format [E0][00][LEN][01][01][name...][00][checksum]
                                copy_read_idx <= JVS_REPORT_POS + 1;    // Start after status and report bytes
                                copy_write_idx <= 8'd0;                 // Start writing at beginning of name array
                                rx_state <= RX_COPY_NAME;               // Switch to name copying state
                            end
                        end else begin
                            // Invalid READID response, signal completion anyway
                            rx_frame_complete <= 1'b1;
                            rx_counter <= 8'h00;
                            rx_state <= RX_IDLE;
                        end
                    end
                    
                    STATE_SEND_CMDREV: begin
                        // Process CMDREV response: E0 00 XX 01 01 YY (where YY is revision in BCD)
                        // Format: [sync][addr][len][status][report][data]
                        // Current expected revision is 1.3, so YY should be 0x13
                        if (rx_buffer[JVS_ADDR_POS] == JVS_HOST_ADDR && rx_buffer[JVS_STATUS_POS] == STATUS_NORMAL && rx_buffer[JVS_LENGTH_POS] >= 4) begin
                            // Check report code for errors
                            if (rx_buffer[JVS_REPORT_POS] == REPORT_NORMAL) begin
                                // Store command revision for current node (current_device_addr - 1 as array index)
                                if (current_device_addr > 0 && current_device_addr <= jvs_node_info_pkg::MAX_JVS_NODES) begin
                                    jvs_nodes_r.node_cmd_ver[current_device_addr - 1] <= rx_buffer[JVS_DATA_START + 1]; // rx_buffer[5] contains revision
                                end
                            end
                        end
                        // Command processed, signal completion
                        rx_frame_complete <= 1'b1;
                        rx_counter <= 8'h00;
                        rx_state <= RX_IDLE;
                    end
                    
                    STATE_SEND_JVSREV: begin
                        // Process JVSREV response: E0 00 XX 01 01 YY (where YY is JVS revision in BCD)
                        // Format: [sync][addr][len][status][report][data]
                        // Current expected revision is 3.0, so YY should be 0x30
                        if (rx_buffer[JVS_ADDR_POS] == JVS_HOST_ADDR && rx_buffer[JVS_STATUS_POS] == STATUS_NORMAL && rx_buffer[JVS_LENGTH_POS] >= 4) begin
                            // Check report code for errors
                            if (rx_buffer[JVS_REPORT_POS] == REPORT_NORMAL) begin
                                // Store JVS revision for current node (current_device_addr - 1 as array index)
                                if (current_device_addr > 0 && current_device_addr <= jvs_node_info_pkg::MAX_JVS_NODES) begin
                                    jvs_nodes_r.node_jvs_ver[current_device_addr - 1] <= rx_buffer[JVS_DATA_START + 1]; // rx_buffer[5] contains revision
                                end
                            end
                        end
                        // Command processed, signal completion
                        rx_frame_complete <= 1'b1;
                        rx_counter <= 8'h00;
                        rx_state <= RX_IDLE;
                    end
                    
                    STATE_SEND_COMMVER: begin
                        // Process COMMVER response: E0 00 XX 01 01 YY (where YY is comm version in BCD)
                        // Format: [sync][addr][len][status][report][data]
                        // Current expected version is 1.0, so YY should be 0x10  
                        if (rx_buffer[JVS_ADDR_POS] == JVS_HOST_ADDR && rx_buffer[JVS_STATUS_POS] == STATUS_NORMAL && rx_buffer[JVS_LENGTH_POS] >= 4) begin
                            // Check report code for errors
                            if (rx_buffer[JVS_REPORT_POS] == REPORT_NORMAL) begin
                                // Store communication version for current node (current_device_addr - 1 as array index)
                                if (current_device_addr > 0 && current_device_addr <= jvs_node_info_pkg::MAX_JVS_NODES) begin
                                    jvs_nodes_r.node_com_ver[current_device_addr - 1] <= rx_buffer[JVS_DATA_START + 1]; // rx_buffer[5] contains version
                                end
                            end
                        end
                        // Command processed, signal completion
                        rx_frame_complete <= 1'b1;
                        rx_counter <= 8'h00;
                        rx_state <= RX_IDLE;
                    end
                    
                    STATE_SEND_FEATCHK: begin
                        // Process FEATCHK response: E0 00 XX 01 01 [function_data...] 00
                        // Format: [sync][addr][len][status][report][function_data...]
                        // Contains 4-byte function descriptors followed by 00 terminator
                        if (rx_buffer[JVS_ADDR_POS] == JVS_HOST_ADDR && rx_buffer[JVS_STATUS_POS] == STATUS_NORMAL && rx_buffer[JVS_LENGTH_POS] >= 4) begin
                            // Check report code for errors
                            if (rx_buffer[JVS_REPORT_POS] == REPORT_NORMAL) begin
                                // Parse feature data - Format: [func_code][param1][param2][param3] repeating, then 00
                                if (current_device_addr > 0 && current_device_addr <= jvs_node_info_pkg::MAX_JVS_NODES) begin
                                    // Parse function blocks: start at JVS_DATA_START + 1 (after report byte)
                                    copy_read_idx <= JVS_DATA_START + 1;  // Start after status and report bytes  
                                    rx_state <= RX_PARSE_FEATURES;        // Switch to feature parsing state
                                end
                            end
                        end else begin
                            // Invalid or error response, signal completion anyway
                            rx_frame_complete <= 1'b1;
                            rx_counter <= 8'h00;
                            rx_state <= RX_IDLE;
                        end
                    end
                    
                    STATE_SEND_INPUTS: begin
                        // Process input data response  
                        if (rx_buffer[JVS_ADDR_POS] == JVS_HOST_ADDR && rx_buffer[JVS_STATUS_POS] == STATUS_NORMAL) begin
                            if (rx_buffer[JVS_REPORT_POS] == REPORT_NORMAL) begin
                                // Start progressive input parsing in RX state machine
                                copy_read_idx <= JVS_STATUS_POS + 1;  // Skip status byte
                                rx_state <= RX_PARSE_INPUTS_START;    // Switch to input parsing state
                            end else begin
                                // Input read failed, signal completion
                                rx_frame_complete <= 1'b1;
                                rx_counter <= 8'h00;
                                rx_state <= RX_IDLE;
                            end
                        end else begin
                            // Invalid response, signal completion anyway
                            rx_frame_complete <= 1'b1;
                            rx_counter <= 8'h00;
                            rx_state <= RX_IDLE;
                        end
                    end

                    default: begin
                        // For other commands (SETADDR, etc.), just acknowledge receipt
                        // No special processing needed
                        rx_frame_complete <= 1'b1;
                        rx_counter <= 8'h00;
                        rx_state <= RX_IDLE;
                    end
                endcase
            end
        end
    end
     
endmodule

//=============================================================================
// END OF JVS CONTROLLER MODULE
//=============================================================================

/*
USAGE NOTES:
============

1. Clock Frequency:
   - Module is designed for 50MHz system clock
   - UART baud rate automatically calculated as MASTER_CLK_FREQ / 115200
   - UART modules has been modified from nandland to handle more than 256 cycles single bit time
   - For different clock frequencies, adjust MASTER_CLK_FREQ parameter

2. RS485 Hardware Requirements:
   - External MAX485 or equivalent RS485 transceiver required
   - Connect o_rx485_dir (SNAC_OUT2) to transceiver DE (Driver Enable) and /RE (Receiver Enable) pins (USB D+, the GREEN one)
   - Connect SNAC_OUT1 to DI (USB D-, the WHITE one)
   - Connect IN4 (USB 3.0 RX+, the YELLOW one from twisted pair on my cable) to RO.
   - In a next release, an additional INPUT (IN7) will be required for JVS SENSE (to support chained JVS boards)

3. JVS Device Compatibility:
   - Tested with Namco Noir Cabinet JVS devices, will soon be tested on Viewlix.
   - Should work with most standard JVS arcade systems
   - Some devices may require timing adjustments

4. Timing Considerations:
   - Initial 5.4s delay ensures system stability
   - 2s delay between RESET requests
   - 10ms timeout prevents freeze on invalid frames

6. Debugging:
   - Monitor o_rx485_dir for transmission timing
   - Monitor in the same time data on A or B.
   - Check UART RX signals for communication issues

7. Extensions:
   - Additional players can be supported by extending frame parsing (up to 4 players)
   - Light GUN can be supported (i have a HUUUUGGGEEE TIME CRISIS 4 cabinet for that)
   - Steering wheel ca be supported (with force feeback ?) i have an Initial D 8 Inifinty cabinet too.
   - More JVS commands can be added to command handling
   - Configurable device addressing possible

JVS FRAME ANALYSIS - ACTUAL CAPTURED TRACES:
=============================================

The following section documents the actual JVS communication traces captured 
during development and testing with a Namco Noir cabinet. These real traces 
has served as reference for current JVS protocol implementation.

COMPLETE COMMUNICATION SEQUENCE:
--------------------------------

1. FIRST RESET COMMAND:
   Raw: E0 FF 03 F0 D9 CB
   Decode:
   - E0: Sync byte (frame start)
   - FF: Broadcast address (all devices)
   - 03: Data length (3 bytes total)
   - F0: Reset command byte 1
   - D9: Reset command byte 2  
   - CB: Checksum (FF+03+F0+D9 = 02CB, low byte = CB)

2. SECOND RESET COMMAND:
   Raw: E0 FF 03 F0 D9 CB
   Decode: Identical to first reset (JVS specification requires double reset)

3. SET ADDRESS COMMAND:
   Raw: E0 FF 03 F1 01 F4
   Decode:
   - E0: Sync byte
   - FF: Broadcast address (for initial address assignment)
   - 03: Data length
   - F1: Set address command
   - 01: Address to assign to device
   - F4: Checksum (FF+03+F1+01 = 01F4, low byte = F4)

4. SET ADDRESS RESPONSE (ACK):
   Raw: E0 00 03 01 01 05
   Decode:
   - E0: Sync byte
   - 00: Master address (device responding to master)
   - 03: Data length
   - 01: Status byte (01 = normal/success)
   - 01: Report data (address accepted)
   - 05: Checksum (00+03+01+01 = 05)

5. UNKNOWN COMMAND:
   Raw: E0 01 04 11 12 13 3B
   Decode:
   - E0: Sync byte
   - 01: Device address
   - 04: Data length
   - 11: Unknown command (possibly device-specific)
   - 12 13: Unknown parameters
   - 3B: Checksum (01+04+11+12+13 = 3B)

6. UNKNOWN COMMAND RESPONSE:
   Raw: E0 00 08 01 01 13 01 30 01 10 5F
   Decode:
   - E0: Sync byte
   - 00: Master address
   - 08: Data length (8 bytes)
   - 01: Status (normal)
   - 01 13 01 30 01 10: Unknown response data
   - 5F: Checksum

7. READ ID COMMAND:
   Raw: E0 01 02 10 13
   Decode:
   - E0: Sync byte
   - 01: Device address
   - 02: Data length
   - 10: Read device ID command
   - 13: Checksum (01+02+10 = 13)

8. READ ID RESPONSE:
   Raw: E0 00 2E 01 01 6E 61 6D 63 6F 20 6C 74 64 2E 3B 4E 41 4A 56 32 3B 56 65 72 31 2E 30 30 3B 4A 50 4E 2C 4D 75 6C 74 69 70 75 72 70 6F 73 65 2E 00 29
   Decode:
   - E0: Sync byte
   - 00: Master address
   - 2E: Data length (46 bytes)
   - 01: Status (normal)
   - 01: Report follows
   - ASCII String: "namco ltd.;NAJV2;Ver1.00;JPN,Multipurpose."
     6E 61 6D 63 6F 20 6C 74 64 2E 3B = "namco ltd.;"
     4E 41 4A 56 32 3B = "NAJV2;"
     56 65 72 31 2E 30 30 3B = "Ver1.00;"
     4A 50 4E 2C 4D 75 6C 74 69 70 75 72 70 6F 73 65 2E = "JPN,Multipurpose."
   - 00: String terminator
   - 29: Checksum

9. CAPABILITIES COMMAND:
   Raw: E0 01 02 14 17
   Decode:
   - E0: Sync byte
   - 01: Device address
   - 02: Data length
   - 14: Get capabilities command
   - 17: Checksum (01+02+14 = 17)

10. CAPABILITIES RESPONSE:
    Raw: E0 00 18 01 01 01 02 0D 00 02 02 00 00 03 08 10 00 12 12 00 00 13 02 00 00 00 82
    Decode:
    - E0: Sync byte
    - 00: Master address
    - 18: Data length (24 bytes)
    - 01: Status (normal)
    - 01: Report follows
    - Capabilities data:
      01: Input function (digital inputs)
      02: Number of players supported (2)
      0D: Button configuration
      00: Reserved
      02: Input function (analog inputs)
      02: Number of analog channels
      00 00: Reserved
      03: Input function (rotary encoders)
      08: Number of rotary channels
      10 00: Rotary configuration
      12: Input function (keypad)
      12: Keypad configuration
      00 00: Reserved
      13: Input function (lightgun)
      02: Number of lightgun inputs
      00 00 00: Reserved
    - 82: Checksum

11. DEVICE INFORMATION COMMAND:
    Raw: E0 01 1C 15 4E 42 47 49 2E 3B 57 69 6E 41 72 63 3B 56 65 72 32 2E 31 3B 4A 50 4E 00 7A
    Decode:
    - E0: Sync byte
    - 01: Device address
    - 1C: Data length (28 bytes)
    - 15: Device information command
    - ASCII String: "NBGI.;WinArc;Ver2.1;JPN"
      4E 42 47 49 2E 3B = "NBGI.;"
      57 69 6E 41 72 63 3B = "WinArc;"
      56 65 72 32 2E 31 3B = "Ver2.1;"
      4A 50 4E = "JPN"
    - 00: String terminator
    - 7A: Checksum

12. DEVICE INFO ACK:
    Raw: E0 00 03 01 01 05
    Decode: Same format as SET ADDRESS response (acknowledgment)

13. READ INPUTS COMMAND:
    Raw: E0 01 12 20 02 02 21 01 22 06 32 02 00 00 33 02 00 00 00 00 EA
    Decode:
    - E0: Sync byte
    - 01: Device address
    - 12: Data length (18 bytes)
    - 20: Read inputs command
    - 02: Number of players (2)
    - 02: Bytes per player (2 bytes each)
    - 21: Coin input command
    - 01: Player 1 button byte request
    - 22: Player 2 system/direction byte request
    - 06: Player 2 button byte request
    - 32: Analog channel 1 request
    - 02: 2 bytes of analog data requested
    - 00 00: Analog channel 1 default values
    - 33: Analog channel 2 request
    - 02: 2 bytes of analog data requested
    - 00 00: Analog channel 2 default values
    - 00 00: Padding
    - EA: Checksum

14. READ INPUTS RESPONSE (no buttons pressed):
    Raw: E0 00 1A 01 01 00 00 00 00 00 01 00 00 01 B5 00 B4 40 B3 C0 B2 80 B5 00 B4 80 01 01 57
    Decode:
    - E0: Sync byte
    - 00: Master address  
    - 1A: Data length (26 bytes)
    - 01: Status (normal)
    - 01: Report data follows
    - Button/Direction Data:
      00: Player 1 buttons (all released)
      00: Player 1 directions (centered/no input)
      00: Player 2 buttons (all released)
      00: Player 2 directions (centered/no input)
      00: Additional system byte
    - Analog Data Block:
      01 00 00 01: Unknown format/header
      B5 00: Analog channel data
      B4 40: Analog channel data
      B3 C0: Analog channel data
      B2 80: Analog channel data (0x80 = center)
      B5 00: Analog channel data
      B4 80: Analog channel data (0x80 = center)
      01 01: Status/footer
    - 57: Checksum

15. READ INPUTS COMMAND (variant):
    Raw: E0 01 12 20 02 02 21 01 22 06 32 02 00 80 33 02 00 00 00 00 6A
    Decode: Similar to #13 but with 80 in analog channel 1 (centered position)

16. READ INPUTS RESPONSE (variant):

CURRENT IMPLEMENTATION SEQUENCE:
===============================

The current implementation uses a simplified JVS sequence focused on input 
polling rather than full protocol compliance. Make it work first then improve it.

IMPLEMENTED COMMAND SEQUENCE:
----------------------------

1. FIRST RESET COMMAND (Implemented):
   Sent: E0 FF 03 F0 D9 CB
   - Full implementation matches captured trace
   - 2 second delay after transmission
   - No response expected (broadcast command)

2. SECOND RESET COMMAND (Implemented):
   Sent: E0 FF 03 F0 D9 CB
   - Identical to first reset
   - 500ms delay after transmission
   - Ensures device is fully reset

3. SET ADDRESS COMMAND (Implemented):
   Sent: E0 FF 03 F1 01 F4
   Expected Response: E0 00 03 01 01 05
   - Waits for ACK response with 2s timeout
   - On success: proceeds to READ ID
   - On timeout: restarts reset sequence
    ⚠️ Should rely on SENSE Line going low to know if all JVS boards are initialized, support only one board for now.

4. READ ID COMMAND (Implemented):
   Sent: E0 01 02 10 13
   Expected Response: E0 00 2E 01 01 [ASCII_STRING] [checksum]
   - Waits for device identification
   - Response parsed but not used for device-specific logic
   - On success: starts input polling loop

5. READ INPUTS COMMAND (Implemented - Simplified):
   Sent: E0 01 12 20 02 02 21 01 22 06 32 02 00 00 33 02 00 00 00 00 EA
   Expected Response: E0 00 1A 01 01 [button_data] [analog_data] [checksum]
   - Continuous 1ms polling for responsive gaming
   - Only processes button/direction data at positions 6-9
   - Analog data received but only basic X/Y sticks mapped
   - 10ms timeout to prevent freeze on invalid frames
   ⚠️Probably a Namco specific, on original hardware the topper light is controlled by JVS (blink when the game is loading).

COMMANDS NOT IMPLEMENTED:
------------------------

The following commands from the full trace are NOT implemented in the current
version, as they are not essential for basic gaming functionality:

- Unknown Command (11 12 13): Device-specific, purpose unclear
- Capabilities Command (14): Not needed for fixed input mapping  
- Device Information Command (15): Additional device info not required

CURRENT LIMITATIONS:
-------------------

1. Button Mapping Incomplete:
   - Only basic D-PAD and face buttons mapped
   - START/SELECT functional but may need position adjustment
   - No support for additional buttons (L1/R1/L2/R2)
   - Player 2 mapping present but not fully tested

2. Analog Support Basic:
   - Only first 4 analog channels mapped (2 sticks)
   - Complex analog format from trace not fully decoded
   - Centering works (0x80) but full range may need calibration

3. Error Handling Simplified:
   - 10ms timeout for all responses (vs proper command-specific timeouts)
   - No retry logic for specific command failures
   - Limited validation of response format

4. Protocol Compliance Partial:
   - Missing capabilities negotiation
   - No device-specific optimizations
   - Simplified frame validation

4. Light gun not supported yet (can not wait to play Duck Hunt on my Time Crisis 4).

5. Steering wheel not supported yet (can not wait to play OutRun with force feedback on my Initial D).

FUTURE IMPROVEMENTS NEEDED:
--------------------------

1. Complete Button Mapping (only 4 buttons for now):

2. Test and Validate Player 2 controls mapping

2. Enhanced Analog Support:
   - Decode complex analog format from real traces
   - Implement proper calibration/centering
   - Support additional analog channels if needed

3. Robust Error Handling:
   - Implement proper retry logic for failed commands
   - Add device-specific timeout handling
   - Improve frame validation and error recovery

4. Protocol Completeness:
   - Add capabilities negotiation for device compatibility
   - Implement device information parsing
   - Support additional JVS commands as needed

DEVELOPMENT STATUS:
------------------
✅ Core Protocol: Working (Reset, Address, ID, Input polling)
✅ Basic Buttons: Working (D-PAD, 4 Face buttons, START)
✅ Basic Analog: Working (2 analog sticks with centering)
✅ Escape Sequences: Working (D0 DF → E0, D0 CF → D0 decoding)
✅ RS485 Timing: Working (Proper setup/hold times)
✅ Performance: Optimized (1ms polling, 10ms timeouts)
✅ FPGA Resources: Optimized with configurable buffer sizes (RX_BUFFER_SIZE, TX_BUFFER_SIZE)

⚠️  Button Mapping: Needs verification (START position unclear)
⚠️  Player 2: Present but not fully tested
⚠️  Advanced Analog: Basic implementation only
⚠️  Error Recovery: Simplified timeout handling

❌ Capabilities: Not implemented
❌ Device Info: Not used for logic
❌ Extended Commands: Not supported
❌ Multi-device: Single device only

BUTTON MAPPING DISCOVERY:
------------------------
During development, the following button positions were empirically determined
by pressing individual buttons and observing the bit changes:

Physical Button -> JVS Data Position -> Analogue Pocket Mapping:
START    -> rx_buffer[6][7] -> p1_btn_state[15]
SELECT   -> rx_buffer[6][6] -> p1_btn_state[14]  
Y        -> rx_buffer[7][5] -> p1_btn_state[7]
X        -> rx_buffer[7][6] -> p1_btn_state[6]
B        -> rx_buffer[6][0] -> p1_btn_state[5]
A        -> rx_buffer[6][1] -> p1_btn_state[4]
RIGHT    -> rx_buffer[6][2] -> p1_btn_state[3] (verified: 0x04 = bit 2)
LEFT     -> rx_buffer[6][3] -> p1_btn_state[2] (verified: 0x08 = bit 3)
DOWN     -> rx_buffer[6][4] -> p1_btn_state[1]
UP       -> rx_buffer[6][5] -> p1_btn_state[0]

TIMING ANALYSIS:
---------------
- JVS communication runs at 115200 baud (8N1)
- RS485 setup time: 10µs before transmission
- RS485 hold time: 30µs after transmission

ERROR CONDITIONS OBSERVED:
--------------------------
- Checksum mismatch: Device ignores frame, no response
- Invalid address: Device ignores frame, no response  
- Timeout scenarios: 10ms timeout prevents system freeze
- Cable disconnection: Continuous timeouts, system continues polling
- Power cycle: Requires full reset sequence (double reset + addressing)

PROTOCOL VARIATIONS:
-------------------
Different JVS devices may implement slight variations:
- Some devices use different button bit positions
- Analog resolution may vary (8-bit is standard)
- Response timing can vary between manufacturers
- Additional data fields for specialized controls (guns, wheels, etc.)

*/