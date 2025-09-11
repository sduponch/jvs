//////////////////////////////////////////////////////////////////////
// JVS Controller Module for Analogizer - ALPHA VERSION
// Partial JVS Master implementation optimized for gaming performance
// 
// ⚠️  ALPHA STATUS - PARTIAL COMMAND IMPLEMENTATION ⚠️
//
// This module implements a JVS (JAMMA Video Standard) Master controller
// that allows connecting JVS arcade cabinets to the Analogue Pocket 
// through the Analogizer.
//
// IMPLEMENTATION STATUS (ALPHA ~30-40% complete):
// Based on JVS Specification v3.0 (25 pages, 49 commands total)
//
// ✅ FULLY IMPLEMENTED (8/49 commands):
//    - Reset (F0) - Double reset sequence with timing delays
//    - Set Address (F1) - Single device addressing
//    - IO Identity (10) - Device name string reading (up to 100 chars)
//    - Command Revision (11) - Format revision detection (BCD)
//    - JVS Revision (12) - Protocol version detection (BCD) 
//    - Communications Version (13) - Communication system version
//    - Feature Check (14) - Complete capability parsing with all function codes
//    - Switch Inputs (20) - Digital buttons (2 players, 13 buttons each)
//    - Analog Inputs (22) - Multi-channel 16-bit analog data
//    - Generic Output 1 (32) - Digital GPIO control (3-byte format)
//    - JVS escape sequences (D0 DF → E0, D0 CF → D0)
//    - RS485 timing control with setup/hold delays
//
// 🟡 PARTIALLY IMPLEMENTED (4/49 commands):
//    - Coin Inputs (21) - Command sent, status parsed but coin data ignored
//    - Screen Position Inputs (25) - Basic X/Y coordinates (16-bit each)
//    - Keycode Inputs (24) - Command sent, response skipped
//    - Misc Switch Inputs (26) - Command sent, response skipped
//
// ❌ NOT IMPLEMENTED (37/49 commands):
//    - Main ID (15) - Send main board identification to device
//    - Rotary Inputs (23) - Rotary encoder data (parsed but ignored)
//    - Remaining Payout (2E) - Medal hopper status and count
//    - Data Retransmit (2F) - Checksum error recovery
//    - Coin management: COINDEC (30), COININC (35), PAYINC (31), PAYDEC (36)
//    - Advanced outputs: OUTPUT2 (37), OUTPUT3 (38), ANLOUT (33), CHAROUT (34)
//    - Communication changes: COMMCHG (F2)
//    - Manufacturer-specific commands (Taito TypeX, Namco, CyberLead LED)
//    - Multi-device addressing (supports single device only)
//    - Dynamic baud rate changes
//    - Error recovery mechanisms
//
// PLANNED RESTRUCTURING:
// This monolithic module will be split into:
//   - jvs_com: Communication layer (UART, framing, escape sequences)
//   - jvs_controller: Protocol layer (commands, parsing, state machines)
//   - SNAC interface abstraction: Review interaction with SNAC module for
//     better abstraction and future portability to MiSTer platform
//
// ARCHITECTURE (Current):
// - RS485 State Machine: Manages transceiver direction and timing
// - Main State Machine: Handles JVS protocol sequence and commands  
// - RX State Machine: Processes incoming JVS responses with full parsing
// - Two-buffer system: Raw buffer + processed buffer for unescaped data
// - Node information management for device capabilities
//
// PROTOCOL COMPLIANCE:
// - Physical Layer: RS-485 at 115200 baud (8N1) ✅
// - Link Layer: SYNC(0xE0) + NODE + LENGTH + DATA + CHECKSUM ✅  
// - Escape sequences: D0 DF → E0, D0 CF → D0 ✅
// - Address assignment: Master=0x00, Slaves=0x01-0x1F ✅
// - Initialization: Double reset + sequential addressing ✅
// - Multi-device chaining: Infrastructure present but single device only
//
// HARDWARE REQUIREMENTS:
// - External MAX485 or equivalent RS485 transceiver
// - Proper 120Ω termination for reliable communication  
// - JVS-compatible arcade cabinet
// - SENSE line connection for proper device chaining (unused in single mode)
//
// I/O BOARD COMPATIBILITY:
// ✅ WORKING:
//    - NAJV2 (Tekken 7) - Full compatibility
//    - NAJV (Time Crisis 4) - Full compatibility  
//    - TAITO CORP Ver2.0 (Viewlix) - Full compatibility
// ❌ NOT WORKING:
//    - "No Brand;NAOMI Converter98701;ver2.0" - Frames ignored
//
// Author: Totaly FuRy - Sebastien DUPONCHEEL (sduponch on GitHub)
// Project: Analogizer JVS Controller
// Status: Alpha - Partial Implementation
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
    // JVS_COM INTERFACE SIGNALS (New modular interface)
    //=========================================================================
    // These signals will be used to communicate with the jvs_com module
    
    // TX interface signals
    logic [7:0] com_tx_data;        // Data byte to transmit  
    logic       com_tx_data_push;   // Pulse to push TX data
    logic       com_tx_cmd_push;    // Pulse to push TX command (stores in FIFO)
    logic [7:0] com_dst_node;       // Destination node address
    logic       com_commit;         // Pulse to commit and transmit frame
    logic       com_tx_ready;       // TX ready to accept data
    
    // RX interface signals
    logic [7:0] com_rx_byte;        // Current data byte from RX
    logic       com_rx_next;        // Pulse to get next RX byte
    logic [7:0] com_rx_remaining;   // Bytes remaining (0 = current is last)
    logic [7:0] com_src_node;       // Source node of response
    logic [7:0] com_src_cmd;        // CMD from command FIFO
    logic       com_src_cmd_next;   // Pulse to get next command from FIFO
    logic [4:0] com_src_cmd_count;  // Number of commands available in FIFO
    logic       com_rx_complete;    // Pulse when RX frame complete
    logic       com_rx_error;       // RX checksum or format error

    //=========================================================================
    // COMMAND BUFFER SYSTEM - Sequential transmission with proper pulses
    //=========================================================================
    // Buffer to stack bytes and send them sequentially to jvs_com
    typedef struct packed {
        logic [7:0] data;      // Byte data
        logic       is_cmd;    // 1=command byte, 0=data byte
    } cmd_buffer_entry_t;
    
    localparam CMD_BUFFER_SIZE = 32;
    cmd_buffer_entry_t cmd_buffer [0:CMD_BUFFER_SIZE-1];
    logic [4:0] cmd_buffer_write_ptr;   // Write pointer (5-bit for overflow detection)
    logic [4:0] cmd_buffer_read_ptr;    // Read pointer
    logic [4:0] cmd_buffer_count;       // Number of entries in buffer
    logic       cmd_buffer_sending;     // Currently sending buffered commands
    logic [7:0] cmd_buffer_dst_node;    // Destination node for current buffer
    
    // Helper signals for buffer operations
    logic       buffer_push_cmd;        // Pulse to push command byte
    logic       buffer_push_data;       // Pulse to push data byte  
    logic [7:0] buffer_push_byte;       // Byte to push
    logic       buffer_commit;          // Pulse to start sending buffer
    logic       buffer_ready;           // Buffer ready to accept new data

    //=========================================================================
    // UART TIMING CONFIGURATION
    //=========================================================================
    // Calculate UART clock divider for 115200 baud rate
    // Formula: UART_CLKS_PER_BIT = System_Clock_Frequency / Baud_Rate
    localparam UART_CLKS_PER_BIT = MASTER_CLK_FREQ / 115200;
    
    //=========================================================================
    // JVS COMMUNICATION MODULE INSTANCE
    //=========================================================================
    // Control signals for JVS communication
    logic [7:0] com_tx_data;        // TX data byte to push
    logic       com_tx_data_push;   // Pulse to push TX data
    logic       com_tx_cmd_push;    // NEW: Pulse to push TX command (stores in FIFO)
    logic [7:0] com_dst_node;       // Destination node address
    logic       com_commit;         // Pulse to commit and transmit frame
    logic       com_tx_ready;       // TX ready to accept data
    
    // RX interface signals
    logic [7:0] com_rx_byte;        // Current RX data byte
    logic       com_rx_next;        // Pulse to get next RX byte
    logic [7:0] com_rx_remaining;   // Bytes remaining in RX frame
    logic [7:0] com_src_node;       // Source node of response
    logic [7:0] com_src_cmd;        // CMD from command FIFO
    logic       com_src_cmd_next;   // NEW: Pulse to get next command from FIFO
    logic [4:0] com_src_cmd_count;  // NEW: Number of commands in FIFO
    logic       com_rx_complete;    // Pulse when RX frame complete
    logic       com_rx_error;       // RX checksum or format error
    
    // Instantiate JVS communication module
    jvs_com #(.JVS_BUFFER_SIZE(256)) jvs_com_inst (
        .clk_sys(i_clk),
        .reset(i_rst),
        
        // UART Physical Interface
        .uart_rx(i_uart_rx),
        .uart_tx(o_uart_tx),
        .uart_rts_n(rs485_rts_n),    // RS485 direction control (active-low)
        .uart_baud_div(UART_CLKS_PER_BIT),       // UART clock divisor for 115200 baud
        
        // TX Interface
        .tx_data(com_tx_data),
        .tx_data_push(com_tx_data_push),
        .tx_cmd_push(com_tx_cmd_push),
        .dst_node(com_dst_node),
        .commit(com_commit),
        .tx_ready(com_tx_ready),
        
        // RX Interface
        .rx_byte(com_rx_byte),
        .rx_next(com_rx_next),
        .rx_remaining(com_rx_remaining),
        .src_node(com_src_node),
        .src_cmd(com_src_cmd),
        .src_cmd_next(com_src_cmd_next),
        .src_cmd_count(com_src_cmd_count),
        .rx_complete(com_rx_complete),
        .rx_error(com_rx_error),
        
        // Debug/Status Interface (unused for now)
        .tx_state_debug(),              // Unconnected
        .rx_state_debug(),              // Unconnected
        .frames_tx_count(),             // Unconnected
        .frames_rx_count(),             // Unconnected
        .checksum_errors_count()        // Unconnected
    );
    
    // Convert RS485 direction signal (jvs_com uses active-low RTS)
    wire rs485_rts_n;
    assign o_rx485_dir = ~rs485_rts_n;  // Convert active-low RTS to active-high DIR

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
    // TX states removed - jvs_com module now handles all transmission
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
    localparam STATE_FIRST_RESET_ARG = 5'h1B; // Push reset argument and commit
    localparam STATE_SECOND_RESET_ARG = 5'h1C; // Push second reset argument and commit  
    localparam STATE_TX_NEXT = 5'h1D; // Generic state for pulse handling and counter increment
    localparam STATE_WAIT_TX_COMPLETE = 5'h1E; // Wait for TX completion
    
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
    // Current state for main protocol state machine
    logic [4:0] main_state;        // Main protocol state
    
    // TX state management for sequential byte transmission
    logic [4:0] return_state;      // State to return to after TX_NEXT
    logic [2:0] cmd_pos;           // Position in current command sequence
    
    // Timing and protocol control
    logic [31:0] delay_counter;    // Multi-purpose delay counter
    logic [31:0] timeout_counter;  // Timeout counter for waiting states
    logic [31:0] poll_timer;       // Timer for input polling frequency
    logic [7:0] current_device_addr; // Address assigned to JVS device (usually 0x01)
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
    // RS485 direction now controlled by jvs_com module

    //=========================================================================
    // RS485 STATE MACHINE
    //=========================================================================
    // Manages RS485 transceiver direction with proper setup and hold timing
    // This is critical for reliable RS485 communication
    
    // RS485 timing and control moved to jvs_com module

    //=========================================================================
    // MAIN STATE MACHINE - JVS PROTOCOL HANDLER
    //=========================================================================
    // Implements the complete JVS initialization sequence and input polling
    
    always @(posedge i_clk) begin

        jvs_data_ready_init <= 1'b0;
        
        // Default: Clear all jvs_com control signals (they are pulses)
        com_tx_data_push <= 1'b0;
        com_tx_cmd_push <= 1'b0;
        com_commit <= 1'b0;
        com_rx_next <= 1'b0;
        com_src_cmd_next <= 1'b0;

        if (i_rst || !i_ena) begin
            // Initialize all state variables on reset
            main_state <= STATE_INIT_DELAY;
            delay_counter <= 32'h0;
            timeout_counter <= 32'h0;
            poll_timer <= 32'h0;
            current_device_addr <= 8'h01;    // Standard JVS device address
            last_tx_state <= 5'h0;
            
            // Initialize TX state management
            return_state <= 5'h0;
            cmd_pos <= 3'h0;
            
            // Initialize jvs_com control signals 
            com_tx_data <= 8'h00;
            com_tx_data_push <= 1'b0;
            com_tx_cmd_push <= 1'b0;
            com_dst_node <= 8'h00;
            com_commit <= 1'b0;
            com_rx_next <= 1'b0;
            com_src_cmd_next <= 1'b0;
        end else begin
            case (main_state)
                //-------------------------------------------------------------
                // IDLE STATE - Continuous input polling for responsive gaming
                //-------------------------------------------------------------
                STATE_IDLE: begin
                    
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
                    // Send first RESET command using sequential byte transmission
                    // JVS requires two reset commands for reliable initialization
                    if (com_tx_ready) begin
                        // Initialize command parameters on first entry
                        if (cmd_pos == 0) begin
                            com_dst_node <= JVS_BROADCAST_ADDR;  // FF - Broadcast to all devices
                            return_state <= STATE_FIRST_RESET;
                        end
                        
                        // Select byte and signal based on position
                        case (cmd_pos)
                            3'd0: begin
                                com_tx_data <= CMD_RESET;     // Reset command (0xF0)
                                com_tx_cmd_push <= 1'b1;     // Push as command
                                main_state <= STATE_TX_NEXT; // Go to TX_NEXT
                            end
                            3'd1: begin
                                com_tx_data <= CMD_RESET_ARG; // Reset argument (0xD9)
                                com_tx_data_push <= 1'b1;    // Push as data  
                                main_state <= STATE_TX_NEXT; // Go to TX_NEXT
                            end
                            default: begin
                                // All bytes sent, commit and transition
                                com_commit <= 1'b1;
                                cmd_pos <= 3'd0;
                                main_state <= STATE_FIRST_RESET_DELAY;
                                last_tx_state <= STATE_FIRST_RESET;
                            end
                        endcase
                    end
                end

                //-------------------------------------------------------------
                // DELAY AFTER FIRST RESET
                //-------------------------------------------------------------
                STATE_FIRST_RESET_DELAY: begin
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
                    // Send second RESET command using sequential byte transmission (identical to first)
                    if (com_tx_ready) begin
                        // Initialize command parameters on first entry
                        if (cmd_pos == 0) begin
                            com_dst_node <= JVS_BROADCAST_ADDR;  // FF - Broadcast to all devices
                            return_state <= STATE_SECOND_RESET;
                        end
                        
                        // Select byte and signal based on position
                        case (cmd_pos)
                            3'd0: begin
                                com_tx_data <= CMD_RESET;     // Reset command (0xF0)
                                com_tx_cmd_push <= 1'b1;     // Push as command
                                main_state <= STATE_TX_NEXT; // Go to TX_NEXT
                            end
                            3'd1: begin
                                com_tx_data <= CMD_RESET_ARG; // Reset argument (0xD9)
                                com_tx_data_push <= 1'b1;    // Push as data  
                                main_state <= STATE_TX_NEXT; // Go to TX_NEXT
                            end
                            default: begin
                                // All bytes sent, commit and transition
                                com_commit <= 1'b1;
                                cmd_pos <= 3'd0;
                                main_state <= STATE_SECOND_RESET_DELAY;
                                last_tx_state <= STATE_SECOND_RESET;
                            end
                        endcase
                    end
                end

                //-------------------------------------------------------------
                // DELAY AFTER SECOND RESET
                //-------------------------------------------------------------
                STATE_SECOND_RESET_DELAY: begin
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
                    // Send SET ADDRESS command using sequential byte transmission
                    // This assigns a unique address (0x01) to the JVS device
                    if (com_tx_ready) begin
                        // Initialize command parameters on first entry
                        if (cmd_pos == 0) begin
                            com_dst_node <= JVS_BROADCAST_ADDR;  // FF - Still broadcast for address assignment
                            return_state <= STATE_SEND_SETADDR;
                        end
                        
                        // Select byte and signal based on position
                        case (cmd_pos)
                            3'd0: begin
                                com_tx_data <= CMD_SETADDR;         // Set address command (0xF1)
                                com_tx_cmd_push <= 1'b1;            // Push as command
                                main_state <= STATE_TX_NEXT;        // Go to TX_NEXT
                            end
                            3'd1: begin
                                com_tx_data <= current_device_addr; // 01 - Address to assign
                                com_tx_data_push <= 1'b1;           // Push as data
                                main_state <= STATE_TX_NEXT;        // Go to TX_NEXT
                            end
                            default: begin
                                // All bytes sent, commit and transition
                                com_commit <= 1'b1;
                                cmd_pos <= 3'd0;
                                main_state <= STATE_WAIT_RX;
                                last_tx_state <= STATE_SEND_SETADDR;
                            end
                        endcase
                    end
                end

                //-------------------------------------------------------------
                // READ ID COMMAND - Request device identification
                //-------------------------------------------------------------
                STATE_SEND_READID: begin
                    // Send READ ID command using sequential byte transmission
                    // This requests the device to send its identification string
                    if (com_tx_ready) begin
                        // Initialize command parameters on first entry
                        if (cmd_pos == 0) begin
                            com_dst_node <= current_device_addr; // 01 - Address specific device
                            return_state <= STATE_SEND_READID;
                        end
                        
                        // Select byte and signal based on position
                        case (cmd_pos)
                            3'd0: begin
                                com_tx_data <= CMD_IOIDENT;          // IO identity command (0x10)
                                com_tx_cmd_push <= 1'b1;             // Push as command
                                main_state <= STATE_TX_NEXT;         // Go to TX_NEXT
                            end
                            default: begin
                                // All bytes sent, commit and transition
                                com_commit <= 1'b1;
                                cmd_pos <= 3'd0;
                                main_state <= STATE_WAIT_RX;
                                last_tx_state <= STATE_SEND_READID;
                            end
                        endcase
                    end
                end

                //-------------------------------------------------------------
                // COMMAND REVISION REQUEST - Get command format revision
                //-------------------------------------------------------------
                STATE_SEND_CMDREV: begin
                    // Send CMDREV command using sequential byte transmission
                    if (com_tx_ready) begin
                        // Initialize command parameters on first entry
                        if (cmd_pos == 0) begin
                            com_dst_node <= current_device_addr; // 01
                            return_state <= STATE_SEND_CMDREV;
                        end
                        
                        // Select byte and signal based on position
                        case (cmd_pos)
                            3'd0: begin
                                com_tx_data <= CMD_CMDREV;          // Command revision command (0x11)
                                com_tx_cmd_push <= 1'b1;            // Push as command
                                main_state <= STATE_TX_NEXT;        // Go to TX_NEXT
                            end
                            default: begin
                                // All bytes sent, commit and transition
                                com_commit <= 1'b1;
                                cmd_pos <= 3'd0;
                                main_state <= STATE_WAIT_RX;
                                last_tx_state <= STATE_SEND_CMDREV;
                            end
                        endcase
                    end
                end

                //-------------------------------------------------------------
                // JVS REVISION REQUEST - Get JVS protocol revision
                //-------------------------------------------------------------
                STATE_SEND_JVSREV: begin
                    // Send JVSREV command using sequential byte transmission
                    if (com_tx_ready) begin
                        // Initialize command parameters on first entry
                        if (cmd_pos == 0) begin
                            com_dst_node <= current_device_addr; // 01
                            return_state <= STATE_SEND_JVSREV;
                        end
                        
                        // Select byte and signal based on position
                        case (cmd_pos)
                            3'd0: begin
                                com_tx_data <= CMD_JVSREV;          // JVS revision command (0x12)
                                com_tx_cmd_push <= 1'b1;            // Push as command
                                main_state <= STATE_TX_NEXT;        // Go to TX_NEXT
                            end
                            default: begin
                                // All bytes sent, commit and transition
                                com_commit <= 1'b1;
                                cmd_pos <= 3'd0;
                                main_state <= STATE_WAIT_RX;
                                last_tx_state <= STATE_SEND_JVSREV;
                            end
                        endcase
                    end
                end

                //-------------------------------------------------------------
                // COMMUNICATIONS VERSION REQUEST - Get communication version
                //-------------------------------------------------------------
                STATE_SEND_COMMVER: begin
                    // Send COMMVER command using sequential byte transmission
                    if (com_tx_ready) begin
                        // Initialize command parameters on first entry
                        if (cmd_pos == 0) begin
                            com_dst_node <= current_device_addr; // 01
                            return_state <= STATE_SEND_COMMVER;
                        end
                        
                        // Select byte and signal based on position
                        case (cmd_pos)
                            3'd0: begin
                                com_tx_data <= CMD_COMMVER;         // Communication version command (0x13)
                                com_tx_cmd_push <= 1'b1;            // Push as command
                                main_state <= STATE_TX_NEXT;        // Go to TX_NEXT
                            end
                            default: begin
                                // All bytes sent, commit and transition
                                com_commit <= 1'b1;
                                cmd_pos <= 3'd0;
                                main_state <= STATE_WAIT_RX;
                                last_tx_state <= STATE_SEND_COMMVER;
                            end
                        endcase
                    end
                end

                //-------------------------------------------------------------
                // FEATURE CHECK REQUEST - Get device capabilities
                //-------------------------------------------------------------
                STATE_SEND_FEATCHK: begin
                    // Send FEATCHK command using sequential byte transmission
                    if (com_tx_ready) begin
                        // Initialize command parameters on first entry
                        if (cmd_pos == 0) begin
                            com_dst_node <= current_device_addr; // 01
                            return_state <= STATE_SEND_FEATCHK;
                        end
                        
                        // Select byte and signal based on position
                        case (cmd_pos)
                            3'd0: begin
                                com_tx_data <= CMD_FEATCHK;         // Feature check command (0x14)
                                com_tx_cmd_push <= 1'b1;            // Push as command
                                main_state <= STATE_TX_NEXT;        // Go to TX_NEXT
                            end
                            default: begin
                                // All bytes sent, commit and transition
                                com_commit <= 1'b1;
                                cmd_pos <= 3'd0;
                                main_state <= STATE_WAIT_RX;
                                last_tx_state <= STATE_SEND_FEATCHK;
                            end
                        endcase
                    end
                end

                //-------------------------------------------------------------
                // READ INPUTS COMMAND - Request current input states
                //-------------------------------------------------------------
                STATE_SEND_INPUTS: begin
                    // Send input commands using new jvs_com interface
                    if (com_tx_ready) begin
                        // Set destination node
                        com_dst_node <= current_device_addr; // 01
                        
                        // Begin progressive state machine - start with switch inputs
                        main_state <= STATE_SEND_INPUTS_SWITCH;
                    end
                end

                // TX states removed - jvs_com module now handles all transmission

                //-------------------------------------------------------------
                // WAIT FOR DEVICE RESPONSE
                //-------------------------------------------------------------
                STATE_WAIT_RX: begin
                    if (com_rx_complete) begin
                        // Process response based on command sent (now available via com_src_cmd)
                        case (com_src_cmd)
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
                        case (com_src_cmd)
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
                        // Push SWINP command and parameters
                        com_tx_data <= CMD_SWINP; // SWINP command (0x20)
                        com_tx_cmd_push <= 1'b1;  // Push as command
                        
                        com_tx_data <= jvs_nodes.node_players[current_device_addr - 1];  // Number of players
                        com_tx_data_push <= 1'b1; // Push as data
                        
                        com_tx_data <= (jvs_nodes.node_buttons[current_device_addr - 1] + 7) / 8; // Compute number of bytes to include all per player bits
                        com_tx_data_push <= 1'b1; // Push as data
                    end
                    main_state <= STATE_SEND_INPUTS_COIN;
                end
                
                STATE_SEND_INPUTS_COIN: begin
                    if (jvs_nodes.node_coin_slots[current_device_addr - 1] > 0) begin
                        // Push coin input command
                        com_tx_data <= CMD_COININP; // COININP command (0x21)
                        com_tx_cmd_push <= 1'b1;    // Push as command
                        
                        com_tx_data <= jvs_nodes.node_coin_slots[current_device_addr - 1]; // Number of coin slots
                        com_tx_data_push <= 1'b1;   // Push as data
                    end
                    main_state <= STATE_SEND_INPUTS_ANALOG;
                end
                
                STATE_SEND_INPUTS_ANALOG: begin
                    if (jvs_nodes.node_analog_channels[current_device_addr - 1] > 0) begin
                        // Push analog input command
                        com_tx_data <= CMD_ANLINP; // ANLINP command (0x22)
                        com_tx_cmd_push <= 1'b1;   // Push as command
                        
                        com_tx_data <= jvs_nodes.node_analog_channels[current_device_addr - 1]; // Number of analog channels
                        com_tx_data_push <= 1'b1;  // Push as data
                    end
                    main_state <= STATE_SEND_INPUTS_ROTARY;
                end
                
                STATE_SEND_INPUTS_ROTARY: begin
                    if (jvs_nodes.node_rotary_channels[current_device_addr - 1] > 0) begin
                        // Push rotary input command
                        com_tx_data <= CMD_ROTINP;  // ROTINP command (0x23)
                        com_tx_cmd_push <= 1'b1;    // Push as command
                        
                        com_tx_data <= jvs_nodes.node_rotary_channels[current_device_addr - 1]; // Number of rotary channels
                        com_tx_data_push <= 1'b1;   // Push as data
                    end
                    main_state <= STATE_SEND_INPUTS_KEYCODE;
                end
                
                STATE_SEND_INPUTS_KEYCODE: begin
                    if (jvs_nodes.node_has_keycode_input[current_device_addr - 1]) begin
                        // Push keycode input command (no parameters)
                        com_tx_data <= CMD_KEYINP;  // KEYINP command (0x24)
                        com_tx_cmd_push <= 1'b1;    // Push as command
                    end
                    main_state <= STATE_SEND_INPUTS_SCREEN;
                end
                
                STATE_SEND_INPUTS_SCREEN: begin
                    if (jvs_nodes.node_has_screen_pos[current_device_addr - 1]) begin
                        // Push screen position input command
                        com_tx_data <= CMD_SCRPOSINP;  // SCRPOSINP command (0x25)
                        com_tx_cmd_push <= 1'b1;       // Push as command
                        
                        com_tx_data <= 8'h01;          // Channel index
                        com_tx_data_push <= 1'b1;      // Push as data
                    end
                    main_state <= STATE_SEND_INPUTS_MISC;
                end
                
                STATE_SEND_INPUTS_MISC: begin
                    if (jvs_nodes.node_misc_digital_inputs[current_device_addr - 1] > 0) begin
                        // Push misc switch input command
                        com_tx_data <= CMD_MISCSWINP;  // MISCSWINP command (0x26)
                        com_tx_cmd_push <= 1'b1;       // Push as command
                        
                        com_tx_data <= (jvs_nodes.node_misc_digital_inputs[current_device_addr - 1] + 7) / 8; // Bytes needed
                        com_tx_data_push <= 1'b1;      // Push as data
                    end
                    main_state <= STATE_SEND_OUTPUT_DIGITAL;
                end
                
                STATE_SEND_OUTPUT_DIGITAL: begin
                    // Check if device has digital outputs
                    if (jvs_nodes.node_digital_outputs[current_device_addr - 1] > 0) begin
                        if (jvs_nodes.node_players[current_device_addr - 1] == 1) begin
                            // Push OUTPUT1 command frame for GPIO control
                            com_tx_data <= CMD_OUTPUT1;         // OUTPUT1 command (0x32)
                            com_tx_cmd_push <= 1'b1;            // Push as command
                            
                            com_tx_data <= 8'h03;               // send 3 bytes (from time crisis 4 capture althouth that FEATCHK report 12 channels/bits?)
                            com_tx_data_push <= 1'b1;           // Push as data
                            
                            com_tx_data <= gpio_output_value;   // Set GPIO1 to current value from SNAC
                            com_tx_data_push <= 1'b1;           // Push as data
                            
                            com_tx_data <= 8'hA0;               // do not know what A is for, but taken from TC4 capture
                            com_tx_data_push <= 1'b1;           // Push as data
                            
                            com_tx_data <= 8'h00;
                            com_tx_data_push <= 1'b1;           // Push as data
                        end
                    end
                    main_state <= STATE_SEND_FINALIZE;
                end

                STATE_SEND_FINALIZE: begin
                    // Commit and transmit frame using new jvs_com interface
                    com_commit <= 1'b1;
                    
                    // Wait for response
                    main_state <= STATE_WAIT_RX;
                    last_tx_state <= STATE_SEND_INPUTS;
                end

                //-------------------------------------------------------------
                // GENERIC TX NEXT STATE - Handles pulse cleanup and position increment
                //-------------------------------------------------------------
                STATE_TX_NEXT: begin
                    // Reset all push signals to 0 (they were set to 1 in calling state)
                    com_tx_cmd_push <= 1'b0;
                    com_tx_data_push <= 1'b0;
                    
                    // Increment position
                    cmd_pos <= cmd_pos + 1;
                    
                    // Return to calling state
                    main_state <= return_state;
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
            // Initialize output registers
            
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
            // RX frame processing managed by jvs_com module
            
            
            // Process incoming frames from jvs_com module
            // For now, simplified processing - full RX parsing needs state machine implementation
            if (com_rx_complete && !com_rx_error) begin
                // Process response based on command that was sent
                case (com_src_cmd)
                    CMD_CMDREV, CMD_JVSREV, CMD_COMMVER: begin
                        // Single byte responses - read version
                        if (com_rx_remaining > 0) begin
                            case (com_src_cmd)
                                CMD_CMDREV: jvs_nodes_r.node_cmd_ver[current_device_addr - 1] <= com_rx_byte;
                                CMD_JVSREV: jvs_nodes_r.node_jvs_ver[current_device_addr - 1] <= com_rx_byte;
                                CMD_COMMVER: jvs_nodes_r.node_com_ver[current_device_addr - 1] <= com_rx_byte;
                            endcase
                        end
                    end
                    
                    CMD_SWINP: begin
                        // Signal input data ready for gaming
                        jvs_data_ready_joy <= 1'b1;
                    end
                    
                    default: begin
                        // For other commands (IOIDENT, FEATCHK, etc.), acknowledge receipt
                        // Full parsing implementation will be added in future updates
                    end
                endcase
                
                // Handle chained commands by advancing to next command
                if (com_src_cmd_count > 1) begin
                    com_src_cmd_next <= 1'b1;
                end
            end
        end
    end
     
endmodule