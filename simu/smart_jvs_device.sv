//////////////////////////////////////////////////////////////////////
// Smart JVS Device - Simulateur JVS Intelligent avec Réponses
//
// Ce module simule un périphérique JVS complet qui peut répondre aux
// commandes du contrôleur JVS, basé sur la capture Naomi Tekken 6.
//
// Fonctionnalités:
// - Décodage UART fiable (utilise uart_rx.v/uart_tx.v)
// - Réponses automatiques aux commandes JVS
// - Simulation d'un board Tekken 6/NAOMI
// - Support des commandes principales: Address Assignment, Device ID, Feature Check
//
// Basé sur: naomi_tekken6_io_board.jvs
//////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
`default_nettype none

module smart_jvs_device #(
    parameter MASTER_CLK_FREQ = 50_000_000
) (
    input logic clk,
    input logic rst,

    // Interface UART (connecté au contrôleur JVS)
    input logic uart_rx_from_master,    // Signal UART du maître
    output logic uart_tx_to_master,     // Réponse au maître

    // Interface de monitoring (pour les tests)
    output logic [7:0] last_received_bytes[0:63], // Buffer des bytes reçus
    output integer bytes_received_count,           // Nombre de bytes reçus
    output logic new_frame_received,               // Pulse quand trame complète
    output logic [7:0] frame_length,               // Longueur de la dernière trame

    // Interface de contrôle
    input logic [7:0] assigned_address,            // Adresse assignée (défaut 0x01)
    input logic auto_respond,                      // Active les réponses automatiques

    // Debug
    output logic uart_rx_active,
    output logic [7:0] current_byte,
    output logic response_sent                     // Pulse quand réponse envoyée
);
    localparam UART_CLKS_PER_BIT = MASTER_CLK_FREQ / 115200;

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

    localparam JVS_OVERHEAD = 8'd2;          // Overhead for length calculation (includes checksum + command byte)


    // Signaux UART RX
    wire uart_rx_dv;
    wire [7:0] uart_rx_byte;

    // Signaux UART TX
    logic uart_tx_dv;
    logic [7:0] uart_tx_byte;
    wire uart_tx_active;
    wire uart_tx_done;

    // Instance du module UART RX réel
    uart_rx #(
        .CLKS_PER_BIT(UART_CLKS_PER_BIT)
    ) uart_rx_inst (
        .i_Clock(clk),
        .i_Rx_Serial(uart_rx_from_master),
        .o_Rx_DV(uart_rx_dv),
        .o_Rx_Byte(uart_rx_byte)
    );

    // Instance du module UART TX réel
    uart_tx #(
        .CLKS_PER_BIT(UART_CLKS_PER_BIT)
    ) uart_tx_inst (
        .i_Clock(clk),
        .i_Tx_DV(uart_tx_dv),
        .i_Tx_Byte(uart_tx_byte),
        .o_Tx_Active(uart_tx_active),
        .o_Tx_Serial(uart_tx_to_master),
        .o_Tx_Done(uart_tx_done)
    );

    // Variables internes de réception
    logic [7:0] received_buffer[0:63];
    logic [7:0] completed_frame_buffer[0:63];
    integer byte_count = 0;
    integer completed_frame_length = 0;
    logic frame_in_progress = 0;
    logic [7:0] expected_frame_length = 0;
    logic [7:0] current_frame_length = 0;
    logic [7:0] device_address = 8'h01; // Adresse par défaut

    logic [7:0] rx_length;         // Length of current incoming frame
    logic [7:0] rx_counter = 8'h00;
    logic [7:0] rx_checksum = 8'h00;
    logic rx_frame_complete;       // Flag indicating frame has been processed and ready for next step

    logic [7:0] copy_read_idx;      // Read index for copy operations
    logic [7:0] copy_write_idx;     // Write index for copy operations


    // État de réception JVS
    typedef enum logic [2:0] {
        RX_IDLE,      // Attendre 0xE0
        RX_READ_ADDR,   // Lire adresse
        RX_READ_SIZE,    // Lire longueur
        RX_READ_DATA,      // Lire données + checksum
        RX_UNESCAPE,
        RX_PROCESS  // Trame complète
    } jvs_rx_state_t;

    jvs_rx_state_t rx_state = RX_IDLE;

    // État de transmission de réponse
    typedef enum logic [2:0] {
        TX_IDLE,
        TX_WAIT_SETUP,      // Attendre setup time RS485 (5ms)
        TX_PREPARE,
        TX_SEND_BYTE,
        TX_WAIT_DONE,
        TX_COMPLETE
    } jvs_tx_state_t;

    jvs_tx_state_t tx_state = TX_IDLE;

    // Buffer de réponse et contrôle
    logic [7:0] rx_buffer[0:63];
    logic [7:0] rx_buffer_raw[0:63];

    integer response_length = 0;
    integer response_index = 0;
    logic need_response = 0;

    // Compteur pour délai RS485 setup time (1ms)
    localparam logic [31:0] SETUP_TIME_CYCLES = MASTER_CLK_FREQ / 1_100_000; // 100us
    integer setup_time_counter = 0;

    // Assignations de sortie
    assign uart_rx_active = uart_rx_dv;
    assign current_byte = uart_rx_byte;
    assign frame_length = current_frame_length;

    // Copier le buffer de trame complètée vers la sortie
    always_comb begin
        for (integer i = 0; i < 64; i++) begin
            last_received_bytes[i] = completed_frame_buffer[i];
        end
        bytes_received_count = completed_frame_length;
    end

    //=========================================================================
    // RÉPONSES JVS PRÉDÉFINIES (basées sur capture Tekken 6)
    //=========================================================================

    // Réponse ACK simple
    function automatic void prepare_ack_response();
        rx_buffer[0] = 8'hE0;           // SYNC
        rx_buffer[1] = 8'h00;           // SOURCE (master)
        rx_buffer[2] = 8'h03;           // LENGTH (3 bytes payload)
        rx_buffer[3] = 8'h01;           // STATUS (Report = 1)
        rx_buffer[4] = 8'h01;           // ACK
        rx_buffer[5] = 8'h05;           // CHECKSUM (00+03+01+01 = 05)
        response_length = 6;
        $display("[SMART_JVS] Préparation réponse ACK");
    endfunction

    // Réponse Device ID (basée sur Tekken 6)
    function automatic void prepare_device_id_response();
        // E0002A01014E6F204272616E643B4E414F4D4920436F6E76657274657239383730313B766572322E303B2000F3
        integer idx = 0;
        logic [7:0] checksum = 0;

        rx_buffer[idx++] = 8'hE0;       // SYNC
        rx_buffer[idx++] = 8'h00;       // SOURCE (master)
        rx_buffer[idx++] = 8'h2A;       // LENGTH (42 bytes)
        rx_buffer[idx++] = 8'h01;       // STATUS (Report = 1)
        rx_buffer[idx++] = 8'h01;       // ACK

        // Device Name: "No Brand;NAOMI Converter987201;ver2.0; "
        rx_buffer[idx++] = 8'h4E; // N
        rx_buffer[idx++] = 8'h6F; // o
        rx_buffer[idx++] = 8'h20; // (space)
        rx_buffer[idx++] = 8'h42; // B
        rx_buffer[idx++] = 8'h72; // r
        rx_buffer[idx++] = 8'h61; // a
        rx_buffer[idx++] = 8'h6E; // n
        rx_buffer[idx++] = 8'h64; // d
        rx_buffer[idx++] = 8'h3B; // ;
        rx_buffer[idx++] = 8'h4E; // N
        rx_buffer[idx++] = 8'h41; // A
        rx_buffer[idx++] = 8'h4F; // O
        rx_buffer[idx++] = 8'h4D; // M
        rx_buffer[idx++] = 8'h49; // I
        rx_buffer[idx++] = 8'h20; // (space)
        rx_buffer[idx++] = 8'h43; // C
        rx_buffer[idx++] = 8'h6F; // o
        rx_buffer[idx++] = 8'h6E; // n
        rx_buffer[idx++] = 8'h76; // v
        rx_buffer[idx++] = 8'h65; // e
        rx_buffer[idx++] = 8'h72; // r
        rx_buffer[idx++] = 8'h74; // t
        rx_buffer[idx++] = 8'h65; // e
        rx_buffer[idx++] = 8'h72; // r
        rx_buffer[idx++] = 8'h39; // 9
        rx_buffer[idx++] = 8'h38; // 8
        rx_buffer[idx++] = 8'h37; // 7
        rx_buffer[idx++] = 8'h30; // 0
        rx_buffer[idx++] = 8'h31; // 1
        rx_buffer[idx++] = 8'h3B; // ;
        rx_buffer[idx++] = 8'h76; // v
        rx_buffer[idx++] = 8'h65; // e
        rx_buffer[idx++] = 8'h72; // r
        rx_buffer[idx++] = 8'h32; // 2
        rx_buffer[idx++] = 8'h2E; // .
        rx_buffer[idx++] = 8'h30; // 0
        rx_buffer[idx++] = 8'h3B; // ;
        rx_buffer[idx++] = 8'h20; // (space)
        rx_buffer[idx++] = 8'h00; // NULL terminator

        // Calculer checksum
        for (integer i = 1; i < idx; i++) begin
            checksum = checksum + rx_buffer[i];
        end
        rx_buffer[idx++] = checksum; // CHECKSUM

        response_length = idx;
        $display("[SMART_JVS] Préparation réponse Device ID (%d bytes)", response_length);
    endfunction

    // Réponse Feature Check (basée sur Tekken 6)
    function automatic void prepare_feature_response();
        // E00010010101020C0002020000030800000030
        integer idx = 0;

        rx_buffer[idx++] = 8'hE0;       // SYNC
        rx_buffer[idx++] = 8'h00;       // SOURCE (master)
        rx_buffer[idx++] = 8'h10;       // LENGTH (16 bytes)
        rx_buffer[idx++] = 8'h01;       // STATUS (Report = 1)
        rx_buffer[idx++] = 8'h01;       // ACK

        // Features from Tekken 6 (exact copy)
        rx_buffer[idx++] = 8'h01;       // Digital inputs
        rx_buffer[idx++] = 8'h02;       // 2 players
        rx_buffer[idx++] = 8'h0C;       // 12 buttons per player
        rx_buffer[idx++] = 8'h00;
        rx_buffer[idx++] = 8'h02;       // Analog inputs
        rx_buffer[idx++] = 8'h02;       // 2 channels
        rx_buffer[idx++] = 8'h00;       //
        rx_buffer[idx++] = 8'h00;       //
        rx_buffer[idx++] = 8'h03;       // Digital outputs
        rx_buffer[idx++] = 8'h08;       // 8 outputs
        rx_buffer[idx++] = 8'h00;       //
        rx_buffer[idx++] = 8'h00;       //
        rx_buffer[idx++] = 8'h00;       // End of features

        // Checksum exact de Tekken 6
        rx_buffer[idx++] = 8'h30;       // CHECKSUM from real capture

        response_length = idx;
        $display("[SMART_JVS] Préparation réponse Feature Check (%d bytes)", response_length);
    endfunction

    // Command revision response (0x11)
    function automatic void prepare_cmdrev_response();
        integer idx = 0;
        logic [7:0] checksum = 0;

        rx_buffer[idx++] = 8'hE0;       // SYNC
        rx_buffer[idx++] = 8'h00;       // SOURCE (master)
        rx_buffer[idx++] = 8'h04;       // LENGTH (4 bytes payload)
        rx_buffer[idx++] = 8'h01;       // STATUS (Report = 1)
        rx_buffer[idx++] = 8'h01;       // REPORT (Normal)
        rx_buffer[idx++] = 8'h13;       // Command revision v1.3 (BCD format)

        // Calculate checksum (excluding SYNC byte)
        for (integer i = 1; i < idx; i++) begin
            checksum = checksum + rx_buffer[i];
        end
        rx_buffer[idx++] = checksum; // CHECKSUM

        response_length = idx;
        $display("[SMART_JVS] Préparation réponse Command Revision (%d bytes) - Rev: 0x%02X", response_length, 8'h13);
    endfunction

    // JVS revision response (0x12)
    function automatic void prepare_jvsrev_response();
        integer idx = 0;
        logic [7:0] checksum = 0;

        rx_buffer[idx++] = 8'hE0;       // SYNC
        rx_buffer[idx++] = 8'h00;       // SOURCE (master)
        rx_buffer[idx++] = 8'h04;       // LENGTH (4 bytes payload)
        rx_buffer[idx++] = 8'h01;       // STATUS (Report = 1)
        rx_buffer[idx++] = 8'h01;       // REPORT (Normal)
        rx_buffer[idx++] = 8'h30;       // JVS revision v3.0 (BCD format)

        // Calculate checksum (excluding SYNC byte)
        for (integer i = 1; i < idx; i++) begin
            checksum = checksum + rx_buffer[i];
        end
        rx_buffer[idx++] = checksum; // CHECKSUM

        response_length = idx;
        $display("[SMART_JVS] Préparation réponse JVS Revision (%d bytes) - Rev: 0x%02X", response_length, 8'h30);
    endfunction

    // Communication version response (0x13)
    function automatic void prepare_commver_response();
        integer idx = 0;
        logic [7:0] checksum = 0;

        rx_buffer[idx++] = 8'hE0;       // SYNC
        rx_buffer[idx++] = 8'h00;       // SOURCE (master)
        rx_buffer[idx++] = 8'h04;       // LENGTH (4 bytes payload)
        rx_buffer[idx++] = 8'h01;       // STATUS (Report = 1)
        rx_buffer[idx++] = 8'h01;       // REPORT (Normal)
        rx_buffer[idx++] = 8'h10;       // Communication version v1.0 (BCD format)

        // Calculate checksum (excluding SYNC byte)
        for (integer i = 1; i < idx; i++) begin
            checksum = checksum + rx_buffer[i];
        end
        rx_buffer[idx++] = checksum; // CHECKSUM

        response_length = idx;
        $display("[SMART_JVS] Préparation réponse Communication Version (%d bytes) - Ver: 0x%02X", response_length, 8'h10);
    endfunction

    //=========================================================================
    // ANALYSEUR DE COMMANDES JVS
    //=========================================================================
    function automatic void analyze_and_respond();
        logic [7:0] cmd;
        logic addressed_to_us;

        if (copy_write_idx >= 4) begin
            addressed_to_us = (rx_buffer[1] == device_address || rx_buffer[1] == 8'hFF);
            cmd = rx_buffer[3];

            $display("[SMART_JVS] Commande reçue: 0x%02X, adressée à: 0x%02X %s",
                    cmd, rx_buffer[1],
                    addressed_to_us ? "(NOUS)" : "(AUTRE)");

            if (addressed_to_us && auto_respond) begin
                case (cmd)
                    8'hF0: begin // CMD_RESET - pas de réponse
                        $display("[SMART_JVS] -> Reset reçu, pas de réponse");
                        need_response = 0;
                    end

                    8'hF1: begin // CMD_SETADDR - Address assignment
                        if (copy_write_idx >= 5) begin
                            device_address = rx_buffer[4];
                            $display("[SMART_JVS] -> Adresse assignée: 0x%02X", device_address);
                            prepare_ack_response();
                            need_response = 1;
                        end
                    end

                    8'h10: begin // CMD_IOIDENT - Device identification
                        $display("[SMART_JVS] -> Demande d'identification");
                        prepare_device_id_response();
                        need_response = 1;
                    end

                    8'h11: begin // CMD_CMDREV - Command revision
                        $display("[SMART_JVS] -> Demande révision commande");
                        prepare_cmdrev_response();
                        need_response = 1;
                    end

                    8'h12: begin // CMD_JVSREV - JVS revision
                        $display("[SMART_JVS] -> Demande révision JVS");
                        prepare_jvsrev_response();
                        need_response = 1;
                    end

                    8'h13: begin // CMD_COMVER - Communication version
                        $display("[SMART_JVS] -> Demande version communication");
                        prepare_commver_response();
                        need_response = 1;
                    end

                    8'h14: begin // CMD_FEATCK - Feature check
                        $display("[SMART_JVS] -> Demande features");
                        prepare_feature_response();
                        need_response = 1;
                    end

                    8'h20: begin // CMD_READ - Read inputs
                        $display("[SMART_JVS] -> Lecture des entrées");
                        // Pour les tests, on peut répondre avec des données factices
                        prepare_ack_response();
                        need_response = 1;
                    end

                    default: begin
                        $display("[SMART_JVS] -> Commande inconnue: 0x%02X", cmd);
                        need_response = 0;
                    end
                endcase
            end else begin
                need_response = 0;
            end
        end
    endfunction

    //=========================================================================
    // MACHINE D'ÉTAT DE RÉCEPTION JVS
    //=========================================================================
    always @(posedge clk) begin
        if (rst) begin
            byte_count <= 0;
            completed_frame_length <= 0;
            rx_state <= RX_IDLE;
            frame_in_progress <= 0;
            expected_frame_length <= 0;
            current_frame_length <= 0;
            new_frame_received <= 0;
            device_address <= assigned_address;
            need_response <= 0;

            // Clear buffers
            for (integer i = 0; i < 64; i++) begin
                received_buffer[i] <= 8'h00;
                completed_frame_buffer[i] <= 8'h00;
            end
        end else begin
            new_frame_received <= 0; // Pulse

            // Traitement des bytes UART reçus
            if (uart_rx_dv) begin
                $display("[SMART_JVS] Byte reçu: 0x%02X - État: %s",
                        uart_rx_byte, rx_state.name());
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
                        if (uart_rx_byte == JVS_BROADCAST_ADDR || uart_rx_byte == 8'h01) begin          // Valid master address
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
                    if (copy_read_idx < 3) begin // JVS_STATUS_POS = 3
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
                        end else begin
                            // Regular data byte, copy as-is
                            rx_buffer[copy_write_idx] <= rx_buffer_raw[copy_read_idx];
                            copy_read_idx <= copy_read_idx + 1;
                            copy_write_idx <= copy_write_idx + 1;
                        end
                    end else begin
                        // Copy final checksum
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
            // RX_PROCESS - Analyze completed frame and prepare response
            //-------------------------------------------------------------
            if (rx_state == RX_PROCESS) begin
                // Stocker la trame complète dans completed_frame_buffer
                for (integer i = 0; i < copy_write_idx && i < 64; i++) begin
                    completed_frame_buffer[i] <= rx_buffer[i];
                end
                completed_frame_length <= copy_write_idx;

                // Analyser la commande et préparer la réponse
                if (auto_respond) begin
                    analyze_and_respond();
                end

                // Signal frame completed
                new_frame_received <= 1'b1;
                rx_frame_complete <= 1'b1;     // Signal frame complete to main state machine
                rx_counter <= 8'h00;           // Reset counter for next frame
                rx_state <= RX_IDLE;           // Return to idle for next frame

                $display("[SMART_JVS] *** TRAME COMPLÈTE (%d bytes) ***", copy_write_idx);
            end
        end
    end

    //=========================================================================
    // MACHINE D'ÉTAT DE TRANSMISSION DE RÉPONSE
    //=========================================================================
    always @(posedge clk) begin
        if (rst) begin
            tx_state <= TX_IDLE;
            uart_tx_dv <= 0;
            response_index <= 0;
            response_sent <= 0;
            setup_time_counter <= 0;
        end else begin
            response_sent <= 0; // Pulse

            case (tx_state)
                TX_IDLE: begin
                    uart_tx_dv <= 0;
                    if (need_response) begin
                        response_index <= 0;
                        setup_time_counter <= 0;
                        tx_state <= TX_WAIT_SETUP;
                        $display("[SMART_JVS] *** RÉPONSE IMMÉDIATE (sans setup time) ***");
                    end
                end

                TX_WAIT_SETUP: begin
                    /*
                    if(setup_time_counter < SETUP_TIME_CYCLES) begin
                        setup_time_counter <= setup_time_counter + 1;
                    end else begin
                        response_index <= 0;
                        tx_state <= TX_PREPARE;
                        $display("[SMART_JVS] *** DÉBUT TRANSMISSION RÉPONSE (après %d cycles) ***", setup_time_counter);
                    end
                    */
                    response_index <= 0;
                        tx_state <= TX_PREPARE;
                        $display("[SMART_JVS] *** DÉBUT TRANSMISSION RÉPONSE (après %d cycles) ***", setup_time_counter);
                end

                TX_PREPARE: begin
                    if (response_index < response_length) begin
                        uart_tx_byte <= rx_buffer[response_index];
                        uart_tx_dv <= 1;
                        tx_state <= TX_SEND_BYTE;
                    end else begin
                        tx_state <= TX_COMPLETE;
                    end
                end

                TX_SEND_BYTE: begin
                    uart_tx_dv <= 0;
                    tx_state <= TX_WAIT_DONE;
                end

                TX_WAIT_DONE: begin
                    if (uart_tx_done) begin
                        $display("[SMART_JVS] Byte transmis [%d]: 0x%02X",
                                response_index, rx_buffer[response_index]);
                        response_index <= response_index + 1;
                        tx_state <= TX_PREPARE;
                    end
                end

                TX_COMPLETE: begin
                    need_response <= 0;
                    response_sent <= 1;
                    tx_state <= TX_IDLE;
                    $display("[SMART_JVS] *** RÉPONSE TRANSMISE (%d bytes) ***", response_length);
                end
            endcase
        end
    end

endmodule

`default_nettype wire