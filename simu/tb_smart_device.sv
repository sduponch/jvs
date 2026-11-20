//////////////////////////////////////////////////////////////////////
// Test 03: Vérification de la séquence d'identification JVS complète
//
// Objectif: Tester la séquence d'initialisation JVS complète avec
// un périphérique intelligent qui peut répondre aux commandes.
//
// Séquence testée (basée sur capture Tekken 6):
// 1. Double Reset (F0 D9) en broadcast (FF)
// 2. Address Assignment (F1 01) en broadcast (FF) -> Réponse ACK
// 3. Device ID Request (10) vers device 01 -> Réponse avec nom device
// 4. Command Revision (11) -> Réponse
// 5. JVS Revision (12) -> Réponse
// 6. Communication Version (13) -> Réponse
// 7. Feature Check (14) -> Réponse avec capabilities
//
// MODIFICATIONS BRAM:
// - node_name stockés en BRAM avec checksum
// - Interface BRAM: node_name_rd_addr et node_name_rd_data
//
//////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
`default_nettype none

import jvs_node_info_pkg::*;

module tb_smart_device;

    // Paramètres de test haute vitesse pour communication bidirectionnelle
    localparam MASTER_CLK_FREQ = 1_000_000;  // 1MHz pour réponses plus rapides
    localparam CLK_PERIOD = 1000;            // 1000ns (1MHz)
    localparam UART_CLKS_PER_BIT = MASTER_CLK_FREQ / 115200; // = 9 cycles/bit

    // Signaux DUT
    logic clk = 0;
    logic rst, ena, stb;
    wire uart_rx, sense;                  // uart_rx maintenant comme wire
    wire uart_tx, rx485_dir;
    logic [7:0] gpio_output_value;
    logic [NAME_BRAM_ADDR_BITS-1:0] node_name_rd_addr;  // Utilise les nouveaux paramètres BRAM

    // Signaux UART bidirectionnels séparés
    wire jvs_to_device;                   // TX du contrôleur vers device
    wire device_to_jvs;                   // TX du device vers contrôleur

    // Sorties DUT (non utilisées dans ce test)
    wire [15:0] p1_btn_state, p2_btn_state, p3_btn_state, p4_btn_state;
    wire [31:0] p1_joy_state, p2_joy_state;
    wire [15:0] screen_pos_x, screen_pos_y;
    wire has_screen_pos;
    wire jvs_data_ready;
    jvs_node_info_t jvs_nodes;
    wire [7:0] node_name_rd_data;

    // Signaux du smart JVS device
    wire [7:0] smart_jvs_bytes[0:63];
    integer smart_jvs_byte_count;
    wire smart_jvs_new_frame;
    wire [7:0] smart_jvs_frame_length;
    wire smart_jvs_uart_active;
    wire [7:0] smart_jvs_current_byte;
    wire smart_jvs_response_sent;

    // Variables de test
    logic test_passed = 0;
    logic test_completed = 0;
    integer commands_received = 0;
    integer responses_sent = 0;
    integer polling_requests = 0; // Compteur de requêtes de polling
    logic [7:0] received_commands[0:15]; // Stockage des commandes reçues

    // Node names are read from the DUT name BRAM (node_name_rd_addr / node_name_rd_data).
    // (A fake BRAM used to drive node_name_rd_data here as well: removed, it conflicted
    // with the DUT output and showed a fake "SEGA ..." name in the log.)

    // Monitoring du signal RS485 direction
    logic rx485_dir_prev = 0;

    // Monitoring des états internes du contrôleur
    logic [4:0] main_state_prev = 5'h0;
    logic [2:0] rx_state_prev = 3'h0;

    // Monitoring des états jvs_com
    logic [3:0] com_tx_state_prev = 4'h0;
    logic [3:0] com_rx_state_prev = 4'h0;

    // Monitoring des états du controller
    logic [4:0] controller_main_state_prev = 5'h0;
    logic com_commit_prev = 1'b0;
    logic com_tx_ready_prev = 1'b1;

    // Timeout et timing
    logic [31:0] timeout_counter = 0;
    localparam TIMEOUT_LIMIT = 32'd50_000_000; // 200s timeout à 250kHz

    // Génération horloge
    always #(CLK_PERIOD/2) clk = ~clk;

    // Connexions UART correctes
    assign jvs_to_device = uart_tx;      // TX du contrôleur
    assign uart_rx = device_to_jvs;      // RX du contrôleur
    assign sense = 1'b1;                 // Signal sense activé

    // Digital output value sent by the controller (OUTPUT1 32): byte 1 = [7:0], byte 2 = [15:8]
    localparam logic [15:0] TEST_DOUT = 16'h0480;
    // Analog output values sent by the controller (ANLOUT 33), channels 0 and 1
    localparam logic [15:0] TEST_AOUT0 = 16'hE000;
    localparam logic [15:0] TEST_AOUT1 = 16'h1234;

    // Simulated board answers (set to 0 to simulate a board that stops answering)
    logic dev_respond = 1'b1;

    // Input polling outputs of the DUT (checked against smart device EXP_* values)
    jvs_bus_state_t bus_state;
    wire [15:0] p1_sw, p2_sw;
    wire [15:0] ana_ch [8];
    wire [MAX_JVS_NODES*8*16-1:0] node_analog;
    wire [4*16-1:0] coin_status;

    // Instance DUT - Utilisation de jvs_ctrl avec nouvelle API BRAM
    jvs_ctrl #(
        .MASTER_CLK_FREQ(MASTER_CLK_FREQ)
    ) dut (
        .i_clk(clk),
        .i_rst(rst),
        .i_ena(ena),
        .i_stb(stb),
        .i_uart_rx(uart_rx),
        .o_uart_tx(uart_tx),
        .i_sense(sense),
        .o_rx485_dir(rx485_dir),
        .i_link(1'b1),
        .o_bus_state(bus_state),
        .player1_input_switch(p1_sw), .player2_input_switch(p2_sw),
        .player3_input_switch(),      .player4_input_switch(),
        .analog_ch1(ana_ch[0]), .analog_ch2(ana_ch[1]), .analog_ch3(ana_ch[2]), .analog_ch4(ana_ch[3]),
        .analog_ch5(ana_ch[4]), .analog_ch6(ana_ch[5]), .analog_ch7(ana_ch[6]), .analog_ch8(ana_ch[7]),
        .node_analog(node_analog),
        .coin_count(), .coin1(), .coin2(), .coin3(), .coin4(),
        .coin_status(coin_status),
        .output_digital_ch1(TEST_DOUT),
        .output_analog({96'h0, TEST_AOUT1, TEST_AOUT0}),
        .has_screen_pos(), .screen_pos_x(), .screen_pos_y(),
        //.gpio_output_value(gpio_output_value),
        .jvs_data_ready(jvs_data_ready),
        .jvs_nodes(jvs_nodes),
        .node_name_rd_data(node_name_rd_data),
        .node_name_rd_addr(node_name_rd_addr)
    );

    //=========================================================================
    // Input polling outputs (checked against the smart device EXP_* values)
    //=========================================================================

    // Expected player switch word from the 2 JVS switch bytes (jvs_ctrl mapping)
    function automatic logic [15:0] exp_player_sw(input logic [15:0] b, input logic p2);
        logic [15:0] w;
        w[15] = b[15]; w[14] = b[14];                          // START, SERVICE
        w[0]  = b[13]; w[1]  = b[12]; w[2] = b[11]; w[3] = b[10]; // UP DOWN LEFT RIGHT
        w[4]  = b[9];  w[5]  = b[8];                           // push1, push2
        w[6]  = b[7];  w[7]  = b[6];  w[8] = b[5]; w[9] = b[4]; w[10] = b[3]; w[11] = b[2];
        w[12] = p2 ? 1'b0 : b[1];                              // P2: [13:12] cleared by jvs_ctrl
        w[13] = p2 ? 1'b0 : b[0];
        return w;
    endfunction

    // Expected node name (smart device IOIDENT answer)
    localparam string EXP_NAME = "No Brand;NAOMI Converter98701;ver2.0; ";

    int checks_failed = 0;
    task automatic check(input string what, input logic [31:0] got, input logic [31:0] exp);
        if (got === exp) $display("[CHECK] PASS  %-28s got 0x%0h", what, got);
        else begin
            $display("[CHECK] FAIL  %-28s got 0x%0h expected 0x%0h", what, got, exp);
            checks_failed++;
        end
    endtask

    task automatic run_checks(input string phase);
        string name_got;
        logic [7:0] ch;
        $display("\n================ JVS CONTROLLER CHECKS (%s) ================", phase);
        // Identification
        check("CMD revision",  jvs_nodes.node_cmd_ver[0], 8'h13);
        check("JVS revision",  jvs_nodes.node_jvs_ver[0], 8'h30);
        check("COM version",   jvs_nodes.node_com_ver[0], 8'h10);
        name_got = "";
        for (int i = 0; i < NODE_NAME_SIZE; i++) begin
            node_name_rd_addr = i;
            repeat (2) @(posedge clk);
            ch = node_name_rd_data;
            if (ch == 8'h00) break;
            name_got = {name_got, (ch >= 8'h20 && ch < 8'h7F) ? string'(ch) : $sformatf("<%02x>", ch)};
        end
        if (name_got == EXP_NAME) $display("[CHECK] PASS  %-28s \"%s\"", "node name", name_got);
        else begin
            $display("[CHECK] FAIL  %-28s \"%s\" expected \"%s\"", "node name", name_got, EXP_NAME);
            checks_failed++;
        end
        // Features
        check("digital outputs (feature)", jvs_nodes.node_digital_outputs[0], smart_jvs.EXP_DOUT_COUNT);
        // Digital output command (32) received by the board
        check("OUTPUT1 frames received > 0", (smart_jvs.dout_rx_frames > 0), 1);
        check("OUTPUT1 byte count", smart_jvs.dout_rx_count, (smart_jvs.EXP_DOUT_COUNT + 7) / 8);
        check("OUTPUT1 data byte 1", smart_jvs.dout_rx_data[0], TEST_DOUT[7:0]);
        check("OUTPUT1 data byte 2", smart_jvs.dout_rx_data[1], TEST_DOUT[15:8]);
        // Analog output command (33) received by the board
        check("analog outputs (feature)", jvs_nodes.node_analog_output_channels[0], smart_jvs.EXP_AOUT_COUNT);
        check("ANLOUT frames received > 0", (smart_jvs.aout_rx_frames > 0), 1);
        check("ANLOUT channel count", smart_jvs.aout_rx_count, smart_jvs.EXP_AOUT_COUNT);
        check("ANLOUT channel 0", smart_jvs.aout_rx_data[0], TEST_AOUT0);
        check("ANLOUT channel 1", smart_jvs.aout_rx_data[1], TEST_AOUT1);
        // Switches
        check("P1 switches", p1_sw, exp_player_sw(smart_jvs.EXP_P1_SW, 1'b0));
        check("P2 switches", p2_sw, exp_player_sw(smart_jvs.EXP_P2_SW, 1'b1));
        // Coins
        for (int c = 0; c < 2; c++)
            check($sformatf("coin slot %0d", c + 1), coin_status[c*16 +: 16], smart_jvs.EXP_COIN[c]);
        // Analog
        for (int c = 0; c < 8; c++)
            check($sformatf("analog CH%0d (node_analog)", c), node_analog[c*16 +: 16], smart_jvs.EXP_ANALOG[c]);
        for (int c = 0; c < 8; c++)
            check($sformatf("analog CH%0d (analog_ch%0d)", c, c + 1), ana_ch[c], smart_jvs.EXP_ANALOG[c]);

    endtask

    // Wait for a few complete polling cycles, check everything, then stop the simulation
    localparam int POLL_CYCLES_BEFORE_CHECK = 3;
    initial begin
        fork
            begin
                wait (bus_state == JVS_READY);
                $display("[CHECK] Bus READY at %0t, waiting %0d polling cycles", $time, POLL_CYCLES_BEFORE_CHECK);
                repeat (POLL_CYCLES_BEFORE_CHECK) @(posedge jvs_data_ready);
                repeat (10) @(posedge clk);
            end
            begin
                #(64'd30_000_000_000); // 30 s simulated (timescale 1ns)
                $display("[CHECK] FAIL  timeout: bus never reached READY (state %s)", bus_state.name());
                checks_failed++;
            end
        join_any
        disable fork;

        run_checks("initial init");
        write_log_dump("sim_jvs_dump.hex");

        // ---- Recovery: the board stops answering while polling, then answers again ----
        $display("[CHECK] Recovery test: board stops answering at %0t", $time);
        dev_respond = 1'b0;
        fork
            begin
                wait (bus_state != JVS_READY);
                $display("[CHECK] Bus left READY (%s) at %0t", bus_state.name(), $time);
                wait (bus_state == JVS_ADDRESSING);
                $display("[CHECK] Re-initialization started at %0t", $time);
            end
            begin
                #(64'd20_000_000_000);
                $display("[CHECK] FAIL  recovery: no re-initialization (state %s)", bus_state.name());
                checks_failed++;
            end
        join_any
        disable fork;
        dev_respond = 1'b1;
        fork
            begin
                wait (bus_state == JVS_READY);
                $display("[CHECK] Bus READY again at %0t", $time);
                repeat (POLL_CYCLES_BEFORE_CHECK) @(posedge jvs_data_ready);
                repeat (10) @(posedge clk);
            end
            begin
                #(64'd40_000_000_000);
                $display("[CHECK] FAIL  recovery: bus never READY again (state %s)", bus_state.name());
                checks_failed++;
            end
        join_any
        disable fork;
        run_checks("after recovery");

        $display("=======================================================");
        if (checks_failed == 0) $display("*** ALL CHECKS PASSED ***");
        else                    $display("*** %0d CHECK(S) FAILED ***", checks_failed);
        $finish;
    end

    //=========================================================================
    // Bus logger (same module as in the core): captures the simulated bus and is
    // written to sim_jvs_dump.bin at the end, in the jvs_dump.bin format
    //=========================================================================
    wire [15:0] log_entries;
    wire [31:0] log_rd_data;
    jvs_bus_logger #(.CLK_FREQ(MASTER_CLK_FREQ), .WORDS(8192)) bus_logger (
        .clk(clk), .enable(1'b1), .restart(1'b0), .hold(1'b0), .ring(1'b0), .sync_start(1'b0),
        .tx_line(uart_tx), .rx_line(device_to_jvs), .dir(rx485_dir),
        .setup_n(sense), .link_n(1'b0),
        .entry_count(log_entries), .full(),
        .rd_clk(clk), .rd_addr(13'd0), .rd_data(log_rd_data)
    );
    task automatic write_log_dump(input string fname);
        int fd;
        fd = $fopen(fname, "w");
        for (int i = 0; i < 8 + 2 * log_entries; i++)
            $fdisplay(fd, "%08h", bus_logger.mem[i]);   // one 32-bit word per line (jvs_dump.py --hex)
        $fclose(fd);
        $display("[CHECK] Bus log: %0d entries written to %s", log_entries, fname);
    endtask

    // Instance Smart JVS Device avec réponses automatiques
    smart_jvs_device #(
        .MASTER_CLK_FREQ(MASTER_CLK_FREQ)
    ) smart_jvs (
        .clk(clk),
        .rst(rst),
        .uart_rx_from_master(jvs_to_device),    // Reçoit TX du contrôleur
        .uart_tx_to_master(device_to_jvs),      // Envoie vers RX du contrôleur
        .last_received_bytes(smart_jvs_bytes),
        .bytes_received_count(smart_jvs_byte_count),
        .new_frame_received(smart_jvs_new_frame),
        .frame_length(smart_jvs_frame_length),
        .assigned_address(8'h01),
        .auto_respond(dev_respond), // Réponses automatiques (coupées pour simuler une perte de liaison)
        .uart_rx_active(smart_jvs_uart_active),
        .current_byte(smart_jvs_current_byte),
        .response_sent(smart_jvs_response_sent)
    );

    // Capture des commandes reçues
    always @(posedge smart_jvs_new_frame) begin
        if (commands_received < 16) begin
            $display("\n[TEST] *** COMMANDE %d REÇUE ***", commands_received + 1);

            // Analyser la commande reçue
            if (smart_jvs_byte_count >= 4) begin
                received_commands[commands_received] = smart_jvs_bytes[3]; // Commande
                $display("[TEST] Commande: 0x%02X, Adresse: 0x%02X, Longueur: %d",
                        smart_jvs_bytes[3], smart_jvs_bytes[1], smart_jvs_bytes[2]);

                // Afficher la trame complète
                $write("[TEST] Trame: ");
                for (integer i = 0; i < smart_jvs_byte_count && i < 8; i++) begin
                    $write("%02X ", smart_jvs_bytes[i]);
                end
                $write("\n");

                // Détecter les commandes répétées pour éviter les boucles
                // EXCEPTION: 0x20 (SWINP) est censée être répétée en boucle - c'est normal
                if (commands_received >= 3 && smart_jvs_bytes[3] != 8'h20) begin
                    // Vérifier si on a la même commande 3 fois de suite (sauf SWINP)
                    if (smart_jvs_bytes[3] == received_commands[commands_received-1] &&
                        smart_jvs_bytes[3] == received_commands[commands_received-2]) begin
                        $display("[TEST] ERREUR: Commande 0x%02X répétée 3 fois - Boucle détectée!", smart_jvs_bytes[3]);
                        $display("[TEST] Test arrêté pour éviter une boucle infinie");
                        analyze_complete_sequence();  // Analyser l'état actuel avant d'arrêter
                        test_passed = 0;
                        test_completed = 1;
                    end
                end

                // Compter les requêtes de polling d'inputs (0x20)
                if (smart_jvs_bytes[3] == 8'h20) begin
                    polling_requests++;
                    $display("[TEST] Requête de polling #%d détectée", polling_requests);

                    // Arrêter après 3 requêtes de polling
                    if (polling_requests >= 3) begin
                        $display("[TEST] 3 requêtes de polling reçues - Test terminé avec succès");
                        analyze_complete_sequence();
                        test_completed = 1;
                    end
                end

                // Arrêter le test si on reçoit un reset après avoir eu des ACKs
                // Cela indique que le contrôleur redémarre sa séquence (fallback)
                if (smart_jvs_bytes[3] == 8'hF0 && responses_sent > 0 && polling_requests == 0) begin
                    $display("[TEST] Reset détecté après %d réponses - Contrôleur redémarre", responses_sent);
                    $display("[TEST] Arrêt du test pour analyser la séquence reçue");
                    analyze_complete_sequence();
                    test_completed = 1;
                end
            end

            commands_received++;
        end
    end

    // Capture des réponses envoyées
    always @(posedge smart_jvs_response_sent) begin
        responses_sent++;
        $display("[TEST] *** RÉPONSE %d ENVOYÉE ***", responses_sent);
        $display("[TEST] Attente pour laisser le contrôleur traiter la réponse...");

        // Afficher l'état après la 6ème réponse (FEATURE CHECK) mais continuer pour polling
        if (responses_sent == 6) begin
            $display("[TEST] Séquence d'identification complète après FEATURE CHECK");
            #1000; // Petit délai pour voir le traitement
            $display("[TEST] STATUS décodé par jvs_com: 0x%02X", dut.com_src_cmd_status);
            $display("[TEST] Commande FIFO: 0x%02X", dut.com_src_cmd);

            // Afficher l'état complet des nodes JVS avec nouvelles structures BRAM
            $display("[TEST] === JVS NODE INFO STATE (COMPLETE) - BRAM VERSION ===");
            for (int dev = 0; dev < MAX_JVS_NODES; dev++) begin
                if (dut.jvs_nodes.node_id[dev] != 8'h00) begin
                    $display("  Device %d:", dev + 1);
                    $display("    Node ID: 0x%02X", dut.jvs_nodes.node_id[dev]);
                    $display("    Node Name Checksum: 0x%04X", dut.jvs_nodes.node_name_checksum[dev]);
                    $display("    Command Revision: 0x%02X", dut.jvs_nodes.node_cmd_ver[dev]);
                    $display("    JVS Revision: 0x%02X", dut.jvs_nodes.node_jvs_ver[dev]);
                    $display("    Communication Version: 0x%02X", dut.jvs_nodes.node_com_ver[dev]);
                    $display("    Players: %d", dut.jvs_nodes.node_players[dev]);
                    $display("    Buttons per player: %d", dut.jvs_nodes.node_buttons[dev]);
                    $display("    Analog channels: %d", dut.jvs_nodes.node_analog_channels[dev]);
                    $display("    Coin slots: %d", dut.jvs_nodes.node_coin_slots[dev]);
                    $display("    Rotary channels: %d", dut.jvs_nodes.node_rotary_channels[dev]);
                    $display("    Analog bits: %d", dut.jvs_nodes.node_analog_bits[dev]);
                    $display("    Has keycode input: %d", dut.jvs_nodes.node_has_keycode_input[dev]);
                    $display("    Has screen position: %d", dut.jvs_nodes.node_has_screen_pos[dev]);
                    $display("    Digital outputs: %d", dut.jvs_nodes.node_digital_outputs[dev]);
                end
            end

            $display("[TEST] Attente des commandes de polling...");
            // Ne pas arrêter - continuer pour permettre le polling
        end
    end

    // Monitoring des accès BRAM
    always @(posedge clk) begin
        if (node_name_rd_addr != 0) begin
            $display("[BRAM] Lecture: addr=0x%03X (%d), data=0x%02X ('%c') (time: %t)",
                    node_name_rd_addr, node_name_rd_addr, node_name_rd_data,
                    (node_name_rd_data >= 32 && node_name_rd_data <= 126) ? node_name_rd_data : ".", $time);
        end
    end

    // Monitoring de l'état du FIFO de commandes
    always @(posedge clk) begin
        if (dut.jvs_com_inst.cmd_fifo_init) begin
            $display("[FIFO] INIT: src_cmd=0x%02X, cmd_read_ptr=%d, cmd_count=%d (time: %t)",
                     dut.jvs_com_inst.src_cmd, dut.jvs_com_inst.cmd_read_ptr,
                     dut.jvs_com_inst.cmd_count, $time);
        end
        if (dut.com_src_cmd_next) begin
            $display("[FIFO] NEXT: src_cmd=0x%02X -> ?, cmd_read_ptr=%d->%d, cmd_count=%d->%d (time: %t)",
                     dut.jvs_com_inst.src_cmd, dut.jvs_com_inst.cmd_read_ptr, dut.jvs_com_inst.cmd_read_ptr + 1,
                     dut.jvs_com_inst.cmd_count, dut.jvs_com_inst.cmd_count - 1, $time);
        end
        if (dut.jvs_com_inst.tx_cmd_push && dut.jvs_com_inst.cmd_count < 16) begin
            $display("[FIFO] PUSH: cmd=0x%02X, cmd_write_ptr=%d->%d, cmd_count=%d->%d (time: %t)",
                     dut.jvs_com_inst.tx_data, dut.jvs_com_inst.cmd_write_ptr, dut.jvs_com_inst.cmd_write_ptr + 1,
                     dut.jvs_com_inst.cmd_count, dut.jvs_com_inst.cmd_count + 1, $time);
        end
    end

    // Délai pour attendre que le contrôleur progresse après les ACKs
    initial begin
        #(64'd30_000_000_000); // 30s timeout plus long
        if (!test_completed) begin
            $display("[TEST] Timeout étendu - Analyse avec les commandes reçues");
            analyze_complete_sequence();
            test_completed = 1;
        end
    end

    //=========================================================================
    // ANALYSE DE LA SÉQUENCE COMPLÈTE D'IDENTIFICATION
    //=========================================================================
    task analyze_complete_sequence();
        logic reset1_ok, reset2_ok, address_ok, devid_ok, cmdrev_ok, jvsrev_ok, commver_ok, featck_ok;

        $display("\n=== ANALYSE DE LA SÉQUENCE D'IDENTIFICATION COMPLÈTE - VERSION BRAM ===");

        $display("Commandes reçues (%d):", commands_received);
        for (integer i = 0; i < commands_received && i < 16; i++) begin
            $display("  [%d] Commande: 0x%02X (%s)", i+1, received_commands[i],
                    get_command_name(received_commands[i]));
        end

        // Vérifier la séquence attendue (avec Communication Version)
        reset1_ok = (commands_received >= 1 && received_commands[0] == 8'hF0);
        reset2_ok = (commands_received >= 2 && received_commands[1] == 8'hF0);
        address_ok = (commands_received >= 3 && received_commands[2] == 8'hF1);
        devid_ok = (commands_received >= 4 && received_commands[3] == 8'h10);
        cmdrev_ok = (commands_received >= 5 && received_commands[4] == 8'h11);
        jvsrev_ok = (commands_received >= 6 && received_commands[5] == 8'h12);
        commver_ok = (commands_received >= 7 && received_commands[6] == 8'h13);
        featck_ok = (commands_received >= 8 && received_commands[7] == 8'h14);

        $display("\nVérification de la séquence:");
        $display("✓ Reset #1 (F0):           %s", reset1_ok ? "CORRECT" : "INCORRECT");
        $display("✓ Reset #2 (F0):           %s", reset2_ok ? "CORRECT" : "INCORRECT");
        $display("✓ Address Assignment (F1): %s", address_ok ? "CORRECT" : "INCORRECT");
        $display("✓ Device ID (10):          %s", devid_ok ? "CORRECT" : "INCORRECT");
        $display("✓ Command Revision (11):   %s", cmdrev_ok ? "CORRECT" : "INCORRECT");
        $display("✓ JVS Revision (12):       %s", jvsrev_ok ? "CORRECT" : "INCORRECT");
        $display("✓ Communication Version (13): %s", commver_ok ? "CORRECT" : "INCORRECT");
        $display("✓ Feature Check (14):      %s", featck_ok ? "CORRECT" : "INCORRECT");

        $display("\nStatistiques:");
        $display("  Commandes reçues: %d", commands_received);
        $display("  Réponses envoyées: %d", responses_sent);
        $display("  Requêtes de polling: %d", polling_requests);

        // Test global - succès si séquence complète ET polling démarré
        test_passed = reset1_ok && reset2_ok && address_ok && devid_ok &&
                      cmdrev_ok && jvsrev_ok && commver_ok && featck_ok &&
                      (responses_sent >= 6) && (polling_requests >= 3); // Séquence complète + polling

        if (test_passed) begin
            $display("\n*** SUCCESS: SÉQUENCE JVS COMPLÈTE + POLLING INPUTS DÉMARRÉ ! ***");
            $display("✓ Double reset correct");
            $display("✓ Assignation d'adresse avec ACK");
            $display("✓ Identification du périphérique");
            $display("✓ Vérification des révisions");
            $display("✓ Lecture des features");
            $display("✓ Communication bidirectionnelle validée");
            $display("✓ Polling des inputs opérationnel (%d requêtes)", polling_requests);
            $display("✓ Interface BRAM fonctionnelle");
        end else begin
            $display("\n*** ERREUR: PROBLÈME DANS LA SÉQUENCE D'IDENTIFICATION ***");
            if (!reset1_ok) $display("✗ Premier reset manquant");
            if (!reset2_ok) $display("✗ Deuxième reset manquant");
            if (!address_ok) $display("✗ Assignation d'adresse manquante");
            if (!devid_ok) $display("✗ Identification device manquante");
            if (!cmdrev_ok) $display("✗ Command revision manquante");
            if (!jvsrev_ok) $display("✗ JVS revision manquante");
            if (!commver_ok) $display("✗ Communication version manquante");
            if (!featck_ok) $display("✗ Feature check manquante");
            if (responses_sent < 6) $display("✗ Pas assez de réponses (%d < 6)", responses_sent);
            if (polling_requests < 3) $display("✗ Polling inputs insuffisant (%d < 3)", polling_requests);
        end

        // Affichage de l'état final de jvs_node_info_pkg avec BRAM
        $display("\n=== ÉTAT FINAL DES INFORMATIONS JVS (jvs_nodes) - VERSION BRAM ===");
        $display("Paramètres BRAM:");
        $display("  MAX_JVS_NODES: %d", MAX_JVS_NODES);
        $display("  NODE_NAME_SIZE: %d", NODE_NAME_SIZE);
        $display("  NAME_BRAM_SIZE: %d (= %d × %d)", NAME_BRAM_SIZE, MAX_JVS_NODES, NODE_NAME_SIZE);
        $display("  NAME_BRAM_ADDR_BITS: %d", NAME_BRAM_ADDR_BITS);
        $display("  CHECKSUM_BITS: 16 (hardcoded)");

        for (integer dev = 0; dev < MAX_JVS_NODES; dev++) begin
            $display("\n--- Device %d (adresse 0x%02X) ---", dev + 1, dev + 1);

            // Checksum du nom (nouvelle feature BRAM)
            $display("  Node Name Checksum: 0x%04X", dut.jvs_nodes.node_name_checksum[dev]);

            // Affichage du nom complet du node depuis la BRAM
            if (dut.jvs_nodes.node_id[dev] != 8'h00) begin
                automatic string node_name_str;
                automatic integer base_addr;
                automatic logic [7:0] char_data;
                automatic integer i;

                node_name_str = "";
                base_addr = dev * NODE_NAME_SIZE;

                for (i = 0; i < NODE_NAME_SIZE; i++) begin
                    char_data = dut.node_name_ram[base_addr + i]; // real DUT name BRAM
                    if (char_data == 8'h00) break; // Fin de chaîne
                    if (char_data >= 32 && char_data <= 126) begin // Caractères imprimables
                        node_name_str = {node_name_str, string'(char_data)};
                    end
                end

                if (node_name_str.len() > 0) begin
                    $display("  Node Name: \"%s\"", node_name_str);
                end else begin
                    $display("  Node Name: <vide ou non lisible>");
                end
            end

            // Versions et révisions parsées
            $display("  Command Revision: 0x%02X", dut.jvs_nodes.node_cmd_ver[dev]);
            $display("  JVS Revision: 0x%02X", dut.jvs_nodes.node_jvs_ver[dev]);
            $display("  Communication Version: 0x%02X", dut.jvs_nodes.node_com_ver[dev]);

            // Capacités du device
            $display("  Joueurs: %d", dut.jvs_nodes.node_players[dev]);
            $display("  Boutons par joueur: %d", dut.jvs_nodes.node_buttons[dev]);
            $display("  Canaux analogiques: %d", dut.jvs_nodes.node_analog_channels[dev]);
            $display("  Bits analogiques: %d", dut.jvs_nodes.node_analog_bits[dev]);
            $display("  Canaux rotatifs: %d", dut.jvs_nodes.node_rotary_channels[dev]);
            $display("  Slots de monnaie: %d", dut.jvs_nodes.node_coin_slots[dev]);
            $display("  Sorties numériques: %d", dut.jvs_nodes.node_digital_outputs[dev]);
            $display("  Canaux sortie analogique: %d", dut.jvs_nodes.node_analog_output_channels[dev]);
            $display("  Entrée keycode: %s", dut.jvs_nodes.node_has_keycode_input[dev] ? "Oui" : "Non");
            $display("  Position écran: %s", dut.jvs_nodes.node_has_screen_pos[dev] ? "Oui" : "Non");
            if (dut.jvs_nodes.node_has_screen_pos[dev]) begin
                $display("    Résolution X: %d bits", dut.jvs_nodes.node_screen_pos_x_bits[dev]);
                $display("    Résolution Y: %d bits", dut.jvs_nodes.node_screen_pos_y_bits[dev]);
            end
            $display("  Entrées misc digital: 0x%04X", dut.jvs_nodes.node_misc_digital_inputs[dev]);
            $display("  Affichage caractères: %s", dut.jvs_nodes.node_has_char_display[dev] ? "Oui" : "Non");
            if (dut.jvs_nodes.node_has_char_display[dev]) begin
                $display("    Largeur: %d", dut.jvs_nodes.node_char_display_width[dev]);
                $display("    Hauteur: %d", dut.jvs_nodes.node_char_display_height[dev]);
                $display("    Type: 0x%02X", dut.jvs_nodes.node_char_display_type[dev]);
            end
            $display("  Support backup: %s", dut.jvs_nodes.node_has_backup[dev] ? "Oui" : "Non");
        end
    endtask

    function automatic string get_command_name(logic [7:0] cmd);
        case (cmd)
            8'hF0: return "RESET";
            8'hF1: return "SET_ADDR";
            8'h10: return "IO_IDENT";
            8'h11: return "CMD_REV";
            8'h12: return "JVS_REV";
            8'h13: return "COM_VER";
            8'h14: return "FEAT_CHK";
            8'h20: return "read_INPUTS";
            default: return "UNKNOWN";
        endcase
    endfunction

    function automatic string get_main_state_name(logic [4:0] state);
        case (state)
            0: return "INIT_DELAY";
            1: return "FIRST_RESET";
            2: return "FIRST_RESET_DELAY";
            3: return "SECOND_RESET";
            4: return "SECOND_RESET_DELAY";
            5: return "SEND_SETADDR";
            6: return "SEND_READID";
            7: return "SEND_CMDREV";
            8: return "SEND_JVSREV";
            9: return "SEND_COMMVER";
            10: return "SEND_FEATCHK";
            11: return "IDLE";
            12: return "SEND_INPUTS";
            13: return "WAIT_TX_SETUP";
            14: return "TRANSMIT_BYTE";
            15: return "WAIT_TX_DONE";
            16: return "WAIT_TX_HOLD";
            17: return "WAIT_RX";
            18: return "SEND_INPUTS_SWITCH";
            19: return "SEND_INPUTS_COIN";
            20: return "SEND_INPUTS_ANALOG";
            21: return "SEND_INPUTS_ROTARY";
            22: return "SEND_INPUTS_KEYCODE";
            23: return "SEND_INPUTS_SCREEN";
            24: return "SEND_INPUTS_MISC";
            25: return "SEND_OUTPUT_DIGITAL";
            26: return "SEND_FINALIZE";
            default: return "UNKNOWN";
        endcase
    endfunction

    function automatic string get_rx_state_name(logic [2:0] state);
        case (state)
            0: return "RX_IDLE";
            1: return "RX_READ_ADDR";
            2: return "RX_READ_SIZE";
            3: return "RX_READ_DATA";
            4: return "RX_UNESCAPE";
            5: return "RX_PROCESS";
            6: return "RX_COPY_NAME";
            default: return "UNKNOWN";
        endcase
    endfunction

    // Monitoring du signal RS485 direction et des états
    always @(posedge clk) begin
        if (rst) begin
            rx485_dir_prev <= 0;
            main_state_prev <= 5'h0;
            rx_state_prev <= 3'h0;
        end else begin
            // Monitor RS485 direction changes
            if (rx485_dir != rx485_dir_prev) begin
                rx485_dir_prev <= rx485_dir;
                $display("[RS485] Direction change: %s (time: %0t)",
                        rx485_dir ? "TX MODE" : "RX MODE", $time);
            end

            // Monitor JVS_COM TX state changes
            if (dut.com_tx_state_debug != com_tx_state_prev) begin
                com_tx_state_prev <= dut.com_tx_state_debug;
                $display("[JVS_COM] TX State: %0d -> %0d (%s) (time: %0t)",
                        com_tx_state_prev, dut.com_tx_state_debug,
                        get_com_tx_state_name(dut.com_tx_state_debug), $time);
            end

            // Monitor JVS_COM RX state changes
            if (dut.com_rx_state_debug != com_rx_state_prev) begin
                com_rx_state_prev <= dut.com_rx_state_debug;
                $display("[JVS_COM] RX State: %0d -> %0d (%s) (time: %0t)",
                        com_rx_state_prev, dut.com_rx_state_debug,
                        get_com_rx_state_name(dut.com_rx_state_debug), $time);
            end

            // Monitor controller main state changes
            if (dut.main_state != controller_main_state_prev) begin
                controller_main_state_prev <= dut.main_state;
//                $display("[CONTROLLER] State change: %0d -> %0d (%s) (time: %0t)",
//                        controller_main_state_prev, dut.main_state,
//                        get_controller_state_name(dut.main_state), $time);
            end

            // Monitor com_commit signal changes
            if (dut.com_commit != com_commit_prev) begin
                com_commit_prev <= dut.com_commit;
                $display("[CONTROLLER] com_commit: %0d -> %0d (time: %0t)",
                        !dut.com_commit, dut.com_commit, $time);
            end

            // Monitor tx_ready signal changes
            if (dut.jvs_com_inst.rs485_tx_enable != com_tx_ready_prev) begin
                com_tx_ready_prev <= dut.jvs_com_inst.rs485_tx_enable;
                $display("[JVS_COM] tx_ready: %0d -> %0d (time: %0t)",
                        !dut.jvs_com_inst.rs485_tx_enable, dut.jvs_com_inst.rs485_tx_enable, $time);
            end

            // Monitor UART TX signals from jvs_com
            if (dut.jvs_com_inst.uart_tx_dv) begin
                $display("[JVS_COM] UART TX DV: byte=0x%02X (time: %0t)",
                        dut.jvs_com_inst.uart_tx_byte, $time);
            end

            // Monitor jvs_com RX complete signal (pulse detection)
            // These signals are checked every clock cycle to catch pulses
        end
    end

    // Separate monitoring for pulse signals that need to be caught every cycle
    always @(posedge clk) begin
        // Monitor RX complete pulse
        if (dut.com_rx_complete) begin
            $display("[JVS_COM] *** RX Complete! *** src_node=0x%02X, src_cmd=0x%02X, rx_byte=0x%02X, remaining=%0d (time: %0t)",
                    dut.com_src_node, dut.com_src_cmd, dut.com_rx_byte, dut.com_rx_remaining, $time);
        end

        // Monitor RX error pulse
        if (dut.com_rx_error) begin
            $display("[JVS_COM] *** RX Error! *** Checksum failed (time: %0t)", $time);
        end

        // Monitor RX_VALIDATE state entry for debugging
        if (dut.com_rx_state_debug == 4'h6) begin // RX_VALIDATE
            $display("[JVS_COM] RX_VALIDATE: calc_checksum=0x%02X, recv_checksum=0x%02X (time: %0t)",
                    dut.jvs_com_inst.rx_checksum_calc, dut.jvs_com_inst.rx_checksum_recv, $time);
        end
    end

    // Timeout counter
    always @(posedge clk) begin
        if (rst) begin
            timeout_counter <= 0;
        end else if (!test_completed) begin
            timeout_counter <= timeout_counter + 1;
            if (timeout_counter >= TIMEOUT_LIMIT) begin
                $display("[TEST] TIMEOUT: Séquence incomplète après timeout");
                $display("[TEST] Commandes reçues: %d, Réponses: %d", commands_received, responses_sent);
                test_completed = 1;
            end
        end
    end

    //=========================================================================
    // SÉQUENCE DE TEST
    //=========================================================================
    initial begin
        $display("\n" + {"="*70});
        $display("TEST 03: Séquence d'identification JVS complète avec Smart Device");
        $display("Vérification: Communication bidirectionnelle complète");
        $display("Fréquence: %0d Hz, UART: %0d cycles/bit", MASTER_CLK_FREQ, UART_CLKS_PER_BIT);
        $display("Version BRAM: node_name stockés avec checksum");
        $display("="*70);

        // Initialisation
        rst = 1;
        ena = 0;
        stb = 0;
        gpio_output_value = 8'h00;
        node_name_rd_addr = {NAME_BRAM_ADDR_BITS{1'b0}};

        repeat(100) @(posedge clk);
        $display("[TEST] Phase 1: Reset terminé");

        rst = 0;
        repeat(50) @(posedge clk);
        $display("[TEST] Phase 2: Module activé");
        ena = 1;

        $display("[TEST] Phase 3: Attente de la séquence d'identification complète...");
        $display("[TEST] Le Smart Device va répondre automatiquement aux commandes");

        // Attendre la séquence complète
        wait(test_completed);

        // Résultat final
        $display("\n" + {"="*70});
        if (test_passed) begin
            $display("*** TEST RÉUSSI: Séquence d'identification JVS complète validée ***");
        end else begin
            $display("*** TEST ÉCHOUÉ: Problème dans la séquence d'identification ***");
        end
        $display("Commandes totales reçues: %d", commands_received);
        $display("Réponses totales envoyées: %d", responses_sent);
        $display("="*70);

        //$finish;
    end

    // Génération VCD
    initial begin
        if ($test$plusargs("vcd")) begin   // vsim ... +vcd to enable (very large file)
            $dumpfile("tb_smart_device.vcd");
            $dumpvars(0, tb_smart_device);
        end
    end

    // Fonctions utilitaires pour l'affichage des états JVS_COM
    function string get_com_tx_state_name(input [3:0] state);
        case (state)
            4'h0: return "TX_IDLE";
            4'h1: return "TX_SETUP";
            4'h2: return "TX_SYNC";
            4'h3: return "TX_NODE";
            4'h4: return "TX_LENGTH";
            4'h5: return "TX_DATA";
            4'h6: return "TX_CHECKSUM";
            4'h7: return "TX_TRANSMIT_BYTE";
            4'h8: return "TX_TRANSMIT_BYTE_DONE";
            4'h9: return "TX_DONE";
            default: return "TX_UNKNOWN";
        endcase
    endfunction

    function string get_com_rx_state_name(input [3:0] state);
        case (state)
            4'h0: return "RX_IDLE";
            4'h1: return "RX_SYNC";
            4'h2: return "RX_NODE";
            4'h3: return "RX_LENGTH";
            4'h4: return "RX_DATA";
            4'h5: return "RX_CHECKSUM";
            4'h6: return "RX_VALIDATE";
            4'h7: return "RX_ESCAPE_WAIT";
            default: return "RX_UNKNOWN";
        endcase
    endfunction

    function string get_controller_state_name(input [4:0] state);
        case (state)
            5'h00: return "IDLE";
            5'h01: return "WAIT_RX";
            5'h02: return "INIT_DELAY";
            5'h03: return "FIRST_RESET";
            5'h04: return "FIRST_RESET_DELAY";
            5'h05: return "SECOND_RESET";
            5'h06: return "SECOND_RESET_DELAY";
            5'h07: return "SEND_SETADDR";
            5'h08: return "PARSE_SETADDR";
            5'h09: return "SEND_IOIDENT";
            5'h0A: return "PARSE_IOIDENT";
            5'h0B: return "SEND_CMDREV";
            5'h0C: return "PARSE_CMDREV";
            5'h0D: return "SEND_JVSREV";
            5'h0E: return "PARSE_COMMVER";
            5'h0F: return "PARSE_JVSREV";
            5'h10: return "SEND_COMMVER";
            5'h11: return "SEND_FEATCHK";
            5'h12: return "SEND_INPUTS";
            5'h13: return "SEND_INPUTS_SWITCH";
            5'h14: return "SEND_INPUTS_COIN";
            5'h15: return "SEND_INPUTS_ANALOG";
            5'h16: return "SEND_INPUTS_ROTARY";
            5'h17: return "SEND_INPUTS_KEYCODE";
            5'h18: return "SEND_INPUTS_SCREEN";
            5'h19: return "SEND_INPUTS_MISC";
            5'h1A: return "SEND_OUTPUT_DIGITAL";
            5'h1B: return "SEND_FINALIZE";
            5'h1C: return "FIRST_RESET_ARG";
            5'h1D: return "TX_NEXT";
            5'h1E: return "RX_NEXT";
            default: return "UNKNOWN";
        endcase
    endfunction

endmodule

`default_nettype wire
