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

module test_03_device_identification;

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

    // BRAM simulée pour les noms des devices
    logic [7:0] name_bram [0:NAME_BRAM_SIZE-1];

    // Initialisation de la BRAM avec des noms de test
    initial begin
        integer i;
        string node0_name;
        string node1_name;

        // Effacer la BRAM
        for (i = 0; i < NAME_BRAM_SIZE; i++) begin
            name_bram[i] = 8'h00;
        end

        // Node 0: "SEGA ENTERPRISES,LTD.;I/O BD JVS;837-13551  ;Ver1.00;98/10"
        node0_name = "SEGA ENTERPRISES,LTD.;I/O BD JVS;837-13551  ;Ver1.00;98/10";
        for (i = 0; i < node0_name.len() && i < NODE_NAME_SIZE; i++) begin
            name_bram[0 * NODE_NAME_SIZE + i] = node0_name[i];
        end

        // Node 1: "NAMCO LTD.;TEKKEN 6;Ver1.00;JPN"
        node1_name = "NAMCO LTD.;TEKKEN 6;Ver1.00;JPN";
        for (i = 0; i < node1_name.len() && i < NODE_NAME_SIZE; i++) begin
            name_bram[1 * NODE_NAME_SIZE + i] = node1_name[i];
        end
    end

    // Interface BRAM - simulation de lecture
    assign node_name_rd_data = name_bram[node_name_rd_addr];

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
        //.gpio_output_value(gpio_output_value),
        .jvs_data_ready(jvs_data_ready),
        .jvs_nodes(jvs_nodes),
        .node_name_rd_data(node_name_rd_data),
        .node_name_rd_addr(node_name_rd_addr)
    );

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
        .auto_respond(1'b1), // Active les réponses automatiques
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
                    char_data = name_bram[base_addr + i];
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
        $dumpfile("test_03_device_identification.vcd");
        $dumpvars(0, test_03_device_identification);
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
