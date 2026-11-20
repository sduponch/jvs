//////////////////////////////////////////////////////////////////////
// tb_capture_replay: replay of a real JVS bus capture
//
// The controller is run against replay_jvs_device, which plays back a capture
// made on real hardware (jvs_dump.bin -> scripts/jvs_dump.py --replay). Checks:
//   - every frame sent by the controller is identical to the captured one
//   - decoded identity / revisions / switches / coins / analog channels match
//     the values found in the capture
// Usage: make replay [REPLAY=captures/<capture>.replay]
//////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none

import jvs_node_info_pkg::*;

module tb_capture_replay;
    localparam int    MASTER_CLK_FREQ = 1_000_000;
    localparam int    CLK_PERIOD      = 1000;
    localparam string REPLAY_FILE     = `ifdef REPLAY_FILE `REPLAY_FILE `else "captures/sega_838-13683B.replay" `endif;

    logic clk = 0;
    logic rst = 1;
    always #(CLK_PERIOD/2) clk = ~clk;

    wire uart_tx, uart_rx, rx485_dir;
    wire sense_in7, link_io3;
    wire replay_done;
    int  tx_frames_checked, tx_mismatches, rx_frames_sent;

    jvs_bus_state_t bus_state;
    jvs_node_info_t jvs_nodes;
    wire [15:0] p_sw [4];
    wire [MAX_JVS_NODES*8*16-1:0] node_analog;
    wire [4*16-1:0] coin_status;
    wire [7:0] node_name_rd_data;
    logic [NAME_BRAM_ADDR_BITS-1:0] node_name_rd_addr = '0;

    jvs_ctrl #(.MASTER_CLK_FREQ(MASTER_CLK_FREQ)) dut (
        .i_clk(clk), .i_rst(rst), .i_ena(1'b1), .i_stb(1'b0),
        .i_uart_rx(uart_rx), .o_uart_tx(uart_tx),
        .i_sense(sense_in7), .i_link(1'b1),
        .o_rx485_dir(rx485_dir), .o_bus_state(bus_state),
        .player1_input_switch(p_sw[0]), .player2_input_switch(p_sw[1]),
        .player3_input_switch(p_sw[2]), .player4_input_switch(p_sw[3]),
        .analog_ch1(), .analog_ch2(), .analog_ch3(), .analog_ch4(),
        .analog_ch5(), .analog_ch6(), .analog_ch7(), .analog_ch8(),
        .node_analog(node_analog),
        .has_screen_pos(), .screen_pos_x(), .screen_pos_y(),
        .coin_count(), .coin1(), .coin2(), .coin3(), .coin4(),
        .coin_status(coin_status),
        .output_digital_ch1(16'h0000),          // same as the capture (32 01 00)
        .output_analog('0),
        .jvs_data_ready(), .jvs_nodes(jvs_nodes),
        .node_name_rd_data(node_name_rd_data), .node_name_rd_addr(node_name_rd_addr)
    );

    replay_jvs_device #(.CLK_FREQ(MASTER_CLK_FREQ), .REPLAY_FILE(REPLAY_FILE)) replay (
        .clk(clk), .rst(rst),
        .uart_rx_from_master(uart_tx), .uart_tx_to_master(uart_rx),
        .sense_in7(sense_in7), .link_io3(link_io3),
        .replay_done(replay_done),
        .tx_frames_checked(tx_frames_checked), .tx_mismatches(tx_mismatches),
        .rx_frames_sent(rx_frames_sent)
    );

    //------------------------------------------------------------------
    // Checks
    //------------------------------------------------------------------
    int checks_failed = 0;
    task automatic check(input string what, input logic [31:0] got, input logic [31:0] exp);
        if (got === exp) $display("[CHECK] PASS  %-28s got 0x%0h", what, got);
        else begin
            $display("[CHECK] FAIL  %-28s got 0x%0h expected 0x%0h", what, got, exp);
            checks_failed++;
        end
    endtask

    function automatic logic [31:0] hexval(input string s);
        string t = "";
        for (int i = 0; i < s.len(); i++) if (s[i] != " ") t = {t, string'(s[i])};
        return t.atohex();
    endfunction

    // Expected player switch word from the 2 JVS switch bytes (jvs_ctrl mapping)
    function automatic logic [15:0] exp_player_sw(input logic [15:0] b, input logic p2);
        logic [15:0] w;
        w[15] = b[15]; w[14] = b[14];
        w[0]  = b[13]; w[1]  = b[12]; w[2] = b[11]; w[3] = b[10];
        w[4]  = b[9];  w[5]  = b[8];
        w[6]  = b[7];  w[7]  = b[6];  w[8] = b[5]; w[9] = b[4]; w[10] = b[3]; w[11] = b[2];
        w[12] = p2 ? 1'b0 : b[1];
        w[13] = p2 ? 1'b0 : b[0];
        return w;
    endfunction

    initial begin
        string name_hex;
        logic [7:0] ch;
        repeat (100) @(posedge clk);
        rst = 0;

        fork
            wait (replay_done);
            begin
                #(64'd60_000_000_000);
                $display("[CHECK] FAIL  timeout: replay not finished (%0d TX frames checked, bus %s)",
                         tx_frames_checked, bus_state.name());
                checks_failed++;
            end
        join_any
        disable fork;
        repeat (200) @(posedge clk);            // let the last answer be parsed

        $display("\n================ REPLAY CHECKS (%s) ================", REPLAY_FILE);
        check("TX frames identical to capture", tx_mismatches, 0);
        $display("[CHECK] INFO  %0d TX frames checked, %0d answers replayed", tx_frames_checked, rx_frames_sent);
        check("bus READY", bus_state == JVS_READY, 1);

        check("nodes addressed", jvs_nodes.node_count, hexval(replay.get_expect("NODES")));

        // Per node checks: players, coin slots and analog channels of chained nodes are
        // stored one after the other by jvs_ctrl (global index = sum of previous nodes)
        begin
            automatic int first_player = 0, first_coin = 0;
            for (int n = 0; n < jvs_nodes.node_count && n < MAX_JVS_NODES; n++) begin
                automatic string pfx = $sformatf("N%0d_", n + 1);
                $display("---- node %0d ----", n + 1);
                name_hex = "";
                for (int i = 0; i < NODE_NAME_SIZE; i++) begin
                    node_name_rd_addr = n * NODE_NAME_SIZE + i;
                    repeat (2) @(posedge clk);
                    ch = node_name_rd_data;
                    if (ch == 8'h00) break;
                    name_hex = {name_hex, $sformatf("%02X", ch)};
                end
                if (name_hex.toupper() == replay.get_expect({pfx, "NAME"}).toupper())
                    $display("[CHECK] PASS  %-28s", {pfx, "name"});
                else begin
                    $display("[CHECK] FAIL  %s\n           got      %s\n           expected %s", {pfx, "name"},
                             name_hex, replay.get_expect({pfx, "NAME"}));
                    checks_failed++;
                end
                check({pfx, "CMD revision"}, jvs_nodes.node_cmd_ver[n], hexval(replay.get_expect({pfx, "CMDREV"})));
                check({pfx, "JVS revision"}, jvs_nodes.node_jvs_ver[n], hexval(replay.get_expect({pfx, "JVSREV"})));
                check({pfx, "COM version"},  jvs_nodes.node_com_ver[n], hexval(replay.get_expect({pfx, "COMVER"})));
                for (int p = 0; p < jvs_nodes.node_players[n]; p++)
                    if (first_player + p < 4)
                        check($sformatf("%sP%0d switches (P%0d)", pfx, p + 1, first_player + p + 1),
                              p_sw[first_player + p],
                              exp_player_sw(16'(hexval(replay.get_expect($sformatf("%sSW%0d", pfx, p)))), (first_player + p) != 0));
                for (int c = 0; c < jvs_nodes.node_coin_slots[n]; c++)
                    if (first_coin + c < 4)
                        check($sformatf("%scoin slot %0d", pfx, c + 1), coin_status[(first_coin + c)*16 +: 16],
                              hexval(replay.get_expect($sformatf("%sCOIN%0d", pfx, c))));
                for (int c = 0; c < jvs_nodes.node_analog_channels[n] && c < 8; c++)
                    check($sformatf("%sanalog CH%0d", pfx, c), node_analog[(n*8 + c)*16 +: 16],
                          hexval(replay.get_expect($sformatf("%sANALOG%0d", pfx, c))));
                first_player += jvs_nodes.node_players[n];
                first_coin   += jvs_nodes.node_coin_slots[n];
            end
        end

        $display("=======================================================");
        if (checks_failed == 0) $display("*** ALL CHECKS PASSED ***");
        else                    $display("*** %0d CHECK(S) FAILED ***", checks_failed);
        $finish;
    end
endmodule
