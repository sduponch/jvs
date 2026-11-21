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
    // Flat storage (simple queues only: Verilator does not handle queues of queues or
    // structs holding a queue): step k = kind / a / b / delay, its bytes are
    // step_bytes[step_start[k] +: step_len[k]]
    byte         step_kind[$];      // "L", "T", "R"
    int          step_a[$], step_b[$], step_delay[$];
    int          step_start[$], step_len[$];
    byte         step_bytes[$];
    string       expect_map[string];

    function automatic string get_expect(input string key);
        return expect_map.exists(key) ? expect_map[key] : "";
    endfunction

    // split a line into space separated tokens, result in toks (no ref argument: not
    // handled by the Verilator CI for a queue of strings)
    string toks[$];
    function automatic void tokenize(input string line);
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
        string t0;                      // first token (Verilator: no char index on an array element)
        fd = $fopen(REPLAY_FILE, "r");
        if (fd == 0) $fatal(1, "[REPLAY] cannot open %s", REPLAY_FILE);
        while (!$feof(fd)) begin
            string tk, v;
            int    first;
            if ($fgets(line, fd) == 0) break;
            tokenize(line);
            if (toks.size() == 0) continue;
            t0 = toks[0];
            if (t0[0] == "#") continue;
            if (t0 == "L" || t0 == "T" || t0 == "R") begin
                step_kind.push_back(t0[0]);
                step_a.push_back(0); step_b.push_back(0); step_delay.push_back(0);
                step_start.push_back(step_bytes.size());
                first = (t0 == "T") ? 1 : 2;        // data bytes start after "T" / "R delay"
                if (t0 == "L") begin
                    tk = toks[1]; step_a[step_a.size() - 1] = tk.atoi();
                    tk = toks[2]; step_b[step_b.size() - 1] = tk.atoi();
                    first = toks.size();            // no data bytes
                end
                if (t0 == "R") begin
                    tk = toks[1]; step_delay[step_delay.size() - 1] = tk.atoi();
                end
                for (int i = first; i < toks.size(); i++) begin
                    tk = toks[i];
                    step_bytes.push_back(byte'(tk.atohex()));
                end
                step_len.push_back(step_bytes.size() - step_start[step_start.size() - 1]);
            end else if (t0 == "E") begin
                v = "";
                for (int i = 2; i < toks.size(); i++) begin
                    tk = toks[i];
                    v = {v, (i > 2) ? " " : "", tk};
                end
                tk = toks[1];
                expect_map[tk] = v;
            end
        end
        $fclose(fd);
        $display("[REPLAY] %s: %0d steps, %0d expected values", REPLAY_FILE, step_kind.size(), expect_map.num());
    end

    //------------------------------------------------------------------
    // Frames sent by the controller (UART decode + JVS framing)
    //------------------------------------------------------------------
    wire       m_dv;
    wire [7:0] m_byte;
    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (.i_Clock(clk), .i_Rx_Serial(uart_rx_from_master), .o_Rx_DV(m_dv), .o_Rx_Byte(m_byte));

    byte  m_frame[$];          // raw bytes of the frame being received
    byte  m_done_bytes[$];     // complete frames, oldest first, one after the other
    int   m_done_len[$];       // length of each complete frame
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
                        foreach (m_frame[i]) m_done_bytes.push_back(m_frame[i]);
                        m_done_len.push_back(m_frame.size());
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
    function automatic string step_hexs(input int k);
        string s = "";
        for (int i = 0; i < step_len[k]; i++) s = {s, $sformatf("%02X ", step_bytes[step_start[k] + i])};
        return s;
    endfunction

    initial begin
        uart_tx_to_master = 1'b1;
        sense_in7 = 1'b1; link_io3 = 1'b1;
        replay_done = 1'b0;
        tx_frames_checked = 0; tx_mismatches = 0; rx_frames_sent = 0;
        @(negedge rst);
        #1;
        for (int k = 0; k < step_kind.size(); k++) begin
            case (step_kind[k])
                "L": begin
                         sense_in7 = step_a[k][0];
                         link_io3  = step_b[k][0];
                         $display("[REPLAY] %0t lines IN7=%0d IO3=%0d", $time, sense_in7, link_io3);
                     end
                "T": begin
                         byte got[$];
                         int  n;
                         logic same;
                         while (m_done_len.size() == 0) @(posedge clk);
                         n = m_done_len.pop_front();
                         got = {};
                         for (int i = 0; i < n; i++) got.push_back(m_done_bytes.pop_front());
                         tx_frames_checked++;
                         same = (n == step_len[k]);
                         for (int i = 0; i < n && same; i++)
                             if (got[i] != step_bytes[step_start[k] + i]) same = 1'b0;
                         if (!same) begin
                             tx_mismatches++;
                             $display("[REPLAY] %0t TX MISMATCH frame %0d\n           got      %s\n           expected %s",
                                      $time, tx_frames_checked, hexs(got), step_hexs(k));
                         end
                     end
                "R": begin
                         repeat (step_delay[k] * (CLK_FREQ / 1_000_000)) @(posedge clk);
                         for (int i = 0; i < step_len[k]; i++) send_byte(step_bytes[step_start[k] + i]);
                         rx_frames_sent++;
                     end
                default: ;
            endcase
        end
        $display("[REPLAY] %0t script done: %0d TX frames checked (%0d mismatches), %0d answers sent",
                 $time, tx_frames_checked, tx_mismatches, rx_frames_sent);
        replay_done = 1'b1;
    end
endmodule
