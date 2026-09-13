// Directed, self-checking unit test for event_rate_window.
// Run with scripts/run_unit_tb.sh (iverilog + vvp).
module tb_event_rate_window;

  localparam CLK_PERIOD = 10;

  logic clk = 0;
  logic rst_n;
  logic [31:0] s_axis_tdata;
  logic        s_axis_tvalid, s_axis_tready, s_axis_tlast;
  logic        close_allowed = 1;

  logic       win_close, win_is_reset, win_last, win_stale;
  logic [7:0] win_id;
  logic [6:0] win_cnt_left, win_cnt_right;
  logic [1:0] win_ev_left, win_ev_right;

  int errors = 0;
  int checks = 0;

  event_rate_window dut (
      .clk,
      .rst_n,
      .s_axis_tdata,
      .s_axis_tvalid,
      .s_axis_tready,
      .s_axis_tlast,
      .close_allowed,
      .win_close,
      .win_is_reset,
      .win_last,
      .win_stale,
      .win_id,
      .win_cnt_left,
      .win_cnt_right,
      .win_ev_left,
      .win_ev_right
  );

  always #(CLK_PERIOD / 2) clk = ~clk;

  // ------------------------------------------------------------ close monitor
  localparam MAX_CLOSES = 256;
  logic [7:0] c_id[MAX_CLOSES];
  logic [6:0] c_cl[MAX_CLOSES], c_cr[MAX_CLOSES];
  logic [1:0] c_el[MAX_CLOSES], c_er[MAX_CLOSES];
  logic       c_last[MAX_CLOSES], c_reset[MAX_CLOSES], c_stale[MAX_CLOSES];
  int n_closes = 0;
  int n_checked = 0;

  always @(posedge clk) begin
    if (rst_n && win_close) begin
      c_id[n_closes]    = win_id;
      c_cl[n_closes]    = win_cnt_left;
      c_cr[n_closes]    = win_cnt_right;
      c_el[n_closes]    = win_ev_left;
      c_er[n_closes]    = win_ev_right;
      c_last[n_closes]  = win_last;
      c_reset[n_closes] = win_is_reset;
      c_stale[n_closes] = win_stale;
      n_closes++;
    end
  end

  // ------------------------------------------------------------ word builders
  function automatic logic [31:0] ev_word(input int x, input int y, input int pol, input int ts);
    ev_word = {1'b1, pol[0], y[6:0], x[6:0], ts[15:0]};
  endfunction

  function automatic logic [31:0] sync_word(input int ts);
    sync_word = {2'b00, 14'd0, ts[15:0]};
  endfunction

  function automatic logic [31:0] reset_word(input int ts);
    reset_word = {2'b01, 14'd0, ts[15:0]};
  endfunction

  function automatic int ts_of(input int wid, input int offset);
    ts_of = ((wid & 8'hFF) << 8) | (offset & 8'hFF);
  endfunction

  // ------------------------------------------------------------ AXIS master
  task automatic send(input logic [31:0] d, input logic last);
    @(negedge clk);
    s_axis_tdata  = d;
    s_axis_tlast  = last;
    s_axis_tvalid = 1'b1;
    @(posedge clk);
    while (!s_axis_tready) @(posedge clk);
    @(negedge clk);
    s_axis_tvalid = 1'b0;
    s_axis_tlast  = 1'b0;
  endtask

  // n events at pixel x in window wid; TLAST on the final one if requested.
  task automatic send_events(input int n, input int x, input int wid, input logic last_on_final);
    for (int i = 0; i < n; i++) send(ev_word(x, 40 + (i % 20), i % 2, ts_of(wid, i)), last_on_final && (i == n - 1));
  endtask

  // ------------------------------------------------------------ checkers
  task automatic expect_close(input int id, input int cl, input int cr, input int el, input int er,
                              input int last, input int is_reset, input int stale, input string label);
    int t = 0;
    while (n_closes <= n_checked && t < 2000) begin
      @(posedge clk);
      t++;
    end
    checks++;
    if (n_closes <= n_checked) begin
      errors++;
      $display("[FAIL] %0t %s: no window close observed", $time, label);
    end else if (c_id[n_checked] !== id[7:0] || c_cl[n_checked] !== cl[6:0] || c_cr[n_checked] !== cr[6:0] ||
                 c_el[n_checked] !== el[1:0] || c_er[n_checked] !== er[1:0] || c_last[n_checked] !== last[0] ||
                 c_reset[n_checked] !== is_reset[0] || c_stale[n_checked] !== stale[0]) begin
      errors++;
      $display("[FAIL] %0t %s: expected id=%0d cl=%0d cr=%0d el=%0d er=%0d last=%0d reset=%0d stale=%0d",
               $time, label, id, cl, cr, el, er, last, is_reset, stale);
      $display("            got id=%0d cl=%0d cr=%0d el=%0d er=%0d last=%0d reset=%0d stale=%0d",
               c_id[n_checked], c_cl[n_checked], c_cr[n_checked], c_el[n_checked], c_er[n_checked],
               c_last[n_checked], c_reset[n_checked], c_stale[n_checked]);
      n_checked++;
    end else begin
      $display("[PASS] %0t %s: id=%0d cl=%0d cr=%0d el=%0d er=%0d last=%0d", $time, label, id, cl, cr, el, er, last);
      n_checked++;
    end
  endtask

  task automatic expect_no_close(input string label);
    repeat (5) @(posedge clk);
    checks++;
    if (n_closes != n_checked) begin
      errors++;
      $display("[FAIL] %0t %s: unexpected window close (id=%0d)", $time, label, c_id[n_checked]);
      n_checked = n_closes;
    end else begin
      $display("[PASS] %0t %s: no close", $time, label);
    end
  endtask

  task automatic expect_tready(input logic expected, input string label);
    checks++;
    if (s_axis_tready !== expected) begin
      errors++;
      $display("[FAIL] %0t %s: expected s_axis_tready=%0d got %0d", $time, label, expected, s_axis_tready);
    end else begin
      $display("[PASS] %0t %s: s_axis_tready=%0d", $time, label, s_axis_tready);
    end
  endtask

  // ------------------------------------------------------------ stimulus
  logic bp_done;

  initial begin
    $dumpfile("build/tb_event_rate_window.vcd");
    $dumpvars(0, tb_event_rate_window);

    rst_n         = 0;
    s_axis_tdata  = '0;
    s_axis_tvalid = 0;
    s_axis_tlast  = 0;
    repeat (2) @(posedge clk);
    @(negedge clk);
    rst_n = 1;
    repeat (2) @(posedge clk);

    // T1: a lone sync word with TLAST closes an empty window.
    send(sync_word(ts_of(0, 0)), 1);
    expect_close(0, 0, 0, 0, 0, 1, 0, 0, "T1 sync+last -> empty window 0");

    // T2: hemisphere split and TLAST close.
    send_events(30, 10, 1, 0);
    send_events(3, 100, 1, 1);
    expect_close(1, 30, 3, 1, 0, 1, 0, 0, "T2 30 left + 3 right, TLAST");

    // T3: quantizer thresholds, windows closed by timestamp boundaries.
    send_events(24, 20, 5, 0);
    send_events(25, 20, 6, 0);
    expect_close(5, 24, 0, 0, 0, 0, 0, 0, "T3 24 events -> bucket 0 (boundary close)");
    send_events(55, 20, 7, 0);
    expect_close(6, 25, 0, 1, 0, 0, 0, 0, "T3 25 events -> bucket 1");
    send_events(85, 20, 8, 0);
    expect_close(7, 55, 0, 2, 0, 0, 0, 0, "T3 55 events -> bucket 2");
    send(sync_word(ts_of(9, 0)), 1);
    expect_close(8, 85, 0, 3, 0, 0, 0, 0, "T3 85 events -> bucket 3");
    expect_close(9, 0, 0, 0, 0, 1, 0, 0, "T3 trailing sync+last -> empty window 9");

    // T4: saturating counter.
    send_events(200, 3, 10, 0);
    send(sync_word(ts_of(10, 255)), 1);
    expect_close(10, 127, 0, 3, 0, 1, 0, 0, "T4 200 events saturate at 127");

    // T5: skipped windows are closed as empty (catch-up), FSM ticks once per window.
    send_events(5, 30, 20, 0);
    send_events(7, 90, 20, 0);
    send(sync_word(ts_of(23, 0)), 0);
    expect_close(20, 5, 7, 0, 0, 0, 0, 0, "T5 window 20 closes on jump to 23");
    expect_close(21, 0, 0, 0, 0, 0, 0, 0, "T5 catch-up: empty window 21");
    expect_close(22, 0, 0, 0, 0, 0, 0, 0, "T5 catch-up: empty window 22");
    expect_no_close("T5 window 23 stays open after catch-up");
    send(sync_word(ts_of(23, 10)), 1);
    expect_close(23, 0, 0, 0, 0, 1, 0, 0, "T5 window 23 closes on TLAST");

    // T6: stale (behind the open window) events are dropped and flagged.
    send_events(3, 10, 40, 0);
    send(ev_word(10, 10, 1, ts_of(39, 200)), 0);
    expect_no_close("T6 stale event does not close the window");
    send_events(2, 100, 40, 1);
    expect_close(40, 3, 2, 0, 0, 1, 0, 1, "T6 stale event dropped, flagged");

    // T7: in-band reset mid-window.
    send_events(10, 10, 50, 0);
    send(reset_word(ts_of(50, 100)), 0);
    expect_close(50, 0, 0, 0, 0, 0, 1, 0, "T7 reset word closes with zero counts");
    send_events(4, 100, 50, 1);
    expect_close(50, 0, 4, 0, 0, 1, 0, 0, "T7 window re-opens fresh after reset");

    // T8: TLAST word held while close_allowed is low; other words keep flowing.
    close_allowed = 0;
    send_events(6, 10, 60, 0);
    send_events(2, 100, 60, 1);
    repeat (3) @(posedge clk);
    expect_tready(0, "T8 closing word parked -> tready low");
    expect_no_close("T8 no close while close_allowed=0");
    @(negedge clk);
    close_allowed = 1;
    expect_close(60, 6, 2, 0, 0, 1, 0, 0, "T8 close proceeds once allowed");

    // T9: window id wrap 255 -> 0 is a normal boundary, not stale.
    send_events(4, 10, 255, 0);
    send_events(1, 100, 0, 0);
    expect_close(255, 4, 0, 0, 0, 0, 0, 0, "T9 wrap: window 255 closes");
    send(sync_word(ts_of(0, 50)), 1);
    expect_close(0, 0, 1, 0, 0, 1, 0, 0, "T9 wrap: window 0 counts the crossing event");

    // T10: reset with TLAST and nothing open.
    send(reset_word(ts_of(77, 0)), 1);
    expect_close(77, 0, 0, 0, 0, 1, 1, 0, "T10 reset+last with no open window");

    // T11: boundary word held while close_allowed is low.
    send_events(3, 10, 90, 0);
    close_allowed = 0;
    bp_done = 0;
    fork
      begin
        send_events(2, 10, 91, 0);
        bp_done = 1;
      end
    join_none
    repeat (6) @(posedge clk);
    expect_tready(0, "T11 boundary word parked -> tready low");
    expect_no_close("T11 no close while close_allowed=0");
    @(negedge clk);
    close_allowed = 1;
    expect_close(90, 3, 0, 0, 0, 0, 0, 0, "T11 boundary close proceeds once allowed");
    wait (bp_done);
    send(sync_word(ts_of(91, 200)), 1);
    expect_close(91, 2, 0, 0, 0, 1, 0, 0, "T11 parked word was counted into window 91");

    repeat (5) @(posedge clk);
    checks++;
    if (n_closes != n_checked) begin
      errors++;
      $display("[FAIL] %0d unexpected extra window closes", n_closes - n_checked);
    end else $display("[PASS] no spurious window closes");

    $display("----------------------------------------");
    if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
    else $display("%0d/%0d CHECKS FAILED", errors, checks);
    $finish;
  end
endmodule
