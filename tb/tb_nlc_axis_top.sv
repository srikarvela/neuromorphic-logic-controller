// Self-checking testbench for nlc_axis_top: the AXI-Stream event pipeline
// end to end, checked against an independent fsm_controller instance
// stepped once per expected window.
//
// Scenarios:
//   A  closed-loop: the same 33-step directed vector sequence as
//      tb/tb_fsm_controller.sv, one DMA-style packet per control step,
//      buckets encoded as event counts (0/30/60/100 events per hemisphere)
//   B  replay: 40 consecutive windows in ONE packet (TLAST only at the end),
//      random buckets, random m_axis backpressure; one decision per window
//   C  in-band reset returns the FSM to FORWARD with a cleared timer
//   D  output stalled (m_axis_tready=0) mid-packet: input backpressures,
//      nothing is lost or duplicated
// Run with scripts/run_unit_tb.sh (iverilog + vvp).
module tb_nlc_axis_top;

  localparam CLK_PERIOD = 10;

  logic clk = 0;
  logic aresetn;
  logic [31:0] s_axis_tdata;
  logic        s_axis_tvalid, s_axis_tready, s_axis_tlast;
  logic [31:0] m_axis_tdata;
  logic        m_axis_tvalid, m_axis_tready, m_axis_tlast;

  int errors = 0;
  int checks = 0;

  nlc_axis_top dut (
      .aclk(clk),
      .aresetn,
      .s_axis_tdata,
      .s_axis_tvalid,
      .s_axis_tready,
      .s_axis_tlast,
      .m_axis_tdata,
      .m_axis_tvalid,
      .m_axis_tready,
      .m_axis_tlast
  );

  // Reference FSM, stepped by the testbench once per expected window.
  logic       ref_rst_n, ref_step;
  logic [1:0] ref_el, ref_er, ref_state;
  logic [2:0] ref_timer;
  fsm_controller ref_fsm (
      .clk,
      .rst_n(ref_rst_n),
      .step_en(ref_step),
      .ev_left(ref_el),
      .ev_right(ref_er),
      .state(ref_state),
      .cmd_forward(),
      .cmd_turn_left(),
      .cmd_turn_right(),
      .cmd_brake(),
      .dbg_brake_timer(ref_timer)
  );

  always #(CLK_PERIOD / 2) clk = ~clk;

  // ------------------------------------------------------------ result monitor
  int bp_mode = 0;  // 0: always ready, 1: random, 2: never
  always @(negedge clk) begin
    case (bp_mode)
      0: m_axis_tready = 1'b1;
      1: m_axis_tready = ($urandom % 3) != 0;
      default: m_axis_tready = 1'b0;
    endcase
  end

  localparam MAX_RES = 512;
  logic [31:0] res_word[MAX_RES];
  logic        res_last[MAX_RES];
  int n_res = 0;
  int n_checked = 0;

  always @(posedge clk) begin
    if (aresetn && m_axis_tvalid && m_axis_tready) begin
      res_word[n_res] = m_axis_tdata;
      res_last[n_res] = m_axis_tlast;
      n_res++;
    end
  end

  // ------------------------------------------------------------ helpers
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
  function automatic int bucket_count(input int bucket);
    case (bucket)
      0: bucket_count = 0;
      1: bucket_count = 30;
      2: bucket_count = 60;
      default: bucket_count = 100;
    endcase
  endfunction

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

  // One window's worth of events for buckets (el, er): left events at x<64,
  // right at x>=64, timestamps increasing through the window; a sync word
  // if there are no events at all. TLAST on the final word if requested.
  task automatic send_window(input int wid, input int el, input int er, input logic last);
    int nl = bucket_count(el);
    int nr = bucket_count(er);
    int n = nl + nr;
    if (n == 0) begin
      send(sync_word(ts_of(wid, 0)), last);
    end else begin
      for (int i = 0; i < n; i++) begin
        int x = (i < nl) ? (5 + (i % 50)) : (70 + (i % 50));
        send(ev_word(x, 30 + (i % 60), i % 2, ts_of(wid, (i * 256) / n)), last && (i == n - 1));
      end
    end
  endtask

  task automatic ref_do_step(input int el, input int er);
    @(negedge clk);
    ref_el   = el[1:0];
    ref_er   = er[1:0];
    ref_step = 1'b1;
    @(negedge clk);
    ref_step = 1'b0;
  endtask

  task automatic ref_do_reset();
    @(negedge clk);
    ref_rst_n = 1'b0;
    @(negedge clk);
    ref_rst_n = 1'b1;
  endtask

  // Wait for the next decision and check every field against expectations
  // and against the reference FSM (which the caller has already stepped).
  task automatic expect_decision(input int wid, input int el, input int er, input int last,
                                 input int exp_state, input int stale, input string label);
    int t = 0;
    logic [31:0] w;
    int d_id, d_el, d_er, d_st, d_tm, d_stale, d_cl, d_cr;
    while (n_res <= n_checked && t < 5000) begin
      @(posedge clk);
      t++;
    end
    checks++;
    if (n_res <= n_checked) begin
      errors++;
      $display("[FAIL] %0t %s: no decision observed", $time, label);
    end else begin
    w       = res_word[n_checked];
    d_id    = w[31:24];
    d_el    = w[23:22];
    d_er    = w[21:20];
    d_st    = w[19:18];
    d_tm    = w[17:15];
    d_stale = w[14];
    d_cl    = w[13:7];
    d_cr    = w[6:0];
    if (d_id != (wid & 8'hFF) || d_el != el || d_er != er || d_st != exp_state || d_stale != stale ||
        d_cl != bucket_count(el) || d_cr != bucket_count(er) || res_last[n_checked] != last[0] ||
        d_st != ref_state || d_tm != ref_timer) begin
      errors++;
      $display("[FAIL] %0t %s: expected id=%0d el=%0d er=%0d state=%0d (ref state=%0d timer=%0d) last=%0d stale=%0d",
               $time, label, wid & 8'hFF, el, er, exp_state, ref_state, ref_timer, last, stale);
      $display("            got id=%0d el=%0d er=%0d state=%0d timer=%0d last=%0d stale=%0d cl=%0d cr=%0d (word=%08x)",
               d_id, d_el, d_er, d_st, d_tm, res_last[n_checked], d_stale, d_cl, d_cr, w);
    end else begin
      $display("[PASS] %0t %s: id=%0d el=%0d er=%0d state=%0d timer=%0d last=%0d", $time, label, d_id, d_el, d_er,
               d_st, d_tm, res_last[n_checked]);
    end
    n_checked++;
    end
  endtask

  // ------------------------------------------------------------ vectors (scenario A)
  // Same sequence as tb/tb_fsm_controller.sv: {ev_left, ev_right, expected state}.
  localparam int N_VEC = 33;
  int vec_el[N_VEC], vec_er[N_VEC], vec_st[N_VEC];
  initial begin
    vec_el[0] = 0; vec_er[0] = 0; vec_st[0] = 0;
    vec_el[1] = 3; vec_er[1] = 0; vec_st[1] = 2;
    vec_el[2] = 3; vec_er[2] = 0; vec_st[2] = 2;
    vec_el[3] = 0; vec_er[3] = 0; vec_st[3] = 0;
    vec_el[4] = 0; vec_er[4] = 3; vec_st[4] = 1;
    vec_el[5] = 0; vec_er[5] = 0; vec_st[5] = 0;
    vec_el[6] = 3; vec_er[6] = 3; vec_st[6] = 3;
    vec_el[7] = 0; vec_er[7] = 0; vec_st[7] = 3;
    vec_el[8] = 0; vec_er[8] = 0; vec_st[8] = 3;
    vec_el[9] = 0; vec_er[9] = 0; vec_st[9] = 3;
    vec_el[10] = 0; vec_er[10] = 0; vec_st[10] = 3;
    vec_el[11] = 0; vec_er[11] = 0; vec_st[11] = 0;
    vec_el[12] = 3; vec_er[12] = 0; vec_st[12] = 2;
    vec_el[13] = 3; vec_er[13] = 3; vec_st[13] = 3;
    vec_el[14] = 0; vec_er[14] = 0; vec_st[14] = 3;
    vec_el[15] = 0; vec_er[15] = 0; vec_st[15] = 3;
    vec_el[16] = 0; vec_er[16] = 0; vec_st[16] = 3;
    vec_el[17] = 0; vec_er[17] = 0; vec_st[17] = 3;
    vec_el[18] = 0; vec_er[18] = 0; vec_st[18] = 0;
    vec_el[19] = 0; vec_er[19] = 3; vec_st[19] = 1;
    vec_el[20] = 3; vec_er[20] = 3; vec_st[20] = 3;
    vec_el[21] = 0; vec_er[21] = 0; vec_st[21] = 3;
    vec_el[22] = 0; vec_er[22] = 0; vec_st[22] = 3;
    vec_el[23] = 0; vec_er[23] = 0; vec_st[23] = 3;
    vec_el[24] = 0; vec_er[24] = 0; vec_st[24] = 3;
    vec_el[25] = 0; vec_er[25] = 0; vec_st[25] = 0;
    vec_el[26] = 3; vec_er[26] = 3; vec_st[26] = 3;
    vec_el[27] = 3; vec_er[27] = 3; vec_st[27] = 3;
    vec_el[28] = 3; vec_er[28] = 3; vec_st[28] = 3;
    vec_el[29] = 3; vec_er[29] = 3; vec_st[29] = 3;
    vec_el[30] = 3; vec_er[30] = 3; vec_st[30] = 3;
    vec_el[31] = 3; vec_er[31] = 3; vec_st[31] = 0;
    vec_el[32] = 3; vec_er[32] = 3; vec_st[32] = 3;
  end

  // ------------------------------------------------------------ stimulus
  int rb_el[40], rb_er[40];
  logic d_done;

  initial begin
    $dumpfile("build/tb_nlc_axis_top.vcd");
    $dumpvars(0, tb_nlc_axis_top);

    aresetn       = 0;
    ref_rst_n     = 0;
    ref_step      = 0;
    ref_el        = 0;
    ref_er        = 0;
    s_axis_tdata  = '0;
    s_axis_tvalid = 0;
    s_axis_tlast  = 0;
    repeat (3) @(posedge clk);
    @(negedge clk);
    aresetn   = 1;
    ref_rst_n = 1;
    repeat (2) @(posedge clk);

    // ---- A: closed-loop, one packet per control step
    $display("--- A: closed-loop directed vectors ---");
    for (int i = 0; i < N_VEC; i++) begin
      send_window(i, vec_el[i], vec_er[i], 1);
      ref_do_step(vec_el[i], vec_er[i]);
      expect_decision(i, vec_el[i], vec_er[i], 1, vec_st[i], 0, $sformatf("A step %0d", i));
    end

    // ---- B: whole-episode replay in one packet with random backpressure
    $display("--- B: single-packet replay, random m_axis backpressure ---");
    bp_mode = 1;
    for (int i = 0; i < 40; i++) begin
      rb_el[i] = $urandom % 4;
      rb_er[i] = $urandom % 4;
      if (i % 7 == 3) begin  // sprinkle in silent windows
        rb_el[i] = 0;
        rb_er[i] = 0;
      end
    end
    fork
      begin
        for (int i = 0; i < 40; i++) send_window(100 + i, rb_el[i], rb_er[i], i == 39);
      end
      begin
        for (int i = 0; i < 40; i++) begin
          ref_do_step(rb_el[i], rb_er[i]);
          expect_decision(100 + i, rb_el[i], rb_er[i], i == 39, ref_state, 0, $sformatf("B window %0d", 100 + i));
        end
      end
    join
    bp_mode = 0;

    // ---- C: in-band reset
    $display("--- C: in-band reset ---");
    send_window(140, 3, 3, 1);
    ref_do_step(3, 3);
    expect_decision(140, 3, 3, 1, 3, 0, "C drive FSM into BRAKE");
    send(reset_word(ts_of(141, 0)), 1);
    ref_do_reset();
    expect_decision(141, 0, 0, 1, 0, 0, "C reset word -> FORWARD, timer cleared");
    send_window(142, 0, 3, 1);
    ref_do_step(0, 3);
    expect_decision(142, 0, 3, 1, 1, 0, "C pipeline live again after reset");

    // ---- D: output stalled mid-packet
    $display("--- D: m_axis stalled mid-packet ---");
    bp_mode = 2;
    d_done  = 0;
    fork
      begin
        send_window(150, 1, 0, 0);
        send_window(151, 0, 2, 0);
        send_window(152, 3, 0, 1);
        d_done = 1;
      end
    join_none
    repeat (400) @(posedge clk);
    checks++;
    if (n_res != n_checked) begin
      errors++;
      $display("[FAIL] D: decision leaked while m_axis_tready=0");
    end else $display("[PASS] D: no decisions while m_axis_tready=0");
    checks++;
    if (d_done) begin
      errors++;
      $display("[FAIL] D: input did not backpressure while output stalled");
    end else $display("[PASS] D: input stalled behind the blocked output");
    bp_mode = 0;
    ref_do_step(1, 0);
    expect_decision(150, 1, 0, 0, ref_state, 0, "D window 150 after release");
    ref_do_step(0, 2);
    expect_decision(151, 0, 2, 0, ref_state, 0, "D window 151 after release");
    ref_do_step(3, 0);
    expect_decision(152, 3, 0, 1, ref_state, 0, "D window 152 after release");
    wait (d_done);

    repeat (10) @(posedge clk);
    checks++;
    if (n_res != n_checked) begin
      errors++;
      $display("[FAIL] %0d unexpected extra decisions", n_res - n_checked);
    end else $display("[PASS] no spurious decisions");

    $display("----------------------------------------");
    if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
    else $display("%0d/%0d CHECKS FAILED", errors, checks);
    $finish;
  end
endmodule
