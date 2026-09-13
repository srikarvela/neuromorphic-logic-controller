// Directed, self-checking unit test for fsm_controller.
// Run with scripts/run_unit_tb.sh (iverilog + vvp).
module tb_fsm_controller;

  localparam CLK_PERIOD   = 10;
  localparam BRAKE_CYCLES = 4;

  logic clk = 0;
  logic rst_n;
  logic step_en = 1;  // every clock is a control step in this unit test
  logic [1:0] ev_left, ev_right;
  logic [1:0] state;
  logic cmd_forward, cmd_turn_left, cmd_turn_right, cmd_brake;

  int errors = 0;
  int checks = 0;

  fsm_controller #(
      .BRAKE_CYCLES(BRAKE_CYCLES)
  ) dut (
      .clk,
      .rst_n,
      .step_en,
      .ev_left,
      .ev_right,
      .state,
      .cmd_forward,
      .cmd_turn_left,
      .cmd_turn_right,
      .cmd_brake,
      .dbg_brake_timer()
  );

  always #(CLK_PERIOD / 2) clk = ~clk;

  // Drive new stimulus on the negedge (half a cycle of margin before the
  // DUT's own posedge-triggered always_ff samples it) to avoid a same-edge
  // race between the testbench and the clocked process.
  task automatic drive(input logic [1:0] el, input logic [1:0] er);
    @(negedge clk);
    ev_left  = el;
    ev_right = er;
  endtask

  // Advance one clock and check state after NBA updates have settled.
  task automatic step_check(input logic [1:0] expected, input string label);
    @(posedge clk);
    #1;
    checks++;
    if (state !== expected) begin
      errors++;
      $display("[FAIL] %0t %s: expected state=%0d got state=%0d", $time, label, expected, state);
    end else begin
      $display("[PASS] %0t %s: state=%0d", $time, label, state);
    end
  endtask

  task automatic check_cmds_match_state();
    checks++;
    if ({cmd_brake, cmd_turn_right, cmd_turn_left, cmd_forward} !==
        {(state == 2'd3), (state == 2'd2), (state == 2'd1), (state == 2'd0)}) begin
      errors++;
      $display("[FAIL] %0t cmd outputs don't match state=%0d", $time, state);
    end
  endtask

  // Whitebox check on the internal brake_timer register (hierarchical
  // reference into the DUT) -- used to confirm the timer is reloaded, not
  // stale, whenever BRAKE_S is (re-)entered.
  task automatic check_timer(input int expected, input string label);
    checks++;
    if (dut.brake_timer !== expected) begin
      errors++;
      $display("[FAIL] %0t %s: expected brake_timer=%0d got %0d", $time, label, expected,
                dut.brake_timer);
    end else begin
      $display("[PASS] %0t %s: brake_timer=%0d", $time, label, dut.brake_timer);
    end
  endtask

  initial begin
    $dumpfile("build/tb_fsm_controller.vcd");
    $dumpvars(0, tb_fsm_controller);

    rst_n    = 0;
    ev_left  = 0;
    ev_right = 0;
    repeat (2) @(posedge clk);
    @(negedge clk);
    rst_n = 1;
    step_check(2'd0, "reset -> FORWARD");
    check_cmds_match_state();

    // Clear skies: stay FORWARD.
    drive(0, 0);
    step_check(2'd0, "no events -> FORWARD");

    // Obstacle on left hemisphere -> steer right, away from it.
    drive(3, 0);
    step_check(2'd2, "obstacle left -> TURN_R");
    check_cmds_match_state();
    step_check(2'd2, "obstacle left persists -> TURN_R holds");

    drive(0, 0);
    step_check(2'd0, "obstacle left clears -> FORWARD");

    // Obstacle on right hemisphere -> steer left, away from it.
    drive(0, 3);
    step_check(2'd1, "obstacle right -> TURN_L");
    check_cmds_match_state();

    drive(0, 0);
    step_check(2'd0, "obstacle right clears -> FORWARD");

    // Critical burst on both sides -> BRAKE, hold for BRAKE_CYCLES, then recover.
    drive(3, 3);
    step_check(2'd3, "critical front -> BRAKE");
    check_cmds_match_state();

    drive(0, 0);
    for (int i = 0; i < BRAKE_CYCLES; i++) begin
      step_check(2'd3, $sformatf("brake timer countdown %0d", i));
    end
    step_check(2'd0, "brake timer expires -> FORWARD");

    // --- Edge case: BRAKE preempts an in-progress TURN_R with a fresh timer,
    // not whatever was left over from any earlier brake.
    drive(3, 0);
    step_check(2'd2, "obstacle left -> TURN_R (preempt setup)");
    drive(3, 3);
    step_check(2'd3, "critical front mid-TURN_R -> BRAKE preempts turn");
    check_timer(BRAKE_CYCLES, "timer freshly loaded on TURN_R->BRAKE preemption");

    drive(0, 0);
    for (int i = 0; i < BRAKE_CYCLES; i++) begin
      step_check(2'd3, $sformatf("preempt-from-TURN_R countdown %0d", i));
    end
    step_check(2'd0, "preempt-from-TURN_R brake expires -> FORWARD");

    // --- Same edge case, mirrored: BRAKE preempts an in-progress TURN_L.
    drive(0, 3);
    step_check(2'd1, "obstacle right -> TURN_L (preempt setup)");
    drive(3, 3);
    step_check(2'd3, "critical front mid-TURN_L -> BRAKE preempts turn");
    check_timer(BRAKE_CYCLES, "timer freshly loaded on TURN_L->BRAKE preemption");

    drive(0, 0);
    for (int i = 0; i < BRAKE_CYCLES; i++) begin
      step_check(2'd3, $sformatf("preempt-from-TURN_L countdown %0d", i));
    end
    step_check(2'd0, "preempt-from-TURN_L brake expires -> FORWARD");

    // --- Edge case: if the obstacle is still critical exactly when the
    // brake timer expires, the FSM takes one cycle in FORWARD before
    // critical_front sends it straight back into a freshly-timed BRAKE.
    // This is a known one-cycle flicker, not a bug -- see docs/architecture.md.
    drive(3, 3);
    step_check(2'd3, "critical sustained -> BRAKE");
    check_timer(BRAKE_CYCLES, "timer freshly loaded entering sustained BRAKE");

    for (int i = 0; i < BRAKE_CYCLES; i++) begin
      step_check(2'd3, $sformatf("sustained-critical countdown %0d", i));
    end
    step_check(2'd0, "timer expiry blip -> FORWARD even though still critical");
    step_check(2'd3, "critical still present -> re-enters BRAKE next cycle");
    check_timer(BRAKE_CYCLES, "timer freshly loaded on flicker re-entry");

    drive(0, 0);

    // --- step_en gating: with step_en low the FSM must freeze completely,
    // both the state register and a mid-countdown brake timer, no matter
    // what the event inputs do. This is what lets the AXI-Stream wrapper
    // advance it exactly once per closed event window.
    do begin  // let the flicker re-entry brake above run out first
      @(posedge clk);
      #1;
    end while (state != 2'd0);
    drive(3, 3);
    step_check(2'd3, "critical front -> BRAKE (step_en gating setup)");
    check_timer(BRAKE_CYCLES, "timer loaded before step_en drops");
    @(negedge clk);
    step_en = 0;
    drive(0, 0);
    for (int i = 0; i < 3; i++) begin
      step_check(2'd3, $sformatf("step_en=0 holds BRAKE %0d", i));
    end
    check_timer(BRAKE_CYCLES, "step_en=0 holds brake_timer");
    @(negedge clk);
    step_en = 1;
    for (int i = 0; i < BRAKE_CYCLES; i++) begin
      step_check(2'd3, $sformatf("step_en=1 resumes countdown %0d", i));
    end
    step_check(2'd0, "resumed countdown expires -> FORWARD");

    $display("----------------------------------------");
    if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
    else $display("%0d/%0d CHECKS FAILED", errors, checks);

    $finish;
  end
endmodule
