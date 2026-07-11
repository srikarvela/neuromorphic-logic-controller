// Directed, self-checking unit test for fsm_controller.
// Run with scripts/run_unit_tb.sh (iverilog + vvp).
module tb_fsm_controller;

  localparam CLK_PERIOD   = 10;
  localparam BRAKE_CYCLES = 4;

  logic clk = 0;
  logic rst_n;
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
      .ev_left,
      .ev_right,
      .state,
      .cmd_forward,
      .cmd_turn_left,
      .cmd_turn_right,
      .cmd_brake
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

    $display("----------------------------------------");
    if (errors == 0) $display("ALL %0d CHECKS PASSED", checks);
    else $display("%0d/%0d CHECKS FAILED", errors, checks);

    $finish;
  end
endmodule
