// Single-step hardware-in-the-loop driver for fsm_controller.
//
// The Python side (sim/cosim_driver.py) owns the agent/environment physics
// and treats this testbench as the "chip": it invokes `vvp` once per
// control step, passing in the FSM's saved state/timer from the previous
// step plus this step's event-rate readings, and parses the single RESULT
// line this testbench prints back.
//
// Statefulness across separate vvp invocations is a testbench-only backdoor
// (not synthesizable): we force the DUT's state/brake_timer registers to
// the saved values immediately after reset, release the force before the
// next clock edge, and let one real posedge run the FSM's normal logic
// from that resumed point.
module tb_cosim_step;

  localparam CLK_PERIOD   = 10;
  localparam BRAKE_CYCLES = 4;

  logic clk = 0;
  logic rst_n;
  logic [1:0] ev_left, ev_right;
  logic [1:0] state;
  logic cmd_forward, cmd_turn_left, cmd_turn_right, cmd_brake;

  int state_in, ev_left_in, ev_right_in, brake_timer_in;

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

  initial begin
    if (!$value$plusargs("state_in=%d", state_in)) state_in = 0;
    if (!$value$plusargs("ev_left=%d", ev_left_in)) ev_left_in = 0;
    if (!$value$plusargs("ev_right=%d", ev_right_in)) ev_right_in = 0;
    if (!$value$plusargs("brake_timer_in=%d", brake_timer_in)) brake_timer_in = 0;

    rst_n    = 0;
    ev_left  = 0;
    ev_right = 0;
    repeat (2) @(posedge clk);

    @(negedge clk);
    rst_n    = 1;
    ev_left  = ev_left_in[1:0];
    ev_right = ev_right_in[1:0];

    // Resume the FSM from the saved state/timer, then release so the
    // upcoming posedge drives it forward with the DUT's own logic.
    force dut.state       = state_in[1:0];
    force dut.brake_timer = brake_timer_in[$bits(dut.brake_timer)-1:0];
    #1;
    release dut.state;
    release dut.brake_timer;

    @(posedge clk);
    #1;
    $display(
        "RESULT state_out=%0d brake_timer_out=%0d cmd_forward=%0d cmd_turn_left=%0d cmd_turn_right=%0d cmd_brake=%0d",
        state, dut.brake_timer, cmd_forward, cmd_turn_left, cmd_turn_right, cmd_brake);
    $finish;
  end
endmodule
