// Event-driven reactive obstacle-avoidance controller.
//
// Inspired by insect optic-flow avoidance: the "brain" receives coarse
// event-rate readings from the left and right halves of an event-camera
// field of view (ev_left/ev_right, higher = more looming activity = closer
// obstacle) and reacts by steering away from the busier side. A burst on
// both sides at once means something is dead ahead, so the FSM brakes for
// a fixed number of control steps before re-evaluating.
//
// One "control step" is one clock edge with step_en high. In the unit
// testbench step_en is tied high so every clock is a step; in the
// AXI-Stream pipeline (rtl/nlc_axis_top.sv) it pulses once per closed
// event window, so the FSM sees one fresh event-rate reading per window
// and BRAKE_CYCLES counts windows, not fabric clocks.
module fsm_controller #(
    parameter int EV_WIDTH        = 2,  // event-rate bucket width (0..2**EV_WIDTH-1)
    parameter int OBSTACLE_THRESH = 2,  // bucket value that counts as "obstacle this side"
    parameter int BRAKE_THRESH    = 3,  // bucket value on BOTH sides that forces a brake
    parameter int BRAKE_CYCLES    = 4   // cycles to hold BRAKE before re-evaluating
) (
    input  logic                clk,
    input  logic                rst_n,
    input  logic                step_en,   // advance one control step this edge
    input  logic [EV_WIDTH-1:0] ev_left,
    input  logic [EV_WIDTH-1:0] ev_right,

    output logic [1:0] state,
    output logic        cmd_forward,
    output logic        cmd_turn_left,
    output logic        cmd_turn_right,
    output logic        cmd_brake,

    // Observability only: the brake countdown, so a wrapper can report it
    // without a hierarchical reference (which Vivado won't synthesize).
    output logic [((BRAKE_CYCLES <= 1) ? 1 : $clog2(BRAKE_CYCLES + 1))-1:0] dbg_brake_timer
);

  localparam logic [1:0] FORWARD = 2'd0;
  localparam logic [1:0] TURN_L  = 2'd1;
  localparam logic [1:0] TURN_R  = 2'd2;
  localparam logic [1:0] BRAKE_S = 2'd3;

  localparam int TIMER_W = (BRAKE_CYCLES <= 1) ? 1 : $clog2(BRAKE_CYCLES + 1);

  logic [1:0] next_state;
  logic [TIMER_W-1:0] brake_timer, brake_timer_next;

  wire obstacle_left  = (ev_left  >= OBSTACLE_THRESH);
  wire obstacle_right = (ev_right >= OBSTACLE_THRESH);
  wire critical_front = (ev_left >= BRAKE_THRESH) && (ev_right >= BRAKE_THRESH);

  always_comb begin
    next_state       = state;
    brake_timer_next = brake_timer;

    case (state)
      FORWARD: begin
        if (critical_front) next_state = BRAKE_S;
        else if (obstacle_left) next_state = TURN_R;  // obstacle on left -> steer right
        else if (obstacle_right) next_state = TURN_L;  // obstacle on right -> steer left
      end

      TURN_R: begin
        if (critical_front) next_state = BRAKE_S;
        else if (!obstacle_left) next_state = FORWARD;
      end

      TURN_L: begin
        if (critical_front) next_state = BRAKE_S;
        else if (!obstacle_right) next_state = FORWARD;
      end

      BRAKE_S: begin
        if (brake_timer == 0) next_state = FORWARD;
        else brake_timer_next = brake_timer - 1'b1;
      end

      default: next_state = FORWARD;
    endcase

    // Load the timer the cycle we enter BRAKE.
    if (next_state == BRAKE_S && state != BRAKE_S)
      brake_timer_next = BRAKE_CYCLES[TIMER_W-1:0];
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state       <= FORWARD;
      brake_timer <= '0;
    end else if (step_en) begin
      state       <= next_state;
      brake_timer <= brake_timer_next;
    end
  end

  assign cmd_forward    = (state == FORWARD);
  assign cmd_turn_left  = (state == TURN_L);
  assign cmd_turn_right = (state == TURN_R);
  assign cmd_brake      = (state == BRAKE_S);
  assign dbg_brake_timer = brake_timer;

endmodule
