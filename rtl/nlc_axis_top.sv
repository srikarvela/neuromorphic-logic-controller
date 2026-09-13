// Neuromorphic Logic Controller, AXI4-Stream event pipeline.
//
//   s_axis (32-bit DVS event words) --> event_rate_window --> fsm_controller
//                                                                |
//   m_axis (32-bit decision words, one per closed window) <------+
//
// This is the synthesizable top that goes on the PYNQ-Z2 between the two
// halves of an AXI DMA: the host streams event packets in over MM2S and
// reads decision words back over S2MM. It is also exactly what the Icarus
// cosim server (tb/tb_cosim_server.sv) wraps, so the simulated and
// on-silicon hardware-in-the-loop paths run the same RTL.
//
// Decision word (m_axis_tdata):
//   [31:24] window id (low 8 bits of the closed window's id)
//   [23:22] ev_left bucket      [21:20] ev_right bucket
//   [19:18] FSM state           [17:15] brake timer (zero-extended)
//   [14]    stale flag
//   [13:7]  left event count    [6:0]   right event count   (saturating, 7-bit)
//
// m_axis_tlast is set on the decision produced by an s_axis_tlast word, so
// one input DMA packet always yields exactly one output DMA packet: a
// single decision in closed-loop mode, or one decision per window when a
// whole recorded episode is replayed in one packet.
//
// Per-window timing (A = the cycle the accumulator decides to close):
//   A   event_rate_window registers win_* fields, drops s_axis_tready if
//       the closing word had to be held
//   A+1 win_close high: FSM takes one step_en (or is reset for a CTRL
//       reset word, via the registered fsm_rst_n flop)
//   A+2 new FSM state/timer captured into the output register
//   A+3 m_axis_tvalid high
// The accumulator can't start another close until the output register has
// drained, so at most one decision is ever in flight and nothing is lost
// under S2MM backpressure -- s_axis_tready stalls instead.
module nlc_axis_top #(
    parameter int OBSTACLE_THRESH = 2,
    parameter int BRAKE_THRESH    = 3,
    parameter int BRAKE_CYCLES    = 4,
    parameter int WINDOW_SHIFT    = 8,
    parameter int X_CENTER        = 64,
    parameter int RATE_T1         = 25,
    parameter int RATE_T2         = 55,
    parameter int RATE_T3         = 85
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:m_axis, ASSOCIATED_RESET aresetn" *)
    input  logic        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  logic        aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TDATA" *)
    input  logic [31:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TVALID" *)
    input  logic        s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TREADY" *)
    output logic        s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TLAST" *)
    input  logic        s_axis_tlast,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TDATA" *)
    output logic [31:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TVALID" *)
    output logic        m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TREADY" *)
    input  logic        m_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TLAST" *)
    output logic        m_axis_tlast
);

  localparam int TS_W     = 16;
  localparam int EV_WIDTH = 2;
  localparam int CNT_W    = 7;
  localparam int WID_W    = TS_W - WINDOW_SHIFT;
  localparam int TIMER_W  = (BRAKE_CYCLES <= 1) ? 1 : $clog2(BRAKE_CYCLES + 1);

`ifndef SYNTHESIS
  initial begin
    if (WID_W != 8) $error("nlc_axis_top: decision word packs an 8-bit window id (got %0d)", WID_W);
    if (TIMER_W > 3) $error("nlc_axis_top: decision word has 3 bits for brake_timer (need %0d)", TIMER_W);
  end
`endif

  // ---------------------------------------------------------------- accumulator
  logic                win_close, win_is_reset, win_last, win_stale;
  logic [WID_W-1:0]    win_id;
  logic [CNT_W-1:0]    win_cnt_left, win_cnt_right;
  logic [EV_WIDTH-1:0] win_ev_left, win_ev_right;

  logic out_valid, out_last, capture_pending;
  logic [31:0] out_data;

  wire close_allowed = !capture_pending && (!out_valid || m_axis_tready);

  event_rate_window #(
      .TS_W(TS_W),
      .WINDOW_SHIFT(WINDOW_SHIFT),
      .X_CENTER(X_CENTER),
      .CNT_W(CNT_W),
      .RATE_T1(RATE_T1),
      .RATE_T2(RATE_T2),
      .RATE_T3(RATE_T3),
      .EV_WIDTH(EV_WIDTH)
  ) u_win (
      .clk(aclk),
      .rst_n(aresetn),
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

  // ---------------------------------------------------------------- FSM
  // In-band reset: a registered active-low reset for the FSM so its async
  // reset pin is driven from a flop, never from combinational logic.
  logic fsm_rst_n;
  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) fsm_rst_n <= 1'b0;
    else          fsm_rst_n <= !(win_close && win_is_reset);
  end

  logic [1:0]         fsm_state;
  logic [TIMER_W-1:0] fsm_timer;
  logic cmd_forward, cmd_turn_left, cmd_turn_right, cmd_brake;

  fsm_controller #(
      .EV_WIDTH(EV_WIDTH),
      .OBSTACLE_THRESH(OBSTACLE_THRESH),
      .BRAKE_THRESH(BRAKE_THRESH),
      .BRAKE_CYCLES(BRAKE_CYCLES)
  ) u_fsm (
      .clk(aclk),
      .rst_n(fsm_rst_n),
      .step_en(win_close && !win_is_reset),
      .ev_left(win_ev_left),
      .ev_right(win_ev_right),
      .state(fsm_state),
      .cmd_forward,
      .cmd_turn_left,
      .cmd_turn_right,
      .cmd_brake,
      .dbg_brake_timer(fsm_timer)
  );

  // ---------------------------------------------------------------- output register
  wire [2:0] timer_field = 3'(fsm_timer);

  always_ff @(posedge aclk or negedge aresetn) begin
    if (!aresetn) begin
      capture_pending <= 1'b0;
      out_valid       <= 1'b0;
      out_last        <= 1'b0;
      out_data        <= '0;
    end else begin
      capture_pending <= win_close;
      if (capture_pending) begin
        out_valid <= 1'b1;
        out_last  <= win_last;
        out_data  <= {win_id[7:0], win_ev_left, win_ev_right, fsm_state, timer_field, win_stale,
                      win_cnt_left[6:0], win_cnt_right[6:0]};
      end else if (out_valid && m_axis_tready) begin
        out_valid <= 1'b0;
      end
    end
  end

  assign m_axis_tdata  = out_data;
  assign m_axis_tvalid = out_valid;
  assign m_axis_tlast  = out_last;

endmodule
