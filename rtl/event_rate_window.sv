// Windowed per-hemisphere event counter: turns a DVS-style event stream
// into the coarse ev_left/ev_right rate buckets fsm_controller consumes.
//
// Input is an AXI4-Stream of 32-bit event words (see docs/architecture.md,
// "Event word format"):
//
//   EVENT word (bit 31 = 1): [30] polarity  [29:23] y  [22:16] x  [15:0] timestamp
//   CTRL  word (bit 31 = 0): [30] reset     [29:16] 0           [15:0] timestamp
//
// Events with x < X_CENTER land in the left hemisphere, the rest on the
// right. A window is 2**WINDOW_SHIFT timestamp ticks wide; its id is the
// timestamp's upper bits. A window closes -- and this module reports its
// two saturating counts plus their quantized buckets for exactly one cycle
// on win_close -- when any of these happen:
//
//   1. a word arrives whose window id differs from the open window's
//      (mid-stream boundary; the word itself is then counted into the new
//      window, and any skipped ids in between are closed as empty windows
//      so the FSM still ticks once per elapsed window),
//   2. the word carries TLAST (end of a DMA packet: the host wants a
//      decision now, whatever the timestamps say), or
//   3. the word is a CTRL reset: counts are cleared and win_is_reset tells
//      the wrapper to reset the FSM instead of stepping it.
//
// CTRL words with reset=0 are sync/heartbeat words: they carry a timestamp
// (so they can open or close a window) but count nothing -- the host uses
// one to close a window in which the sensor produced no events at all.
//
// A word whose window id is behind the open window (modular compare,
// upper half of the id space counts as "behind") is stale: it is dropped
// and the window's win_stale flag is set so the host can see it happened.
//
// The wrapper throttles closes with close_allowed (its one-deep output
// register must be free). Words that don't close a window keep flowing
// while a close is stalled; a closing word is held in the input register
// until the close can go ahead, which backpressures s_axis_tready.
module event_rate_window #(
    parameter int TS_W         = 16,  // timestamp bits in the word
    parameter int WINDOW_SHIFT = 8,   // window = 2**WINDOW_SHIFT ticks
    parameter int X_W          = 7,   // sensor x bits (128 px wide, DVS128-like)
    parameter int X_CENTER     = 64,  // x < X_CENTER -> left hemisphere
    parameter int CNT_W        = 7,   // per-hemisphere saturating counter width
    parameter int RATE_T1      = 25,  // count >= T1 -> bucket 1
    parameter int RATE_T2      = 55,  // count >= T2 -> bucket 2
    parameter int RATE_T3      = 85,  // count >= T3 -> bucket 3
    parameter int EV_WIDTH     = 2
) (
    input  logic clk,
    input  logic rst_n,

    input  logic [31:0] s_axis_tdata,
    input  logic        s_axis_tvalid,
    output logic        s_axis_tready,
    input  logic        s_axis_tlast,

    input  logic                       close_allowed,
    output logic                       win_close,     // 1-cycle pulse, fields below valid
    output logic                       win_is_reset,  // this close came from a CTRL reset word
    output logic                       win_last,      // this close came from a TLAST word
    output logic                       win_stale,     // >= 1 stale word dropped in this window
    output logic [TS_W-WINDOW_SHIFT-1:0] win_id,
    output logic [CNT_W-1:0]           win_cnt_left,
    output logic [CNT_W-1:0]           win_cnt_right,
    output logic [EV_WIDTH-1:0]        win_ev_left,
    output logic [EV_WIDTH-1:0]        win_ev_right
);

  localparam int WID_W = TS_W - WINDOW_SHIFT;

  // ---------------------------------------------------------------- input register
  logic [31:0] q_data;
  logic        q_last, q_valid;
  logic        q_consume;

  assign s_axis_tready = !q_valid || q_consume;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      q_valid <= 1'b0;
      q_data  <= '0;
      q_last  <= 1'b0;
    end else if (s_axis_tvalid && s_axis_tready) begin
      q_valid <= 1'b1;
      q_data  <= s_axis_tdata;
      q_last  <= s_axis_tlast;
    end else if (q_consume) begin
      q_valid <= 1'b0;
    end
  end

  // ---------------------------------------------------------------- decode
  wire             q_is_event = q_data[31];
  wire             q_reset    = !q_data[31] && q_data[30];
  wire [X_W-1:0]   q_x        = q_data[16 +: X_W];
  wire             q_left     = (q_x < X_CENTER);
  wire [WID_W-1:0] q_wid      = q_data[WINDOW_SHIFT +: WID_W];

  // ---------------------------------------------------------------- window state
  logic             win_open;
  logic [WID_W-1:0] cur_id;
  logic [CNT_W-1:0] cnt_left, cnt_right;
  logic             stale_sticky;

  wire [WID_W-1:0] wid_delta  = q_wid - cur_id;
  wire             q_stale    = win_open && !q_reset && wid_delta[WID_W-1];
  wire             q_boundary = win_open && !q_reset && (q_wid != cur_id) && !q_stale;

  wire can_close = close_allowed && !win_close;

  // counts including this cycle's event (if it is accepted and counted)
  wire count_now_left  = q_valid && q_consume && q_is_event && !q_stale &&  q_left;
  wire count_now_right = q_valid && q_consume && q_is_event && !q_stale && !q_left;
  wire [CNT_W-1:0] cnt_left_inc  = (&cnt_left)  ? cnt_left  : cnt_left  + 1'b1;
  wire [CNT_W-1:0] cnt_right_inc = (&cnt_right) ? cnt_right : cnt_right + 1'b1;
  wire [CNT_W-1:0] cnt_left_now  = count_now_left  ? cnt_left_inc  : cnt_left;
  wire [CNT_W-1:0] cnt_right_now = count_now_right ? cnt_right_inc : cnt_right;

  // ---------------------------------------------------------------- decision
  logic do_close, close_reset, close_last, clear_counts, open_next, take_id;

  always_comb begin
    q_consume    = 1'b0;
    do_close     = 1'b0;
    close_reset  = 1'b0;
    close_last   = 1'b0;
    clear_counts = 1'b0;
    open_next    = win_open;
    take_id      = 1'b0;

    if (q_valid) begin
      if (q_boundary) begin
        // Close the open window first; the word stays parked in the input
        // register and is re-evaluated against cur_id+1 next cycle (this
        // is what walks through any skipped, empty windows).
        if (can_close) begin
          do_close     = 1'b1;
          clear_counts = 1'b1;
        end
      end else if ((q_last || q_reset) && !can_close) begin
        // Word would close the window but the output side is busy: hold.
      end else begin
        q_consume = 1'b1;
        if (q_reset) begin
          do_close     = 1'b1;
          close_reset  = 1'b1;
          close_last   = q_last;
          clear_counts = 1'b1;
          open_next    = 1'b0;
        end else begin
          if (!win_open) begin
            open_next = 1'b1;
            take_id   = 1'b1;
          end
          if (q_last) begin
            do_close     = 1'b1;
            close_last   = 1'b1;
            clear_counts = 1'b1;
            open_next    = 1'b0;
          end
        end
      end
    end
  end

  function automatic logic [EV_WIDTH-1:0] quantize(input logic [CNT_W-1:0] c);
    if (c >= RATE_T3) quantize = EV_WIDTH'(3);
    else if (c >= RATE_T2) quantize = EV_WIDTH'(2);
    else if (c >= RATE_T1) quantize = EV_WIDTH'(1);
    else quantize = '0;
  endfunction

  // Counts reported for a reset close are zero by definition.
  wire [CNT_W-1:0] rep_left  = close_reset ? '0 : cnt_left_now;
  wire [CNT_W-1:0] rep_right = close_reset ? '0 : cnt_right_now;
  wire             rep_stale = !close_reset && (stale_sticky || (q_valid && q_consume && q_stale));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      win_open      <= 1'b0;
      cur_id        <= '0;
      cnt_left      <= '0;
      cnt_right     <= '0;
      stale_sticky  <= 1'b0;
      win_close     <= 1'b0;
      win_is_reset  <= 1'b0;
      win_last      <= 1'b0;
      win_stale     <= 1'b0;
      win_id        <= '0;
      win_cnt_left  <= '0;
      win_cnt_right <= '0;
      win_ev_left   <= '0;
      win_ev_right  <= '0;
    end else begin
      win_close <= do_close;

      if (do_close) begin
        win_is_reset  <= close_reset;
        win_last      <= close_last;
        win_stale     <= rep_stale;
        win_id        <= (take_id || close_reset) ? q_wid : cur_id;
        win_cnt_left  <= rep_left;
        win_cnt_right <= rep_right;
        win_ev_left   <= quantize(rep_left);
        win_ev_right  <= quantize(rep_right);
      end

      win_open <= open_next;
      if (take_id) cur_id <= q_wid;
      else if (do_close && q_boundary) cur_id <= cur_id + 1'b1;

      if (clear_counts) begin
        cnt_left     <= '0;
        cnt_right    <= '0;
        stale_sticky <= 1'b0;
      end else begin
        cnt_left  <= cnt_left_now;
        cnt_right <= cnt_right_now;
        if (q_valid && q_consume && q_stale) stale_sticky <= 1'b1;
      end
    end
  end

endmodule
