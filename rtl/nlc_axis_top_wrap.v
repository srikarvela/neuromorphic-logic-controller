// Plain Verilog-2001 shell around nlc_axis_top for Vivado IP Integrator.
//
// A block-design module reference (create_bd_cell -type module -reference)
// must have a Verilog or VHDL top file; SystemVerilog is rejected
// ([filemgmt 56-195]). This wrapper only re-declares the ports with the
// interface attributes Vivado uses to infer the two AXI4-Stream buses and
// passes everything straight through. All logic lives in the .sv files.
`default_nettype none
module nlc_axis_top_wrap #(
    parameter OBSTACLE_THRESH = 2,
    parameter BRAKE_THRESH    = 3,
    parameter BRAKE_CYCLES    = 4,
    parameter WINDOW_SHIFT    = 8,
    parameter X_CENTER        = 64,
    parameter RATE_T1         = 25,
    parameter RATE_T2         = 55,
    parameter RATE_T3         = 85
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:m_axis, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TDATA" *)
    input  wire [31:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TVALID" *)
    input  wire        s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TREADY" *)
    output wire        s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TLAST" *)
    input  wire        s_axis_tlast,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TDATA" *)
    output wire [31:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TVALID" *)
    output wire        m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TREADY" *)
    input  wire        m_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TLAST" *)
    output wire        m_axis_tlast
);

  nlc_axis_top #(
      .OBSTACLE_THRESH(OBSTACLE_THRESH),
      .BRAKE_THRESH(BRAKE_THRESH),
      .BRAKE_CYCLES(BRAKE_CYCLES),
      .WINDOW_SHIFT(WINDOW_SHIFT),
      .X_CENTER(X_CENTER),
      .RATE_T1(RATE_T1),
      .RATE_T2(RATE_T2),
      .RATE_T3(RATE_T3)
  ) u_nlc (
      .aclk(aclk),
      .aresetn(aresetn),
      .s_axis_tdata(s_axis_tdata),
      .s_axis_tvalid(s_axis_tvalid),
      .s_axis_tready(s_axis_tready),
      .s_axis_tlast(s_axis_tlast),
      .m_axis_tdata(m_axis_tdata),
      .m_axis_tvalid(m_axis_tvalid),
      .m_axis_tready(m_axis_tready),
      .m_axis_tlast(m_axis_tlast)
  );

endmodule
`default_nettype wire
