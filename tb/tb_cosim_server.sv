// Persistent hardware-in-the-loop server around nlc_axis_top.
//
// The Python side (sim/engines.py::IcarusEngine) launches one `vvp` process
// per session and talks to it over stdin/stdout with a line protocol that
// mirrors what the PYNQ DMA driver does with buffers:
//
//   stdin  ->  "<8 hex digits> <last>"   one AXI-Stream beat (event word + TLAST)
//   stdin  ->  "Q"                       finish
//   stdout <-  "RESULT word=<8 hex> last=<0|1>"   one m_axis beat (decision word)
//
// The RTL runs continuously inside this one simulation: FSM state and
// brake timer persist between packets in the DUT's own registers, exactly
// as they do on the FPGA, so there is no force/release or state
// save/restore anywhere. After a TLAST beat the server keeps clocking
// until the decision carrying m_axis_tlast has been printed, then blocks
// on the next stdin line (simulation time only advances while stimulus is
// flowing).
//
// Compile: scripts/build_cosim.sh   Run: vvp -n build/tb_cosim_server.vvp
module tb_cosim_server;

  localparam CLK_PERIOD  = 10;
  localparam LAST_TIMEOUT_CYCLES = 100000;  // upper bound on catch-up windows per packet

  logic clk = 0;
  logic aresetn;

  logic [31:0] s_axis_tdata;
  logic        s_axis_tvalid, s_axis_tready, s_axis_tlast;
  logic [31:0] m_axis_tdata;
  logic        m_axis_tvalid, m_axis_tready, m_axis_tlast;

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

  always #(CLK_PERIOD / 2) clk = ~clk;

  // Decision monitor: print every m_axis beat as it happens.
  int last_seen = 0;
  assign m_axis_tready = 1'b1;
  always @(posedge clk) begin
    if (aresetn && m_axis_tvalid && m_axis_tready) begin
      $display("RESULT word=%08x last=%0d", m_axis_tdata, m_axis_tlast);
      $fflush();
      if (m_axis_tlast) last_seen++;
    end
  end

  // One AXI-Stream beat: drive on negedge, hold until the DUT accepts it.
  task automatic send_beat(input logic [31:0] data, input logic last);
    @(negedge clk);
    s_axis_tdata  = data;
    s_axis_tlast  = last;
    s_axis_tvalid = 1'b1;
    @(posedge clk);
    while (!s_axis_tready) @(posedge clk);
    @(negedge clk);
    s_axis_tvalid = 1'b0;
    s_axis_tlast  = 1'b0;
  endtask

  integer fd, n, cycles, want_last;
  reg [8*128-1:0] line;
  reg [31:0] word;
  integer last;
  string vcd_path;

  initial begin
    if ($value$plusargs("vcd=%s", vcd_path)) begin
      $dumpfile(vcd_path);
      $dumpvars(0, tb_cosim_server);
    end

    fd = $fopen("/dev/stdin", "r");
    if (fd == 0) begin
      $display("ERROR cannot open stdin");
      $finish;
    end

    aresetn       = 0;
    s_axis_tdata  = '0;
    s_axis_tvalid = 0;
    s_axis_tlast  = 0;
    repeat (4) @(posedge clk);
    @(negedge clk);
    aresetn = 1;
    repeat (2) @(posedge clk);
    $display("READY");
    $fflush();

    forever begin
      n = $fgets(line, fd);
      if (n == 0) begin
        $display("EOF");
        $finish;
      end
      if (line[8*n-1 -: 8] == "Q") begin
        $display("QUIT");
        $finish;
      end
      if ($sscanf(line, "%h %d", word, last) != 2) begin
        $display("ERROR unparseable line");
        $fflush();
      end else begin
        want_last = last_seen + (last ? 1 : 0);
        send_beat(word, last[0]);
        if (last) begin
          cycles = 0;
          while (last_seen < want_last && cycles < LAST_TIMEOUT_CYCLES) begin
            @(posedge clk);
            cycles++;
          end
          if (last_seen < want_last) begin
            $display("ERROR timeout waiting for tlast decision");
            $fflush();
          end
        end
      end
    end
  end
endmodule
