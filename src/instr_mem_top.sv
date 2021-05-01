
module instr_mem_top
(
  input clk_i,
  input rst_ni,

  input  logic        req,
  input  logic [11:0] addr,
  input  logic [31:0] wdata,
  output logic [31:0] rdata,
  output logic        rvalid,
  input  logic [3:0]  we
);

  always_ff @(posedge clk_i) begin
  if (!rst_ni) begin
    rvalid <= 1'b0;
  end else if (we) begin
    rvalid <= 1'b0;
  end else begin 
    rvalid <= req;
  end
  end

DFFRAM iccm (

    .CLK    (clk_i),
    .EN     (req), // chip enable
    .WE     (we), //write mask
    .Di     (wdata), //data input
    .Do     (rdata), // data output
    .A      (addr) // address
);

endmodule