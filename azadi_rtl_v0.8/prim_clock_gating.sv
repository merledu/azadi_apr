

module prim_clock_gating (
  input  logic clk_i,
  input  logic en_i,
  input  logic test_en_i,
  output logic clk_o
);

sky130_fd_sc_hd__dlclkp_1 CG( .CLK(clk_i), .GCLK(clk_o), .GATE(en_i | test_en_i));

endmodule
