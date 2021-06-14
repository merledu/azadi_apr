module sram_top(
  `ifdef USE_POWER_PINS
      inout VPWR,
      inout VGND,
  `endif
    clk_i,web_i,wmask_i,addr_i,din_i,dout_o
  );

  input       clk_i;
  input       web_i;
  input [3:0] wmask_i;
  input [10:0]addr_i;
  input [31:0]din_i;
  output[31:0]dout_o;

  wire csb;
  assign csb = addr_i[10];

  wire [9:0]addr;
  assign addr = addr_i[9:0];

  wire [31:0] dout_1, dout_2;

  sky130_sram_4kbyte_1rw1r_32x1024_8 SRAM1(
  `ifdef USE_POWER_PINS
      .vccd1(VPWR),
      .vssd1(VGND),
  `endif
    // Port 0: RW
    .clk0(clk_i),
    .csb0(csb),
    .web0(web_i),
    .wmask0(wmask_i),
    .addr0(addr),
    .din0(din_i),
    .dout0(dout_1),
    // Port 1: R
    .clk1(1'b0),
    .csb1(1'b1),
    .addr1(11'b0),
    .dout1()
  );

  assign dout_o = csb ? dout_2 : dout_1;

  sky130_sram_4kbyte_1rw1r_32x1024_8 SRAM2(
  `ifdef USE_POWER_PINS
      .vccd1(VPWR),
      .vssd1(VGND),
  `endif
    // Port 0: RW
    .clk0(clk_i),
    .csb0(~csb),
    .web0(web_i),
    .wmask0(wmask_i),
    .addr0(addr),
    .din0(din_i),
    .dout0(dout_2),
    // Port 1: R
    .clk1(1'b0),
    .csb1(1'b1),
    .addr1(11'b0),
    .dout1()
  );

endmodule
