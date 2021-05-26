`include "/home/merl/github_repos/azadi/src/spi_host/rtl/spi_defines.v"
module azadi_top_arty7(

  input  clock,
  input  reset,

  // gpio
  inout  logic [31:0] gpio,
  // jtag interface 
  input  logic       jtag_tck_i,
  input  logic       jtag_tms_i,
  input  logic       jtag_trst_ni,
  input  logic       jtag_tdi_i,
  output logic       jtag_tdo_o,

  // uart-periph interface
  output logic       uart_tx,
  input  logic       uart_rx,

  // PWM interface  

  output logic       pwm_o,
  output logic       pwm_o_2,

  // SPI interface

  output logic    [`SPI_SS_NB-1:0] ss_o,        
  output logic                     sclk_o,      
  output logic                     sd_o,       
  input  logic                     sd_i,

  // i2c interface 

  inout  logic scl_pad,
  inout  logic sda_pad
  
);

logic [31:0] gpio_i;
logic [31:0] gpio_o;
logic [32:0] gpio_oe;

logic scl_pad_i;
logic scl_pad_o;
logic scl_padoen_op;

logic sda_pad_i;
logic sda_pad_o;
logic sda_padoen_o;

logic clk_i;
logic reset_n;
assign reset_n = ~reset;

//gpio buffers
IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst0(
    .O  (gpio_i[0]),//Bufferoutput
    .IO (gpio[0]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[0]),//Bufferinput
    .T  (gpio_oe[0])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst1(
    .O  (gpio_i[1]),//Bufferoutput
    .IO (gpio[1]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[1]),//Bufferinput
    .T  (gpio_oe[1])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst2(
    .O  (gpio_i[2]),//Bufferoutput
    .IO (gpio[2]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[2]),//Bufferinput
    .T  (gpio_oe[2])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst3(
    .O  (gpio_i[3]),//Bufferoutput
    .IO (gpio[3]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[3]),//Bufferinput
    .T  (gpio_oe[3])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_ins4(
    .O  (gpio_i[4]),//Bufferoutput
    .IO (gpio[4]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[4]),//Bufferinput
    .T  (gpio_oe[4])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_ins5(
    .O  (gpio_i[5]),//Bufferoutput
    .IO (gpio[5]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[5]),//Bufferinput
    .T  (gpio_oe[5])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_ins6(
    .O  (gpio_i[6]),//Bufferoutput
    .IO (gpio[6]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[6]),//Bufferinput
    .T  (gpio_oe[6])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_ins7(
    .O  (gpio_i[7]),//Bufferoutput
    .IO (gpio[7]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[7]),//Bufferinput
    .T  (gpio_oe[7])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_ins8(
    .O  (gpio_i[8]),//Bufferoutput
    .IO (gpio[8]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[8]),//Bufferinput
    .T  (gpio_oe[8])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_ins9(
    .O  (gpio_i[9]),//Bufferoutput
    .IO (gpio[9]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[9]),//Bufferinput
    .T  (gpio_oe[9])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst10(
    .O  (gpio_i[10]),//Bufferoutput
    .IO (gpio[10]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[10]),//Bufferinput
    .T  (gpio_oe[10])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst11(
    .O  (gpio_i[11]),//Bufferoutput
    .IO (gpio[11]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[11]),//Bufferinput
    .T  (gpio_oe[11])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst12(
    .O  (gpio_i[12]),//Bufferoutput
    .IO (gpio[12]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[12]),//Bufferinput
    .T  (gpio_oe[12])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst13(
    .O  (gpio_i[13]),//Bufferoutput
    .IO (gpio[13]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[13]),//Bufferinput
    .T  (gpio_oe[13])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst14(
    .O  (gpio_i[14]),//Bufferoutput
    .IO (gpio[14]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[14]),//Bufferinput
    .T  (gpio_oe[14])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst15(
    .O  (gpio_i[15]),//Bufferoutput
    .IO (gpio[15]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[15]),//Bufferinput
    .T  (gpio_oe[15])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst16(
    .O  (gpio_i[16]),//Bufferoutput
    .IO (gpio[16]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[16]),//Bufferinput
    .T  (gpio_oe[16])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst17(
    .O  (gpio_i[17]),//Bufferoutput
    .IO (gpio[17]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[17]),//Bufferinput
    .T  (gpio_oe[17])//3-stateenableinput,high=input,low=output
);
IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst18(
    .O  (gpio_i[18]),//Bufferoutput
    .IO (gpio[18]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[18]),//Bufferinput
    .T  (gpio_oe[18])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst19(
    .O  (gpio_i[19]),//Bufferoutput
    .IO (gpio[19]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[19]),//Bufferinput
    .T  (gpio_oe[19])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst20(
    .O  (gpio_i[20]),//Bufferoutput
    .IO (gpio[20]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[2]),//Bufferinput
    .T  (gpio_oe[20])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst21(
    .O  (gpio_i[21]),//Bufferoutput
    .IO (gpio[21]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[21]),//Bufferinput
    .T  (gpio_oe[21])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst22(
    .O  (gpio_i[22]),//Bufferoutput
    .IO (gpio[22]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[22]),//Bufferinput
    .T  (gpio_oe[22])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst23(
    .O  (gpio_i[23]),//Bufferoutput
    .IO (gpio[23]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[23]),//Bufferinput
    .T  (gpio_oe[23])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst24(
    .O  (gpio_i[24]),//Bufferoutput
    .IO (gpio[24]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[24]),//Bufferinput
    .T  (gpio_oe[24])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst25(
    .O  (gpio_i[25]),//Bufferoutput
    .IO (gpio[25]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[25]),//Bufferinput
    .T  (gpio_oe[25])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst26(
    .O  (gpio_i[26]),//Bufferoutput
    .IO (gpio[26]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[26]),//Bufferinput
    .T  (gpio_oe[26])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst27(
    .O  (gpio_i[27]),//Bufferoutput
    .IO (gpio[27]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[27]),//Bufferinput
    .T  (gpio_oe[27])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst28(
    .O  (gpio_i[28]),//Bufferoutput
    .IO (gpio[28]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[28]),//Bufferinput
    .T  (gpio_oe[28])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst29(
    .O  (gpio_i[29]),//Bufferoutput
    .IO (gpio[29]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[29]),//Bufferinput
    .T  (gpio_oe[29])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst30(
    .O  (gpio_i[30]),//Bufferoutput
    .IO (gpio[30]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[30]),//Bufferinput
    .T  (gpio_oe[30])//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst31(
    .O  (gpio_i[31]),//Bufferoutput
    .IO (gpio[31]),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (gpio_o[31]),//Bufferinput
    .T  (gpio_oe[31])//3-stateenableinput,high=input,low=output
);

// i2c buffers
IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst32(
    .O  (scl_pad_i),//Bufferoutput
    .IO (scl_pad),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (scl_pad_o),//Bufferinput
    .T  (scl_padoen_o)//3-stateenableinput,high=input,low=output
);

IOBUF#(
    .DRIVE(12),//Specifytheoutputdrivestrength
    .IBUF_LOW_PWR("TRUE"),//LowPower-"TRUE",HighPerforrmance="FALSE"
    .IOSTANDARD("DEFAULT"),//SpecifytheI/Ostandard
    .SLEW("SLOW")//Specifytheoutputslewrate
)IOBUF_inst33(
    .O  (sda_pad_i),//Bufferoutput
    .IO (sda_pad),//Bufferinoutport(connectdirectlytotop-levelport)
    .I  (sda_pad_o),//Bufferinput
    .T  (sda_padoen_o)//3-stateenableinput,high=input,low=output
);

// clock ip
clk_wiz_0 clock_ip(
  // Clock out ports
  .clk_out1   (clk_i),
  // Status and control signals
  .resetn     (reset_n),
  .locked     (),
 // Clock in ports
  .clk_in1    (clock)
 );

 

localparam logic [31:0] JTAG_IDCODE = {
  4'h0,     // Version
  16'h4F54, // Part Number: "OT"
  11'h426,  // Manufacturer Identity: Google
  1'b1      // (fixed)
};


azadi_soc_top #(
  
  .JTAG_ID       (JTAG_IDCODE),
  .DirectDmiTap  ('0)
) u_soc_top (
  .clk_i         (clk_i ),
  .rst_ni        (reset_n),

  .gpio_i        (gpio_o),
  .gpio_o        (gpio_o),
  .gpio_oe       (gpio_oe),

  // jtag interface 
  .jtag_tck_i    (jtag_tck_i),
  .jtag_tms_i    (jtag_tms_i),
  .jtag_trst_ni  (jtag_trst_ni),
  .jtag_tdi_i    (jtag_tdi_i),
  .jtag_tdo_o    (jtag_tdo_o),

  // uart-periph interface
  .uart_tx       (uart_tx),
  .uart_rx       (uart_rx),

  // PWM interface  

  .pwm_o         (pwm_o),
  .pwm_o_2       (pwm_o_2),

  // SPI interface

  .ss_o          (ss_o),        
  .sclk_o        (sclk_o),      
  .sd_o          (sd_o),       
  .sd_i          (sd_i),

  // i2c interface 

  .scl_pad_i     (scl_pad_i),
  .scl_pad_o     (scl_pad_o),
  .scl_padoen_o  (scl_padoen_o),

  .sda_pad_i     (sda_pad_i), 
  .sda_pad_o     (sda_pad_o),
  .sda_padoen_o  (sda_padoen_o) 

);


endmodule