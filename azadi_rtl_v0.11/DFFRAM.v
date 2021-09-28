/*
    Copyright ©2020-2021 The American University in Cairo and the Cloud V Project.

    This file is part of the DFFRAM Memory Compiler.
    See https://github.com/Cloud-V/DFFRAM for further info.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

// Add 1x2 binary decoder
//`default_nettype none
`define USE_COSTUM
`ifdef USE_COSTUM

module DFFRAM #(parameter  USE_LATCH=1,
                            WSIZE=1 )(
`ifdef USE_POWER_PINS
    input VPWR,
    input VGND,
`endif
    input CLK,
    input [3:0] WE,
    input EN,
    input [31:0] Di,
    output reg [31:0] Do,
    input [10:0] A
);
  

reg [31:0] mem [0:2047];

always @(posedge CLK) begin
    if (EN == 1'b1) begin
        Do <= mem[A];
        if (WE[0]) mem[A][ 7: 0] <= Di[ 7: 0];
        if (WE[1]) mem[A][15: 8] <= Di[15: 8];
        if (WE[2]) mem[A][23:16] <= Di[23:16];
        if (WE[3]) mem[A][31:24] <= Di[31:24];
    end 
end
endmodule

`else

module DEC1x2 (
    input           EN,
    input           A,
    output [1:0]    SEL
);
    sky130_fd_sc_hd__and2b_2 AND0 ( .X(SEL[0]), .A_N(A), .B(EN) );
    sky130_fd_sc_hd__and2_2 AND1 ( .X(SEL[1]), .A(A) , .B(EN) );

endmodule

module DEC2x4 (
    input           EN,
    input   [1:0]   A,
    output  [3:0]   SEL
);
    sky130_fd_sc_hd__nor3b_4    AND0 ( .Y(SEL[0]), .A(A[0]),   .B(A[1]), .C_N(EN) );
    sky130_fd_sc_hd__and3b_4    AND1 ( .X(SEL[1]), .A_N(A[1]), .B(A[0]), .C(EN) );
    sky130_fd_sc_hd__and3b_4    AND2 ( .X(SEL[2]), .A_N(A[0]), .B(A[1]), .C(EN) );
    sky130_fd_sc_hd__and3_4     AND3 ( .X(SEL[3]), .A(A[1]),   .B(A[0]), .C(EN) );
    
endmodule

module DEC3x8 (
    input           EN,
    input [2:0]     A,
    output [7:0]    SEL
);

    wire [2:0]  A_buf;
    wire        EN_buf;

    sky130_fd_sc_hd__clkbuf_2 ABUF[2:0] (.X(A_buf), .A(A));
    sky130_fd_sc_hd__clkbuf_2 ENBUF (.X(EN_buf), .A(EN));
    
    sky130_fd_sc_hd__nor4b_2   AND0 ( .Y(SEL[0])  , .A(A_buf[0]), .B(A_buf[1])  , .C(A_buf[2]), .D_N(EN_buf) ); // 000
    sky130_fd_sc_hd__and4bb_2   AND1 ( .X(SEL[1])  , .A_N(A_buf[2]), .B_N(A_buf[1]), .C(A_buf[0])  , .D(EN_buf) ); // 001
    sky130_fd_sc_hd__and4bb_2   AND2 ( .X(SEL[2])  , .A_N(A_buf[2]), .B_N(A_buf[0]), .C(A_buf[1])  , .D(EN_buf) ); // 010
    sky130_fd_sc_hd__and4b_2    AND3 ( .X(SEL[3])  , .A_N(A_buf[2]), .B(A_buf[1]), .C(A_buf[0])  , .D(EN_buf) );   // 011
    sky130_fd_sc_hd__and4bb_2   AND4 ( .X(SEL[4])  , .A_N(A_buf[0]), .B_N(A_buf[1]), .C(A_buf[2])  , .D(EN_buf) ); // 100
    sky130_fd_sc_hd__and4b_2    AND5 ( .X(SEL[5])  , .A_N(A_buf[1]), .B(A_buf[0]), .C(A_buf[2])  , .D(EN_buf) );   // 101
    sky130_fd_sc_hd__and4b_2    AND6 ( .X(SEL[6])  , .A_N(A_buf[0]), .B(A_buf[1]), .C(A_buf[2])  , .D(EN_buf) );   // 110
    sky130_fd_sc_hd__and4_2     AND7 ( .X(SEL[7])  , .A(A_buf[0]), .B(A_buf[1]), .C(A_buf[2])  , .D(EN_buf) ); // 111
endmodule

module MUX4x1 #(parameter   WIDTH=32)
(
    input   wire [WIDTH-1:0]     A0, A1, A2, A3,
    input   wire [1:0]          S,
    output  wire [WIDTH-1:0]     X
);
    localparam SIZE = WIDTH/8;
    wire [SIZE-1:0] SEL0, SEL1;
    sky130_fd_sc_hd__clkbuf_2 SEL0BUF[SIZE-1:0] (.X(SEL0), .A(S[0]));
    sky130_fd_sc_hd__clkbuf_2 SEL1BUF[SIZE-1:0] (.X(SEL1), .A(S[1]));
    generate
        genvar i;
        for(i=0; i<SIZE; i=i+1) begin : M
            sky130_fd_sc_hd__mux4_1 MUX[7:0] (
                    .A0(A0[(i+1)*8-1:i*8]), 
                    .A1(A1[(i+1)*8-1:i*8]), 
                    .A2(A2[(i+1)*8-1:i*8]), 
                    .A3(A3[(i+1)*8-1:i*8]), 
                    .S0(SEL0[i]), 
                    .S1(SEL1[i]), 
                    .X(X[(i+1)*8-1:i*8]) );        
        end
    endgenerate
endmodule

module MUX2x1 #(parameter   WIDTH=32)
(
    input   wire [WIDTH-1:0]     A0, A1, A2, A3,
    input   wire           S,
    output  wire [WIDTH-1:0]     X
);
    localparam SIZE = WIDTH/8;
    wire [SIZE-1:0] SEL;
    sky130_fd_sc_hd__clkbuf_2 SELBUF[SIZE-1:0] (.X(SEL), .A(S));
    generate
        genvar i;
        for(i=0; i<SIZE; i=i+1) begin : M
            sky130_fd_sc_hd__mux2_1 MUX[7:0] (.A0(A0[(i+1)*8-1:i*8]), .A1(A1[(i+1)*8-1:i*8]), .S(SEL[i]), .X(X[(i+1)*8-1:i*8]) );
        end
    endgenerate
endmodule

module BYTE #(  parameter   USE_LATCH=1)( 
    input   wire        CLK,    // FO: 1
    input   wire        WE,     // FO: 1
    input   wire        SEL,    // FO: 2
    input   wire [7:0]  Di,     // FO: 1
    output  wire [7:0]  Do
);

    wire [7:0]  q_wire;
    wire        we_wire;
    wire        SEL_B;
    wire        GCLK;
    wire        CLK_B;

    generate 
        genvar i;

        if(USE_LATCH == 1) begin
            sky130_fd_sc_hd__inv_1 CLKINV(.Y(CLK_B), .A(CLK));
            sky130_fd_sc_hd__dlclkp_1 CG( .CLK(CLK_B), .GCLK(GCLK), .GATE(we_wire) );
        end else
            sky130_fd_sc_hd__dlclkp_1 CG( .CLK(CLK), .GCLK(GCLK), .GATE(we_wire) );
    
        sky130_fd_sc_hd__inv_1 SELINV(.Y(SEL_B), .A(SEL));
        sky130_fd_sc_hd__and2_1 CGAND( .A(SEL), .B(WE), .X(we_wire) );
    
        for(i=0; i<8; i=i+1) begin : BIT
            if(USE_LATCH == 0)
                sky130_fd_sc_hd__dfxtp_1 FF ( .D(Di[i]), .Q(q_wire[i]), .CLK(GCLK) );
            else 
                sky130_fd_sc_hd__dlxtp_1 LATCH (.Q(q_wire[i]), .D(Di[i]), .GATE(GCLK) );
            sky130_fd_sc_hd__ebufn_2 OBUF ( .A(q_wire[i]), .Z(Do[i]), .TE_B(SEL_B) );
        end
    endgenerate 
  
endmodule

module WORD #( parameter    USE_LATCH=0,
                            WSIZE=1 ) (
    input   wire                CLK,    // FO: 1
    input   wire [WSIZE-1:0]     WE,     // FO: 1
    input   wire                SEL,    // FO: 1
    input   wire [(WSIZE*8-1):0] Di,     // FO: 1
    output  wire [(WSIZE*8-1):0] Do
);

    wire SEL_buf;
    wire CLK_buf;
    sky130_fd_sc_hd__clkbuf_2 SELBUF (.X(SEL_buf), .A(SEL));
    sky130_fd_sc_hd__clkbuf_1 CLKBUF (.X(CLK_buf), .A(CLK));
    generate
        genvar i;
            for(i=0; i<WSIZE; i=i+1) begin : BYTE
                BYTE #(.USE_LATCH(USE_LATCH)) B ( .CLK(CLK_buf), .WE(WE[i]), .SEL(SEL_buf), .Di(Di[(i+1)*8-1:i*8]), .Do(Do[(i+1)*8-1:i*8]) );
            end
    endgenerate
    
endmodule 

module RAM8 #( parameter    USE_LATCH=1,
                            WSIZE=1 ) (
    input   wire                CLK,    // FO: 1
    input   wire [WSIZE-1:0]     WE,     // FO: 1
    input                       EN,     // EN: 1
    input   wire [2:0]          A,      // A: 1
    input   wire [(WSIZE*8-1):0] Di,     // FO: 1
    output  wire [(WSIZE*8-1):0] Do
);

    wire    [7:0]        SEL;
    wire    [WSIZE-1:0]   WE_buf; 
    wire                 CLK_buf;

    DEC3x8 DEC (.EN(EN), .A(A), .SEL(SEL));
    sky130_fd_sc_hd__clkbuf_2 WEBUF[WSIZE-1:0] (.X(WE_buf), .A(WE));
    sky130_fd_sc_hd__clkbuf_2 CLKBUF (.X(CLK_buf), .A(CLK));

    generate
        genvar i;
        for (i=0; i< 8; i=i+1) begin : WORD
            WORD #(.USE_LATCH(USE_LATCH), .WSIZE(WSIZE)) W ( .CLK(CLK_buf), .WE(WE_buf), .SEL(SEL[i]), .Di(Di), .Do(Do) );
        end
    endgenerate

endmodule

// 4 x RAM8 slices (128 bytes) with registered outout 
module RAM32 #( parameter   USE_LATCH=1,
                            WSIZE=1 ) 
(
    input   wire                CLK,    // FO: 1
    input   wire [WSIZE-1:0]     WE,     // FO: 1
    input                       EN,     // FO: 1
    input   wire [4:0]          A,      // FO: 1
    input   wire [(WSIZE*8-1):0] Di,     // FO: 1
    output  wire [(WSIZE*8-1):0] Do
    
);
    wire [3:0]          SEL;
    wire [4:0]          A_buf;
    wire                CLK_buf;
    wire [WSIZE-1:0]     WE_buf;
    wire                EN_buf;

    wire [(WSIZE*8-1):0] Do_pre;
    wire [(WSIZE*8-1):0] Di_buf;

    // Buffers
    // Di Buffers
    sky130_fd_sc_hd__clkbuf_16  DIBUF[(WSIZE*8-1):0] (.X(Di_buf), .A(Di));
    // Control signals buffers
    sky130_fd_sc_hd__clkbuf_2   CLKBUF              (.X(CLK_buf), .A(CLK));
    sky130_fd_sc_hd__clkbuf_2   WEBUF[(WSIZE-1):0]   (.X(WE_buf), .A(WE));
    sky130_fd_sc_hd__clkbuf_2   ABUF[4:0]           (.X(A_buf), .A(A[4:0]));
    sky130_fd_sc_hd__clkbuf_2   ENBUF               (.X(EN_buf), .A(EN));

    DEC2x4 DEC (.EN(EN_buf), .A(A_buf[4:3]), .SEL(SEL));

    generate
        genvar i;
        for (i=0; i< 4; i=i+1) begin : SLICE
            RAM8 #(.USE_LATCH(USE_LATCH), .WSIZE(WSIZE)) RAM8 (.CLK(CLK_buf), .WE(WE_buf),.EN(SEL[i]), .Di(Di_buf), .Do(Do_pre), .A(A_buf[2:0]) ); 
        end
    endgenerate

    // Ensure that the Do_pre lines are not floating when EN = 0
    wire [WSIZE-1:0] lo;
    wire [WSIZE-1:0] float_buf_en;
    sky130_fd_sc_hd__clkbuf_2   FBUFENBUF[WSIZE-1:0] ( .X(float_buf_en), .A(EN) );
    sky130_fd_sc_hd__conb_1     TIE[WSIZE-1:0] (.LO(lo), .HI());

    // Following split by group because each is done by one TIE CELL and ONE CLKINV_4
    // Provides default values for floating lines (lo)
    generate
        for (i=0; i< WSIZE; i=i+1) begin : BYTE
            sky130_fd_sc_hd__ebufn_2 FLOATBUF[(8*(i+1))-1:8*i] ( .A( lo[i] ), .Z(Do_pre[(8*(i+1))-1:8*i]), .TE_B(float_buf_en[i]) );        
        end
    endgenerate
    
    sky130_fd_sc_hd__dfxtp_1 Do_FF[WSIZE*8-1:0] ( .D(Do_pre), .Q(Do), .CLK(CLK) );

endmodule

/*
    4 x RAM32 Blocks
*/

module RAM128 #(parameter   USE_LATCH=1,
                            WSIZE=1 ) 
(
    input   wire                CLK,    // FO: 1
    input   wire [WSIZE-1:0]     WE,     // FO: 1
    input                       EN,     // FO: 1
    input   wire [6:0]          A,      // FO: 1
    input   wire [(WSIZE*8-1):0] Di,     // FO: 1
    output  wire [(WSIZE*8-1):0] Do
);

    wire                        CLK_buf;
    wire [WSIZE-1:0]            WE_buf;
    wire                        EN_buf;
    wire [6:0]                  A_buf;
    wire [(WSIZE*8-1):0]        Di_buf;
    wire [3:0]                  SEL;
    wire [(WSIZE*8-1):0]        Do_pre[3:0]; 
                            
    // Buffers
    sky130_fd_sc_hd__clkbuf_16  DIBUF[(WSIZE*8-1):0] (.X(Di_buf),  .A(Di));
    sky130_fd_sc_hd__clkbuf_4   CLKBUF              (.X(CLK_buf), .A(CLK));
    sky130_fd_sc_hd__clkbuf_2   WEBUF[WSIZE-1:0]     (.X(WE_buf),  .A(WE));
    sky130_fd_sc_hd__clkbuf_2   ENBUF               (.X(EN_buf),  .A(EN));
    sky130_fd_sc_hd__clkbuf_2   ABUF[6:0]           (.X(A_buf),   .A(A));

    DEC2x4 DEC (.EN(EN_buf), .A(A_buf[6:5]), .SEL(SEL));

     generate
        genvar i;
        for (i=0; i< 4; i=i+1) begin : BLOCK
            RAM32 #(.USE_LATCH(USE_LATCH), .WSIZE(WSIZE)) RAM32 (.CLK(CLK_buf), .EN(SEL[i]), .WE(WE_buf), .Di(Di_buf), .Do(Do_pre[i]), .A(A_buf[4:0]) );        
        end
     endgenerate

    // Output MUX    
    MUX4x1 #(.WIDTH(WSIZE*8)) DoMUX ( .A0(Do_pre[0]), .A1(Do_pre[1]), .A2(Do_pre[2]), .A3(Do_pre[3]), .S(A_buf[6:5]), .X(Do) );

endmodule

module RAM256 #(parameter   USE_LATCH=1,
                            WSIZE=1 ) 
(
    input   wire                    CLK,    // FO: 2
    input   wire [WSIZE-1:0]        WE,     // FO: 2
    input                           EN,     // FO: 2
    input   wire [7:0]              A,      // FO: 5
    input   wire [(WSIZE*8-1):0]    Di,     // FO: 2
    output  wire [(WSIZE*8-1):0]    Do

);

    wire [1:0]              SEL;
    wire [(WSIZE*8-1):0]    Do_pre[1:0]; 

    // 1x2 DEC
    DEC1x2 DEC (.EN(EN), .A(A[7]), .SEL(SEL));

    generate
        genvar i;
        for (i=0; i< 2; i=i+1) begin : BLOCK
            RAM128 #(.USE_LATCH(USE_LATCH), .WSIZE(WSIZE)) RAM128 (.CLK(CLK), .EN(SEL[i]), .WE(WE), .Di(Di), .Do(Do_pre[i]), .A(A[6:0]) );        
        end
     endgenerate

    // Output MUX    
    MUX2x1 #(.WIDTH(WSIZE*8)) DoMUX ( .A0(Do_pre[0]), .A1(Do_pre[1]), .S(A[7]), .X(Do) );

endmodule

module RAM512 #(parameter   USE_LATCH=1,
                            WSIZE=1 ) 
(
    input   wire                    CLK,    // FO: 4
    input   wire [WSIZE-1:0]        WE,     // FO: 4
    input                           EN,     // FO: 4
    input   wire [8:0]              A,      // FO: 5
    input   wire [(WSIZE*8-1):0]    Di,     // FO: 4
    output  wire [(WSIZE*8-1):0]    Do 
);

    wire [3:0]              SEL;
    wire [(WSIZE*8-1):0]    Do_pre[3:0]; 

    DEC2x4 DEC (.EN(EN), .A(A[8:7]), .SEL(SEL));

    generate
        genvar i;
        for (i=0; i< 4; i=i+1) begin : BLOCK
            RAM128 #(.USE_LATCH(USE_LATCH), .WSIZE(WSIZE)) RAM128 (.CLK(CLK), .EN(SEL[i]), .WE(WE), .Di(Di), .Do(Do_pre[i]), .A(A[6:0]) );        
        end
     endgenerate

    // Output MUX    
    MUX4x1 #(.WIDTH(WSIZE*8)) DoMUX ( .A0(Do_pre[0]), .A1(Do_pre[1]), .A2(Do_pre[2]), .A3(Do_pre[3]), .S(A[8:7]), .X(Do) );

endmodule


module DFFRAM #(parameter  USE_LATCH=1,
                            WSIZE=1 ) 
(
    input   wire                    CLK,    // FO: 1
    input   wire [WSIZE-1:0]        WE,     // FO: 1
    input                           EN,     // FO: 1
    input   wire [9:0]              A,      // FO: 1
    input   wire [(WSIZE*8-1):0]    Di,     // FO: 1
    output  wire [(WSIZE*8-1):0]    Do  
);

    wire                    CLK_buf;
    wire [WSIZE-1:0]        WE_buf;
    wire                    EN_buf;
    wire [9:0]              A_buf;
    wire [(WSIZE*8-1):0]    Di_buf;
    wire [1:0]              SEL;
    wire [(WSIZE*8-1):0]    Do_pre[1:0]; 
                            
    // Buffers
    sky130_fd_sc_hd__clkbuf_16  DIBUF[(WSIZE*8-1):0] (.X(Di_buf),  .A(Di));
    sky130_fd_sc_hd__clkbuf_4   CLKBUF              (.X(CLK_buf), .A(CLK));
    sky130_fd_sc_hd__clkbuf_2   WEBUF[WSIZE-1:0]     (.X(WE_buf),  .A(WE));
    sky130_fd_sc_hd__clkbuf_2   ENBUF               (.X(EN_buf),  .A(EN));
    sky130_fd_sc_hd__clkbuf_2   ABUF[9:0]           (.X(A_buf),   .A(A));

    // 1x2 DEC
    DEC1x2 DEC (.EN(EN_buf), .A(A[9]), .SEL(SEL));

     generate
        genvar i;
        for (i=0; i< 2; i=i+1) begin : BLOCK
            RAM512 #(.USE_LATCH(USE_LATCH), .WSIZE(WSIZE)) RAM32 (.CLK(CLK_buf), .EN(SEL[i]), .WE(WE_buf), .Di(Di_buf), .Do(Do_pre[i]), .A(A_buf[8:0]) );        
        end
     endgenerate

    // Output MUX    
    MUX2x1 #(.WIDTH(WSIZE*8)) DoMUX ( .A0(Do_pre[0]), .A1(Do_pre[1]), .S(A_buf[9]), .X(Do) );
    

endmodule

`endif
