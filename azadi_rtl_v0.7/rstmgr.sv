
// basic reset managemnet logic for azadi

module rstmgr(

    input logic clk_i, //system clock
    input logic rst_ni, // system reset
    input logic prog_rst_ni,
  
    input  logic  ndmreset, // non-debug module reset
    output logic  sys_rst_ni // reset for system except debug module
);

  logic rst_d, rst_q;
  logic rst_fd, rst_fq; // follower flip flop
  
  always_comb begin
    if(!rst_ni) begin
      rst_d = 1'b0;
    end else 
    if(ndmreset) begin
      rst_d = 1'b0;
    end else 
    if(!prog_rst_ni)begin
      rst_d = 1'b0;
    end else begin
      rst_d = 1'b1;
    end
  end
  
  always_ff @(posedge clk_i ) begin
   if(~rst_ni) begin
      rst_q <= 1'b0;
   end else begin
      rst_q <= rst_d;
   end
    
  end

  assign rst_fd = rst_q;
  always_ff @(posedge clk_i ) begin
    if(~rst_ni) begin
      rst_fq <= 1'b0;
    end else begin
      rst_fq <= rst_fd;
    end 
  end

  assign sys_rst_ni = rst_fq;

endmodule
