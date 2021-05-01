
// basic rst_ni managemnet logic for azadi

module rstmgr(

    input clk_i, //system clk_i
    input rst_ni, // system rst_ni

    input  logic  ndmreset, // non-debug module rst_ni
    output logic  sys_rst_ni // rst_ni for system except debug module
);

  logic rst_d, rst_q;
  logic rst_fd, rst_fq; // follower flip flop
  
  always_comb begin
    if(!rst_ni) begin
      rst_d = 1'b0;
    end else 
      if(ndmreset) begin
        rst_d = 1'b0;
      end else begin
        rst_d = 1'b1;
      end
  end
  
  always_ff @(posedge clk_i ) begin
    rst_q <= rst_d;
  end

  assign rst_fd = rst_q;
  always_ff @(posedge clk_i ) begin
    rst_fq <= rst_fd;
  end

  assign sys_rst_ni = rst_fq;

endmodule