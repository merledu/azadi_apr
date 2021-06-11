
// basic reset managemnet logic for azadi

module rstmgr(

    input logic clk_i, //system clock
    input logic rst_ni, // system reset
    input logic prog_rst_ni,
  
    input  logic  ndmreset, // non-debug module reset
    output logic  sys_rst_ni // reset for system except debug module
);


  always_comb begin
    if(!rst_ni) begin
      sys_rst_ni = 1'b0;
    end else begin 
    	if(!prog_rst_ni) begin
      	   sys_rst_ni = 1'b0;
    	end else begin
	   if(ndmreset)begin
      	     sys_rst_ni = 1'b0;
    	   end else begin
      	     sys_rst_ni = prog_rst_ni;
    	   end
	end
    end
  end
  

endmodule
