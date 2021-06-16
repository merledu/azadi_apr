
// basic reset managemnet logic for azadi

module rstmgr(

    input clk_i, //system clock
    input rst_ni, // system reset
    input prog_rst_ni,
  
    output logic  sys_rst_ni // reset for system except debug module
);
  typedef enum logic [1:0] {
  RESET, IDLE, PROG, RUN
} rst_fsm_e;

rst_fsm_e rst_fsm_cs, rst_fsm_ns;
logic rst_run_d, rst_run_q;

always_comb begin : comb_part
  // default values
  rst_fsm_ns  = rst_fsm_cs;
  sys_rst_ni  = 1'b0;

  unique case (rst_fsm_cs)
    RESET: begin
      sys_rst_ni   = 1'b0;
      rst_fsm_ns   = IDLE;
    end
    IDLE: begin
      sys_rst_ni   = 1'b0;
      rst_run_d    = 1'b0;
      if (rst_run_q) begin 
        rst_fsm_ns   = RUN;
      end else if (!prog_rst_ni) begin
        rst_fsm_ns = PROG;
      end else begin
        rst_fsm_ns = IDLE;
      end
    end
    PROG: begin
      sys_rst_ni  = 1'b0;
      rst_run_d   = 1'b0;
      if (!prog_rst_ni) begin
        rst_fsm_ns = PROG;
      end else begin
        rst_fsm_ns = RUN;
      end
    end
    RUN: begin
      sys_rst_ni  = 1'b1;
      rst_run_d   = 1'b0;
      if (!rst_ni) begin
	rst_run_d   = 1'b1;
	rst_fsm_ns  = RESET;
      end else if (!prog_rst_ni) begin
	rst_fsm_ns  = PROG;
      end else begin
	rst_fsm_ns  = RUN;
      end
    end
    default: begin
      rst_fsm_ns  = rst_fsm_cs;
      sys_rst_ni  = 1'b0;
      rst_run_d   = 1'b0;
    end
  endcase 
end

always_ff @(posedge clk_i or negedge rst_ni) begin : seq_part 
  if (!rst_ni) begin
    rst_fsm_cs  <= RESET;
    rst_run_q   <= 1'b0;
  end else begin
    rst_fsm_cs  <= rst_fsm_ns;
    rst_run_q   <= rst_run_d;
  end
end

endmodule
