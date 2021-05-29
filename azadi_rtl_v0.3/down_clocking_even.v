/*Down clocking module
Output clock frequency is the original frequency divided by an even number
*/
module	down_clocking_even(
input	clk_i,
input	rst_ni,
input	[15:0]i_divisor,
output	o_clk
);

wire	[15:0]divisor;
wire	borrow;

minus_one	minus_one_0(
.i_operand(i_divisor),
.o_result(divisor),
.o_borrow(borrow)
);

wire	go;
assign	go=((i_divisor!=0)&&rst_ni);
reg	[15:0]ct;
reg	clk;
always@(posedge clk_i )
	if(!rst_ni)begin
		ct<=0;
		clk<=0;
	end
	else if(go)begin
		if(ct>=divisor)begin
			ct<=0;
			clk<=~clk;
		end
		else ct<=ct+1;
	end
assign	o_clk=go?clk:clk_i;
endmodule