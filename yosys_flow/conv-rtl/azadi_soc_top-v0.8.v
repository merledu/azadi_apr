module control_mvp (
	Clk_CI,
	Rst_RBI,
	Div_start_SI,
	Sqrt_start_SI,
	Start_SI,
	Kill_SI,
	Special_case_SBI,
	Special_case_dly_SBI,
	Precision_ctl_SI,
	Format_sel_SI,
	Numerator_DI,
	Exp_num_DI,
	Denominator_DI,
	Exp_den_DI,
	Div_start_dly_SO,
	Sqrt_start_dly_SO,
	Div_enable_SO,
	Sqrt_enable_SO,
	Full_precision_SO,
	FP32_SO,
	FP64_SO,
	FP16_SO,
	FP16ALT_SO,
	Ready_SO,
	Done_SO,
	Mant_result_prenorm_DO,
	Exp_result_prenorm_DO
);
	input wire Clk_CI;
	input wire Rst_RBI;
	input wire Div_start_SI;
	input wire Sqrt_start_SI;
	input wire Start_SI;
	input wire Kill_SI;
	input wire Special_case_SBI;
	input wire Special_case_dly_SBI;
	input wire [C_PC - 1:0] Precision_ctl_SI;
	input wire [1:0] Format_sel_SI;
	input wire [C_MANT_FP64:0] Numerator_DI;
	input wire [C_EXP_FP64:0] Exp_num_DI;
	input wire [C_MANT_FP64:0] Denominator_DI;
	input wire [C_EXP_FP64:0] Exp_den_DI;
	output wire Div_start_dly_SO;
	output wire Sqrt_start_dly_SO;
	output reg Div_enable_SO;
	output reg Sqrt_enable_SO;
	output wire Full_precision_SO;
	output wire FP32_SO;
	output wire FP64_SO;
	output wire FP16_SO;
	output wire FP16ALT_SO;
	output reg Ready_SO;
	output reg Done_SO;
	output reg [C_MANT_FP64 + 4:0] Mant_result_prenorm_DO;
	output wire [C_EXP_FP64 + 1:0] Exp_result_prenorm_DO;
	reg [C_MANT_FP64 + 5:0] Partial_remainder_DN;
	reg [C_MANT_FP64 + 5:0] Partial_remainder_DP;
	reg [C_MANT_FP64 + 4:0] Quotient_DP;
	wire [C_MANT_FP64 + 1:0] Numerator_se_D;
	wire [C_MANT_FP64 + 1:0] Denominator_se_D;
	reg [C_MANT_FP64 + 1:0] Denominator_se_DB;
	assign Numerator_se_D = {1'b0, Numerator_DI};
	assign Denominator_se_D = {1'b0, Denominator_DI};
	always @(*)
		if (FP32_SO)
			Denominator_se_DB = {~Denominator_se_D[C_MANT_FP64 + 1:C_MANT_FP64 - C_MANT_FP32], {C_MANT_FP64 - C_MANT_FP32 {1'b0}}};
		else if (FP64_SO)
			Denominator_se_DB = ~Denominator_se_D;
		else if (FP16_SO)
			Denominator_se_DB = {~Denominator_se_D[C_MANT_FP64 + 1:C_MANT_FP64 - C_MANT_FP16], {C_MANT_FP64 - C_MANT_FP16 {1'b0}}};
		else
			Denominator_se_DB = {~Denominator_se_D[C_MANT_FP64 + 1:C_MANT_FP64 - C_MANT_FP16ALT], {C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}};
	wire [C_MANT_FP64 + 1:0] Mant_D_sqrt_Norm;
	assign Mant_D_sqrt_Norm = (Exp_num_DI[0] ? {1'b0, Numerator_DI} : {Numerator_DI, 1'b0});
	reg [1:0] Format_sel_S;
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Format_sel_S <= 'b0;
		else if (Start_SI && Ready_SO)
			Format_sel_S <= Format_sel_SI;
		else
			Format_sel_S <= Format_sel_S;
	assign FP32_SO = Format_sel_S == 2'b00;
	assign FP64_SO = Format_sel_S == 2'b01;
	assign FP16_SO = Format_sel_S == 2'b10;
	assign FP16ALT_SO = Format_sel_S == 2'b11;
	reg [C_PC - 1:0] Precision_ctl_S;
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Precision_ctl_S <= 'b0;
		else if (Start_SI && Ready_SO)
			Precision_ctl_S <= Precision_ctl_SI;
		else
			Precision_ctl_S <= Precision_ctl_S;
	assign Full_precision_SO = Precision_ctl_S == 6'h00;
	reg [5:0] State_ctl_S;
	wire [5:0] State_Two_iteration_unit_S;
	wire [5:0] State_Four_iteration_unit_S;
	assign State_Two_iteration_unit_S = Precision_ctl_S[C_PC - 1:1];
	assign State_Four_iteration_unit_S = Precision_ctl_S[C_PC - 1:2];
	always @(*)
		case (Iteration_unit_num_S)
			2'b00:
				case (Format_sel_S)
					2'b00:
						if (Full_precision_SO)
							State_ctl_S = 6'h1b;
						else
							State_ctl_S = Precision_ctl_S;
					2'b01:
						if (Full_precision_SO)
							State_ctl_S = 6'h38;
						else
							State_ctl_S = Precision_ctl_S;
					2'b10:
						if (Full_precision_SO)
							State_ctl_S = 6'h0e;
						else
							State_ctl_S = Precision_ctl_S;
					2'b11:
						if (Full_precision_SO)
							State_ctl_S = 6'h0b;
						else
							State_ctl_S = Precision_ctl_S;
				endcase
			2'b01:
				case (Format_sel_S)
					2'b00:
						if (Full_precision_SO)
							State_ctl_S = 6'h0d;
						else
							State_ctl_S = State_Two_iteration_unit_S;
					2'b01:
						if (Full_precision_SO)
							State_ctl_S = 6'h1b;
						else
							State_ctl_S = State_Two_iteration_unit_S;
					2'b10:
						if (Full_precision_SO)
							State_ctl_S = 6'h06;
						else
							State_ctl_S = State_Two_iteration_unit_S;
					2'b11:
						if (Full_precision_SO)
							State_ctl_S = 6'h05;
						else
							State_ctl_S = State_Two_iteration_unit_S;
				endcase
			2'b10:
				case (Format_sel_S)
					2'b00:
						case (Precision_ctl_S)
							6'h00: State_ctl_S = 6'h08;
							6'h06, 6'h07, 6'h08: State_ctl_S = 6'h02;
							6'h09, 6'h0a, 6'h0b: State_ctl_S = 6'h03;
							6'h0c, 6'h0d, 6'h0e: State_ctl_S = 6'h04;
							6'h0f, 6'h10, 6'h11: State_ctl_S = 6'h05;
							6'h12, 6'h13, 6'h14: State_ctl_S = 6'h06;
							6'h15, 6'h16, 6'h17: State_ctl_S = 6'h07;
							default: State_ctl_S = 6'h08;
						endcase
					2'b01:
						case (Precision_ctl_S)
							6'h00: State_ctl_S = 6'h12;
							6'h06, 6'h07, 6'h08: State_ctl_S = 6'h02;
							6'h09, 6'h0a, 6'h0b: State_ctl_S = 6'h03;
							6'h0c, 6'h0d, 6'h0e: State_ctl_S = 6'h04;
							6'h0f, 6'h10, 6'h11: State_ctl_S = 6'h05;
							6'h12, 6'h13, 6'h14: State_ctl_S = 6'h06;
							6'h15, 6'h16, 6'h17: State_ctl_S = 6'h07;
							6'h18, 6'h19, 6'h1a: State_ctl_S = 6'h08;
							6'h1b, 6'h1c, 6'h1d: State_ctl_S = 6'h09;
							6'h1e, 6'h1f, 6'h20: State_ctl_S = 6'h0a;
							6'h21, 6'h22, 6'h23: State_ctl_S = 6'h0b;
							6'h24, 6'h25, 6'h26: State_ctl_S = 6'h0c;
							6'h27, 6'h28, 6'h29: State_ctl_S = 6'h0d;
							6'h2a, 6'h2b, 6'h2c: State_ctl_S = 6'h0e;
							6'h2d, 6'h2e, 6'h2f: State_ctl_S = 6'h0f;
							6'h30, 6'h31, 6'h32: State_ctl_S = 6'h10;
							6'h33, 6'h34, 6'h35: State_ctl_S = 6'h11;
							default: State_ctl_S = 6'h12;
						endcase
					2'b10:
						case (Precision_ctl_S)
							6'h00: State_ctl_S = 6'h04;
							6'h06, 6'h07, 6'h08: State_ctl_S = 6'h02;
							6'h09, 6'h0a, 6'h0b: State_ctl_S = 6'h03;
							default: State_ctl_S = 6'h04;
						endcase
					2'b11:
						case (Precision_ctl_S)
							6'h00: State_ctl_S = 6'h03;
							6'h06, 6'h07, 6'h08: State_ctl_S = 6'h02;
							default: State_ctl_S = 6'h03;
						endcase
				endcase
			2'b11:
				case (Format_sel_S)
					2'b00:
						if (Full_precision_SO)
							State_ctl_S = 6'h06;
						else
							State_ctl_S = State_Four_iteration_unit_S;
					2'b01:
						if (Full_precision_SO)
							State_ctl_S = 6'h0d;
						else
							State_ctl_S = State_Four_iteration_unit_S;
					2'b10:
						if (Full_precision_SO)
							State_ctl_S = 6'h03;
						else
							State_ctl_S = State_Four_iteration_unit_S;
					2'b11:
						if (Full_precision_SO)
							State_ctl_S = 6'h02;
						else
							State_ctl_S = State_Four_iteration_unit_S;
				endcase
		endcase
	reg Div_start_dly_S;
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Div_start_dly_S <= 1'b0;
		else if (Div_start_SI && Ready_SO)
			Div_start_dly_S <= 1'b1;
		else
			Div_start_dly_S <= 1'b0;
	assign Div_start_dly_SO = Div_start_dly_S;
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Div_enable_SO <= 1'b0;
		else if (Kill_SI)
			Div_enable_SO <= 1'b0;
		else if (Div_start_SI && Ready_SO)
			Div_enable_SO <= 1'b1;
		else if (Done_SO)
			Div_enable_SO <= 1'b0;
		else
			Div_enable_SO <= Div_enable_SO;
	reg Sqrt_start_dly_S;
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Sqrt_start_dly_S <= 1'b0;
		else if (Sqrt_start_SI && Ready_SO)
			Sqrt_start_dly_S <= 1'b1;
		else
			Sqrt_start_dly_S <= 1'b0;
	assign Sqrt_start_dly_SO = Sqrt_start_dly_S;
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Sqrt_enable_SO <= 1'b0;
		else if (Kill_SI)
			Sqrt_enable_SO <= 1'b0;
		else if (Sqrt_start_SI && Ready_SO)
			Sqrt_enable_SO <= 1'b1;
		else if (Done_SO)
			Sqrt_enable_SO <= 1'b0;
		else
			Sqrt_enable_SO <= Sqrt_enable_SO;
	reg [5:0] Crtl_cnt_S;
	wire Start_dly_S;
	assign Start_dly_S = Div_start_dly_S | Sqrt_start_dly_S;
	wire Fsm_enable_S;
	assign Fsm_enable_S = ((Start_dly_S | |Crtl_cnt_S) && ~Kill_SI) && Special_case_dly_SBI;
	wire Final_state_S;
	assign Final_state_S = Crtl_cnt_S == State_ctl_S;
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Crtl_cnt_S <= {6 {1'sb0}};
		else if (Final_state_S | Kill_SI)
			Crtl_cnt_S <= {6 {1'sb0}};
		else if (Fsm_enable_S)
			Crtl_cnt_S <= Crtl_cnt_S + 1;
		else
			Crtl_cnt_S <= {6 {1'sb0}};
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Done_SO <= 1'b0;
		else if (Start_SI && Ready_SO) begin
			if (~Special_case_SBI)
				Done_SO <= 1'b1;
			else
				Done_SO <= 1'b0;
		end
		else if (Final_state_S)
			Done_SO <= 1'b1;
		else
			Done_SO <= 1'b0;
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Ready_SO <= 1'b1;
		else if (Start_SI && Ready_SO) begin
			if (~Special_case_SBI)
				Ready_SO <= 1'b1;
			else
				Ready_SO <= 1'b0;
		end
		else if (Final_state_S | Kill_SI)
			Ready_SO <= 1'b1;
		else
			Ready_SO <= Ready_SO;
	wire Qcnt_one_0;
	wire Qcnt_one_1;
	wire [1:0] Qcnt_one_2;
	wire [2:0] Qcnt_one_3;
	wire [3:0] Qcnt_one_4;
	wire [4:0] Qcnt_one_5;
	wire [5:0] Qcnt_one_6;
	wire [6:0] Qcnt_one_7;
	wire [7:0] Qcnt_one_8;
	wire [8:0] Qcnt_one_9;
	wire [9:0] Qcnt_one_10;
	wire [10:0] Qcnt_one_11;
	wire [11:0] Qcnt_one_12;
	wire [12:0] Qcnt_one_13;
	wire [13:0] Qcnt_one_14;
	wire [14:0] Qcnt_one_15;
	wire [15:0] Qcnt_one_16;
	wire [16:0] Qcnt_one_17;
	wire [17:0] Qcnt_one_18;
	wire [18:0] Qcnt_one_19;
	wire [19:0] Qcnt_one_20;
	wire [20:0] Qcnt_one_21;
	wire [21:0] Qcnt_one_22;
	wire [22:0] Qcnt_one_23;
	wire [23:0] Qcnt_one_24;
	wire [24:0] Qcnt_one_25;
	wire [25:0] Qcnt_one_26;
	wire [26:0] Qcnt_one_27;
	wire [27:0] Qcnt_one_28;
	wire [28:0] Qcnt_one_29;
	wire [29:0] Qcnt_one_30;
	wire [30:0] Qcnt_one_31;
	wire [31:0] Qcnt_one_32;
	wire [32:0] Qcnt_one_33;
	wire [33:0] Qcnt_one_34;
	wire [34:0] Qcnt_one_35;
	wire [35:0] Qcnt_one_36;
	wire [36:0] Qcnt_one_37;
	wire [37:0] Qcnt_one_38;
	wire [38:0] Qcnt_one_39;
	wire [39:0] Qcnt_one_40;
	wire [40:0] Qcnt_one_41;
	wire [41:0] Qcnt_one_42;
	wire [42:0] Qcnt_one_43;
	wire [43:0] Qcnt_one_44;
	wire [44:0] Qcnt_one_45;
	wire [45:0] Qcnt_one_46;
	wire [46:0] Qcnt_one_47;
	wire [47:0] Qcnt_one_48;
	wire [48:0] Qcnt_one_49;
	wire [49:0] Qcnt_one_50;
	wire [50:0] Qcnt_one_51;
	wire [51:0] Qcnt_one_52;
	wire [52:0] Qcnt_one_53;
	wire [53:0] Qcnt_one_54;
	wire [54:0] Qcnt_one_55;
	wire [55:0] Qcnt_one_56;
	wire [56:0] Qcnt_one_57;
	wire [57:0] Qcnt_one_58;
	wire [58:0] Qcnt_one_59;
	wire [59:0] Qcnt_one_60;
	wire [1:0] Qcnt_two_0;
	wire [2:0] Qcnt_two_1;
	wire [4:0] Qcnt_two_2;
	wire [6:0] Qcnt_two_3;
	wire [8:0] Qcnt_two_4;
	wire [10:0] Qcnt_two_5;
	wire [12:0] Qcnt_two_6;
	wire [14:0] Qcnt_two_7;
	wire [16:0] Qcnt_two_8;
	wire [18:0] Qcnt_two_9;
	wire [20:0] Qcnt_two_10;
	wire [22:0] Qcnt_two_11;
	wire [24:0] Qcnt_two_12;
	wire [26:0] Qcnt_two_13;
	wire [28:0] Qcnt_two_14;
	wire [30:0] Qcnt_two_15;
	wire [32:0] Qcnt_two_16;
	wire [34:0] Qcnt_two_17;
	wire [36:0] Qcnt_two_18;
	wire [38:0] Qcnt_two_19;
	wire [40:0] Qcnt_two_20;
	wire [42:0] Qcnt_two_21;
	wire [44:0] Qcnt_two_22;
	wire [46:0] Qcnt_two_23;
	wire [48:0] Qcnt_two_24;
	wire [50:0] Qcnt_two_25;
	wire [52:0] Qcnt_two_26;
	wire [54:0] Qcnt_two_27;
	wire [56:0] Qcnt_two_28;
	wire [2:0] Qcnt_three_0;
	wire [4:0] Qcnt_three_1;
	wire [7:0] Qcnt_three_2;
	wire [10:0] Qcnt_three_3;
	wire [13:0] Qcnt_three_4;
	wire [16:0] Qcnt_three_5;
	wire [19:0] Qcnt_three_6;
	wire [22:0] Qcnt_three_7;
	wire [25:0] Qcnt_three_8;
	wire [28:0] Qcnt_three_9;
	wire [31:0] Qcnt_three_10;
	wire [34:0] Qcnt_three_11;
	wire [37:0] Qcnt_three_12;
	wire [40:0] Qcnt_three_13;
	wire [43:0] Qcnt_three_14;
	wire [46:0] Qcnt_three_15;
	wire [49:0] Qcnt_three_16;
	wire [52:0] Qcnt_three_17;
	wire [55:0] Qcnt_three_18;
	wire [58:0] Qcnt_three_19;
	wire [61:0] Qcnt_three_20;
	wire [3:0] Qcnt_four_0;
	wire [6:0] Qcnt_four_1;
	wire [10:0] Qcnt_four_2;
	wire [14:0] Qcnt_four_3;
	wire [18:0] Qcnt_four_4;
	wire [22:0] Qcnt_four_5;
	wire [26:0] Qcnt_four_6;
	wire [30:0] Qcnt_four_7;
	wire [34:0] Qcnt_four_8;
	wire [38:0] Qcnt_four_9;
	wire [42:0] Qcnt_four_10;
	wire [46:0] Qcnt_four_11;
	wire [50:0] Qcnt_four_12;
	wire [54:0] Qcnt_four_13;
	wire [58:0] Qcnt_four_14;
	wire [C_MANT_FP64 + 5:0] Sqrt_R0;
	reg [C_MANT_FP64 + 5:0] Sqrt_Q0;
	reg [C_MANT_FP64 + 5:0] Q_sqrt0;
	reg [C_MANT_FP64 + 5:0] Q_sqrt_com_0;
	wire [C_MANT_FP64 + 5:0] Sqrt_R1;
	reg [C_MANT_FP64 + 5:0] Sqrt_Q1;
	reg [C_MANT_FP64 + 5:0] Q_sqrt1;
	reg [C_MANT_FP64 + 5:0] Q_sqrt_com_1;
	wire [C_MANT_FP64 + 5:0] Sqrt_R2;
	reg [C_MANT_FP64 + 5:0] Sqrt_Q2;
	reg [C_MANT_FP64 + 5:0] Q_sqrt2;
	reg [C_MANT_FP64 + 5:0] Q_sqrt_com_2;
	wire [C_MANT_FP64 + 5:0] Sqrt_R3;
	reg [C_MANT_FP64 + 5:0] Sqrt_Q3;
	reg [C_MANT_FP64 + 5:0] Q_sqrt3;
	reg [C_MANT_FP64 + 5:0] Q_sqrt_com_3;
	wire [C_MANT_FP64 + 5:0] Sqrt_R4;
	reg [1:0] Sqrt_DI [3:0];
	wire [1:0] Sqrt_DO [3:0];
	wire Sqrt_carry_DO;
	wire [C_MANT_FP64 + 5:0] Iteration_cell_a_D [3:0];
	wire [C_MANT_FP64 + 5:0] Iteration_cell_b_D [3:0];
	wire [C_MANT_FP64 + 5:0] Iteration_cell_a_BMASK_D [3:0];
	wire [C_MANT_FP64 + 5:0] Iteration_cell_b_BMASK_D [3:0];
	wire Iteration_cell_carry_D [3:0];
	wire [C_MANT_FP64 + 5:0] Iteration_cell_sum_D [3:0];
	wire [C_MANT_FP64 + 5:0] Iteration_cell_sum_AMASK_D [3:0];
	reg [3:0] Sqrt_quotinent_S;
	always @(*)
		case (Format_sel_S)
			2'b00: begin
				Sqrt_quotinent_S = {~Iteration_cell_sum_AMASK_D[0][C_MANT_FP32 + 5], ~Iteration_cell_sum_AMASK_D[1][C_MANT_FP32 + 5], ~Iteration_cell_sum_AMASK_D[2][C_MANT_FP32 + 5], ~Iteration_cell_sum_AMASK_D[3][C_MANT_FP32 + 5]};
				Q_sqrt_com_0 = {{C_MANT_FP64 - C_MANT_FP32 {1'b0}}, ~Q_sqrt0[C_MANT_FP32 + 5:0]};
				Q_sqrt_com_1 = {{C_MANT_FP64 - C_MANT_FP32 {1'b0}}, ~Q_sqrt1[C_MANT_FP32 + 5:0]};
				Q_sqrt_com_2 = {{C_MANT_FP64 - C_MANT_FP32 {1'b0}}, ~Q_sqrt2[C_MANT_FP32 + 5:0]};
				Q_sqrt_com_3 = {{C_MANT_FP64 - C_MANT_FP32 {1'b0}}, ~Q_sqrt3[C_MANT_FP32 + 5:0]};
			end
			2'b01: begin
				Sqrt_quotinent_S = {Iteration_cell_carry_D[0], Iteration_cell_carry_D[1], Iteration_cell_carry_D[2], Iteration_cell_carry_D[3]};
				Q_sqrt_com_0 = ~Q_sqrt0;
				Q_sqrt_com_1 = ~Q_sqrt1;
				Q_sqrt_com_2 = ~Q_sqrt2;
				Q_sqrt_com_3 = ~Q_sqrt3;
			end
			2'b10: begin
				Sqrt_quotinent_S = {~Iteration_cell_sum_AMASK_D[0][C_MANT_FP16 + 5], ~Iteration_cell_sum_AMASK_D[1][C_MANT_FP16 + 5], ~Iteration_cell_sum_AMASK_D[2][C_MANT_FP16 + 5], ~Iteration_cell_sum_AMASK_D[3][C_MANT_FP16 + 5]};
				Q_sqrt_com_0 = {{C_MANT_FP64 - C_MANT_FP16 {1'b0}}, ~Q_sqrt0[C_MANT_FP16 + 5:0]};
				Q_sqrt_com_1 = {{C_MANT_FP64 - C_MANT_FP16 {1'b0}}, ~Q_sqrt1[C_MANT_FP16 + 5:0]};
				Q_sqrt_com_2 = {{C_MANT_FP64 - C_MANT_FP16 {1'b0}}, ~Q_sqrt2[C_MANT_FP16 + 5:0]};
				Q_sqrt_com_3 = {{C_MANT_FP64 - C_MANT_FP16 {1'b0}}, ~Q_sqrt3[C_MANT_FP16 + 5:0]};
			end
			2'b11: begin
				Sqrt_quotinent_S = {~Iteration_cell_sum_AMASK_D[0][C_MANT_FP16ALT + 5], ~Iteration_cell_sum_AMASK_D[1][C_MANT_FP16ALT + 5], ~Iteration_cell_sum_AMASK_D[2][C_MANT_FP16ALT + 5], ~Iteration_cell_sum_AMASK_D[3][C_MANT_FP16ALT + 5]};
				Q_sqrt_com_0 = {{C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}, ~Q_sqrt0[C_MANT_FP16ALT + 5:0]};
				Q_sqrt_com_1 = {{C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}, ~Q_sqrt1[C_MANT_FP16ALT + 5:0]};
				Q_sqrt_com_2 = {{C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}, ~Q_sqrt2[C_MANT_FP16ALT + 5:0]};
				Q_sqrt_com_3 = {{C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}, ~Q_sqrt3[C_MANT_FP16ALT + 5:0]};
			end
		endcase
	assign Qcnt_one_0 = 1'b0;
	assign Qcnt_one_1 = {Quotient_DP[0]};
	assign Qcnt_one_2 = {Quotient_DP[1:0]};
	assign Qcnt_one_3 = {Quotient_DP[2:0]};
	assign Qcnt_one_4 = {Quotient_DP[3:0]};
	assign Qcnt_one_5 = {Quotient_DP[4:0]};
	assign Qcnt_one_6 = {Quotient_DP[5:0]};
	assign Qcnt_one_7 = {Quotient_DP[6:0]};
	assign Qcnt_one_8 = {Quotient_DP[7:0]};
	assign Qcnt_one_9 = {Quotient_DP[8:0]};
	assign Qcnt_one_10 = {Quotient_DP[9:0]};
	assign Qcnt_one_11 = {Quotient_DP[10:0]};
	assign Qcnt_one_12 = {Quotient_DP[11:0]};
	assign Qcnt_one_13 = {Quotient_DP[12:0]};
	assign Qcnt_one_14 = {Quotient_DP[13:0]};
	assign Qcnt_one_15 = {Quotient_DP[14:0]};
	assign Qcnt_one_16 = {Quotient_DP[15:0]};
	assign Qcnt_one_17 = {Quotient_DP[16:0]};
	assign Qcnt_one_18 = {Quotient_DP[17:0]};
	assign Qcnt_one_19 = {Quotient_DP[18:0]};
	assign Qcnt_one_20 = {Quotient_DP[19:0]};
	assign Qcnt_one_21 = {Quotient_DP[20:0]};
	assign Qcnt_one_22 = {Quotient_DP[21:0]};
	assign Qcnt_one_23 = {Quotient_DP[22:0]};
	assign Qcnt_one_24 = {Quotient_DP[23:0]};
	assign Qcnt_one_25 = {Quotient_DP[24:0]};
	assign Qcnt_one_26 = {Quotient_DP[25:0]};
	assign Qcnt_one_27 = {Quotient_DP[26:0]};
	assign Qcnt_one_28 = {Quotient_DP[27:0]};
	assign Qcnt_one_29 = {Quotient_DP[28:0]};
	assign Qcnt_one_30 = {Quotient_DP[29:0]};
	assign Qcnt_one_31 = {Quotient_DP[30:0]};
	assign Qcnt_one_32 = {Quotient_DP[31:0]};
	assign Qcnt_one_33 = {Quotient_DP[32:0]};
	assign Qcnt_one_34 = {Quotient_DP[33:0]};
	assign Qcnt_one_35 = {Quotient_DP[34:0]};
	assign Qcnt_one_36 = {Quotient_DP[35:0]};
	assign Qcnt_one_37 = {Quotient_DP[36:0]};
	assign Qcnt_one_38 = {Quotient_DP[37:0]};
	assign Qcnt_one_39 = {Quotient_DP[38:0]};
	assign Qcnt_one_40 = {Quotient_DP[39:0]};
	assign Qcnt_one_41 = {Quotient_DP[40:0]};
	assign Qcnt_one_42 = {Quotient_DP[41:0]};
	assign Qcnt_one_43 = {Quotient_DP[42:0]};
	assign Qcnt_one_44 = {Quotient_DP[43:0]};
	assign Qcnt_one_45 = {Quotient_DP[44:0]};
	assign Qcnt_one_46 = {Quotient_DP[45:0]};
	assign Qcnt_one_47 = {Quotient_DP[46:0]};
	assign Qcnt_one_48 = {Quotient_DP[47:0]};
	assign Qcnt_one_49 = {Quotient_DP[48:0]};
	assign Qcnt_one_50 = {Quotient_DP[49:0]};
	assign Qcnt_one_51 = {Quotient_DP[50:0]};
	assign Qcnt_one_52 = {Quotient_DP[51:0]};
	assign Qcnt_one_53 = {Quotient_DP[52:0]};
	assign Qcnt_one_54 = {Quotient_DP[53:0]};
	assign Qcnt_one_55 = {Quotient_DP[54:0]};
	assign Qcnt_one_56 = {Quotient_DP[55:0]};
	assign Qcnt_one_57 = {Quotient_DP[56:0]};
	assign Qcnt_two_0 = {1'b0, Sqrt_quotinent_S[3]};
	assign Qcnt_two_1 = {Quotient_DP[1:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_2 = {Quotient_DP[3:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_3 = {Quotient_DP[5:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_4 = {Quotient_DP[7:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_5 = {Quotient_DP[9:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_6 = {Quotient_DP[11:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_7 = {Quotient_DP[13:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_8 = {Quotient_DP[15:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_9 = {Quotient_DP[17:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_10 = {Quotient_DP[19:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_11 = {Quotient_DP[21:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_12 = {Quotient_DP[23:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_13 = {Quotient_DP[25:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_14 = {Quotient_DP[27:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_15 = {Quotient_DP[29:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_16 = {Quotient_DP[31:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_17 = {Quotient_DP[33:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_18 = {Quotient_DP[35:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_19 = {Quotient_DP[37:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_20 = {Quotient_DP[39:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_21 = {Quotient_DP[41:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_22 = {Quotient_DP[43:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_23 = {Quotient_DP[45:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_24 = {Quotient_DP[47:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_25 = {Quotient_DP[49:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_26 = {Quotient_DP[51:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_27 = {Quotient_DP[53:0], Sqrt_quotinent_S[3]};
	assign Qcnt_two_28 = {Quotient_DP[55:0], Sqrt_quotinent_S[3]};
	assign Qcnt_three_0 = {1'b0, Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_1 = {Quotient_DP[2:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_2 = {Quotient_DP[5:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_3 = {Quotient_DP[8:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_4 = {Quotient_DP[11:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_5 = {Quotient_DP[14:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_6 = {Quotient_DP[17:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_7 = {Quotient_DP[20:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_8 = {Quotient_DP[23:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_9 = {Quotient_DP[26:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_10 = {Quotient_DP[29:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_11 = {Quotient_DP[32:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_12 = {Quotient_DP[35:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_13 = {Quotient_DP[38:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_14 = {Quotient_DP[41:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_15 = {Quotient_DP[44:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_16 = {Quotient_DP[47:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_17 = {Quotient_DP[50:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_18 = {Quotient_DP[53:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_three_19 = {Quotient_DP[56:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2]};
	assign Qcnt_four_0 = {1'b0, Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_1 = {Quotient_DP[3:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_2 = {Quotient_DP[7:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_3 = {Quotient_DP[11:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_4 = {Quotient_DP[15:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_5 = {Quotient_DP[19:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_6 = {Quotient_DP[23:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_7 = {Quotient_DP[27:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_8 = {Quotient_DP[31:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_9 = {Quotient_DP[35:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_10 = {Quotient_DP[39:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_11 = {Quotient_DP[43:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_12 = {Quotient_DP[47:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_13 = {Quotient_DP[51:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	assign Qcnt_four_14 = {Quotient_DP[55:0], Sqrt_quotinent_S[3], Sqrt_quotinent_S[2], Sqrt_quotinent_S[1]};
	always @(*)
		case (Iteration_unit_num_S)
			2'b00:
				case (Crtl_cnt_S)
					6'b000000: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 + 1:C_MANT_FP64];
						Q_sqrt0 = {{C_MANT_FP64 + 5 {1'b0}}, Qcnt_one_0};
						Sqrt_Q0 = Q_sqrt_com_0;
					end
					6'b000001: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 1:C_MANT_FP64 - 2];
						Q_sqrt0 = {{C_MANT_FP64 + 5 {1'b0}}, Qcnt_one_1};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b000010: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 3:C_MANT_FP64 - 4];
						Q_sqrt0 = {{C_MANT_FP64 + 4 {1'b0}}, Qcnt_one_2};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b000011: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 5:C_MANT_FP64 - 6];
						Q_sqrt0 = {{C_MANT_FP64 + 3 {1'b0}}, Qcnt_one_3};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b000100: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 7:C_MANT_FP64 - 8];
						Q_sqrt0 = {{C_MANT_FP64 + 2 {1'b0}}, Qcnt_one_4};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b000101: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 9:C_MANT_FP64 - 10];
						Q_sqrt0 = {{C_MANT_FP64 + 1 {1'b0}}, Qcnt_one_5};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b000110: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 11:C_MANT_FP64 - 12];
						Q_sqrt0 = {{C_MANT_FP64 {1'b0}}, Qcnt_one_6};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b000111: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 13:C_MANT_FP64 - 14];
						Q_sqrt0 = {{C_MANT_FP64 - 1 {1'b0}}, Qcnt_one_7};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b001000: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 15:C_MANT_FP64 - 16];
						Q_sqrt0 = {{C_MANT_FP64 - 2 {1'b0}}, Qcnt_one_8};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b001001: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 17:C_MANT_FP64 - 18];
						Q_sqrt0 = {{C_MANT_FP64 - 3 {1'b0}}, Qcnt_one_9};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b001010: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 19:C_MANT_FP64 - 20];
						Q_sqrt0 = {{C_MANT_FP64 - 4 {1'b0}}, Qcnt_one_10};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b001011: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 21:C_MANT_FP64 - 22];
						Q_sqrt0 = {{C_MANT_FP64 - 5 {1'b0}}, Qcnt_one_11};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b001100: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 23:C_MANT_FP64 - 24];
						Q_sqrt0 = {{C_MANT_FP64 - 6 {1'b0}}, Qcnt_one_12};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b001101: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 25:C_MANT_FP64 - 26];
						Q_sqrt0 = {{C_MANT_FP64 - 7 {1'b0}}, Qcnt_one_13};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b001110: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 27:C_MANT_FP64 - 28];
						Q_sqrt0 = {{C_MANT_FP64 - 8 {1'b0}}, Qcnt_one_14};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b001111: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 29:C_MANT_FP64 - 30];
						Q_sqrt0 = {{C_MANT_FP64 - 9 {1'b0}}, Qcnt_one_15};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b010000: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 31:C_MANT_FP64 - 32];
						Q_sqrt0 = {{C_MANT_FP64 - 10 {1'b0}}, Qcnt_one_16};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b010001: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 33:C_MANT_FP64 - 34];
						Q_sqrt0 = {{C_MANT_FP64 - 11 {1'b0}}, Qcnt_one_17};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b010010: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 35:C_MANT_FP64 - 36];
						Q_sqrt0 = {{C_MANT_FP64 - 12 {1'b0}}, Qcnt_one_18};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b010011: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 37:C_MANT_FP64 - 38];
						Q_sqrt0 = {{C_MANT_FP64 - 13 {1'b0}}, Qcnt_one_19};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b010100: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 39:C_MANT_FP64 - 40];
						Q_sqrt0 = {{C_MANT_FP64 - 14 {1'b0}}, Qcnt_one_20};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b010101: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 41:C_MANT_FP64 - 42];
						Q_sqrt0 = {{C_MANT_FP64 - 15 {1'b0}}, Qcnt_one_21};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b010110: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 43:C_MANT_FP64 - 44];
						Q_sqrt0 = {{C_MANT_FP64 - 16 {1'b0}}, Qcnt_one_22};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b010111: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 45:C_MANT_FP64 - 46];
						Q_sqrt0 = {{C_MANT_FP64 - 17 {1'b0}}, Qcnt_one_23};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b011000: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 47:C_MANT_FP64 - 48];
						Q_sqrt0 = {{C_MANT_FP64 - 18 {1'b0}}, Qcnt_one_24};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b011001: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 49:C_MANT_FP64 - 50];
						Q_sqrt0 = {{C_MANT_FP64 - 19 {1'b0}}, Qcnt_one_25};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b011010: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 51:C_MANT_FP64 - 52];
						Q_sqrt0 = {{C_MANT_FP64 - 20 {1'b0}}, Qcnt_one_26};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b011011: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 21 {1'b0}}, Qcnt_one_27};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b011100: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 22 {1'b0}}, Qcnt_one_28};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b011101: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 23 {1'b0}}, Qcnt_one_29};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b011110: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 24 {1'b0}}, Qcnt_one_30};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b011111: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 25 {1'b0}}, Qcnt_one_31};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b100000: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 26 {1'b0}}, Qcnt_one_32};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b100001: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 27 {1'b0}}, Qcnt_one_33};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b100010: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 28 {1'b0}}, Qcnt_one_34};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b100011: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 29 {1'b0}}, Qcnt_one_35};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b100100: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 30 {1'b0}}, Qcnt_one_36};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b100101: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 31 {1'b0}}, Qcnt_one_37};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b100110: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 32 {1'b0}}, Qcnt_one_38};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b100111: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 33 {1'b0}}, Qcnt_one_39};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b101000: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 34 {1'b0}}, Qcnt_one_40};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b101001: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 35 {1'b0}}, Qcnt_one_41};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b101010: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 36 {1'b0}}, Qcnt_one_42};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b101011: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 37 {1'b0}}, Qcnt_one_43};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b101100: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 38 {1'b0}}, Qcnt_one_44};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b101101: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 39 {1'b0}}, Qcnt_one_45};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b101110: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 40 {1'b0}}, Qcnt_one_46};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b101111: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 41 {1'b0}}, Qcnt_one_47};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b110000: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 42 {1'b0}}, Qcnt_one_48};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b110001: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 43 {1'b0}}, Qcnt_one_49};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b110010: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 44 {1'b0}}, Qcnt_one_50};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b110011: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 45 {1'b0}}, Qcnt_one_51};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b110100: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 46 {1'b0}}, Qcnt_one_52};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b110101: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 47 {1'b0}}, Qcnt_one_53};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b110110: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 48 {1'b0}}, Qcnt_one_54};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b110111: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 49 {1'b0}}, Qcnt_one_55};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					6'b111000: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 50 {1'b0}}, Qcnt_one_56};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
					end
					default: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {((C_MANT_FP64 + 5) >= 0 ? C_MANT_FP64 + 6 : 1 - (C_MANT_FP64 + 5)) {1'sb0}};
						Sqrt_Q0 = {((C_MANT_FP64 + 5) >= 0 ? C_MANT_FP64 + 6 : 1 - (C_MANT_FP64 + 5)) {1'sb0}};
					end
				endcase
			2'b01:
				case (Crtl_cnt_S)
					6'b000000: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 + 1:C_MANT_FP64];
						Q_sqrt0 = {{C_MANT_FP64 + 5 {1'b0}}, Qcnt_two_0[1]};
						Sqrt_Q0 = Q_sqrt_com_0;
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 1:C_MANT_FP64 - 2];
						Q_sqrt1 = {{C_MANT_FP64 + 4 {1'b0}}, Qcnt_two_0[1:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b000001: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 3:C_MANT_FP64 - 4];
						Q_sqrt0 = {{C_MANT_FP64 + 4 {1'b0}}, Qcnt_two_1[2:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 5:C_MANT_FP64 - 6];
						Q_sqrt1 = {{C_MANT_FP64 + 3 {1'b0}}, Qcnt_two_1[2:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b000010: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 7:C_MANT_FP64 - 8];
						Q_sqrt0 = {{C_MANT_FP64 + 2 {1'b0}}, Qcnt_two_2[4:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 9:C_MANT_FP64 - 10];
						Q_sqrt1 = {{C_MANT_FP64 + 1 {1'b0}}, Qcnt_two_2[4:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b000011: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 11:C_MANT_FP64 - 12];
						Q_sqrt0 = {{C_MANT_FP64 {1'b0}}, Qcnt_two_3[6:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 13:C_MANT_FP64 - 14];
						Q_sqrt1 = {{C_MANT_FP64 - 1 {1'b0}}, Qcnt_two_3[6:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b000100: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 15:C_MANT_FP64 - 16];
						Q_sqrt0 = {{C_MANT_FP64 - 2 {1'b0}}, Qcnt_two_4[8:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 17:C_MANT_FP64 - 18];
						Q_sqrt1 = {{C_MANT_FP64 - 3 {1'b0}}, Qcnt_two_4[8:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b000101: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 19:C_MANT_FP64 - 20];
						Q_sqrt0 = {{C_MANT_FP64 - 4 {1'b0}}, Qcnt_two_5[10:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 21:C_MANT_FP64 - 22];
						Q_sqrt1 = {{C_MANT_FP64 - 5 {1'b0}}, Qcnt_two_5[10:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b000110: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 23:C_MANT_FP64 - 24];
						Q_sqrt0 = {{C_MANT_FP64 - 6 {1'b0}}, Qcnt_two_6[12:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 25:C_MANT_FP64 - 26];
						Q_sqrt1 = {{C_MANT_FP64 - 7 {1'b0}}, Qcnt_two_6[12:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b000111: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 27:C_MANT_FP64 - 28];
						Q_sqrt0 = {{C_MANT_FP64 - 8 {1'b0}}, Qcnt_two_7[14:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 29:C_MANT_FP64 - 30];
						Q_sqrt1 = {{C_MANT_FP64 - 9 {1'b0}}, Qcnt_two_7[14:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b001000: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 31:C_MANT_FP64 - 32];
						Q_sqrt0 = {{C_MANT_FP64 - 10 {1'b0}}, Qcnt_two_8[16:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 33:C_MANT_FP64 - 34];
						Q_sqrt1 = {{C_MANT_FP64 - 11 {1'b0}}, Qcnt_two_8[16:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b001001: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 35:C_MANT_FP64 - 36];
						Q_sqrt0 = {{C_MANT_FP64 - 12 {1'b0}}, Qcnt_two_9[18:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 37:C_MANT_FP64 - 38];
						Q_sqrt1 = {{C_MANT_FP64 - 13 {1'b0}}, Qcnt_two_9[18:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b001010: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 39:C_MANT_FP64 - 40];
						Q_sqrt0 = {{C_MANT_FP64 - 14 {1'b0}}, Qcnt_two_10[20:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 41:C_MANT_FP64 - 42];
						Q_sqrt1 = {{C_MANT_FP64 - 15 {1'b0}}, Qcnt_two_10[20:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b001011: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 43:C_MANT_FP64 - 44];
						Q_sqrt0 = {{C_MANT_FP64 - 16 {1'b0}}, Qcnt_two_11[22:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 45:C_MANT_FP64 - 46];
						Q_sqrt1 = {{C_MANT_FP64 - 17 {1'b0}}, Qcnt_two_11[22:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b001100: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 47:C_MANT_FP64 - 48];
						Q_sqrt0 = {{C_MANT_FP64 - 18 {1'b0}}, Qcnt_two_12[24:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 49:C_MANT_FP64 - 50];
						Q_sqrt1 = {{C_MANT_FP64 - 19 {1'b0}}, Qcnt_two_12[24:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b001101: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 51:C_MANT_FP64 - 52];
						Q_sqrt0 = {{C_MANT_FP64 - 20 {1'b0}}, Qcnt_two_13[26:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 21 {1'b0}}, Qcnt_two_13[26:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b001110: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 22 {1'b0}}, Qcnt_two_14[28:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 23 {1'b0}}, Qcnt_two_14[28:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b001111: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 24 {1'b0}}, Qcnt_two_15[30:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 25 {1'b0}}, Qcnt_two_15[30:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b010000: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 26 {1'b0}}, Qcnt_two_16[32:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 27 {1'b0}}, Qcnt_two_16[32:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b010001: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 28 {1'b0}}, Qcnt_two_17[34:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 29 {1'b0}}, Qcnt_two_17[34:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b010010: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 30 {1'b0}}, Qcnt_two_18[36:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 31 {1'b0}}, Qcnt_two_18[36:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b010011: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 32 {1'b0}}, Qcnt_two_19[38:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 33 {1'b0}}, Qcnt_two_19[38:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b010100: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 34 {1'b0}}, Qcnt_two_20[40:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 35 {1'b0}}, Qcnt_two_20[40:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b010101: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 36 {1'b0}}, Qcnt_two_21[42:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 37 {1'b0}}, Qcnt_two_21[42:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b010110: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 38 {1'b0}}, Qcnt_two_22[44:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 39 {1'b0}}, Qcnt_two_22[44:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b010111: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 40 {1'b0}}, Qcnt_two_23[46:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 41 {1'b0}}, Qcnt_two_23[46:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b011000: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 42 {1'b0}}, Qcnt_two_24[48:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 43 {1'b0}}, Qcnt_two_24[48:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b011001: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 44 {1'b0}}, Qcnt_two_25[50:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 45 {1'b0}}, Qcnt_two_25[50:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b011010: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 46 {1'b0}}, Qcnt_two_26[52:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 47 {1'b0}}, Qcnt_two_26[52:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b011011: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 48 {1'b0}}, Qcnt_two_27[54:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 49 {1'b0}}, Qcnt_two_27[54:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					6'b011100: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 50 {1'b0}}, Qcnt_two_28[56:1]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 51 {1'b0}}, Qcnt_two_28[56:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
					default: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 + 1:C_MANT_FP64];
						Q_sqrt0 = {{C_MANT_FP64 + 5 {1'b0}}, Qcnt_two_0[1]};
						Sqrt_Q0 = Q_sqrt_com_0;
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 1:C_MANT_FP64 - 2];
						Q_sqrt1 = {{C_MANT_FP64 + 4 {1'b0}}, Qcnt_two_0[1:0]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
					end
				endcase
			2'b10:
				case (Crtl_cnt_S)
					6'b000000: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 + 1:C_MANT_FP64];
						Q_sqrt0 = {{C_MANT_FP64 + 5 {1'b0}}, Qcnt_three_0[2]};
						Sqrt_Q0 = Q_sqrt_com_0;
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 1:C_MANT_FP64 - 2];
						Q_sqrt1 = {{C_MANT_FP64 + 4 {1'b0}}, Qcnt_three_0[2:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 3:C_MANT_FP64 - 4];
						Q_sqrt2 = {{C_MANT_FP64 + 3 {1'b0}}, Qcnt_three_0[2:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b000001: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 5:C_MANT_FP64 - 6];
						Q_sqrt0 = {{C_MANT_FP64 + 2 {1'b0}}, Qcnt_three_1[4:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 7:C_MANT_FP64 - 8];
						Q_sqrt1 = {{C_MANT_FP64 + 1 {1'b0}}, Qcnt_three_1[4:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 9:C_MANT_FP64 - 10];
						Q_sqrt2 = {{C_MANT_FP64 {1'b0}}, Qcnt_three_1[4:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b000010: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 11:C_MANT_FP64 - 12];
						Q_sqrt0 = {{C_MANT_FP64 - 1 {1'b0}}, Qcnt_three_2[7:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 13:C_MANT_FP64 - 14];
						Q_sqrt1 = {{C_MANT_FP64 - 2 {1'b0}}, Qcnt_three_2[7:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 15:C_MANT_FP64 - 16];
						Q_sqrt2 = {{C_MANT_FP64 - 3 {1'b0}}, Qcnt_three_2[7:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b000011: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 17:C_MANT_FP64 - 18];
						Q_sqrt0 = {{C_MANT_FP64 - 4 {1'b0}}, Qcnt_three_3[10:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 19:C_MANT_FP64 - 20];
						Q_sqrt1 = {{C_MANT_FP64 - 5 {1'b0}}, Qcnt_three_3[10:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 21:C_MANT_FP64 - 22];
						Q_sqrt2 = {{C_MANT_FP64 - 6 {1'b0}}, Qcnt_three_3[10:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b000100: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 23:C_MANT_FP64 - 24];
						Q_sqrt0 = {{C_MANT_FP64 - 7 {1'b0}}, Qcnt_three_4[13:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 25:C_MANT_FP64 - 26];
						Q_sqrt1 = {{C_MANT_FP64 - 8 {1'b0}}, Qcnt_three_4[13:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 27:C_MANT_FP64 - 28];
						Q_sqrt2 = {{C_MANT_FP64 - 9 {1'b0}}, Qcnt_three_4[13:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b000101: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 29:C_MANT_FP64 - 30];
						Q_sqrt0 = {{C_MANT_FP64 - 10 {1'b0}}, Qcnt_three_5[16:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 31:C_MANT_FP64 - 32];
						Q_sqrt1 = {{C_MANT_FP64 - 11 {1'b0}}, Qcnt_three_5[16:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 33:C_MANT_FP64 - 34];
						Q_sqrt2 = {{C_MANT_FP64 - 12 {1'b0}}, Qcnt_three_5[16:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b000110: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 35:C_MANT_FP64 - 36];
						Q_sqrt0 = {{C_MANT_FP64 - 13 {1'b0}}, Qcnt_three_6[19:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 37:C_MANT_FP64 - 38];
						Q_sqrt1 = {{C_MANT_FP64 - 14 {1'b0}}, Qcnt_three_6[19:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 39:C_MANT_FP64 - 40];
						Q_sqrt2 = {{C_MANT_FP64 - 15 {1'b0}}, Qcnt_three_6[19:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b000111: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 41:C_MANT_FP64 - 42];
						Q_sqrt0 = {{C_MANT_FP64 - 16 {1'b0}}, Qcnt_three_7[22:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 43:C_MANT_FP64 - 44];
						Q_sqrt1 = {{C_MANT_FP64 - 17 {1'b0}}, Qcnt_three_7[22:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 45:C_MANT_FP64 - 46];
						Q_sqrt2 = {{C_MANT_FP64 - 18 {1'b0}}, Qcnt_three_7[22:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b001000: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 47:C_MANT_FP64 - 48];
						Q_sqrt0 = {{C_MANT_FP64 - 19 {1'b0}}, Qcnt_three_8[25:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 49:C_MANT_FP64 - 50];
						Q_sqrt1 = {{C_MANT_FP64 - 20 {1'b0}}, Qcnt_three_8[25:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 51:C_MANT_FP64 - 52];
						Q_sqrt2 = {{C_MANT_FP64 - 21 {1'b0}}, Qcnt_three_8[25:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b001001: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 22 {1'b0}}, Qcnt_three_9[28:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 23 {1'b0}}, Qcnt_three_9[28:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 24 {1'b0}}, Qcnt_three_9[28:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b001010: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 25 {1'b0}}, Qcnt_three_10[31:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 26 {1'b0}}, Qcnt_three_10[31:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 27 {1'b0}}, Qcnt_three_10[31:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b001011: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 28 {1'b0}}, Qcnt_three_11[34:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 29 {1'b0}}, Qcnt_three_11[34:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 30 {1'b0}}, Qcnt_three_11[34:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b001100: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 31 {1'b0}}, Qcnt_three_12[37:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 32 {1'b0}}, Qcnt_three_12[37:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 33 {1'b0}}, Qcnt_three_12[37:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b001101: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 34 {1'b0}}, Qcnt_three_13[40:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 35 {1'b0}}, Qcnt_three_13[40:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 36 {1'b0}}, Qcnt_three_13[40:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b001110: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 37 {1'b0}}, Qcnt_three_14[43:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 38 {1'b0}}, Qcnt_three_14[43:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 39 {1'b0}}, Qcnt_three_14[43:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b001111: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 40 {1'b0}}, Qcnt_three_15[46:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 41 {1'b0}}, Qcnt_three_15[46:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 42 {1'b0}}, Qcnt_three_15[46:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b010000: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 43 {1'b0}}, Qcnt_three_16[49:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 44 {1'b0}}, Qcnt_three_16[49:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 45 {1'b0}}, Qcnt_three_16[49:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b010001: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 46 {1'b0}}, Qcnt_three_17[52:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 47 {1'b0}}, Qcnt_three_17[52:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 48 {1'b0}}, Qcnt_three_17[52:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					6'b010010: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 49 {1'b0}}, Qcnt_three_18[55:2]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 50 {1'b0}}, Qcnt_three_18[55:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 51 {1'b0}}, Qcnt_three_18[55:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
					default: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 + 1:C_MANT_FP64];
						Q_sqrt0 = {{C_MANT_FP64 + 5 {1'b0}}, Qcnt_three_0[2]};
						Sqrt_Q0 = Q_sqrt_com_0;
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 1:C_MANT_FP64 - 2];
						Q_sqrt1 = {{C_MANT_FP64 + 4 {1'b0}}, Qcnt_three_0[2:1]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 3:C_MANT_FP64 - 4];
						Q_sqrt2 = {{C_MANT_FP64 + 3 {1'b0}}, Qcnt_three_0[2:0]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
					end
				endcase
			2'b11:
				case (Crtl_cnt_S)
					6'b000000: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 + 1:C_MANT_FP64];
						Q_sqrt0 = {{C_MANT_FP64 + 5 {1'b0}}, Qcnt_four_0[3]};
						Sqrt_Q0 = Q_sqrt_com_0;
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 1:C_MANT_FP64 - 2];
						Q_sqrt1 = {{C_MANT_FP64 + 4 {1'b0}}, Qcnt_four_0[3:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 3:C_MANT_FP64 - 4];
						Q_sqrt2 = {{C_MANT_FP64 + 3 {1'b0}}, Qcnt_four_0[3:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = Mant_D_sqrt_Norm[C_MANT_FP64 - 5:C_MANT_FP64 - 6];
						Q_sqrt3 = {{C_MANT_FP64 + 2 {1'b0}}, Qcnt_four_0[3:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b000001: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 7:C_MANT_FP64 - 8];
						Q_sqrt0 = {{C_MANT_FP64 + 1 {1'b0}}, Qcnt_four_1[6:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 9:C_MANT_FP64 - 10];
						Q_sqrt1 = {{C_MANT_FP64 {1'b0}}, Qcnt_four_1[6:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 11:C_MANT_FP64 - 12];
						Q_sqrt2 = {{C_MANT_FP64 - 1 {1'b0}}, Qcnt_four_1[6:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = Mant_D_sqrt_Norm[C_MANT_FP64 - 13:C_MANT_FP64 - 14];
						Q_sqrt3 = {{C_MANT_FP64 - 2 {1'b0}}, Qcnt_four_1[6:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b000010: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 15:C_MANT_FP64 - 16];
						Q_sqrt0 = {{C_MANT_FP64 - 3 {1'b0}}, Qcnt_four_2[10:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 17:C_MANT_FP64 - 18];
						Q_sqrt1 = {{C_MANT_FP64 - 4 {1'b0}}, Qcnt_four_2[10:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 19:C_MANT_FP64 - 20];
						Q_sqrt2 = {{C_MANT_FP64 - 5 {1'b0}}, Qcnt_four_2[10:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = Mant_D_sqrt_Norm[C_MANT_FP64 - 21:C_MANT_FP64 - 22];
						Q_sqrt3 = {{C_MANT_FP64 - 6 {1'b0}}, Qcnt_four_2[10:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b000011: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 23:C_MANT_FP64 - 24];
						Q_sqrt0 = {{C_MANT_FP64 - 7 {1'b0}}, Qcnt_four_3[14:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 25:C_MANT_FP64 - 26];
						Q_sqrt1 = {{C_MANT_FP64 - 8 {1'b0}}, Qcnt_four_3[14:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 27:C_MANT_FP64 - 28];
						Q_sqrt2 = {{C_MANT_FP64 - 9 {1'b0}}, Qcnt_four_3[14:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = Mant_D_sqrt_Norm[C_MANT_FP64 - 29:C_MANT_FP64 - 30];
						Q_sqrt3 = {{C_MANT_FP64 - 10 {1'b0}}, Qcnt_four_3[14:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b000100: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 31:C_MANT_FP64 - 32];
						Q_sqrt0 = {{C_MANT_FP64 - 11 {1'b0}}, Qcnt_four_4[18:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 33:C_MANT_FP64 - 34];
						Q_sqrt1 = {{C_MANT_FP64 - 12 {1'b0}}, Qcnt_four_4[18:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 35:C_MANT_FP64 - 36];
						Q_sqrt2 = {{C_MANT_FP64 - 13 {1'b0}}, Qcnt_four_4[18:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = Mant_D_sqrt_Norm[C_MANT_FP64 - 37:C_MANT_FP64 - 38];
						Q_sqrt3 = {{C_MANT_FP64 - 14 {1'b0}}, Qcnt_four_4[18:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b000101: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 39:C_MANT_FP64 - 40];
						Q_sqrt0 = {{C_MANT_FP64 - 15 {1'b0}}, Qcnt_four_5[22:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 41:C_MANT_FP64 - 42];
						Q_sqrt1 = {{C_MANT_FP64 - 16 {1'b0}}, Qcnt_four_5[22:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 43:C_MANT_FP64 - 44];
						Q_sqrt2 = {{C_MANT_FP64 - 17 {1'b0}}, Qcnt_four_5[22:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = Mant_D_sqrt_Norm[C_MANT_FP64 - 45:C_MANT_FP64 - 46];
						Q_sqrt3 = {{C_MANT_FP64 - 18 {1'b0}}, Qcnt_four_5[22:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b000110: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 - 47:C_MANT_FP64 - 48];
						Q_sqrt0 = {{C_MANT_FP64 - 19 {1'b0}}, Qcnt_four_6[26:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 49:C_MANT_FP64 - 50];
						Q_sqrt1 = {{C_MANT_FP64 - 20 {1'b0}}, Qcnt_four_6[26:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 51:C_MANT_FP64 - 52];
						Q_sqrt2 = {{C_MANT_FP64 - 21 {1'b0}}, Qcnt_four_6[26:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = 2'b00;
						Q_sqrt3 = {{C_MANT_FP64 - 22 {1'b0}}, Qcnt_four_6[26:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b000111: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 23 {1'b0}}, Qcnt_four_7[30:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 24 {1'b0}}, Qcnt_four_7[30:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 25 {1'b0}}, Qcnt_four_7[30:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = 2'b00;
						Q_sqrt3 = {{C_MANT_FP64 - 26 {1'b0}}, Qcnt_four_7[30:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b001000: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 27 {1'b0}}, Qcnt_four_8[34:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 28 {1'b0}}, Qcnt_four_8[34:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 29 {1'b0}}, Qcnt_four_8[34:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = 2'b00;
						Q_sqrt3 = {{C_MANT_FP64 - 30 {1'b0}}, Qcnt_four_8[34:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b001001: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 31 {1'b0}}, Qcnt_four_9[38:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 32 {1'b0}}, Qcnt_four_9[38:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 33 {1'b0}}, Qcnt_four_9[38:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = 2'b00;
						Q_sqrt3 = {{C_MANT_FP64 - 34 {1'b0}}, Qcnt_four_9[38:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b001010: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 35 {1'b0}}, Qcnt_four_10[42:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 36 {1'b0}}, Qcnt_four_10[42:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 37 {1'b0}}, Qcnt_four_10[42:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = 2'b00;
						Q_sqrt3 = {{C_MANT_FP64 - 38 {1'b0}}, Qcnt_four_10[42:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b001011: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 39 {1'b0}}, Qcnt_four_11[46:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 40 {1'b0}}, Qcnt_four_11[46:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 41 {1'b0}}, Qcnt_four_11[46:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = 2'b00;
						Q_sqrt3 = {{C_MANT_FP64 - 42 {1'b0}}, Qcnt_four_11[46:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b001100: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 43 {1'b0}}, Qcnt_four_12[50:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 44 {1'b0}}, Qcnt_four_12[50:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 45 {1'b0}}, Qcnt_four_12[50:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = 2'b00;
						Q_sqrt3 = {{C_MANT_FP64 - 46 {1'b0}}, Qcnt_four_12[50:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					6'b001101: begin
						Sqrt_DI[0] = 2'b00;
						Q_sqrt0 = {{C_MANT_FP64 - 47 {1'b0}}, Qcnt_four_13[54:3]};
						Sqrt_Q0 = (Quotient_DP[0] ? Q_sqrt_com_0 : Q_sqrt0);
						Sqrt_DI[1] = 2'b00;
						Q_sqrt1 = {{C_MANT_FP64 - 48 {1'b0}}, Qcnt_four_13[54:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = 2'b00;
						Q_sqrt2 = {{C_MANT_FP64 - 49 {1'b0}}, Qcnt_four_13[54:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = 2'b00;
						Q_sqrt3 = {{C_MANT_FP64 - 50 {1'b0}}, Qcnt_four_13[54:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
					default: begin
						Sqrt_DI[0] = Mant_D_sqrt_Norm[C_MANT_FP64 + 1:C_MANT_FP64];
						Q_sqrt0 = {{C_MANT_FP64 + 5 {1'b0}}, Qcnt_four_0[3]};
						Sqrt_Q0 = Q_sqrt_com_0;
						Sqrt_DI[1] = Mant_D_sqrt_Norm[C_MANT_FP64 - 1:C_MANT_FP64 - 2];
						Q_sqrt1 = {{C_MANT_FP64 + 4 {1'b0}}, Qcnt_four_0[3:2]};
						Sqrt_Q1 = (Sqrt_quotinent_S[3] ? Q_sqrt_com_1 : Q_sqrt1);
						Sqrt_DI[2] = Mant_D_sqrt_Norm[C_MANT_FP64 - 3:C_MANT_FP64 - 4];
						Q_sqrt2 = {{C_MANT_FP64 + 3 {1'b0}}, Qcnt_four_0[3:1]};
						Sqrt_Q2 = (Sqrt_quotinent_S[2] ? Q_sqrt_com_2 : Q_sqrt2);
						Sqrt_DI[3] = Mant_D_sqrt_Norm[C_MANT_FP64 - 5:C_MANT_FP64 - 6];
						Q_sqrt3 = {{C_MANT_FP64 + 2 {1'b0}}, Qcnt_four_0[3:0]};
						Sqrt_Q3 = (Sqrt_quotinent_S[1] ? Q_sqrt_com_3 : Q_sqrt3);
					end
				endcase
		endcase
	assign Sqrt_R0 = (Sqrt_start_dly_S ? {((C_MANT_FP64 + 5) >= 0 ? C_MANT_FP64 + 6 : 1 - (C_MANT_FP64 + 5)) {1'sb0}} : {Partial_remainder_DP[C_MANT_FP64 + 5:0]});
	assign Sqrt_R1 = {Iteration_cell_sum_AMASK_D[0][C_MANT_FP64 + 5], Iteration_cell_sum_AMASK_D[0][C_MANT_FP64 + 2:0], Sqrt_DO[0]};
	assign Sqrt_R2 = {Iteration_cell_sum_AMASK_D[1][C_MANT_FP64 + 5], Iteration_cell_sum_AMASK_D[1][C_MANT_FP64 + 2:0], Sqrt_DO[1]};
	assign Sqrt_R3 = {Iteration_cell_sum_AMASK_D[2][C_MANT_FP64 + 5], Iteration_cell_sum_AMASK_D[2][C_MANT_FP64 + 2:0], Sqrt_DO[2]};
	assign Sqrt_R4 = {Iteration_cell_sum_AMASK_D[3][C_MANT_FP64 + 5], Iteration_cell_sum_AMASK_D[3][C_MANT_FP64 + 2:0], Sqrt_DO[3]};
	wire [C_MANT_FP64 + 5:0] Denominator_se_format_DB;
	assign Denominator_se_format_DB = {Denominator_se_DB[C_MANT_FP64 + 1:C_MANT_FP64 - C_MANT_FP16ALT], {(FP16ALT_SO ? FP16ALT_SO : Denominator_se_DB[(C_MANT_FP64 - C_MANT_FP16ALT) - 1])}, Denominator_se_DB[(C_MANT_FP64 - C_MANT_FP16ALT) - 2:C_MANT_FP64 - C_MANT_FP16], {(FP16_SO ? FP16_SO : Denominator_se_DB[(C_MANT_FP64 - C_MANT_FP16) - 1])}, Denominator_se_DB[(C_MANT_FP64 - C_MANT_FP16) - 2:C_MANT_FP64 - C_MANT_FP32], {(FP32_SO ? FP32_SO : Denominator_se_DB[(C_MANT_FP64 - C_MANT_FP32) - 1])}, Denominator_se_DB[(C_MANT_FP64 - C_MANT_FP32) - 2:C_MANT_FP64 - C_MANT_FP64], FP64_SO, 3'b000};
	wire [C_MANT_FP64 + 5:0] First_iteration_cell_div_a_D;
	wire [C_MANT_FP64 + 5:0] First_iteration_cell_div_b_D;
	wire Sel_b_for_first_S;
	assign First_iteration_cell_div_a_D = (Div_start_dly_S ? {Numerator_se_D[C_MANT_FP64 + 1:C_MANT_FP64 - C_MANT_FP16ALT], {(FP16ALT_SO ? FP16ALT_SO : Numerator_se_D[(C_MANT_FP64 - C_MANT_FP16ALT) - 1])}, Numerator_se_D[(C_MANT_FP64 - C_MANT_FP16ALT) - 2:C_MANT_FP64 - C_MANT_FP16], {(FP16_SO ? FP16_SO : Numerator_se_D[(C_MANT_FP64 - C_MANT_FP16) - 1])}, Numerator_se_D[(C_MANT_FP64 - C_MANT_FP16) - 2:C_MANT_FP64 - C_MANT_FP32], {(FP32_SO ? FP32_SO : Numerator_se_D[(C_MANT_FP64 - C_MANT_FP32) - 1])}, Numerator_se_D[(C_MANT_FP64 - C_MANT_FP32) - 2:C_MANT_FP64 - C_MANT_FP64], FP64_SO, 3'b000} : {Partial_remainder_DP[C_MANT_FP64 + 4:(C_MANT_FP64 - C_MANT_FP16ALT) + 3], {(FP16ALT_SO ? Quotient_DP[0] : Partial_remainder_DP[(C_MANT_FP64 - C_MANT_FP16ALT) + 2])}, Partial_remainder_DP[(C_MANT_FP64 - C_MANT_FP16ALT) + 1:(C_MANT_FP64 - C_MANT_FP16) + 3], {(FP16_SO ? Quotient_DP[0] : Partial_remainder_DP[(C_MANT_FP64 - C_MANT_FP16) + 2])}, Partial_remainder_DP[(C_MANT_FP64 - C_MANT_FP16) + 1:(C_MANT_FP64 - C_MANT_FP32) + 3], {(FP32_SO ? Quotient_DP[0] : Partial_remainder_DP[(C_MANT_FP64 - C_MANT_FP32) + 2])}, Partial_remainder_DP[(C_MANT_FP64 - C_MANT_FP32) + 1:(C_MANT_FP64 - C_MANT_FP64) + 3], FP64_SO && Quotient_DP[0], 3'b000});
	assign Sel_b_for_first_S = (Div_start_dly_S ? 1 : Quotient_DP[0]);
	assign First_iteration_cell_div_b_D = (Sel_b_for_first_S ? Denominator_se_format_DB : {Denominator_se_D, 4'b0000});
	assign Iteration_cell_a_BMASK_D[0] = (Sqrt_enable_SO ? Sqrt_R0 : {First_iteration_cell_div_a_D});
	assign Iteration_cell_b_BMASK_D[0] = (Sqrt_enable_SO ? Sqrt_Q0 : {First_iteration_cell_div_b_D});
	wire [C_MANT_FP64 + 5:0] Sec_iteration_cell_div_a_D;
	wire [C_MANT_FP64 + 5:0] Sec_iteration_cell_div_b_D;
	wire Sel_b_for_sec_S;
	generate
		if (|Iteration_unit_num_S) begin
			assign Sel_b_for_sec_S = ~Iteration_cell_sum_AMASK_D[0][C_MANT_FP64 + 5];
			assign Sec_iteration_cell_div_a_D = {Iteration_cell_sum_AMASK_D[0][C_MANT_FP64 + 4:(C_MANT_FP64 - C_MANT_FP16ALT) + 3], {(FP16ALT_SO ? Sel_b_for_sec_S : Iteration_cell_sum_AMASK_D[0][(C_MANT_FP64 - C_MANT_FP16ALT) + 2])}, Iteration_cell_sum_AMASK_D[0][(C_MANT_FP64 - C_MANT_FP16ALT) + 1:(C_MANT_FP64 - C_MANT_FP16) + 3], {(FP16_SO ? Sel_b_for_sec_S : Iteration_cell_sum_AMASK_D[0][(C_MANT_FP64 - C_MANT_FP16) + 2])}, Iteration_cell_sum_AMASK_D[0][(C_MANT_FP64 - C_MANT_FP16) + 1:(C_MANT_FP64 - C_MANT_FP32) + 3], {(FP32_SO ? Sel_b_for_sec_S : Iteration_cell_sum_AMASK_D[0][(C_MANT_FP64 - C_MANT_FP32) + 2])}, Iteration_cell_sum_AMASK_D[0][(C_MANT_FP64 - C_MANT_FP32) + 1:(C_MANT_FP64 - C_MANT_FP64) + 3], FP64_SO && Sel_b_for_sec_S, 3'b000};
			assign Sec_iteration_cell_div_b_D = (Sel_b_for_sec_S ? Denominator_se_format_DB : {Denominator_se_D, 4'b0000});
			assign Iteration_cell_a_BMASK_D[1] = (Sqrt_enable_SO ? Sqrt_R1 : {Sec_iteration_cell_div_a_D});
			assign Iteration_cell_b_BMASK_D[1] = (Sqrt_enable_SO ? Sqrt_Q1 : {Sec_iteration_cell_div_b_D});
		end
	endgenerate
	wire [C_MANT_FP64 + 5:0] Thi_iteration_cell_div_a_D;
	wire [C_MANT_FP64 + 5:0] Thi_iteration_cell_div_b_D;
	wire Sel_b_for_thi_S;
	generate
		if ((Iteration_unit_num_S == 2'b10) | (Iteration_unit_num_S == 2'b11)) begin
			assign Sel_b_for_thi_S = ~Iteration_cell_sum_AMASK_D[1][C_MANT_FP64 + 5];
			assign Thi_iteration_cell_div_a_D = {Iteration_cell_sum_AMASK_D[1][C_MANT_FP64 + 4:(C_MANT_FP64 - C_MANT_FP16ALT) + 3], {(FP16ALT_SO ? Sel_b_for_thi_S : Iteration_cell_sum_AMASK_D[1][(C_MANT_FP64 - C_MANT_FP16ALT) + 2])}, Iteration_cell_sum_AMASK_D[1][(C_MANT_FP64 - C_MANT_FP16ALT) + 1:(C_MANT_FP64 - C_MANT_FP16) + 3], {(FP16_SO ? Sel_b_for_thi_S : Iteration_cell_sum_AMASK_D[1][(C_MANT_FP64 - C_MANT_FP16) + 2])}, Iteration_cell_sum_AMASK_D[1][(C_MANT_FP64 - C_MANT_FP16) + 1:(C_MANT_FP64 - C_MANT_FP32) + 3], {(FP32_SO ? Sel_b_for_thi_S : Iteration_cell_sum_AMASK_D[1][(C_MANT_FP64 - C_MANT_FP32) + 2])}, Iteration_cell_sum_AMASK_D[1][(C_MANT_FP64 - C_MANT_FP32) + 1:(C_MANT_FP64 - C_MANT_FP64) + 3], FP64_SO && Sel_b_for_thi_S, 3'b000};
			assign Thi_iteration_cell_div_b_D = (Sel_b_for_thi_S ? Denominator_se_format_DB : {Denominator_se_D, 4'b0000});
			assign Iteration_cell_a_BMASK_D[2] = (Sqrt_enable_SO ? Sqrt_R2 : {Thi_iteration_cell_div_a_D});
			assign Iteration_cell_b_BMASK_D[2] = (Sqrt_enable_SO ? Sqrt_Q2 : {Thi_iteration_cell_div_b_D});
		end
	endgenerate
	wire [C_MANT_FP64 + 5:0] Fou_iteration_cell_div_a_D;
	wire [C_MANT_FP64 + 5:0] Fou_iteration_cell_div_b_D;
	wire Sel_b_for_fou_S;
	generate
		if (Iteration_unit_num_S == 2'b11) begin
			assign Sel_b_for_fou_S = ~Iteration_cell_sum_AMASK_D[2][C_MANT_FP64 + 5];
			assign Fou_iteration_cell_div_a_D = {Iteration_cell_sum_AMASK_D[2][C_MANT_FP64 + 4:(C_MANT_FP64 - C_MANT_FP16ALT) + 3], {(FP16ALT_SO ? Sel_b_for_fou_S : Iteration_cell_sum_AMASK_D[2][(C_MANT_FP64 - C_MANT_FP16ALT) + 2])}, Iteration_cell_sum_AMASK_D[2][(C_MANT_FP64 - C_MANT_FP16ALT) + 1:(C_MANT_FP64 - C_MANT_FP16) + 3], {(FP16_SO ? Sel_b_for_fou_S : Iteration_cell_sum_AMASK_D[2][(C_MANT_FP64 - C_MANT_FP16) + 2])}, Iteration_cell_sum_AMASK_D[2][(C_MANT_FP64 - C_MANT_FP16) + 1:(C_MANT_FP64 - C_MANT_FP32) + 3], {(FP32_SO ? Sel_b_for_fou_S : Iteration_cell_sum_AMASK_D[2][(C_MANT_FP64 - C_MANT_FP32) + 2])}, Iteration_cell_sum_AMASK_D[2][(C_MANT_FP64 - C_MANT_FP32) + 1:(C_MANT_FP64 - C_MANT_FP64) + 3], FP64_SO && Sel_b_for_fou_S, 3'b000};
			assign Fou_iteration_cell_div_b_D = (Sel_b_for_fou_S ? Denominator_se_format_DB : {Denominator_se_D, 4'b0000});
			assign Iteration_cell_a_BMASK_D[3] = (Sqrt_enable_SO ? Sqrt_R3 : {Fou_iteration_cell_div_a_D});
			assign Iteration_cell_b_BMASK_D[3] = (Sqrt_enable_SO ? Sqrt_Q3 : {Fou_iteration_cell_div_b_D});
		end
	endgenerate
	wire [C_MANT_FP64 + 5:0] Mask_bits_ctl_S;
	assign Mask_bits_ctl_S = 58'h3ffffffffffffff;
	wire Div_enable_SI [3:0];
	wire Div_start_dly_SI [3:0];
	wire Sqrt_enable_SI [3:0];
	generate
		genvar i;
		genvar j;
		for (i = 0; i <= Iteration_unit_num_S; i = i + 1) begin
			for (j = 0; j <= (C_MANT_FP64 + 5); j = j + 1) begin
				assign Iteration_cell_a_D[i][j] = Mask_bits_ctl_S[j] && Iteration_cell_a_BMASK_D[i][j];
				assign Iteration_cell_b_D[i][j] = Mask_bits_ctl_S[j] && Iteration_cell_b_BMASK_D[i][j];
				assign Iteration_cell_sum_AMASK_D[i][j] = Mask_bits_ctl_S[j] && Iteration_cell_sum_D[i][j];
			end
			assign Div_enable_SI[i] = Div_enable_SO;
			assign Div_start_dly_SI[i] = Div_start_dly_S;
			assign Sqrt_enable_SI[i] = Sqrt_enable_SO;
			iteration_div_sqrt_mvp #(.WIDTH(C_MANT_FP64 + 6)) iteration_div_sqrt(
				.A_DI(Iteration_cell_a_D[i]),
				.B_DI(Iteration_cell_b_D[i]),
				.Div_enable_SI(Div_enable_SI[i]),
				.Div_start_dly_SI(Div_start_dly_SI[i]),
				.Sqrt_enable_SI(Sqrt_enable_SI[i]),
				.D_DI(Sqrt_DI[i]),
				.D_DO(Sqrt_DO[i]),
				.Sum_DO(Iteration_cell_sum_D[i]),
				.Carry_out_DO(Iteration_cell_carry_D[i])
			);
		end
	endgenerate
	always @(*)
		case (Iteration_unit_num_S)
			2'b00:
				if (Fsm_enable_S)
					Partial_remainder_DN = (Sqrt_enable_SO ? Sqrt_R1 : Iteration_cell_sum_AMASK_D[0]);
				else
					Partial_remainder_DN = Partial_remainder_DP;
			2'b01:
				if (Fsm_enable_S)
					Partial_remainder_DN = (Sqrt_enable_SO ? Sqrt_R2 : Iteration_cell_sum_AMASK_D[1]);
				else
					Partial_remainder_DN = Partial_remainder_DP;
			2'b10:
				if (Fsm_enable_S)
					Partial_remainder_DN = (Sqrt_enable_SO ? Sqrt_R3 : Iteration_cell_sum_AMASK_D[2]);
				else
					Partial_remainder_DN = Partial_remainder_DP;
			2'b11:
				if (Fsm_enable_S)
					Partial_remainder_DN = (Sqrt_enable_SO ? Sqrt_R4 : Iteration_cell_sum_AMASK_D[3]);
				else
					Partial_remainder_DN = Partial_remainder_DP;
		endcase
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Partial_remainder_DP <= {((C_MANT_FP64 + 5) >= 0 ? C_MANT_FP64 + 6 : 1 - (C_MANT_FP64 + 5)) {1'sb0}};
		else
			Partial_remainder_DP <= Partial_remainder_DN;
	reg [C_MANT_FP64 + 4:0] Quotient_DN;
	always @(*)
		case (Iteration_unit_num_S)
			2'b00:
				if (Fsm_enable_S)
					Quotient_DN = (Sqrt_enable_SO ? {Quotient_DP[C_MANT_FP64 + 3:0], Sqrt_quotinent_S[3]} : {Quotient_DP[C_MANT_FP64 + 3:0], Iteration_cell_carry_D[0]});
				else
					Quotient_DN = Quotient_DP;
			2'b01:
				if (Fsm_enable_S)
					Quotient_DN = (Sqrt_enable_SO ? {Quotient_DP[C_MANT_FP64 + 2:0], Sqrt_quotinent_S[3:2]} : {Quotient_DP[C_MANT_FP64 + 2:0], Iteration_cell_carry_D[0], Iteration_cell_carry_D[1]});
				else
					Quotient_DN = Quotient_DP;
			2'b10:
				if (Fsm_enable_S)
					Quotient_DN = (Sqrt_enable_SO ? {Quotient_DP[C_MANT_FP64 + 1:0], Sqrt_quotinent_S[3:1]} : {Quotient_DP[C_MANT_FP64 + 1:0], Iteration_cell_carry_D[0], Iteration_cell_carry_D[1], Iteration_cell_carry_D[2]});
				else
					Quotient_DN = Quotient_DP;
			2'b11:
				if (Fsm_enable_S)
					Quotient_DN = (Sqrt_enable_SO ? {Quotient_DP[C_MANT_FP64:0], Sqrt_quotinent_S} : {Quotient_DP[C_MANT_FP64:0], Iteration_cell_carry_D[0], Iteration_cell_carry_D[1], Iteration_cell_carry_D[2], Iteration_cell_carry_D[3]});
				else
					Quotient_DN = Quotient_DP;
		endcase
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Quotient_DP <= {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
		else
			Quotient_DP <= Quotient_DN;
	generate
		if (Iteration_unit_num_S == 2'b00) always @(*)
			case (Format_sel_S)
				2'b00:
					case (Precision_ctl_S)
						6'h00: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 + 4:0], {C_MANT_FP64 - C_MANT_FP32 {1'b0}}};
						6'h17: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32:0], {(C_MANT_FP64 - C_MANT_FP32) + 4 {1'b0}}};
						6'h16: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 1:0], {(C_MANT_FP64 - C_MANT_FP32) + 5 {1'b0}}};
						6'h15: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 2:0], {(C_MANT_FP64 - C_MANT_FP32) + 6 {1'b0}}};
						6'h14: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 3:0], {(C_MANT_FP64 - C_MANT_FP32) + 7 {1'b0}}};
						6'h13: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 4:0], {(C_MANT_FP64 - C_MANT_FP32) + 8 {1'b0}}};
						6'h12: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 5:0], {(C_MANT_FP64 - C_MANT_FP32) + 9 {1'b0}}};
						6'h11: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 6:0], {(C_MANT_FP64 - C_MANT_FP32) + 10 {1'b0}}};
						6'h10: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 7:0], {(C_MANT_FP64 - C_MANT_FP32) + 11 {1'b0}}};
						6'h0f: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 8:0], {(C_MANT_FP64 - C_MANT_FP32) + 12 {1'b0}}};
						6'h0e: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 9:0], {(C_MANT_FP64 - C_MANT_FP32) + 13 {1'b0}}};
						6'h0d: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 10:0], {(C_MANT_FP64 - C_MANT_FP32) + 14 {1'b0}}};
						6'h0c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 11:0], {(C_MANT_FP64 - C_MANT_FP32) + 15 {1'b0}}};
						6'h0b: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 12:0], {(C_MANT_FP64 - C_MANT_FP32) + 16 {1'b0}}};
						6'h0a: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 13:0], {(C_MANT_FP64 - C_MANT_FP32) + 17 {1'b0}}};
						6'h09: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 14:0], {(C_MANT_FP64 - C_MANT_FP32) + 18 {1'b0}}};
						6'h08: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 15:0], {(C_MANT_FP64 - C_MANT_FP32) + 19 {1'b0}}};
						6'h07: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 16:0], {(C_MANT_FP64 - C_MANT_FP32) + 20 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 + 4:0], {C_MANT_FP64 - C_MANT_FP32 {1'b0}}};
					endcase
				2'b01:
					case (Precision_ctl_S)
						6'h00: Mant_result_prenorm_DO = Quotient_DP[C_MANT_FP64 + 4:0];
						6'h34: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64:0], {4 {1'b0}}};
						6'h33: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 1:0], {5 {1'b0}}};
						6'h32: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 2:0], {6 {1'b0}}};
						6'h31: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 3:0], {7 {1'b0}}};
						6'h30: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 4:0], {8 {1'b0}}};
						6'h2f: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 5:0], {9 {1'b0}}};
						6'h2e: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 6:0], {10 {1'b0}}};
						6'h2d: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 7:0], {11 {1'b0}}};
						6'h2c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 8:0], {12 {1'b0}}};
						6'h2b: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 9:0], {13 {1'b0}}};
						6'h2a: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 10:0], {14 {1'b0}}};
						6'h29: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 11:0], {15 {1'b0}}};
						6'h28: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 12:0], {16 {1'b0}}};
						6'h27: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 13:0], {17 {1'b0}}};
						6'h26: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 14:0], {18 {1'b0}}};
						6'h25: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 15:0], {19 {1'b0}}};
						6'h24: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 16:0], {20 {1'b0}}};
						6'h23: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 17:0], {21 {1'b0}}};
						6'h22: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 18:0], {22 {1'b0}}};
						6'h21: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 19:0], {23 {1'b0}}};
						6'h20: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 20:0], {24 {1'b0}}};
						6'h1f: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 21:0], {25 {1'b0}}};
						6'h1e: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 22:0], {26 {1'b0}}};
						6'h1d: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 23:0], {27 {1'b0}}};
						6'h1c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 24:0], {28 {1'b0}}};
						6'h1b: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 25:0], {29 {1'b0}}};
						6'h1a: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 26:0], {30 {1'b0}}};
						6'h19: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 27:0], {31 {1'b0}}};
						6'h18: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 28:0], {32 {1'b0}}};
						6'h17: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 29:0], {33 {1'b0}}};
						6'h16: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 30:0], {34 {1'b0}}};
						6'h15: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 31:0], {35 {1'b0}}};
						6'h14: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 32:0], {36 {1'b0}}};
						6'h13: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 33:0], {37 {1'b0}}};
						6'h12: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 34:0], {38 {1'b0}}};
						6'h11: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 35:0], {39 {1'b0}}};
						6'h10: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 36:0], {40 {1'b0}}};
						6'h0f: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 37:0], {41 {1'b0}}};
						6'h0e: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 38:0], {42 {1'b0}}};
						6'h0d: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 39:0], {43 {1'b0}}};
						6'h0c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 40:0], {44 {1'b0}}};
						6'h0b: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 41:0], {45 {1'b0}}};
						6'h0a: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 42:0], {46 {1'b0}}};
						6'h09: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 43:0], {47 {1'b0}}};
						6'h08: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 44:0], {48 {1'b0}}};
						6'h07: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 45:0], {49 {1'b0}}};
						default: Mant_result_prenorm_DO = Quotient_DP[C_MANT_FP64 + 4:0];
					endcase
				2'b10:
					case (Precision_ctl_S)
						6'b000000: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 + 4:0], {C_MANT_FP64 - C_MANT_FP16 {1'b0}}};
						6'h0a: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16:0], {(C_MANT_FP64 - C_MANT_FP16) + 4 {1'b0}}};
						6'h09: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 - 1:0], {(C_MANT_FP64 - C_MANT_FP16) + 5 {1'b0}}};
						6'h08: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 - 2:0], {(C_MANT_FP64 - C_MANT_FP16) + 6 {1'b0}}};
						6'h07: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 - 3:0], {(C_MANT_FP64 - C_MANT_FP16) + 7 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 + 4:0], {C_MANT_FP64 - C_MANT_FP16 {1'b0}}};
					endcase
				2'b11:
					case (Precision_ctl_S)
						6'b000000: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT + 4:0], {C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}};
						6'h07: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT:0], {(C_MANT_FP64 - C_MANT_FP16ALT) + 4 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT + 4:0], {C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}};
					endcase
			endcase
	endgenerate
	generate
		if (Iteration_unit_num_S == 2'b01) always @(*)
			case (Format_sel_S)
				2'b00:
					case (Precision_ctl_S)
						6'h00: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 + 4:0], {C_MANT_FP64 - C_MANT_FP32 {1'b0}}};
						6'h17, 6'h16: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32:0], {(C_MANT_FP64 - C_MANT_FP32) + 4 {1'b0}}};
						6'h15, 6'h14: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 2:0], {(C_MANT_FP64 - C_MANT_FP32) + 6 {1'b0}}};
						6'h13, 6'h12: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 4:0], {(C_MANT_FP64 - C_MANT_FP32) + 8 {1'b0}}};
						6'h11, 6'h10: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 6:0], {(C_MANT_FP64 - C_MANT_FP32) + 10 {1'b0}}};
						6'h0f, 6'h0e: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 8:0], {(C_MANT_FP64 - C_MANT_FP32) + 12 {1'b0}}};
						6'h0d, 6'h0c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 10:0], {(C_MANT_FP64 - C_MANT_FP32) + 14 {1'b0}}};
						6'h0b, 6'h0a: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 12:0], {(C_MANT_FP64 - C_MANT_FP32) + 16 {1'b0}}};
						6'h09, 6'h08: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 14:0], {(C_MANT_FP64 - C_MANT_FP32) + 18 {1'b0}}};
						6'h07, 6'h06: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 16:0], {(C_MANT_FP64 - C_MANT_FP32) + 20 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 + 4:0], {C_MANT_FP64 - C_MANT_FP32 {1'b0}}};
					endcase
				2'b01:
					case (Precision_ctl_S)
						6'h00: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 + 3:0], 1'b0};
						6'h34: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 + 1:1], {4 {1'b0}}};
						6'h33, 6'h32: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 1:0], {5 {1'b0}}};
						6'h31, 6'h30: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 3:0], {7 {1'b0}}};
						6'h2f, 6'h2e: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 5:0], {9 {1'b0}}};
						6'h2d, 6'h2c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 7:0], {11 {1'b0}}};
						6'h2b, 6'h2a: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 9:0], {13 {1'b0}}};
						6'h29, 6'h28: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 11:0], {15 {1'b0}}};
						6'h27, 6'h26: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 13:0], {17 {1'b0}}};
						6'h25, 6'h24: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 15:0], {19 {1'b0}}};
						6'h23, 6'h22: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 17:0], {21 {1'b0}}};
						6'h21, 6'h20: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 19:0], {23 {1'b0}}};
						6'h1f, 6'h1e: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 21:0], {25 {1'b0}}};
						6'h1d, 6'h1c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 23:0], {27 {1'b0}}};
						6'h1b, 6'h1a: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 25:0], {29 {1'b0}}};
						6'h19, 6'h18: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 27:0], {31 {1'b0}}};
						6'h17, 6'h16: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 29:0], {33 {1'b0}}};
						6'h15, 6'h14: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 31:0], {35 {1'b0}}};
						6'h13, 6'h12: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 33:0], {37 {1'b0}}};
						6'h11, 6'h10: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 35:0], {39 {1'b0}}};
						6'h0f, 6'h0e: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 37:0], {41 {1'b0}}};
						6'h0d, 6'h0c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 39:0], {43 {1'b0}}};
						6'h0b, 6'h0a: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 41:0], {45 {1'b0}}};
						6'h09, 6'h08: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 43:0], {47 {1'b0}}};
						6'h07: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 45:0], {49 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 + 3:0], 1'b0};
					endcase
				2'b10:
					case (Precision_ctl_S)
						6'b000000: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 + 3:0], {(C_MANT_FP64 - C_MANT_FP16) + 1 {1'b0}}};
						6'h0a: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 + 1:1], {(C_MANT_FP64 - C_MANT_FP16) + 4 {1'b0}}};
						6'h09, 6'h08: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 - 1:0], {(C_MANT_FP64 - C_MANT_FP16) + 5 {1'b0}}};
						6'h07: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 - 3:0], {(C_MANT_FP64 - C_MANT_FP16) + 7 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 + 4:0], {C_MANT_FP64 - C_MANT_FP16 {1'b0}}};
					endcase
				2'b11:
					case (Precision_ctl_S)
						6'b000000: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT + 4:0], {C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}};
						6'h07: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT:0], {(C_MANT_FP64 - C_MANT_FP16ALT) + 4 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT + 4:0], {C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}};
					endcase
			endcase
	endgenerate
	generate
		if (Iteration_unit_num_S == 2'b10) always @(*)
			case (Format_sel_S)
				2'b00:
					case (Precision_ctl_S)
						6'h00: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 + 3:0], {(C_MANT_FP64 - C_MANT_FP32) + 1 {1'b0}}};
						6'h17, 6'h16, 6'h15: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32:0], {(C_MANT_FP64 - C_MANT_FP32) + 4 {1'b0}}};
						6'h14, 6'h13, 6'h12: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 3:0], {(C_MANT_FP64 - C_MANT_FP32) + 7 {1'b0}}};
						6'h11, 6'h10, 6'h0f: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 6:0], {(C_MANT_FP64 - C_MANT_FP32) + 10 {1'b0}}};
						6'h0e, 6'h0d, 6'h0c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 9:0], {(C_MANT_FP64 - C_MANT_FP32) + 13 {1'b0}}};
						6'h0b, 6'h0a, 6'h09: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 12:0], {(C_MANT_FP64 - C_MANT_FP32) + 16 {1'b0}}};
						6'h08, 6'h07, 6'h06: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 15:0], {(C_MANT_FP64 - C_MANT_FP32) + 19 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 + 3:0], {(C_MANT_FP64 - C_MANT_FP32) + 1 {1'b0}}};
					endcase
				2'b01:
					case (Precision_ctl_S)
						6'h00: Mant_result_prenorm_DO = Quotient_DP[C_MANT_FP64 + 4:0];
						6'h34, 6'h33: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 + 1:1], {4 {1'b0}}};
						6'h32, 6'h31, 6'h30: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 2:0], {6 {1'b0}}};
						6'h2f, 6'h2e, 6'h2d: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 5:0], {9 {1'b0}}};
						6'h2c, 6'h2b, 6'h2a: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 8:0], {12 {1'b0}}};
						6'h29, 6'h28, 6'h27: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 11:0], {15 {1'b0}}};
						6'h26, 6'h25, 6'h24: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 14:0], {18 {1'b0}}};
						6'h23, 6'h22, 6'h21: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 17:0], {21 {1'b0}}};
						6'h20, 6'h1f, 6'h1e: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 20:0], {24 {1'b0}}};
						6'h1d, 6'h1c, 6'h1b: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 23:0], {27 {1'b0}}};
						6'h1a, 6'h19, 6'h18: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 26:0], {30 {1'b0}}};
						6'h17, 6'h16, 6'h15: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 29:0], {33 {1'b0}}};
						6'h14, 6'h13, 6'h12: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 32:0], {36 {1'b0}}};
						6'h11, 6'h10, 6'h0f: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 35:0], {39 {1'b0}}};
						6'h0e, 6'h0d, 6'h0c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 38:0], {42 {1'b0}}};
						6'h0b, 6'h0a, 6'h09: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 41:0], {45 {1'b0}}};
						6'h08, 6'h07, 6'h06: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 44:0], {48 {1'b0}}};
						default: Mant_result_prenorm_DO = Quotient_DP[C_MANT_FP64 + 4:0];
					endcase
				2'b10:
					case (Precision_ctl_S)
						6'b000000: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 + 4:0], {C_MANT_FP64 - C_MANT_FP16 {1'b0}}};
						6'h0a, 6'h09: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 + 1:1], {(C_MANT_FP64 - C_MANT_FP16) + 4 {1'b0}}};
						6'h08, 6'h07, 6'h06: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 - 2:0], {(C_MANT_FP64 - C_MANT_FP16) + 6 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 + 4:0], {C_MANT_FP64 - C_MANT_FP16 {1'b0}}};
					endcase
				2'b11:
					case (Precision_ctl_S)
						6'b000000: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT + 4:0], {C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}};
						6'h07, 6'h06: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT + 1:1], {(C_MANT_FP64 - C_MANT_FP16ALT) + 4 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT + 4:0], {C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}};
					endcase
			endcase
	endgenerate
	generate
		if (Iteration_unit_num_S == 2'b11) always @(*)
			case (Format_sel_S)
				2'b00:
					case (Precision_ctl_S)
						6'h00: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 + 4:0], {C_MANT_FP64 - C_MANT_FP32 {1'b0}}};
						6'h17, 6'h16, 6'h15, 6'h14: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32:0], {(C_MANT_FP64 - C_MANT_FP32) + 4 {1'b0}}};
						6'h13, 6'h12, 6'h11, 6'h10: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 4:0], {(C_MANT_FP64 - C_MANT_FP32) + 8 {1'b0}}};
						6'h0f, 6'h0e, 6'h0d, 6'h0c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 8:0], {(C_MANT_FP64 - C_MANT_FP32) + 12 {1'b0}}};
						6'h0b, 6'h0a, 6'h09, 6'h08: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 12:0], {(C_MANT_FP64 - C_MANT_FP32) + 16 {1'b0}}};
						6'h07, 6'h06: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 - 16:0], {(C_MANT_FP64 - C_MANT_FP32) + 20 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32 + 4:0], {C_MANT_FP64 - C_MANT_FP32 {1'b0}}};
					endcase
				2'b01:
					case (Precision_ctl_S)
						6'h00: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 + 3:0], 1'b0};
						6'h34: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 + 3:0], 1'b0};
						6'h33, 6'h32, 6'h31, 6'h30: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 1:0], {5 {1'b0}}};
						6'h2f, 6'h2e, 6'h2d, 6'h2c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 5:0], {9 {1'b0}}};
						6'h2b, 6'h2a, 6'h29, 6'h28: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 9:0], {13 {1'b0}}};
						6'h27, 6'h26, 6'h25, 6'h24: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 13:0], {17 {1'b0}}};
						6'h23, 6'h22, 6'h21, 6'h20: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 17:0], {21 {1'b0}}};
						6'h1f, 6'h1e, 6'h1d, 6'h1c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 21:0], {25 {1'b0}}};
						6'h1b, 6'h1a, 6'h19, 6'h18: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 25:0], {29 {1'b0}}};
						6'h17, 6'h16, 6'h15, 6'h14: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 29:0], {33 {1'b0}}};
						6'h13, 6'h12, 6'h11, 6'h10: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 33:0], {37 {1'b0}}};
						6'h0f, 6'h0e, 6'h0d, 6'h0c: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 37:0], {41 {1'b0}}};
						6'h0b, 6'h0a, 6'h09, 6'h08: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 41:0], {45 {1'b0}}};
						6'h07, 6'h06: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 - 45:0], {49 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64 + 3:0], 1'b0};
					endcase
				2'b10:
					case (Precision_ctl_S)
						6'b000000: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 + 5:0], {(C_MANT_FP64 - C_MANT_FP16) - 1 {1'b0}}};
						6'h0a, 6'h09, 6'h08: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 + 1:1], {(C_MANT_FP64 - C_MANT_FP16) + 4 {1'b0}}};
						6'h07, 6'h06: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 - 3:0], {(C_MANT_FP64 - C_MANT_FP16) + 7 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16 + 5:0], {(C_MANT_FP64 - C_MANT_FP16) - 1 {1'b0}}};
					endcase
				2'b11:
					case (Precision_ctl_S)
						6'b000000: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT + 4:0], {C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}};
						6'h07, 6'h06: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT:0], {(C_MANT_FP64 - C_MANT_FP16ALT) + 4 {1'b0}}};
						default: Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT + 4:0], {C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}};
					endcase
			endcase
	endgenerate
	wire [C_EXP_FP64 + 1:0] Exp_result_prenorm_DN;
	reg [C_EXP_FP64 + 1:0] Exp_result_prenorm_DP;
	wire [C_EXP_FP64 + 1:0] Exp_add_a_D;
	wire [C_EXP_FP64 + 1:0] Exp_add_b_D;
	wire [C_EXP_FP64 + 1:0] Exp_add_c_D;
	integer C_BIAS_AONE;
	integer C_HALF_BIAS;
	always @(*)
		case (Format_sel_S)
			2'b00: begin
				C_BIAS_AONE = C_BIAS_AONE_FP32;
				C_HALF_BIAS = C_HALF_BIAS_FP32;
			end
			2'b01: begin
				C_BIAS_AONE = C_BIAS_AONE_FP64;
				C_HALF_BIAS = C_HALF_BIAS_FP64;
			end
			2'b10: begin
				C_BIAS_AONE = C_BIAS_AONE_FP16;
				C_HALF_BIAS = C_HALF_BIAS_FP16;
			end
			2'b11: begin
				C_BIAS_AONE = C_BIAS_AONE_FP16ALT;
				C_HALF_BIAS = C_HALF_BIAS_FP16ALT;
			end
		endcase
	assign Exp_add_a_D = {(Sqrt_start_dly_S ? {Exp_num_DI[C_EXP_FP64], Exp_num_DI[C_EXP_FP64], Exp_num_DI[C_EXP_FP64], Exp_num_DI[C_EXP_FP64:1]} : {Exp_num_DI[C_EXP_FP64], Exp_num_DI[C_EXP_FP64], Exp_num_DI})};
	assign Exp_add_b_D = {(Sqrt_start_dly_S ? {1'b0, {C_EXP_ZERO_FP64}, Exp_num_DI[0]} : {~Exp_den_DI[C_EXP_FP64], ~Exp_den_DI[C_EXP_FP64], ~Exp_den_DI})};
	assign Exp_add_c_D = {(Div_start_dly_S ? {C_BIAS_AONE} : {C_HALF_BIAS})};
	assign Exp_result_prenorm_DN = (Start_dly_S ? {(Exp_add_a_D + Exp_add_b_D) + Exp_add_c_D} : Exp_result_prenorm_DP);
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Exp_result_prenorm_DP <= {((C_EXP_FP64 + 1) >= 0 ? C_EXP_FP64 + 2 : 1 - (C_EXP_FP64 + 1)) {1'sb0}};
		else
			Exp_result_prenorm_DP <= Exp_result_prenorm_DN;
	assign Exp_result_prenorm_DO = Exp_result_prenorm_DP;
endmodule
module div_sqrt_mvp_wrapper (
	Clk_CI,
	Rst_RBI,
	Div_start_SI,
	Sqrt_start_SI,
	Operand_a_DI,
	Operand_b_DI,
	RM_SI,
	Precision_ctl_SI,
	Format_sel_SI,
	Kill_SI,
	Result_DO,
	Fflags_SO,
	Ready_SO,
	Done_SO
);
	parameter PrePipeline_depth_S = 0;
	parameter PostPipeline_depth_S = 2;
	input wire Clk_CI;
	input wire Rst_RBI;
	input wire Div_start_SI;
	input wire Sqrt_start_SI;
	input wire [C_OP_FP64 - 1:0] Operand_a_DI;
	input wire [C_OP_FP64 - 1:0] Operand_b_DI;
	input wire [C_RM - 1:0] RM_SI;
	input wire [C_PC - 1:0] Precision_ctl_SI;
	input wire [C_FS - 1:0] Format_sel_SI;
	input wire Kill_SI;
	output wire [C_OP_FP64 - 1:0] Result_DO;
	output wire [4:0] Fflags_SO;
	output wire Ready_SO;
	output wire Done_SO;
	reg Div_start_S_S;
	reg Sqrt_start_S_S;
	reg [C_OP_FP64 - 1:0] Operand_a_S_D;
	reg [C_OP_FP64 - 1:0] Operand_b_S_D;
	reg [C_RM - 1:0] RM_S_S;
	reg [C_PC - 1:0] Precision_ctl_S_S;
	reg [C_FS - 1:0] Format_sel_S_S;
	reg Kill_S_S;
	wire [C_OP_FP64 - 1:0] Result_D;
	wire Ready_S;
	wire Done_S;
	wire [4:0] Fflags_S;
	generate
		if (PrePipeline_depth_S == 1) begin
			div_sqrt_top_mvp div_top_U0(
				.Clk_CI(Clk_CI),
				.Rst_RBI(Rst_RBI),
				.Div_start_SI(Div_start_S_S),
				.Sqrt_start_SI(Sqrt_start_S_S),
				.Operand_a_DI(Operand_a_S_D),
				.Operand_b_DI(Operand_b_S_D),
				.RM_SI(RM_S_S),
				.Precision_ctl_SI(Precision_ctl_S_S),
				.Format_sel_SI(Format_sel_S_S),
				.Kill_SI(Kill_S_S),
				.Result_DO(Result_D),
				.Fflags_SO(Fflags_S),
				.Ready_SO(Ready_S),
				.Done_SO(Done_S)
			);
			always @(posedge Clk_CI or negedge Rst_RBI)
				if (~Rst_RBI) begin
					Div_start_S_S <= 1'b0;
					Sqrt_start_S_S <= 1'b0;
					Operand_a_S_D <= {C_OP_FP64 {1'sb0}};
					Operand_b_S_D <= {C_OP_FP64 {1'sb0}};
					RM_S_S <= 1'b0;
					Precision_ctl_S_S <= {C_PC {1'sb0}};
					Format_sel_S_S <= {C_FS {1'sb0}};
					Kill_S_S <= 1'b0;
				end
				else begin
					Div_start_S_S <= Div_start_SI;
					Sqrt_start_S_S <= Sqrt_start_SI;
					Operand_a_S_D <= Operand_a_DI;
					Operand_b_S_D <= Operand_b_DI;
					RM_S_S <= RM_SI;
					Precision_ctl_S_S <= Precision_ctl_SI;
					Format_sel_S_S <= Format_sel_SI;
					Kill_S_S <= Kill_SI;
				end
		end
		else div_sqrt_top_mvp div_top_U0(
			.Clk_CI(Clk_CI),
			.Rst_RBI(Rst_RBI),
			.Div_start_SI(Div_start_SI),
			.Sqrt_start_SI(Sqrt_start_SI),
			.Operand_a_DI(Operand_a_DI),
			.Operand_b_DI(Operand_b_DI),
			.RM_SI(RM_SI),
			.Precision_ctl_SI(Precision_ctl_SI),
			.Format_sel_SI(Format_sel_SI),
			.Kill_SI(Kill_SI),
			.Result_DO(Result_D),
			.Fflags_SO(Fflags_S),
			.Ready_SO(Ready_S),
			.Done_SO(Done_S)
		);
	endgenerate
	reg [C_OP_FP64 - 1:0] Result_dly_S_D;
	reg Ready_dly_S_S;
	reg Done_dly_S_S;
	reg [4:0] Fflags_dly_S_S;
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI) begin
			Result_dly_S_D <= {C_OP_FP64 {1'sb0}};
			Ready_dly_S_S <= 1'b0;
			Done_dly_S_S <= 1'b0;
			Fflags_dly_S_S <= 1'b0;
		end
		else begin
			Result_dly_S_D <= Result_D;
			Ready_dly_S_S <= Ready_S;
			Done_dly_S_S <= Done_S;
			Fflags_dly_S_S <= Fflags_S;
		end
	reg [C_OP_FP64 - 1:0] Result_dly_D_D;
	reg Ready_dly_D_S;
	reg Done_dly_D_S;
	reg [4:0] Fflags_dly_D_S;
	generate
		if (PostPipeline_depth_S == 2) begin
			always @(posedge Clk_CI or negedge Rst_RBI)
				if (~Rst_RBI) begin
					Result_dly_D_D <= {C_OP_FP64 {1'sb0}};
					Ready_dly_D_S <= 1'b0;
					Done_dly_D_S <= 1'b0;
					Fflags_dly_D_S <= 1'b0;
				end
				else begin
					Result_dly_D_D <= Result_dly_S_D;
					Ready_dly_D_S <= Ready_dly_S_S;
					Done_dly_D_S <= Done_dly_S_S;
					Fflags_dly_D_S <= Fflags_dly_S_S;
				end
			assign Result_DO = Result_dly_D_D;
			assign Ready_SO = Ready_dly_D_S;
			assign Done_SO = Done_dly_D_S;
			assign Fflags_SO = Fflags_dly_D_S;
		end
		else begin
			assign Result_DO = Result_dly_S_D;
			assign Ready_SO = Ready_dly_S_S;
			assign Done_SO = Done_dly_S_S;
			assign Fflags_SO = Fflags_dly_S_S;
		end
	endgenerate
endmodule
module div_sqrt_top_mvp (
	Clk_CI,
	Rst_RBI,
	Div_start_SI,
	Sqrt_start_SI,
	Operand_a_DI,
	Operand_b_DI,
	RM_SI,
	Precision_ctl_SI,
	Format_sel_SI,
	Kill_SI,
	Result_DO,
	Fflags_SO,
	Ready_SO,
	Done_SO
);
	input wire Clk_CI;
	input wire Rst_RBI;
	input wire Div_start_SI;
	input wire Sqrt_start_SI;
	input wire [C_OP_FP64 - 1:0] Operand_a_DI;
	input wire [C_OP_FP64 - 1:0] Operand_b_DI;
	input wire [C_RM - 1:0] RM_SI;
	input wire [C_PC - 1:0] Precision_ctl_SI;
	input wire [C_FS - 1:0] Format_sel_SI;
	input wire Kill_SI;
	output wire [C_OP_FP64 - 1:0] Result_DO;
	output wire [4:0] Fflags_SO;
	output wire Ready_SO;
	output wire Done_SO;
	wire [C_EXP_FP64:0] Exp_a_D;
	wire [C_EXP_FP64:0] Exp_b_D;
	wire [C_MANT_FP64:0] Mant_a_D;
	wire [C_MANT_FP64:0] Mant_b_D;
	wire [C_EXP_FP64 + 1:0] Exp_z_D;
	wire [C_MANT_FP64 + 4:0] Mant_z_D;
	wire Sign_z_D;
	wire Start_S;
	wire [C_RM - 1:0] RM_dly_S;
	wire Div_enable_S;
	wire Sqrt_enable_S;
	wire Inf_a_S;
	wire Inf_b_S;
	wire Zero_a_S;
	wire Zero_b_S;
	wire NaN_a_S;
	wire NaN_b_S;
	wire SNaN_S;
	wire Special_case_SB;
	wire Special_case_dly_SB;
	wire Full_precision_S;
	wire FP32_S;
	wire FP64_S;
	wire FP16_S;
	wire FP16ALT_S;
	preprocess_mvp preprocess_U0(
		.Clk_CI(Clk_CI),
		.Rst_RBI(Rst_RBI),
		.Div_start_SI(Div_start_SI),
		.Sqrt_start_SI(Sqrt_start_SI),
		.Ready_SI(Ready_SO),
		.Operand_a_DI(Operand_a_DI),
		.Operand_b_DI(Operand_b_DI),
		.RM_SI(RM_SI),
		.Format_sel_SI(Format_sel_SI),
		.Start_SO(Start_S),
		.Exp_a_DO_norm(Exp_a_D),
		.Exp_b_DO_norm(Exp_b_D),
		.Mant_a_DO_norm(Mant_a_D),
		.Mant_b_DO_norm(Mant_b_D),
		.RM_dly_SO(RM_dly_S),
		.Sign_z_DO(Sign_z_D),
		.Inf_a_SO(Inf_a_S),
		.Inf_b_SO(Inf_b_S),
		.Zero_a_SO(Zero_a_S),
		.Zero_b_SO(Zero_b_S),
		.NaN_a_SO(NaN_a_S),
		.NaN_b_SO(NaN_b_S),
		.SNaN_SO(SNaN_S),
		.Special_case_SBO(Special_case_SB),
		.Special_case_dly_SBO(Special_case_dly_SB)
	);
	nrbd_nrsc_mvp nrbd_nrsc_U0(
		.Clk_CI(Clk_CI),
		.Rst_RBI(Rst_RBI),
		.Div_start_SI(Div_start_SI),
		.Sqrt_start_SI(Sqrt_start_SI),
		.Start_SI(Start_S),
		.Kill_SI(Kill_SI),
		.Special_case_SBI(Special_case_SB),
		.Special_case_dly_SBI(Special_case_dly_SB),
		.Div_enable_SO(Div_enable_S),
		.Sqrt_enable_SO(Sqrt_enable_S),
		.Precision_ctl_SI(Precision_ctl_SI),
		.Format_sel_SI(Format_sel_SI),
		.Exp_a_DI(Exp_a_D),
		.Exp_b_DI(Exp_b_D),
		.Mant_a_DI(Mant_a_D),
		.Mant_b_DI(Mant_b_D),
		.Full_precision_SO(Full_precision_S),
		.FP32_SO(FP32_S),
		.FP64_SO(FP64_S),
		.FP16_SO(FP16_S),
		.FP16ALT_SO(FP16ALT_S),
		.Ready_SO(Ready_SO),
		.Done_SO(Done_SO),
		.Exp_z_DO(Exp_z_D),
		.Mant_z_DO(Mant_z_D)
	);
	norm_div_sqrt_mvp fpu_norm_U0(
		.Mant_in_DI(Mant_z_D),
		.Exp_in_DI(Exp_z_D),
		.Sign_in_DI(Sign_z_D),
		.Div_enable_SI(Div_enable_S),
		.Sqrt_enable_SI(Sqrt_enable_S),
		.Inf_a_SI(Inf_a_S),
		.Inf_b_SI(Inf_b_S),
		.Zero_a_SI(Zero_a_S),
		.Zero_b_SI(Zero_b_S),
		.NaN_a_SI(NaN_a_S),
		.NaN_b_SI(NaN_b_S),
		.SNaN_SI(SNaN_S),
		.RM_SI(RM_dly_S),
		.Full_precision_SI(Full_precision_S),
		.FP32_SI(FP32_S),
		.FP64_SI(FP64_S),
		.FP16_SI(FP16_S),
		.FP16ALT_SI(FP16ALT_S),
		.Result_DO(Result_DO),
		.Fflags_SO(Fflags_SO)
	);
endmodule
module fpnew_classifier (
	operands_i,
	is_boxed_i,
	info_o
);
	localparam [31:0] fpnew_pkg_NUM_FP_FORMATS = 5;
	localparam [31:0] fpnew_pkg_FP_FORMAT_BITS = 3;
	function automatic [2:0] sv2v_cast_9E068;
		input reg [2:0] inp;
		sv2v_cast_9E068 = inp;
	endfunction
	parameter [2:0] FpFormat = sv2v_cast_9E068(0);
	parameter [31:0] NumOperands = 1;
	localparam [319:0] fpnew_pkg_FP_ENCODINGS = 320'h8000000170000000b00000034000000050000000a00000005000000020000000800000007;
	function automatic [31:0] fpnew_pkg_fp_width;
		input reg [2:0] fmt;
		fpnew_pkg_fp_width = (fpnew_pkg_FP_ENCODINGS[((4 - fmt) * 64) + 63-:32] + fpnew_pkg_FP_ENCODINGS[((4 - fmt) * 64) + 31-:32]) + 1;
	endfunction
	localparam [31:0] WIDTH = fpnew_pkg_fp_width(FpFormat);
	input wire [(NumOperands * WIDTH) - 1:0] operands_i;
	input wire [NumOperands - 1:0] is_boxed_i;
	output reg [(NumOperands * 8) - 1:0] info_o;
	function automatic [31:0] fpnew_pkg_exp_bits;
		input reg [2:0] fmt;
		fpnew_pkg_exp_bits = fpnew_pkg_FP_ENCODINGS[((4 - fmt) * 64) + 63-:32];
	endfunction
	localparam [31:0] EXP_BITS = fpnew_pkg_exp_bits(FpFormat);
	function automatic [31:0] fpnew_pkg_man_bits;
		input reg [2:0] fmt;
		fpnew_pkg_man_bits = fpnew_pkg_FP_ENCODINGS[((4 - fmt) * 64) + 31-:32];
	endfunction
	localparam [31:0] MAN_BITS = fpnew_pkg_man_bits(FpFormat);
	generate
		genvar op;
		function automatic signed [31:0] sv2v_cast_32_signed;
			input reg signed [31:0] inp;
			sv2v_cast_32_signed = inp;
		endfunction
		for (op = 0; op < sv2v_cast_32_signed(NumOperands); op = op + 1) begin : gen_num_values
			reg [((1 + EXP_BITS) + MAN_BITS) - 1:0] value;
			reg is_boxed;
			reg is_normal;
			reg is_inf;
			reg is_nan;
			reg is_signalling;
			reg is_quiet;
			reg is_zero;
			reg is_subnormal;
			always @(*) begin : classify_input
				value = operands_i[op * WIDTH+:WIDTH];
				is_boxed = is_boxed_i[op];
				is_normal = (is_boxed && (value[EXP_BITS + (MAN_BITS - 1)-:((EXP_BITS + (MAN_BITS - 1)) >= MAN_BITS ? ((EXP_BITS + (MAN_BITS - 1)) - MAN_BITS) + 1 : (MAN_BITS - (EXP_BITS + (MAN_BITS - 1))) + 1)] != {((EXP_BITS + (MAN_BITS - 1)) >= MAN_BITS ? ((EXP_BITS + (MAN_BITS - 1)) - MAN_BITS) + 1 : (MAN_BITS - (EXP_BITS + (MAN_BITS - 1))) + 1) {1'sb0}})) && (value[EXP_BITS + (MAN_BITS - 1)-:((EXP_BITS + (MAN_BITS - 1)) >= MAN_BITS ? ((EXP_BITS + (MAN_BITS - 1)) - MAN_BITS) + 1 : (MAN_BITS - (EXP_BITS + (MAN_BITS - 1))) + 1)] != {((EXP_BITS + (MAN_BITS - 1)) >= MAN_BITS ? ((EXP_BITS + (MAN_BITS - 1)) - MAN_BITS) + 1 : (MAN_BITS - (EXP_BITS + (MAN_BITS - 1))) + 1) {1'sb1}});
				is_zero = (is_boxed && (value[EXP_BITS + (MAN_BITS - 1)-:((EXP_BITS + (MAN_BITS - 1)) >= MAN_BITS ? ((EXP_BITS + (MAN_BITS - 1)) - MAN_BITS) + 1 : (MAN_BITS - (EXP_BITS + (MAN_BITS - 1))) + 1)] == {((EXP_BITS + (MAN_BITS - 1)) >= MAN_BITS ? ((EXP_BITS + (MAN_BITS - 1)) - MAN_BITS) + 1 : (MAN_BITS - (EXP_BITS + (MAN_BITS - 1))) + 1) {1'sb0}})) && (value[MAN_BITS - 1-:MAN_BITS] == {MAN_BITS {1'sb0}});
				is_subnormal = (is_boxed && (value[EXP_BITS + (MAN_BITS - 1)-:((EXP_BITS + (MAN_BITS - 1)) >= MAN_BITS ? ((EXP_BITS + (MAN_BITS - 1)) - MAN_BITS) + 1 : (MAN_BITS - (EXP_BITS + (MAN_BITS - 1))) + 1)] == {((EXP_BITS + (MAN_BITS - 1)) >= MAN_BITS ? ((EXP_BITS + (MAN_BITS - 1)) - MAN_BITS) + 1 : (MAN_BITS - (EXP_BITS + (MAN_BITS - 1))) + 1) {1'sb0}})) && !is_zero;
				is_inf = is_boxed && ((value[EXP_BITS + (MAN_BITS - 1)-:((EXP_BITS + (MAN_BITS - 1)) >= MAN_BITS ? ((EXP_BITS + (MAN_BITS - 1)) - MAN_BITS) + 1 : (MAN_BITS - (EXP_BITS + (MAN_BITS - 1))) + 1)] == {((EXP_BITS + (MAN_BITS - 1)) >= MAN_BITS ? ((EXP_BITS + (MAN_BITS - 1)) - MAN_BITS) + 1 : (MAN_BITS - (EXP_BITS + (MAN_BITS - 1))) + 1) {1'sb1}}) && (value[MAN_BITS - 1-:MAN_BITS] == {MAN_BITS {1'sb0}}));
				is_nan = !is_boxed || ((value[EXP_BITS + (MAN_BITS - 1)-:((EXP_BITS + (MAN_BITS - 1)) >= MAN_BITS ? ((EXP_BITS + (MAN_BITS - 1)) - MAN_BITS) + 1 : (MAN_BITS - (EXP_BITS + (MAN_BITS - 1))) + 1)] == {((EXP_BITS + (MAN_BITS - 1)) >= MAN_BITS ? ((EXP_BITS + (MAN_BITS - 1)) - MAN_BITS) + 1 : (MAN_BITS - (EXP_BITS + (MAN_BITS - 1))) + 1) {1'sb1}}) && (value[MAN_BITS - 1-:MAN_BITS] != {MAN_BITS {1'sb0}}));
				is_signalling = (is_boxed && is_nan) && (value[(MAN_BITS - 1) - ((MAN_BITS - 1) - (MAN_BITS - 1))] == 1'b0);
				is_quiet = is_nan && !is_signalling;
				info_o[(op * 8) + 7] = is_normal;
				info_o[(op * 8) + 6] = is_subnormal;
				info_o[(op * 8) + 5] = is_zero;
				info_o[(op * 8) + 4] = is_inf;
				info_o[(op * 8) + 3] = is_nan;
				info_o[(op * 8) + 2] = is_signalling;
				info_o[(op * 8) + 1] = is_quiet;
				info_o[op * 8] = is_boxed;
			end
		end
	endgenerate
endmodule
module fpnew_rounding (
	abs_value_i,
	sign_i,
	round_sticky_bits_i,
	rnd_mode_i,
	effective_subtraction_i,
	abs_rounded_o,
	sign_o,
	exact_zero_o
);
	parameter [31:0] AbsWidth = 2;
	input wire [AbsWidth - 1:0] abs_value_i;
	input wire sign_i;
	input wire [1:0] round_sticky_bits_i;
	input wire [2:0] rnd_mode_i;
	input wire effective_subtraction_i;
	output wire [AbsWidth - 1:0] abs_rounded_o;
	output wire sign_o;
	output wire exact_zero_o;
	reg round_up;
	localparam [0:0] fpnew_pkg_DONT_CARE = 1'b1;
	localparam [2:0] fpnew_pkg_RDN = 3'b010;
	localparam [2:0] fpnew_pkg_RMM = 3'b100;
	localparam [2:0] fpnew_pkg_RNE = 3'b000;
	localparam [2:0] fpnew_pkg_RTZ = 3'b001;
	localparam [2:0] fpnew_pkg_RUP = 3'b011;
	always @(*) begin : rounding_decision
		case (rnd_mode_i)
			fpnew_pkg_RNE:
				case (round_sticky_bits_i)
					2'b00, 2'b01: round_up = 1'b0;
					2'b10: round_up = abs_value_i[0];
					2'b11: round_up = 1'b1;
					default: round_up = fpnew_pkg_DONT_CARE;
				endcase
			fpnew_pkg_RTZ: round_up = 1'b0;
			fpnew_pkg_RDN: round_up = (|round_sticky_bits_i ? sign_i : 1'b0);
			fpnew_pkg_RUP: round_up = (|round_sticky_bits_i ? ~sign_i : 1'b0);
			fpnew_pkg_RMM: round_up = round_sticky_bits_i[1];
			default: round_up = fpnew_pkg_DONT_CARE;
		endcase
	end
	assign abs_rounded_o = abs_value_i + round_up;
	assign exact_zero_o = (abs_value_i == {AbsWidth {1'sb0}}) && (round_sticky_bits_i == {2 {1'sb0}});
	assign sign_o = (exact_zero_o && effective_subtraction_i ? rnd_mode_i == fpnew_pkg_RDN : sign_i);
endmodule
module iteration_div_sqrt_mvp (
	A_DI,
	B_DI,
	Div_enable_SI,
	Div_start_dly_SI,
	Sqrt_enable_SI,
	D_DI,
	D_DO,
	Sum_DO,
	Carry_out_DO
);
	parameter WIDTH = 25;
	input wire [WIDTH - 1:0] A_DI;
	input wire [WIDTH - 1:0] B_DI;
	input wire Div_enable_SI;
	input wire Div_start_dly_SI;
	input wire Sqrt_enable_SI;
	input wire [1:0] D_DI;
	output wire [1:0] D_DO;
	output wire [WIDTH - 1:0] Sum_DO;
	output wire Carry_out_DO;
	wire D_carry_D;
	wire Sqrt_cin_D;
	wire Cin_D;
	assign D_DO[0] = ~D_DI[0];
	assign D_DO[1] = ~(D_DI[1] ^ D_DI[0]);
	assign D_carry_D = D_DI[1] | D_DI[0];
	assign Sqrt_cin_D = Sqrt_enable_SI && D_carry_D;
	assign Cin_D = (Div_enable_SI ? 1'b0 : Sqrt_cin_D);
	assign {Carry_out_DO, Sum_DO} = (A_DI + B_DI) + Cin_D;
endmodule
module lzc (
	in_i,
	cnt_o,
	empty_o
);
	parameter [31:0] WIDTH = 2;
	parameter [0:0] MODE = 1'b0;
	function automatic [31:0] cf_math_pkg_idx_width;
		input reg [31:0] num_idx;
		cf_math_pkg_idx_width = (num_idx > 32'd1 ? $unsigned($clog2(num_idx)) : 32'd1);
	endfunction
	parameter [31:0] CNT_WIDTH = cf_math_pkg_idx_width(WIDTH);
	input wire [WIDTH - 1:0] in_i;
	output wire [CNT_WIDTH - 1:0] cnt_o;
	output wire empty_o;
	generate
		if (WIDTH == 1) begin : gen_degenerate_lzc
			assign cnt_o[0] = !in_i[0];
			assign empty_o = !in_i[0];
		end
		else begin : gen_lzc
			localparam [31:0] NumLevels = $clog2(WIDTH);
			wire [(WIDTH * NumLevels) - 1:0] index_lut;
			wire [(2 ** NumLevels) - 1:0] sel_nodes;
			wire [((2 ** NumLevels) * NumLevels) - 1:0] index_nodes;
			reg [WIDTH - 1:0] in_tmp;
			always @(*) begin : flip_vector
				begin : sv2v_autoblock_4
					reg [31:0] i;
					for (i = 0; i < WIDTH; i = i + 1)
						in_tmp[i] = (MODE ? in_i[(WIDTH - 1) - i] : in_i[i]);
				end
			end
			genvar j;
			for (j = 0; $unsigned(j) < WIDTH; j = j + 1) begin : g_index_lut
				function automatic [NumLevels - 1:0] sv2v_cast_4C5E6;
					input reg [NumLevels - 1:0] inp;
					sv2v_cast_4C5E6 = inp;
				endfunction
				assign index_lut[j * NumLevels+:NumLevels] = sv2v_cast_4C5E6($unsigned(j));
			end
			genvar level;
			for (level = 0; $unsigned(level) < NumLevels; level = level + 1) begin : g_levels
				if ($unsigned(level) == (NumLevels - 1)) begin : g_last_level
					genvar k;
					for (k = 0; k < (2 ** level); k = k + 1) begin : g_level
						if (($unsigned(k) * 2) < (WIDTH - 1)) begin : g_reduce
							assign sel_nodes[((2 ** level) - 1) + k] = in_tmp[k * 2] | in_tmp[(k * 2) + 1];
							assign index_nodes[(((2 ** level) - 1) + k) * NumLevels+:NumLevels] = (in_tmp[k * 2] == 1'b1 ? index_lut[(k * 2) * NumLevels+:NumLevels] : index_lut[((k * 2) + 1) * NumLevels+:NumLevels]);
						end
						if (($unsigned(k) * 2) == (WIDTH - 1)) begin : g_base
							assign sel_nodes[((2 ** level) - 1) + k] = in_tmp[k * 2];
							assign index_nodes[(((2 ** level) - 1) + k) * NumLevels+:NumLevels] = index_lut[(k * 2) * NumLevels+:NumLevels];
						end
						if (($unsigned(k) * 2) > (WIDTH - 1)) begin : g_out_of_range
							assign sel_nodes[((2 ** level) - 1) + k] = 1'b0;
							assign index_nodes[(((2 ** level) - 1) + k) * NumLevels+:NumLevels] = {NumLevels {1'sb0}};
						end
					end
				end
				else begin : g_not_last_level
					genvar l;
					for (l = 0; l < (2 ** level); l = l + 1) begin : g_level
						assign sel_nodes[((2 ** level) - 1) + l] = sel_nodes[((2 ** (level + 1)) - 1) + (l * 2)] | sel_nodes[(((2 ** (level + 1)) - 1) + (l * 2)) + 1];
						assign index_nodes[(((2 ** level) - 1) + l) * NumLevels+:NumLevels] = (sel_nodes[((2 ** (level + 1)) - 1) + (l * 2)] == 1'b1 ? index_nodes[(((2 ** (level + 1)) - 1) + (l * 2)) * NumLevels+:NumLevels] : index_nodes[((((2 ** (level + 1)) - 1) + (l * 2)) + 1) * NumLevels+:NumLevels]);
					end
				end
			end
			assign cnt_o = (NumLevels > $unsigned(0) ? index_nodes[0+:NumLevels] : {$clog2(WIDTH) {1'b0}});
			assign empty_o = (NumLevels > $unsigned(0) ? ~sel_nodes[0] : ~(|in_i));
		end
	endgenerate
endmodule
module norm_div_sqrt_mvp (
	Mant_in_DI,
	Exp_in_DI,
	Sign_in_DI,
	Div_enable_SI,
	Sqrt_enable_SI,
	Inf_a_SI,
	Inf_b_SI,
	Zero_a_SI,
	Zero_b_SI,
	NaN_a_SI,
	NaN_b_SI,
	SNaN_SI,
	RM_SI,
	Full_precision_SI,
	FP32_SI,
	FP64_SI,
	FP16_SI,
	FP16ALT_SI,
	Result_DO,
	Fflags_SO
);
	input wire [C_MANT_FP64 + 4:0] Mant_in_DI;
	input wire signed [C_EXP_FP64 + 1:0] Exp_in_DI;
	input wire Sign_in_DI;
	input wire Div_enable_SI;
	input wire Sqrt_enable_SI;
	input wire Inf_a_SI;
	input wire Inf_b_SI;
	input wire Zero_a_SI;
	input wire Zero_b_SI;
	input wire NaN_a_SI;
	input wire NaN_b_SI;
	input wire SNaN_SI;
	input wire [C_RM - 1:0] RM_SI;
	input wire Full_precision_SI;
	input wire FP32_SI;
	input wire FP64_SI;
	input wire FP16_SI;
	input wire FP16ALT_SI;
	output reg [C_EXP_FP64 + C_MANT_FP64:0] Result_DO;
	output wire [4:0] Fflags_SO;
	reg Sign_res_D;
	reg NV_OP_S;
	reg Exp_OF_S;
	reg Exp_UF_S;
	reg Div_Zero_S;
	wire In_Exact_S;
	reg [C_MANT_FP64:0] Mant_res_norm_D;
	reg [C_EXP_FP64 - 1:0] Exp_res_norm_D;
	wire [C_EXP_FP64 + 1:0] Exp_Max_RS_FP64_D;
	wire [C_EXP_FP32 + 1:0] Exp_Max_RS_FP32_D;
	wire [C_EXP_FP16 + 1:0] Exp_Max_RS_FP16_D;
	wire [C_EXP_FP16ALT + 1:0] Exp_Max_RS_FP16ALT_D;
	assign Exp_Max_RS_FP64_D = (Exp_in_DI[C_EXP_FP64:0] + C_MANT_FP64) + 1;
	assign Exp_Max_RS_FP32_D = (Exp_in_DI[C_EXP_FP32:0] + C_MANT_FP32) + 1;
	assign Exp_Max_RS_FP16_D = (Exp_in_DI[C_EXP_FP16:0] + C_MANT_FP16) + 1;
	assign Exp_Max_RS_FP16ALT_D = (Exp_in_DI[C_EXP_FP16ALT:0] + C_MANT_FP16ALT) + 1;
	wire [C_EXP_FP64 + 1:0] Num_RS_D;
	assign Num_RS_D = ~Exp_in_DI + 2;
	wire [C_MANT_FP64:0] Mant_RS_D;
	wire [C_MANT_FP64 + 4:0] Mant_forsticky_D;
	assign {Mant_RS_D, Mant_forsticky_D} = {Mant_in_DI, {C_MANT_FP64 + 1 {1'b0}}} >> Num_RS_D;
	wire [C_EXP_FP64 + 1:0] Exp_subOne_D;
	assign Exp_subOne_D = Exp_in_DI - 1;
	reg [1:0] Mant_lower_D;
	reg Mant_sticky_bit_D;
	reg [C_MANT_FP64 + 4:0] Mant_forround_D;
	always @(*)
		if (NaN_a_SI) begin
			Div_Zero_S = 1'b0;
			Exp_OF_S = 1'b0;
			Exp_UF_S = 1'b0;
			Mant_res_norm_D = {1'b0, C_MANT_NAN_FP64};
			Exp_res_norm_D = {C_EXP_FP64 {1'sb1}};
			Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
			Sign_res_D = 1'b0;
			NV_OP_S = SNaN_SI;
		end
		else if (NaN_b_SI) begin
			Div_Zero_S = 1'b0;
			Exp_OF_S = 1'b0;
			Exp_UF_S = 1'b0;
			Mant_res_norm_D = {1'b0, C_MANT_NAN_FP64};
			Exp_res_norm_D = {C_EXP_FP64 {1'sb1}};
			Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
			Sign_res_D = 1'b0;
			NV_OP_S = SNaN_SI;
		end
		else if (Inf_a_SI) begin
			if (Div_enable_SI && Inf_b_SI) begin
				Div_Zero_S = 1'b0;
				Exp_OF_S = 1'b0;
				Exp_UF_S = 1'b0;
				Mant_res_norm_D = {1'b0, C_MANT_NAN_FP64};
				Exp_res_norm_D = {C_EXP_FP64 {1'sb1}};
				Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
				Sign_res_D = 1'b0;
				NV_OP_S = 1'b1;
			end
			else if (Sqrt_enable_SI && Sign_in_DI) begin
				Div_Zero_S = 1'b0;
				Exp_OF_S = 1'b0;
				Exp_UF_S = 1'b0;
				Mant_res_norm_D = {1'b0, C_MANT_NAN_FP64};
				Exp_res_norm_D = {C_EXP_FP64 {1'sb1}};
				Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
				Sign_res_D = 1'b0;
				NV_OP_S = 1'b1;
			end
			else begin
				Div_Zero_S = 1'b0;
				Exp_OF_S = 1'b1;
				Exp_UF_S = 1'b0;
				Mant_res_norm_D = {(C_MANT_FP64 >= 0 ? C_MANT_FP64 + 1 : 1 - C_MANT_FP64) {1'sb0}};
				Exp_res_norm_D = {C_EXP_FP64 {1'sb1}};
				Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
				Sign_res_D = Sign_in_DI;
				NV_OP_S = 1'b0;
			end
		end
		else if (Div_enable_SI && Inf_b_SI) begin
			Div_Zero_S = 1'b0;
			Exp_OF_S = 1'b1;
			Exp_UF_S = 1'b0;
			Mant_res_norm_D = {(C_MANT_FP64 >= 0 ? C_MANT_FP64 + 1 : 1 - C_MANT_FP64) {1'sb0}};
			Exp_res_norm_D = {C_EXP_FP64 {1'sb0}};
			Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
			Sign_res_D = Sign_in_DI;
			NV_OP_S = 1'b0;
		end
		else if (Zero_a_SI) begin
			if (Div_enable_SI && Zero_b_SI) begin
				Div_Zero_S = 1'b1;
				Exp_OF_S = 1'b0;
				Exp_UF_S = 1'b0;
				Mant_res_norm_D = {1'b0, C_MANT_NAN_FP64};
				Exp_res_norm_D = {C_EXP_FP64 {1'sb1}};
				Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
				Sign_res_D = 1'b0;
				NV_OP_S = 1'b1;
			end
			else begin
				Div_Zero_S = 1'b0;
				Exp_OF_S = 1'b0;
				Exp_UF_S = 1'b0;
				Mant_res_norm_D = {(C_MANT_FP64 >= 0 ? C_MANT_FP64 + 1 : 1 - C_MANT_FP64) {1'sb0}};
				Exp_res_norm_D = {C_EXP_FP64 {1'sb0}};
				Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
				Sign_res_D = Sign_in_DI;
				NV_OP_S = 1'b0;
			end
		end
		else if (Div_enable_SI && Zero_b_SI) begin
			Div_Zero_S = 1'b1;
			Exp_OF_S = 1'b0;
			Exp_UF_S = 1'b0;
			Mant_res_norm_D = {(C_MANT_FP64 >= 0 ? C_MANT_FP64 + 1 : 1 - C_MANT_FP64) {1'sb0}};
			Exp_res_norm_D = {C_EXP_FP64 {1'sb1}};
			Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
			Sign_res_D = Sign_in_DI;
			NV_OP_S = 1'b0;
		end
		else if (Sign_in_DI && Sqrt_enable_SI) begin
			Div_Zero_S = 1'b0;
			Exp_OF_S = 1'b0;
			Exp_UF_S = 1'b0;
			Mant_res_norm_D = {1'b0, C_MANT_NAN_FP64};
			Exp_res_norm_D = {C_EXP_FP64 {1'sb1}};
			Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
			Sign_res_D = 1'b0;
			NV_OP_S = 1'b1;
		end
		else if (Exp_in_DI[C_EXP_FP64:0] == {(C_EXP_FP64 >= 0 ? C_EXP_FP64 + 1 : 1 - C_EXP_FP64) {1'sb0}}) begin
			if (Mant_in_DI != {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}}) begin
				Div_Zero_S = 1'b0;
				Exp_OF_S = 1'b0;
				Exp_UF_S = 1'b1;
				Mant_res_norm_D = {1'b0, Mant_in_DI[C_MANT_FP64 + 4:5]};
				Exp_res_norm_D = {C_EXP_FP64 {1'sb0}};
				Mant_forround_D = {Mant_in_DI[4:0], {C_MANT_FP64 {1'b0}}};
				Sign_res_D = Sign_in_DI;
				NV_OP_S = 1'b0;
			end
			else begin
				Div_Zero_S = 1'b0;
				Exp_OF_S = 1'b0;
				Exp_UF_S = 1'b0;
				Mant_res_norm_D = {(C_MANT_FP64 >= 0 ? C_MANT_FP64 + 1 : 1 - C_MANT_FP64) {1'sb0}};
				Exp_res_norm_D = {C_EXP_FP64 {1'sb0}};
				Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
				Sign_res_D = Sign_in_DI;
				NV_OP_S = 1'b0;
			end
		end
		else if ((Exp_in_DI[C_EXP_FP64:0] == C_EXP_ONE_FP64) && ~Mant_in_DI[C_MANT_FP64 + 4]) begin
			Div_Zero_S = 1'b0;
			Exp_OF_S = 1'b0;
			Exp_UF_S = 1'b1;
			Mant_res_norm_D = Mant_in_DI[C_MANT_FP64 + 4:4];
			Exp_res_norm_D = {C_EXP_FP64 {1'sb0}};
			Mant_forround_D = {Mant_in_DI[3:0], {C_MANT_FP64 + 1 {1'b0}}};
			Sign_res_D = Sign_in_DI;
			NV_OP_S = 1'b0;
		end
		else if (Exp_in_DI[C_EXP_FP64 + 1]) begin
			Div_Zero_S = 1'b0;
			Exp_OF_S = 1'b0;
			Exp_UF_S = 1'b1;
			Mant_res_norm_D = {Mant_RS_D[C_MANT_FP64:0]};
			Exp_res_norm_D = {C_EXP_FP64 {1'sb0}};
			Mant_forround_D = {Mant_forsticky_D[C_MANT_FP64 + 4:0]};
			Sign_res_D = Sign_in_DI;
			NV_OP_S = 1'b0;
		end
		else if ((((Exp_in_DI[C_EXP_FP32] && FP32_SI) | (Exp_in_DI[C_EXP_FP64] && FP64_SI)) | (Exp_in_DI[C_EXP_FP16] && FP16_SI)) | (Exp_in_DI[C_EXP_FP16ALT] && FP16ALT_SI)) begin
			Div_Zero_S = 1'b0;
			Exp_OF_S = 1'b1;
			Exp_UF_S = 1'b0;
			Mant_res_norm_D = {(C_MANT_FP64 >= 0 ? C_MANT_FP64 + 1 : 1 - C_MANT_FP64) {1'sb0}};
			Exp_res_norm_D = {C_EXP_FP64 {1'sb1}};
			Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
			Sign_res_D = Sign_in_DI;
			NV_OP_S = 1'b0;
		end
		else if (((((Exp_in_DI[C_EXP_FP32 - 1:0] == {C_EXP_FP32 {1'sb1}}) && FP32_SI) | ((Exp_in_DI[C_EXP_FP64 - 1:0] == {C_EXP_FP64 {1'sb1}}) && FP64_SI)) | ((Exp_in_DI[C_EXP_FP16 - 1:0] == {C_EXP_FP16 {1'sb1}}) && FP16_SI)) | ((Exp_in_DI[C_EXP_FP16ALT - 1:0] == {C_EXP_FP16ALT {1'sb1}}) && FP16ALT_SI)) begin
			if (~Mant_in_DI[C_MANT_FP64 + 4]) begin
				Div_Zero_S = 1'b0;
				Exp_OF_S = 1'b0;
				Exp_UF_S = 1'b0;
				Mant_res_norm_D = Mant_in_DI[C_MANT_FP64 + 3:3];
				Exp_res_norm_D = Exp_subOne_D;
				Mant_forround_D = {Mant_in_DI[2:0], {C_MANT_FP64 + 2 {1'b0}}};
				Sign_res_D = Sign_in_DI;
				NV_OP_S = 1'b0;
			end
			else if (Mant_in_DI != {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}}) begin
				Div_Zero_S = 1'b0;
				Exp_OF_S = 1'b1;
				Exp_UF_S = 1'b0;
				Mant_res_norm_D = {(C_MANT_FP64 >= 0 ? C_MANT_FP64 + 1 : 1 - C_MANT_FP64) {1'sb0}};
				Exp_res_norm_D = {C_EXP_FP64 {1'sb1}};
				Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
				Sign_res_D = Sign_in_DI;
				NV_OP_S = 1'b0;
			end
			else begin
				Div_Zero_S = 1'b0;
				Exp_OF_S = 1'b1;
				Exp_UF_S = 1'b0;
				Mant_res_norm_D = {(C_MANT_FP64 >= 0 ? C_MANT_FP64 + 1 : 1 - C_MANT_FP64) {1'sb0}};
				Exp_res_norm_D = {C_EXP_FP64 {1'sb1}};
				Mant_forround_D = {((C_MANT_FP64 + 4) >= 0 ? C_MANT_FP64 + 5 : 1 - (C_MANT_FP64 + 4)) {1'sb0}};
				Sign_res_D = Sign_in_DI;
				NV_OP_S = 1'b0;
			end
		end
		else if (Mant_in_DI[C_MANT_FP64 + 4]) begin
			Div_Zero_S = 1'b0;
			Exp_OF_S = 1'b0;
			Exp_UF_S = 1'b0;
			Mant_res_norm_D = Mant_in_DI[C_MANT_FP64 + 4:4];
			Exp_res_norm_D = Exp_in_DI[C_EXP_FP64 - 1:0];
			Mant_forround_D = {Mant_in_DI[3:0], {C_MANT_FP64 + 1 {1'b0}}};
			Sign_res_D = Sign_in_DI;
			NV_OP_S = 1'b0;
		end
		else begin
			Div_Zero_S = 1'b0;
			Exp_OF_S = 1'b0;
			Exp_UF_S = 1'b0;
			Mant_res_norm_D = Mant_in_DI[C_MANT_FP64 + 3:3];
			Exp_res_norm_D = Exp_subOne_D;
			Mant_forround_D = {Mant_in_DI[2:0], {C_MANT_FP64 + 2 {1'b0}}};
			Sign_res_D = Sign_in_DI;
			NV_OP_S = 1'b0;
		end
	reg [C_MANT_FP64:0] Mant_upper_D;
	wire [C_MANT_FP64 + 1:0] Mant_upperRounded_D;
	reg Mant_roundUp_S;
	wire Mant_rounded_S;
	always @(*)
		if (FP32_SI) begin
			Mant_upper_D = {Mant_res_norm_D[C_MANT_FP64:C_MANT_FP64 - C_MANT_FP32], {C_MANT_FP64 - C_MANT_FP32 {1'b0}}};
			Mant_lower_D = Mant_res_norm_D[(C_MANT_FP64 - C_MANT_FP32) - 1:(C_MANT_FP64 - C_MANT_FP32) - 2];
			Mant_sticky_bit_D = |Mant_res_norm_D[(C_MANT_FP64 - C_MANT_FP32) - 3:0];
		end
		else if (FP64_SI) begin
			Mant_upper_D = Mant_res_norm_D[C_MANT_FP64:0];
			Mant_lower_D = Mant_forround_D[C_MANT_FP64 + 4:C_MANT_FP64 + 3];
			Mant_sticky_bit_D = |Mant_forround_D[C_MANT_FP64 + 3:0];
		end
		else if (FP16_SI) begin
			Mant_upper_D = {Mant_res_norm_D[C_MANT_FP64:C_MANT_FP64 - C_MANT_FP16], {C_MANT_FP64 - C_MANT_FP16 {1'b0}}};
			Mant_lower_D = Mant_res_norm_D[(C_MANT_FP64 - C_MANT_FP16) - 1:(C_MANT_FP64 - C_MANT_FP16) - 2];
			Mant_sticky_bit_D = |Mant_res_norm_D[(C_MANT_FP64 - C_MANT_FP16) - 3:30];
		end
		else begin
			Mant_upper_D = {Mant_res_norm_D[C_MANT_FP64:C_MANT_FP64 - C_MANT_FP16ALT], {C_MANT_FP64 - C_MANT_FP16ALT {1'b0}}};
			Mant_lower_D = Mant_res_norm_D[(C_MANT_FP64 - C_MANT_FP16ALT) - 1:(C_MANT_FP64 - C_MANT_FP16ALT) - 2];
			Mant_sticky_bit_D = |Mant_res_norm_D[(C_MANT_FP64 - C_MANT_FP16ALT) - 3:30];
		end
	assign Mant_rounded_S = |Mant_lower_D | Mant_sticky_bit_D;
	always @(*) begin
		Mant_roundUp_S = 1'b0;
		case (RM_SI)
			C_RM_NEAREST: Mant_roundUp_S = Mant_lower_D[1] && ((Mant_lower_D[0] | Mant_sticky_bit_D) | ((((FP32_SI && Mant_upper_D[C_MANT_FP64 - C_MANT_FP32]) | (FP64_SI && Mant_upper_D[0])) | (FP16_SI && Mant_upper_D[C_MANT_FP64 - C_MANT_FP16])) | (FP16ALT_SI && Mant_upper_D[C_MANT_FP64 - C_MANT_FP16ALT])));
			C_RM_TRUNC: Mant_roundUp_S = 0;
			C_RM_PLUSINF: Mant_roundUp_S = Mant_rounded_S & ~Sign_in_DI;
			C_RM_MINUSINF: Mant_roundUp_S = Mant_rounded_S & Sign_in_DI;
			default: Mant_roundUp_S = 0;
		endcase
	end
	wire Mant_renorm_S;
	wire [C_MANT_FP64:0] Mant_roundUp_Vector_S;
	assign Mant_roundUp_Vector_S = {7'h00, FP16ALT_SI && Mant_roundUp_S, 2'h0, FP16_SI && Mant_roundUp_S, 12'h000, FP32_SI && Mant_roundUp_S, 28'h0000000, FP64_SI && Mant_roundUp_S};
	assign Mant_upperRounded_D = Mant_upper_D + Mant_roundUp_Vector_S;
	assign Mant_renorm_S = Mant_upperRounded_D[C_MANT_FP64 + 1];
	wire [C_MANT_FP64 - 1:0] Mant_res_round_D;
	wire [C_EXP_FP64 - 1:0] Exp_res_round_D;
	assign Mant_res_round_D = (Mant_renorm_S ? Mant_upperRounded_D[C_MANT_FP64:1] : Mant_upperRounded_D[C_MANT_FP64 - 1:0]);
	assign Exp_res_round_D = Exp_res_norm_D + Mant_renorm_S;
	wire [C_MANT_FP64 - 1:0] Mant_before_format_ctl_D;
	wire [C_EXP_FP64 - 1:0] Exp_before_format_ctl_D;
	assign Mant_before_format_ctl_D = (Full_precision_SI ? Mant_res_round_D : Mant_res_norm_D);
	assign Exp_before_format_ctl_D = (Full_precision_SI ? Exp_res_round_D : Exp_res_norm_D);
	always @(*)
		if (FP32_SI)
			Result_DO = {32'hffffffff, Sign_res_D, Exp_before_format_ctl_D[C_EXP_FP32 - 1:0], Mant_before_format_ctl_D[C_MANT_FP64 - 1:C_MANT_FP64 - C_MANT_FP32]};
		else if (FP64_SI)
			Result_DO = {Sign_res_D, Exp_before_format_ctl_D[C_EXP_FP64 - 1:0], Mant_before_format_ctl_D[C_MANT_FP64 - 1:0]};
		else if (FP16_SI)
			Result_DO = {48'hffffffffffff, Sign_res_D, Exp_before_format_ctl_D[C_EXP_FP16 - 1:0], Mant_before_format_ctl_D[C_MANT_FP64 - 1:C_MANT_FP64 - C_MANT_FP16]};
		else
			Result_DO = {48'hffffffffffff, Sign_res_D, Exp_before_format_ctl_D[C_EXP_FP16ALT - 1:0], Mant_before_format_ctl_D[C_MANT_FP64 - 1:C_MANT_FP64 - C_MANT_FP16ALT]};
	assign In_Exact_S = ~Full_precision_SI | Mant_rounded_S;
	assign Fflags_SO = {NV_OP_S, Div_Zero_S, Exp_OF_S, Exp_UF_S, In_Exact_S};
endmodule
module nrbd_nrsc_mvp (
	Clk_CI,
	Rst_RBI,
	Div_start_SI,
	Sqrt_start_SI,
	Start_SI,
	Kill_SI,
	Special_case_SBI,
	Special_case_dly_SBI,
	Precision_ctl_SI,
	Format_sel_SI,
	Mant_a_DI,
	Mant_b_DI,
	Exp_a_DI,
	Exp_b_DI,
	Div_enable_SO,
	Sqrt_enable_SO,
	Full_precision_SO,
	FP32_SO,
	FP64_SO,
	FP16_SO,
	FP16ALT_SO,
	Ready_SO,
	Done_SO,
	Mant_z_DO,
	Exp_z_DO
);
	input wire Clk_CI;
	input wire Rst_RBI;
	input wire Div_start_SI;
	input wire Sqrt_start_SI;
	input wire Start_SI;
	input wire Kill_SI;
	input wire Special_case_SBI;
	input wire Special_case_dly_SBI;
	input wire [C_PC - 1:0] Precision_ctl_SI;
	input wire [1:0] Format_sel_SI;
	input wire [C_MANT_FP64:0] Mant_a_DI;
	input wire [C_MANT_FP64:0] Mant_b_DI;
	input wire [C_EXP_FP64:0] Exp_a_DI;
	input wire [C_EXP_FP64:0] Exp_b_DI;
	output wire Div_enable_SO;
	output wire Sqrt_enable_SO;
	output wire Full_precision_SO;
	output wire FP32_SO;
	output wire FP64_SO;
	output wire FP16_SO;
	output wire FP16ALT_SO;
	output wire Ready_SO;
	output wire Done_SO;
	output wire [C_MANT_FP64 + 4:0] Mant_z_DO;
	output wire [C_EXP_FP64 + 1:0] Exp_z_DO;
	wire Div_start_dly_S;
	wire Sqrt_start_dly_S;
	control_mvp control_U0(
		.Clk_CI(Clk_CI),
		.Rst_RBI(Rst_RBI),
		.Div_start_SI(Div_start_SI),
		.Sqrt_start_SI(Sqrt_start_SI),
		.Start_SI(Start_SI),
		.Kill_SI(Kill_SI),
		.Special_case_SBI(Special_case_SBI),
		.Special_case_dly_SBI(Special_case_dly_SBI),
		.Precision_ctl_SI(Precision_ctl_SI),
		.Format_sel_SI(Format_sel_SI),
		.Numerator_DI(Mant_a_DI),
		.Exp_num_DI(Exp_a_DI),
		.Denominator_DI(Mant_b_DI),
		.Exp_den_DI(Exp_b_DI),
		.Div_start_dly_SO(Div_start_dly_S),
		.Sqrt_start_dly_SO(Sqrt_start_dly_S),
		.Div_enable_SO(Div_enable_SO),
		.Sqrt_enable_SO(Sqrt_enable_SO),
		.Full_precision_SO(Full_precision_SO),
		.FP32_SO(FP32_SO),
		.FP64_SO(FP64_SO),
		.FP16_SO(FP16_SO),
		.FP16ALT_SO(FP16ALT_SO),
		.Ready_SO(Ready_SO),
		.Done_SO(Done_SO),
		.Mant_result_prenorm_DO(Mant_z_DO),
		.Exp_result_prenorm_DO(Exp_z_DO)
	);
endmodule
module preprocess_mvp (
	Clk_CI,
	Rst_RBI,
	Div_start_SI,
	Sqrt_start_SI,
	Ready_SI,
	Operand_a_DI,
	Operand_b_DI,
	RM_SI,
	Format_sel_SI,
	Start_SO,
	Exp_a_DO_norm,
	Exp_b_DO_norm,
	Mant_a_DO_norm,
	Mant_b_DO_norm,
	RM_dly_SO,
	Sign_z_DO,
	Inf_a_SO,
	Inf_b_SO,
	Zero_a_SO,
	Zero_b_SO,
	NaN_a_SO,
	NaN_b_SO,
	SNaN_SO,
	Special_case_SBO,
	Special_case_dly_SBO
);
	input wire Clk_CI;
	input wire Rst_RBI;
	input wire Div_start_SI;
	input wire Sqrt_start_SI;
	input wire Ready_SI;
	input wire [C_OP_FP64 - 1:0] Operand_a_DI;
	input wire [C_OP_FP64 - 1:0] Operand_b_DI;
	input wire [C_RM - 1:0] RM_SI;
	input wire [C_FS - 1:0] Format_sel_SI;
	output wire Start_SO;
	output wire [C_EXP_FP64:0] Exp_a_DO_norm;
	output wire [C_EXP_FP64:0] Exp_b_DO_norm;
	output wire [C_MANT_FP64:0] Mant_a_DO_norm;
	output wire [C_MANT_FP64:0] Mant_b_DO_norm;
	output wire [C_RM - 1:0] RM_dly_SO;
	output wire Sign_z_DO;
	output wire Inf_a_SO;
	output wire Inf_b_SO;
	output wire Zero_a_SO;
	output wire Zero_b_SO;
	output wire NaN_a_SO;
	output wire NaN_b_SO;
	output wire SNaN_SO;
	output wire Special_case_SBO;
	output reg Special_case_dly_SBO;
	wire Hb_a_D;
	wire Hb_b_D;
	reg [C_EXP_FP64 - 1:0] Exp_a_D;
	reg [C_EXP_FP64 - 1:0] Exp_b_D;
	reg [C_MANT_FP64 - 1:0] Mant_a_NonH_D;
	reg [C_MANT_FP64 - 1:0] Mant_b_NonH_D;
	wire [C_MANT_FP64:0] Mant_a_D;
	wire [C_MANT_FP64:0] Mant_b_D;
	reg Sign_a_D;
	reg Sign_b_D;
	wire Start_S;
	always @(*)
		case (Format_sel_SI)
			2'b00: begin
				Sign_a_D = Operand_a_DI[C_OP_FP32 - 1];
				Sign_b_D = Operand_b_DI[C_OP_FP32 - 1];
				Exp_a_D = {3'h0, Operand_a_DI[C_OP_FP32 - 2:C_MANT_FP32]};
				Exp_b_D = {3'h0, Operand_b_DI[C_OP_FP32 - 2:C_MANT_FP32]};
				Mant_a_NonH_D = {Operand_a_DI[C_MANT_FP32 - 1:0], 29'h00000000};
				Mant_b_NonH_D = {Operand_b_DI[C_MANT_FP32 - 1:0], 29'h00000000};
			end
			2'b01: begin
				Sign_a_D = Operand_a_DI[C_OP_FP64 - 1];
				Sign_b_D = Operand_b_DI[C_OP_FP64 - 1];
				Exp_a_D = Operand_a_DI[C_OP_FP64 - 2:C_MANT_FP64];
				Exp_b_D = Operand_b_DI[C_OP_FP64 - 2:C_MANT_FP64];
				Mant_a_NonH_D = Operand_a_DI[C_MANT_FP64 - 1:0];
				Mant_b_NonH_D = Operand_b_DI[C_MANT_FP64 - 1:0];
			end
			2'b10: begin
				Sign_a_D = Operand_a_DI[C_OP_FP16 - 1];
				Sign_b_D = Operand_b_DI[C_OP_FP16 - 1];
				Exp_a_D = {6'h00, Operand_a_DI[C_OP_FP16 - 2:C_MANT_FP16]};
				Exp_b_D = {6'h00, Operand_b_DI[C_OP_FP16 - 2:C_MANT_FP16]};
				Mant_a_NonH_D = {Operand_a_DI[C_MANT_FP16 - 1:0], 42'h00000000000};
				Mant_b_NonH_D = {Operand_b_DI[C_MANT_FP16 - 1:0], 42'h00000000000};
			end
			2'b11: begin
				Sign_a_D = Operand_a_DI[C_OP_FP16ALT - 1];
				Sign_b_D = Operand_b_DI[C_OP_FP16ALT - 1];
				Exp_a_D = {3'h0, Operand_a_DI[C_OP_FP16ALT - 2:C_MANT_FP16ALT]};
				Exp_b_D = {3'h0, Operand_b_DI[C_OP_FP16ALT - 2:C_MANT_FP16ALT]};
				Mant_a_NonH_D = {Operand_a_DI[C_MANT_FP16ALT - 1:0], 45'h000000000000};
				Mant_b_NonH_D = {Operand_b_DI[C_MANT_FP16ALT - 1:0], 45'h000000000000};
			end
		endcase
	assign Mant_a_D = {Hb_a_D, Mant_a_NonH_D};
	assign Mant_b_D = {Hb_b_D, Mant_b_NonH_D};
	assign Hb_a_D = |Exp_a_D;
	assign Hb_b_D = |Exp_b_D;
	assign Start_S = Div_start_SI | Sqrt_start_SI;
	reg Mant_a_prenorm_zero_S;
	reg Mant_b_prenorm_zero_S;
	wire Exp_a_prenorm_zero_S;
	wire Exp_b_prenorm_zero_S;
	assign Exp_a_prenorm_zero_S = ~Hb_a_D;
	assign Exp_b_prenorm_zero_S = ~Hb_b_D;
	reg Exp_a_prenorm_Inf_NaN_S;
	reg Exp_b_prenorm_Inf_NaN_S;
	wire Mant_a_prenorm_QNaN_S;
	wire Mant_a_prenorm_SNaN_S;
	wire Mant_b_prenorm_QNaN_S;
	wire Mant_b_prenorm_SNaN_S;
	assign Mant_a_prenorm_QNaN_S = Mant_a_NonH_D[C_MANT_FP64 - 1] && ~(|Mant_a_NonH_D[C_MANT_FP64 - 2:0]);
	assign Mant_a_prenorm_SNaN_S = ~Mant_a_NonH_D[C_MANT_FP64 - 1] && |Mant_a_NonH_D[C_MANT_FP64 - 2:0];
	assign Mant_b_prenorm_QNaN_S = Mant_b_NonH_D[C_MANT_FP64 - 1] && ~(|Mant_b_NonH_D[C_MANT_FP64 - 2:0]);
	assign Mant_b_prenorm_SNaN_S = ~Mant_b_NonH_D[C_MANT_FP64 - 1] && |Mant_b_NonH_D[C_MANT_FP64 - 2:0];
	always @(*)
		case (Format_sel_SI)
			2'b00: begin
				Mant_a_prenorm_zero_S = Operand_a_DI[C_MANT_FP32 - 1:0] == C_MANT_ZERO_FP32;
				Mant_b_prenorm_zero_S = Operand_b_DI[C_MANT_FP32 - 1:0] == C_MANT_ZERO_FP32;
				Exp_a_prenorm_Inf_NaN_S = Operand_a_DI[C_OP_FP32 - 2:C_MANT_FP32] == C_EXP_INF_FP32;
				Exp_b_prenorm_Inf_NaN_S = Operand_b_DI[C_OP_FP32 - 2:C_MANT_FP32] == C_EXP_INF_FP32;
			end
			2'b01: begin
				Mant_a_prenorm_zero_S = Operand_a_DI[C_MANT_FP64 - 1:0] == C_MANT_ZERO_FP64;
				Mant_b_prenorm_zero_S = Operand_b_DI[C_MANT_FP64 - 1:0] == C_MANT_ZERO_FP64;
				Exp_a_prenorm_Inf_NaN_S = Operand_a_DI[C_OP_FP64 - 2:C_MANT_FP64] == C_EXP_INF_FP64;
				Exp_b_prenorm_Inf_NaN_S = Operand_b_DI[C_OP_FP64 - 2:C_MANT_FP64] == C_EXP_INF_FP64;
			end
			2'b10: begin
				Mant_a_prenorm_zero_S = Operand_a_DI[C_MANT_FP16 - 1:0] == C_MANT_ZERO_FP16;
				Mant_b_prenorm_zero_S = Operand_b_DI[C_MANT_FP16 - 1:0] == C_MANT_ZERO_FP16;
				Exp_a_prenorm_Inf_NaN_S = Operand_a_DI[C_OP_FP16 - 2:C_MANT_FP16] == C_EXP_INF_FP16;
				Exp_b_prenorm_Inf_NaN_S = Operand_b_DI[C_OP_FP16 - 2:C_MANT_FP16] == C_EXP_INF_FP16;
			end
			2'b11: begin
				Mant_a_prenorm_zero_S = Operand_a_DI[C_MANT_FP16ALT - 1:0] == C_MANT_ZERO_FP16ALT;
				Mant_b_prenorm_zero_S = Operand_b_DI[C_MANT_FP16ALT - 1:0] == C_MANT_ZERO_FP16ALT;
				Exp_a_prenorm_Inf_NaN_S = Operand_a_DI[C_OP_FP16ALT - 2:C_MANT_FP16ALT] == C_EXP_INF_FP16ALT;
				Exp_b_prenorm_Inf_NaN_S = Operand_b_DI[C_OP_FP16ALT - 2:C_MANT_FP16ALT] == C_EXP_INF_FP16ALT;
			end
		endcase
	wire Zero_a_SN;
	reg Zero_a_SP;
	wire Zero_b_SN;
	reg Zero_b_SP;
	wire Inf_a_SN;
	reg Inf_a_SP;
	wire Inf_b_SN;
	reg Inf_b_SP;
	wire NaN_a_SN;
	reg NaN_a_SP;
	wire NaN_b_SN;
	reg NaN_b_SP;
	wire SNaN_SN;
	reg SNaN_SP;
	assign Zero_a_SN = (Start_S && Ready_SI ? Exp_a_prenorm_zero_S && Mant_a_prenorm_zero_S : Zero_a_SP);
	assign Zero_b_SN = (Start_S && Ready_SI ? Exp_b_prenorm_zero_S && Mant_b_prenorm_zero_S : Zero_b_SP);
	assign Inf_a_SN = (Start_S && Ready_SI ? Exp_a_prenorm_Inf_NaN_S && Mant_a_prenorm_zero_S : Inf_a_SP);
	assign Inf_b_SN = (Start_S && Ready_SI ? Exp_b_prenorm_Inf_NaN_S && Mant_b_prenorm_zero_S : Inf_b_SP);
	assign NaN_a_SN = (Start_S && Ready_SI ? Exp_a_prenorm_Inf_NaN_S && ~Mant_a_prenorm_zero_S : NaN_a_SP);
	assign NaN_b_SN = (Start_S && Ready_SI ? Exp_b_prenorm_Inf_NaN_S && ~Mant_b_prenorm_zero_S : NaN_b_SP);
	assign SNaN_SN = (Start_S && Ready_SI ? (Mant_a_prenorm_SNaN_S && NaN_a_SN) | (Mant_b_prenorm_SNaN_S && NaN_b_SN) : SNaN_SP);
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI) begin
			Zero_a_SP <= 1'b0;
			Zero_b_SP <= 1'b0;
			Inf_a_SP <= 1'b0;
			Inf_b_SP <= 1'b0;
			NaN_a_SP <= 1'b0;
			NaN_b_SP <= 1'b0;
			SNaN_SP <= 1'b0;
		end
		else begin
			Inf_a_SP <= Inf_a_SN;
			Inf_b_SP <= Inf_b_SN;
			Zero_a_SP <= Zero_a_SN;
			Zero_b_SP <= Zero_b_SN;
			NaN_a_SP <= NaN_a_SN;
			NaN_b_SP <= NaN_b_SN;
			SNaN_SP <= SNaN_SN;
		end
	assign Special_case_SBO = ~{(Div_start_SI ? ((((Zero_a_SN | Zero_b_SN) | Inf_a_SN) | Inf_b_SN) | NaN_a_SN) | NaN_b_SN : ((Zero_a_SN | Inf_a_SN) | NaN_a_SN) | Sign_a_D)} && (Start_S && Ready_SI);
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Special_case_dly_SBO <= 1'b0;
		else if (Start_S && Ready_SI)
			Special_case_dly_SBO <= Special_case_SBO;
		else if (Special_case_dly_SBO)
			Special_case_dly_SBO <= 1'b1;
		else
			Special_case_dly_SBO <= 1'b0;
	reg Sign_z_DN;
	reg Sign_z_DP;
	always @(*)
		if (Div_start_SI && Ready_SI)
			Sign_z_DN = Sign_a_D ^ Sign_b_D;
		else if (Sqrt_start_SI && Ready_SI)
			Sign_z_DN = Sign_a_D;
		else
			Sign_z_DN = Sign_z_DP;
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Sign_z_DP <= 1'b0;
		else
			Sign_z_DP <= Sign_z_DN;
	reg [C_RM - 1:0] RM_DN;
	reg [C_RM - 1:0] RM_DP;
	always @(*)
		if (Start_S && Ready_SI)
			RM_DN = RM_SI;
		else
			RM_DN = RM_DP;
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			RM_DP <= {C_RM {1'sb0}};
		else
			RM_DP <= RM_DN;
	assign RM_dly_SO = RM_DP;
	wire [5:0] Mant_leadingOne_a;
	wire [5:0] Mant_leadingOne_b;
	wire Mant_zero_S_a;
	wire Mant_zero_S_b;
	lzc #(
		.WIDTH(C_MANT_FP64 + 1),
		.MODE(1)
	) LOD_Ua(
		.in_i(Mant_a_D),
		.cnt_o(Mant_leadingOne_a),
		.empty_o(Mant_zero_S_a)
	);
	wire [C_MANT_FP64:0] Mant_a_norm_DN;
	reg [C_MANT_FP64:0] Mant_a_norm_DP;
	assign Mant_a_norm_DN = (Start_S && Ready_SI ? Mant_a_D << Mant_leadingOne_a : Mant_a_norm_DP);
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Mant_a_norm_DP <= {(C_MANT_FP64 >= 0 ? C_MANT_FP64 + 1 : 1 - C_MANT_FP64) {1'sb0}};
		else
			Mant_a_norm_DP <= Mant_a_norm_DN;
	wire [C_EXP_FP64:0] Exp_a_norm_DN;
	reg [C_EXP_FP64:0] Exp_a_norm_DP;
	assign Exp_a_norm_DN = (Start_S && Ready_SI ? (Exp_a_D - Mant_leadingOne_a) + |Mant_leadingOne_a : Exp_a_norm_DP);
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Exp_a_norm_DP <= {(C_EXP_FP64 >= 0 ? C_EXP_FP64 + 1 : 1 - C_EXP_FP64) {1'sb0}};
		else
			Exp_a_norm_DP <= Exp_a_norm_DN;
	lzc #(
		.WIDTH(C_MANT_FP64 + 1),
		.MODE(1)
	) LOD_Ub(
		.in_i(Mant_b_D),
		.cnt_o(Mant_leadingOne_b),
		.empty_o(Mant_zero_S_b)
	);
	wire [C_MANT_FP64:0] Mant_b_norm_DN;
	reg [C_MANT_FP64:0] Mant_b_norm_DP;
	assign Mant_b_norm_DN = (Start_S && Ready_SI ? Mant_b_D << Mant_leadingOne_b : Mant_b_norm_DP);
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Mant_b_norm_DP <= {(C_MANT_FP64 >= 0 ? C_MANT_FP64 + 1 : 1 - C_MANT_FP64) {1'sb0}};
		else
			Mant_b_norm_DP <= Mant_b_norm_DN;
	wire [C_EXP_FP64:0] Exp_b_norm_DN;
	reg [C_EXP_FP64:0] Exp_b_norm_DP;
	assign Exp_b_norm_DN = (Start_S && Ready_SI ? (Exp_b_D - Mant_leadingOne_b) + |Mant_leadingOne_b : Exp_b_norm_DP);
	always @(posedge Clk_CI or negedge Rst_RBI)
		if (~Rst_RBI)
			Exp_b_norm_DP <= {(C_EXP_FP64 >= 0 ? C_EXP_FP64 + 1 : 1 - C_EXP_FP64) {1'sb0}};
		else
			Exp_b_norm_DP <= Exp_b_norm_DN;
	assign Start_SO = Start_S;
	assign Exp_a_DO_norm = Exp_a_norm_DP;
	assign Exp_b_DO_norm = Exp_b_norm_DP;
	assign Mant_a_DO_norm = Mant_a_norm_DP;
	assign Mant_b_DO_norm = Mant_b_norm_DP;
	assign Sign_z_DO = Sign_z_DP;
	assign Inf_a_SO = Inf_a_SP;
	assign Inf_b_SO = Inf_b_SP;
	assign Zero_a_SO = Zero_a_SP;
	assign Zero_b_SO = Zero_b_SP;
	assign NaN_a_SO = NaN_a_SP;
	assign NaN_b_SO = NaN_b_SP;
	assign SNaN_SO = SNaN_SP;
endmodule
