

module gpio_reg_top (
  input clk_i,
  input rst_ni,

  // Below Regster interface can be changed
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,
  // To HW
  output gpio_reg_pkg::gpio_reg2hw_t reg2hw, // Write
  input  gpio_reg_pkg::gpio_hw2reg_t hw2reg, // Read

  // Config
  input devmode_i // If 1, explicit error return for unmapped register access
);

  import gpio_reg_pkg::* ;

  localparam int AW = 6;
  localparam int DW = 32;
  localparam int DBW = DW/8;                    // Byte Width

  // register signals
  logic           reg_we;
  logic           reg_re;
  logic [AW-1:0]  reg_addr;
  logic [DW-1:0]  reg_wdata;
  logic [DBW-1:0] reg_be;
  logic [DW-1:0]  reg_rdata;
  logic           reg_error;

  logic          addrmiss, wr_err;

  logic [DW-1:0] reg_rdata_next;

  tlul_pkg::tl_h2d_t tl_reg_h2d;
  tlul_pkg::tl_d2h_t tl_reg_d2h;

  assign tl_reg_h2d = tl_i;
  assign tl_o       = tl_reg_d2h;

  tlul_adapter_reg #(
    .RegAw(AW),
    .RegDw(DW)
  ) u_reg_if (
    .clk_i,
    .rst_ni,

    .tl_i (tl_reg_h2d),
    .tl_o (tl_reg_d2h),

    .we_o    (reg_we),
    .re_o    (reg_re),
    .addr_o  (reg_addr),
    .wdata_o (reg_wdata),
    .be_o    (reg_be),
    .rdata_i (reg_rdata),
    .error_i (reg_error)
  );

  assign reg_rdata = reg_rdata_next ;
  assign reg_error = (devmode_i & addrmiss) | wr_err ;

  // Define SW related signals
  // Format: <reg>_<field>_{wd|we|qs}
  //        or <reg>_{wd|we|qs} if field == 1 or 0
  logic [31:0] intr_state_qs;
  logic [31:0] intr_state_wd;
  logic intr_state_we;
  logic [31:0] intr_enable_qs;
  logic [31:0] intr_enable_wd;
  logic intr_enable_we;
  logic [31:0] intr_test_wd;
  logic intr_test_we;
  logic [31:0] data_in_qs;
  logic [31:0] direct_out_qs;
  logic [31:0] direct_out_wd;
  logic direct_out_we;
  logic direct_out_re;
  logic [15:0] masked_out_lower_data_qs;
  logic [15:0] masked_out_lower_data_wd;
  logic masked_out_lower_data_we;
  logic masked_out_lower_data_re;
  logic [15:0] masked_out_lower_mask_wd;
  logic masked_out_lower_mask_we;
  logic [15:0] masked_out_upper_data_qs;
  logic [15:0] masked_out_upper_data_wd;
  logic masked_out_upper_data_we;
  logic masked_out_upper_data_re;
  logic [15:0] masked_out_upper_mask_wd;
  logic masked_out_upper_mask_we;
  logic [31:0] direct_oe_qs;
  logic [31:0] direct_oe_wd;
  logic direct_oe_we;
  logic direct_oe_re;
  logic [15:0] masked_oe_lower_data_qs;
  logic [15:0] masked_oe_lower_data_wd;
  logic masked_oe_lower_data_we;
  logic masked_oe_lower_data_re;
  logic [15:0] masked_oe_lower_mask_qs;
  logic [15:0] masked_oe_lower_mask_wd;
  logic masked_oe_lower_mask_we;
  logic masked_oe_lower_mask_re;
  logic [15:0] masked_oe_upper_data_qs;
  logic [15:0] masked_oe_upper_data_wd;
  logic masked_oe_upper_data_we;
  logic masked_oe_upper_data_re;
  logic [15:0] masked_oe_upper_mask_qs;
  logic [15:0] masked_oe_upper_mask_wd;
  logic masked_oe_upper_mask_we;
  logic masked_oe_upper_mask_re;
  logic [31:0] intr_ctrl_en_rising_qs;
  logic [31:0] intr_ctrl_en_rising_wd;
  logic intr_ctrl_en_rising_we;
  logic [31:0] intr_ctrl_en_falling_qs;
  logic [31:0] intr_ctrl_en_falling_wd;
  logic intr_ctrl_en_falling_we;
  logic [31:0] intr_ctrl_en_lvlhigh_qs;
  logic [31:0] intr_ctrl_en_lvlhigh_wd;
  logic intr_ctrl_en_lvlhigh_we;
  logic [31:0] intr_ctrl_en_lvllow_qs;
  logic [31:0] intr_ctrl_en_lvllow_wd;
  logic intr_ctrl_en_lvllow_we;
  logic [31:0] ctrl_en_input_filter_qs;
  logic [31:0] ctrl_en_input_filter_wd;
  logic ctrl_en_input_filter_we;

  // Register instances
  // R[intr_state]: V(False)

  prim_subreg #(
    .DW      (32),
    .SWACCESS("W1C"),
    .RESVAL  (32'h0)
  ) u_intr_state (
    .clk_i   (clk_i    ),
    .rst_ni  (rst_ni  ),

    // from register interface
    .we     (intr_state_we),
    .wd     (intr_state_wd),

    // from internal hardware
    .de     (hw2reg.intr_state.de),
    .d      (hw2reg.intr_state.d ),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_state.q ),

    // to register interface (read)
    .qs     (intr_state_qs)
  );


  // R[intr_enable]: V(False)

  prim_subreg #(
    .DW      (32),
    .SWACCESS("RW"),
    .RESVAL  (32'h0)
  ) u_intr_enable (
    .clk_i   (clk_i    ),
    .rst_ni  (rst_ni  ),

    // from register interface
    .we     (intr_enable_we),
    .wd     (intr_enable_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0  ),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_enable.q ),

    // to register interface (read)
    .qs     (intr_enable_qs)
  );


  // R[intr_test]: V(True)

  prim_subreg_ext #(
    .DW    (32)
  ) u_intr_test (
    .re     (1'b0),
    .we     (intr_test_we),
    .wd     (intr_test_wd),
    .d      ('0),
    .qre    (),
    .qe     (reg2hw.intr_test.qe),
    .q      (reg2hw.intr_test.q ),
    .qs     ()
  );


  // R[data_in]: V(False)

  prim_subreg #(
    .DW      (32),
    .SWACCESS("RO"),
    .RESVAL  (32'h0)
  ) u_data_in (
    .clk_i   (clk_i    ),
    .rst_ni  (rst_ni  ),

    .we     (1'b0),
    .wd     ('0  ),

    // from internal hardware
    .de     (hw2reg.data_in.de),
    .d      (hw2reg.data_in.d ),

    // to internal hardware
    .qe     (),
    .q      (),

    // to register interface (read)
    .qs     (data_in_qs)
  );


  // R[direct_out]: V(True)

  prim_subreg_ext #(
    .DW    (32)
  ) u_direct_out (
    .re     (direct_out_re),
    .we     (direct_out_we),
    .wd     (direct_out_wd),
    .d      (hw2reg.direct_out.d),
    .qre    (),
    .qe     (reg2hw.direct_out.qe),
    .q      (reg2hw.direct_out.q ),
    .qs     (direct_out_qs)
  );


  // R[masked_out_lower]: V(True)

  //   F[data]: 15:0
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_out_lower_data (
    .re     (masked_out_lower_data_re),
    .we     (masked_out_lower_data_we),
    .wd     (masked_out_lower_data_wd),
    .d      (hw2reg.masked_out_lower.data.d),
    .qre    (),
    .qe     (reg2hw.masked_out_lower.data.qe),
    .q      (reg2hw.masked_out_lower.data.q ),
    .qs     (masked_out_lower_data_qs)
  );


  //   F[mask]: 31:16
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_out_lower_mask (
    .re     (1'b0),
    .we     (masked_out_lower_mask_we),
    .wd     (masked_out_lower_mask_wd),
    .d      (hw2reg.masked_out_lower.mask.d),
    .qre    (),
    .qe     (reg2hw.masked_out_lower.mask.qe),
    .q      (reg2hw.masked_out_lower.mask.q ),
    .qs     ()
  );


  // R[masked_out_upper]: V(True)

  //   F[data]: 15:0
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_out_upper_data (
    .re     (masked_out_upper_data_re),
    .we     (masked_out_upper_data_we),
    .wd     (masked_out_upper_data_wd),
    .d      (hw2reg.masked_out_upper.data.d),
    .qre    (),
    .qe     (reg2hw.masked_out_upper.data.qe),
    .q      (reg2hw.masked_out_upper.data.q ),
    .qs     (masked_out_upper_data_qs)
  );


  //   F[mask]: 31:16
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_out_upper_mask (
    .re     (1'b0),
    .we     (masked_out_upper_mask_we),
    .wd     (masked_out_upper_mask_wd),
    .d      (hw2reg.masked_out_upper.mask.d),
    .qre    (),
    .qe     (reg2hw.masked_out_upper.mask.qe),
    .q      (reg2hw.masked_out_upper.mask.q ),
    .qs     ()
  );


  // R[direct_oe]: V(True)

  prim_subreg_ext #(
    .DW    (32)
  ) u_direct_oe (
    .re     (direct_oe_re),
    .we     (direct_oe_we),
    .wd     (direct_oe_wd),
    .d      (hw2reg.direct_oe.d),
    .qre    (),
    .qe     (reg2hw.direct_oe.qe),
    .q      (reg2hw.direct_oe.q ),
    .qs     (direct_oe_qs)
  );


  // R[masked_oe_lower]: V(True)

  //   F[data]: 15:0
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_oe_lower_data (
    .re     (masked_oe_lower_data_re),
    .we     (masked_oe_lower_data_we),
    .wd     (masked_oe_lower_data_wd),
    .d      (hw2reg.masked_oe_lower.data.d),
    .qre    (),
    .qe     (reg2hw.masked_oe_lower.data.qe),
    .q      (reg2hw.masked_oe_lower.data.q ),
    .qs     (masked_oe_lower_data_qs)
  );


  //   F[mask]: 31:16
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_oe_lower_mask (
    .re     (masked_oe_lower_mask_re),
    .we     (masked_oe_lower_mask_we),
    .wd     (masked_oe_lower_mask_wd),
    .d      (hw2reg.masked_oe_lower.mask.d),
    .qre    (),
    .qe     (reg2hw.masked_oe_lower.mask.qe),
    .q      (reg2hw.masked_oe_lower.mask.q ),
    .qs     (masked_oe_lower_mask_qs)
  );


  // R[masked_oe_upper]: V(True)

  //   F[data]: 15:0
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_oe_upper_data (
    .re     (masked_oe_upper_data_re),
    .we     (masked_oe_upper_data_we),
    .wd     (masked_oe_upper_data_wd),
    .d      (hw2reg.masked_oe_upper.data.d),
    .qre    (),
    .qe     (reg2hw.masked_oe_upper.data.qe),
    .q      (reg2hw.masked_oe_upper.data.q ),
    .qs     (masked_oe_upper_data_qs)
  );


  //   F[mask]: 31:16
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_oe_upper_mask (
    .re     (masked_oe_upper_mask_re),
    .we     (masked_oe_upper_mask_we),
    .wd     (masked_oe_upper_mask_wd),
    .d      (hw2reg.masked_oe_upper.mask.d),
    .qre    (),
    .qe     (reg2hw.masked_oe_upper.mask.qe),
    .q      (reg2hw.masked_oe_upper.mask.q ),
    .qs     (masked_oe_upper_mask_qs)
  );


  // R[intr_ctrl_en_rising]: V(False)

  prim_subreg #(
    .DW      (32),
    .SWACCESS("RW"),
    .RESVAL  (32'h0)
  ) u_intr_ctrl_en_rising (
    .clk_i   (clk_i    ),
    .rst_ni  (rst_ni  ),

    // from register interface
    .we     (intr_ctrl_en_rising_we),
    .wd     (intr_ctrl_en_rising_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0  ),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_ctrl_en_rising.q ),

    // to register interface (read)
    .qs     (intr_ctrl_en_rising_qs)
  );


  // R[intr_ctrl_en_falling]: V(False)

  prim_subreg #(
    .DW      (32),
    .SWACCESS("RW"),
    .RESVAL  (32'h0)
  ) u_intr_ctrl_en_falling (
    .clk_i   (clk_i    ),
    .rst_ni  (rst_ni  ),

    // from register interface
    .we     (intr_ctrl_en_falling_we),
    .wd     (intr_ctrl_en_falling_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0  ),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_ctrl_en_falling.q ),

    // to register interface (read)
    .qs     (intr_ctrl_en_falling_qs)
  );


  // R[intr_ctrl_en_lvlhigh]: V(False)

  prim_subreg #(
    .DW      (32),
    .SWACCESS("RW"),
    .RESVAL  (32'h0)
  ) u_intr_ctrl_en_lvlhigh (
    .clk_i   (clk_i    ),
    .rst_ni  (rst_ni  ),

    // from register interface
    .we     (intr_ctrl_en_lvlhigh_we),
    .wd     (intr_ctrl_en_lvlhigh_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0  ),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_ctrl_en_lvlhigh.q ),

    // to register interface (read)
    .qs     (intr_ctrl_en_lvlhigh_qs)
  );


  // R[intr_ctrl_en_lvllow]: V(False)

  prim_subreg #(
    .DW      (32),
    .SWACCESS("RW"),
    .RESVAL  (32'h0)
  ) u_intr_ctrl_en_lvllow (
    .clk_i   (clk_i    ),
    .rst_ni  (rst_ni  ),

    // from register interface
    .we     (intr_ctrl_en_lvllow_we),
    .wd     (intr_ctrl_en_lvllow_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0  ),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_ctrl_en_lvllow.q ),

    // to register interface (read)
    .qs     (intr_ctrl_en_lvllow_qs)
  );


  // R[ctrl_en_input_filter]: V(False)

  prim_subreg #(
    .DW      (32),
    .SWACCESS("RW"),
    .RESVAL  (32'h0)
  ) u_ctrl_en_input_filter (
    .clk_i   (clk_i    ),
    .rst_ni  (rst_ni  ),

    // from register interface
    .we     (ctrl_en_input_filter_we),
    .wd     (ctrl_en_input_filter_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0  ),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.ctrl_en_input_filter.q ),

    // to register interface (read)
    .qs     (ctrl_en_input_filter_qs)
  );




  logic [14:0] addr_hit;
  always_comb begin
    addr_hit = '0;
    addr_hit[ 0] = (reg_addr == GPIO_INTR_STATE_OFFSET);
    addr_hit[ 1] = (reg_addr == GPIO_INTR_ENABLE_OFFSET);
    addr_hit[ 2] = (reg_addr == GPIO_INTR_TEST_OFFSET);
    addr_hit[ 3] = (reg_addr == GPIO_DATA_IN_OFFSET);
    addr_hit[ 4] = (reg_addr == GPIO_DIRECT_OUT_OFFSET);
    addr_hit[ 5] = (reg_addr == GPIO_MASKED_OUT_LOWER_OFFSET);
    addr_hit[ 6] = (reg_addr == GPIO_MASKED_OUT_UPPER_OFFSET);
    addr_hit[ 7] = (reg_addr == GPIO_DIRECT_OE_OFFSET);
    addr_hit[ 8] = (reg_addr == GPIO_MASKED_OE_LOWER_OFFSET);
    addr_hit[ 9] = (reg_addr == GPIO_MASKED_OE_UPPER_OFFSET);
    addr_hit[10] = (reg_addr == GPIO_INTR_CTRL_EN_RISING_OFFSET);
    addr_hit[11] = (reg_addr == GPIO_INTR_CTRL_EN_FALLING_OFFSET);
    addr_hit[12] = (reg_addr == GPIO_INTR_CTRL_EN_LVLHIGH_OFFSET);
    addr_hit[13] = (reg_addr == GPIO_INTR_CTRL_EN_LVLLOW_OFFSET);
    addr_hit[14] = (reg_addr == GPIO_CTRL_EN_INPUT_FILTER_OFFSET);
  end

  assign addrmiss = (reg_re || reg_we) ? ~|addr_hit : 1'b0 ;

  // Check sub-word write is permitted
  always_comb begin
    wr_err = 1'b0;
    if (addr_hit[ 0] && reg_we && (GPIO_PERMIT[ 0] != (GPIO_PERMIT[ 0] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[ 1] && reg_we && (GPIO_PERMIT[ 1] != (GPIO_PERMIT[ 1] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[ 2] && reg_we && (GPIO_PERMIT[ 2] != (GPIO_PERMIT[ 2] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[ 3] && reg_we && (GPIO_PERMIT[ 3] != (GPIO_PERMIT[ 3] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[ 4] && reg_we && (GPIO_PERMIT[ 4] != (GPIO_PERMIT[ 4] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[ 5] && reg_we && (GPIO_PERMIT[ 5] != (GPIO_PERMIT[ 5] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[ 6] && reg_we && (GPIO_PERMIT[ 6] != (GPIO_PERMIT[ 6] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[ 7] && reg_we && (GPIO_PERMIT[ 7] != (GPIO_PERMIT[ 7] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[ 8] && reg_we && (GPIO_PERMIT[ 8] != (GPIO_PERMIT[ 8] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[ 9] && reg_we && (GPIO_PERMIT[ 9] != (GPIO_PERMIT[ 9] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[10] && reg_we && (GPIO_PERMIT[10] != (GPIO_PERMIT[10] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[11] && reg_we && (GPIO_PERMIT[11] != (GPIO_PERMIT[11] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[12] && reg_we && (GPIO_PERMIT[12] != (GPIO_PERMIT[12] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[13] && reg_we && (GPIO_PERMIT[13] != (GPIO_PERMIT[13] & reg_be))) wr_err = 1'b1 ;
    if (addr_hit[14] && reg_we && (GPIO_PERMIT[14] != (GPIO_PERMIT[14] & reg_be))) wr_err = 1'b1 ;
    if (reg_wdata > 32'h000FFFFF) wr_err = 1'b1;
  end

  assign intr_state_we = addr_hit[0] & reg_we & ~wr_err;
  assign intr_state_wd = reg_wdata[31:0];

  assign intr_enable_we = addr_hit[1] & reg_we & ~wr_err;
  assign intr_enable_wd = reg_wdata[31:0];

  assign intr_test_we = addr_hit[2] & reg_we & ~wr_err;
  assign intr_test_wd = reg_wdata[31:0];


  assign direct_out_we = addr_hit[4] & reg_we & ~wr_err;
  assign direct_out_wd = reg_wdata[31:0];
  assign direct_out_re = addr_hit[4] && reg_re;

  assign masked_out_lower_data_we = addr_hit[5] & reg_we & ~wr_err;
  assign masked_out_lower_data_wd = reg_wdata[9:0];
  assign masked_out_lower_data_re = addr_hit[5] && reg_re;

  assign masked_out_lower_mask_we = addr_hit[5] & reg_we & ~wr_err;
  assign masked_out_lower_mask_wd = {12'b111111111111,reg_wdata[19:10]};

  assign masked_out_upper_data_we = addr_hit[6] & reg_we & ~wr_err;
  assign masked_out_upper_data_wd = reg_wdata[15:0];
  assign masked_out_upper_data_re = addr_hit[6] && reg_re;

  assign masked_out_upper_mask_we = addr_hit[6] & reg_we & ~wr_err;
  assign masked_out_upper_mask_wd = reg_wdata[31:16];

  assign direct_oe_we = addr_hit[7] & reg_we & ~wr_err;
  assign direct_oe_wd = reg_wdata[31:0];
  assign direct_oe_re = addr_hit[7] && reg_re;

  assign masked_oe_lower_data_we = addr_hit[8] & reg_we & ~wr_err;
  assign masked_oe_lower_data_wd = reg_wdata[15:0];
  assign masked_oe_lower_data_re = addr_hit[8] && reg_re;

  assign masked_oe_lower_mask_we = addr_hit[8] & reg_we & ~wr_err;
  assign masked_oe_lower_mask_wd = reg_wdata[31:16];
  assign masked_oe_lower_mask_re = addr_hit[8] && reg_re;

  assign masked_oe_upper_data_we = addr_hit[9] & reg_we & ~wr_err;
  assign masked_oe_upper_data_wd = reg_wdata[15:0];
  assign masked_oe_upper_data_re = addr_hit[9] && reg_re;

  assign masked_oe_upper_mask_we = addr_hit[9] & reg_we & ~wr_err;
  assign masked_oe_upper_mask_wd = reg_wdata[31:16];
  assign masked_oe_upper_mask_re = addr_hit[9] && reg_re;

  assign intr_ctrl_en_rising_we = addr_hit[10] & reg_we & ~wr_err;
  assign intr_ctrl_en_rising_wd = reg_wdata[31:0];

  assign intr_ctrl_en_falling_we = addr_hit[11] & reg_we & ~wr_err;
  assign intr_ctrl_en_falling_wd = reg_wdata[31:0];

  assign intr_ctrl_en_lvlhigh_we = addr_hit[12] & reg_we & ~wr_err;
  assign intr_ctrl_en_lvlhigh_wd = reg_wdata[31:0];

  assign intr_ctrl_en_lvllow_we = addr_hit[13] & reg_we & ~wr_err;
  assign intr_ctrl_en_lvllow_wd = reg_wdata[31:0];

  assign ctrl_en_input_filter_we = addr_hit[14] & reg_we & ~wr_err;
  assign ctrl_en_input_filter_wd = reg_wdata[31:0];

  // Read data return
  always_comb begin
    reg_rdata_next = '0;
    unique case (1'b1)
      addr_hit[0]: begin
        reg_rdata_next[31:0] = intr_state_qs;
      end

      addr_hit[1]: begin
        reg_rdata_next[31:0] = intr_enable_qs;
      end

      addr_hit[2]: begin
        reg_rdata_next[31:0] = '0;
      end

      addr_hit[3]: begin
        reg_rdata_next[31:0] = data_in_qs;
      end

      addr_hit[4]: begin
        reg_rdata_next[31:0] = direct_out_qs;
      end

      addr_hit[5]: begin
        reg_rdata_next[9:0] = masked_out_lower_data_qs;
        reg_rdata_next[31:10] = 22'b1111111111110000000000;
      end

      addr_hit[6]: begin
        reg_rdata_next[15:0] = masked_out_upper_data_qs;
        reg_rdata_next[31:16] = '0;
      end

      addr_hit[7]: begin
        reg_rdata_next[31:0] = direct_oe_qs;
      end

      addr_hit[8]: begin
        reg_rdata_next[15:0] = masked_oe_lower_data_qs;
        reg_rdata_next[31:16] = masked_oe_lower_mask_qs;
      end

      addr_hit[9]: begin
        reg_rdata_next[15:0] = masked_oe_upper_data_qs;
        reg_rdata_next[31:16] = masked_oe_upper_mask_qs;
      end

      addr_hit[10]: begin
        reg_rdata_next[31:0] = intr_ctrl_en_rising_qs;
      end

      addr_hit[11]: begin
        reg_rdata_next[31:0] = intr_ctrl_en_falling_qs;
      end

      addr_hit[12]: begin
        reg_rdata_next[31:0] = intr_ctrl_en_lvlhigh_qs;
      end

      addr_hit[13]: begin
        reg_rdata_next[31:0] = intr_ctrl_en_lvllow_qs;
      end

      addr_hit[14]: begin
        reg_rdata_next[31:0] = ctrl_en_input_filter_qs;
      end

      default: begin
        reg_rdata_next = '1;
      end
    endcase
  end

endmodule
