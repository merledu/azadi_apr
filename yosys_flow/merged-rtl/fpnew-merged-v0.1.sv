// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// This file contains all div_sqrt_top_mvp parameters
// Authors    : Lei Li  (lile@iis.ee.ethz.ch)

package defs_div_sqrt_mvp;

   // op command
   localparam C_RM                  = 3;
   localparam C_RM_NEAREST          = 3'h0;
   localparam C_RM_TRUNC            = 3'h1;
   localparam C_RM_PLUSINF          = 3'h2;
   localparam C_RM_MINUSINF         = 3'h3;
   localparam C_PC                  = 6; // Precision Control
   localparam C_FS                  = 2; // Format Selection
   localparam C_IUNC                = 2; // Iteration Unit Number Control
   localparam Iteration_unit_num_S  = 2'b10;

   // FP64
   localparam C_OP_FP64             = 64;
   localparam C_MANT_FP64           = 52;
   localparam C_EXP_FP64            = 11;
   localparam C_BIAS_FP64           = 1023;
   localparam C_BIAS_AONE_FP64      = 11'h400;
   localparam C_HALF_BIAS_FP64      = 511;
   localparam C_EXP_ZERO_FP64       = 11'h000;
   localparam C_EXP_ONE_FP64        = 13'h001; // Bit width is in agreement with in norm
   localparam C_EXP_INF_FP64        = 11'h7FF;
   localparam C_MANT_ZERO_FP64      = 52'h0;
   localparam C_MANT_NAN_FP64       = 52'h8_0000_0000_0000;
   localparam C_PZERO_FP64          = 64'h0000_0000_0000_0000;
   localparam C_MZERO_FP64          = 64'h8000_0000_0000_0000;
   localparam C_QNAN_FP64           = 64'h7FF8_0000_0000_0000;

   // FP32
   localparam C_OP_FP32             = 32;
   localparam C_MANT_FP32           = 23;
   localparam C_EXP_FP32            = 8;
   localparam C_BIAS_FP32           = 127;
   localparam C_BIAS_AONE_FP32      = 8'h80;
   localparam C_HALF_BIAS_FP32      = 63;
   localparam C_EXP_ZERO_FP32       = 8'h00;
   localparam C_EXP_INF_FP32        = 8'hFF;
   localparam C_MANT_ZERO_FP32      = 23'h0;
   localparam C_PZERO_FP32          = 32'h0000_0000;
   localparam C_MZERO_FP32          = 32'h8000_0000;
   localparam C_QNAN_FP32           = 32'h7FC0_0000;

   // FP16
   localparam C_OP_FP16             = 16;
   localparam C_MANT_FP16           = 10;
   localparam C_EXP_FP16            = 5;
   localparam C_BIAS_FP16           = 15;
   localparam C_BIAS_AONE_FP16      = 5'h10;
   localparam C_HALF_BIAS_FP16      = 7;
   localparam C_EXP_ZERO_FP16       = 5'h00;
   localparam C_EXP_INF_FP16        = 5'h1F;
   localparam C_MANT_ZERO_FP16      = 10'h0;
   localparam C_PZERO_FP16          = 16'h0000;
   localparam C_MZERO_FP16          = 16'h8000;
   localparam C_QNAN_FP16           = 16'h7E00;

   // FP16alt
   localparam C_OP_FP16ALT           = 16;
   localparam C_MANT_FP16ALT         = 7;
   localparam C_EXP_FP16ALT          = 8;
   localparam C_BIAS_FP16ALT         = 127;
   localparam C_BIAS_AONE_FP16ALT    = 8'h80;
   localparam C_HALF_BIAS_FP16ALT    = 63;
   localparam C_EXP_ZERO_FP16ALT     = 8'h00;
   localparam C_EXP_INF_FP16ALT      = 8'hFF;
   localparam C_MANT_ZERO_FP16ALT    = 7'h0;
   localparam C_QNAN_FP16ALT         = 16'h7FC0;

endpackage : defs_div_sqrt_mvp
// Copyright 2019 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Stefan Mach <smach@iis.ee.ethz.ch>

package fpnew_pkg;

  // ---------
  // FP TYPES
  // ---------
  // | Enumerator | Format           | Width  | EXP_BITS | MAN_BITS
  // |:----------:|------------------|-------:|:--------:|:--------:
  // | FP32       | IEEE binary32    | 32 bit | 8        | 23
  // | FP64       | IEEE binary64    | 64 bit | 11       | 52
  // | FP16       | IEEE binary16    | 16 bit | 5        | 10
  // | FP8        | binary8          |  8 bit | 5        | 2
  // | FP16ALT    | binary16alt      | 16 bit | 8        | 7
  // *NOTE:* Add new formats only at the end of the enumeration for backwards compatibilty!

  // Encoding for a format
  typedef struct packed {
    int unsigned exp_bits;
    int unsigned man_bits;
  } fp_encoding_t;

  localparam int unsigned NUM_FP_FORMATS = 5; // change me to add formats
  localparam int unsigned FP_FORMAT_BITS = $clog2(NUM_FP_FORMATS);

  // FP formats
  typedef enum logic [FP_FORMAT_BITS-1:0] {
    FP32    = 'd0,
    FP64    = 'd1,
    FP16    = 'd2,
    FP8     = 'd3,
    FP16ALT = 'd4
    // add new formats here
  } fp_format_e;

  // Encodings for supported FP formats
  localparam fp_encoding_t [0:NUM_FP_FORMATS-1] FP_ENCODINGS  = '{
    '{8,  23}, // IEEE binary32 (single)
    '{11, 52}, // IEEE binary64 (double)
    '{5,  10}, // IEEE binary16 (half)
    '{5,  2},  // custom binary8
    '{8,  7}   // custom binary16alt
    // add new formats here
  };

  typedef logic [0:NUM_FP_FORMATS-1]       fmt_logic_t;    // Logic indexed by FP format (for masks)
  typedef logic [0:NUM_FP_FORMATS-1][31:0] fmt_unsigned_t; // Unsigned indexed by FP format

  localparam fmt_logic_t CPK_FORMATS = 5'b11000; // FP32 and FP64 can provide CPK only

  // ---------
  // INT TYPES
  // ---------
  // | Enumerator | Width  |
  // |:----------:|-------:|
  // | INT8       |  8 bit |
  // | INT16      | 16 bit |
  // | INT32      | 32 bit |
  // | INT64      | 64 bit |
  // *NOTE:* Add new formats only at the end of the enumeration for backwards compatibilty!

  localparam int unsigned NUM_INT_FORMATS = 4; // change me to add formats
  localparam int unsigned INT_FORMAT_BITS = $clog2(NUM_INT_FORMATS);

  // Int formats
  typedef enum logic [INT_FORMAT_BITS-1:0] {
    INT8,
    INT16,
    INT32,
    INT64
    // add new formats here
  } int_format_e;

  // Returns the width of an INT format by index
  function automatic int unsigned int_width(int_format_e ifmt);
    unique case (ifmt)
      INT8:  return 8;
      INT16: return 16;
      INT32: return 32;
      INT64: return 64;
      default: begin
        // pragma translate_off
        $fatal(1, "Invalid INT format supplied");
        // pragma translate_on
        // just return any integer to avoid any latches
        // hopefully this error is caught by simulation
        return INT8;
      end
    endcase
  endfunction

  typedef logic [0:NUM_INT_FORMATS-1] ifmt_logic_t; // Logic indexed by INT format (for masks)

  // --------------
  // FP OPERATIONS
  // --------------
  localparam int unsigned NUM_OPGROUPS = 4;

  // Each FP operation belongs to an operation group
  typedef enum logic [1:0] {
    ADDMUL, DIVSQRT, NONCOMP, CONV
  } opgroup_e;

  localparam int unsigned OP_BITS = 4;

  typedef enum logic [OP_BITS-1:0] {
    FMADD, FNMSUB, ADD, MUL,     // ADDMUL operation group
    DIV, SQRT,                   // DIVSQRT operation group
    SGNJ, MINMAX, CMP, CLASSIFY, // NONCOMP operation group
    F2F, F2I, I2F, CPKAB, CPKCD  // CONV operation group
  } operation_e;

  // -------------------
  // RISC-V FP-SPECIFIC
  // -------------------
  // Rounding modes
  typedef enum logic [2:0] {
    RNE = 3'b000,
    RTZ = 3'b001,
    RDN = 3'b010,
    RUP = 3'b011,
    RMM = 3'b100,
    DYN = 3'b111
  } roundmode_e;

  // Status flags
  typedef struct packed {
    logic NV; // Invalid
    logic DZ; // Divide by zero
    logic OF; // Overflow
    logic UF; // Underflow
    logic NX; // Inexact
  } status_t;

  // Information about a floating point value
  typedef struct packed {
    logic is_normal;     // is the value normal
    logic is_subnormal;  // is the value subnormal
    logic is_zero;       // is the value zero
    logic is_inf;        // is the value infinity
    logic is_nan;        // is the value NaN
    logic is_signalling; // is the value a signalling NaN
    logic is_quiet;      // is the value a quiet NaN
    logic is_boxed;      // is the value properly NaN-boxed (RISC-V specific)
  } fp_info_t;

  // Classification mask
  typedef enum logic [9:0] {
    NEGINF     = 10'b00_0000_0001,
    NEGNORM    = 10'b00_0000_0010,
    NEGSUBNORM = 10'b00_0000_0100,
    NEGZERO    = 10'b00_0000_1000,
    POSZERO    = 10'b00_0001_0000,
    POSSUBNORM = 10'b00_0010_0000,
    POSNORM    = 10'b00_0100_0000,
    POSINF     = 10'b00_1000_0000,
    SNAN       = 10'b01_0000_0000,
    QNAN       = 10'b10_0000_0000
  } classmask_e;

  // ------------------
  // FPU configuration
  // ------------------
  // Pipelining registers can be inserted (at elaboration time) into operational units
  typedef enum logic [1:0] {
    BEFORE,     // registers are inserted at the inputs of the unit
    AFTER,      // registers are inserted at the outputs of the unit
    INSIDE,     // registers are inserted at predetermined (suboptimal) locations in the unit
    DISTRIBUTED // registers are evenly distributed, INSIDE >= AFTER >= BEFORE
  } pipe_config_t;

  // Arithmetic units can be arranged in parallel (per format), merged (multi-format) or not at all.
  typedef enum logic [1:0] {
    DISABLED, // arithmetic units are not generated
    PARALLEL, // arithmetic units are generated in prallel slices, one for each format
    MERGED    // arithmetic units are contained within a merged unit holding multiple formats
  } unit_type_t;

  // Array of unit types indexed by format
  typedef unit_type_t [0:NUM_FP_FORMATS-1] fmt_unit_types_t;

  // Array of format-specific unit types by opgroup
  typedef fmt_unit_types_t [0:NUM_OPGROUPS-1] opgrp_fmt_unit_types_t;
  // same with unsigned
  typedef fmt_unsigned_t [0:NUM_OPGROUPS-1] opgrp_fmt_unsigned_t;

  // FPU configuration: features
  typedef struct packed {
    int unsigned Width;
    logic        EnableVectors;
    logic        EnableNanBox;
    fmt_logic_t  FpFmtMask;
    ifmt_logic_t IntFmtMask;
  } fpu_features_t;

  localparam fpu_features_t RV64D = '{
    Width:         64,
    EnableVectors: 1'b0,
    EnableNanBox:  1'b1,
    FpFmtMask:     5'b11000,
    IntFmtMask:    4'b0011
  };

  localparam fpu_features_t RV32D = '{
    Width:         64,
    EnableVectors: 1'b1,
    EnableNanBox:  1'b1,
    FpFmtMask:     5'b11000,
    IntFmtMask:    4'b0010
  };

  localparam fpu_features_t RV32F = '{
    Width:         32,
    EnableVectors: 1'b0,
    EnableNanBox:  1'b1,
    FpFmtMask:     5'b10000,
    IntFmtMask:    4'b0010
  };

  localparam fpu_features_t RV64D_Xsflt = '{
    Width:         64,
    EnableVectors: 1'b1,
    EnableNanBox:  1'b1,
    FpFmtMask:     5'b11111,
    IntFmtMask:    4'b1111
  };

  localparam fpu_features_t RV32F_Xsflt = '{
    Width:         32,
    EnableVectors: 1'b1,
    EnableNanBox:  1'b1,
    FpFmtMask:     5'b10111,
    IntFmtMask:    4'b1110
  };

  localparam fpu_features_t RV32F_Xf16alt_Xfvec = '{
    Width:         32,
    EnableVectors: 1'b1,
    EnableNanBox:  1'b1,
    FpFmtMask:     5'b10001,
    IntFmtMask:    4'b0110
  };


  // FPU configuraion: implementation
  typedef struct packed {
    opgrp_fmt_unsigned_t   PipeRegs;
    opgrp_fmt_unit_types_t UnitTypes;
    pipe_config_t          PipeConfig;
  } fpu_implementation_t;

  localparam fpu_implementation_t DEFAULT_NOREGS = '{
    PipeRegs:   '{default: 0},
    UnitTypes:  '{'{default: PARALLEL}, // ADDMUL
                  '{default: MERGED},   // DIVSQRT
                  '{default: PARALLEL}, // NONCOMP
                  '{default: MERGED}},  // CONV
    PipeConfig: BEFORE
  };

  localparam fpu_implementation_t DEFAULT_SNITCH = '{
    PipeRegs:   '{default: 1},
    UnitTypes:  '{'{default: PARALLEL}, // ADDMUL
                  '{default: DISABLED}, // DIVSQRT
                  '{default: PARALLEL}, // NONCOMP
                  '{default: MERGED}},  // CONV
    PipeConfig: BEFORE
  };

  // -----------------------
  // Synthesis optimization
  // -----------------------
  localparam logic DONT_CARE = 1'b1; // the value to assign as don't care

  // -------------------------
  // General helper functions
  // -------------------------
  function automatic int minimum(int a, int b);
    return (a < b) ? a : b;
  endfunction

  function automatic int maximum(int a, int b);
    return (a > b) ? a : b;
  endfunction

  // -------------------------------------------
  // Helper functions for FP formats and values
  // -------------------------------------------
  // Returns the width of a FP format
  function automatic int unsigned fp_width(fp_format_e fmt);
    return FP_ENCODINGS[fmt].exp_bits + FP_ENCODINGS[fmt].man_bits + 1;
  endfunction

  // Returns the widest FP format present
  function automatic int unsigned max_fp_width(fmt_logic_t cfg);
    automatic int unsigned res = 0;
    for (int unsigned i = 0; i < NUM_FP_FORMATS; i++)
      if (cfg[i])
        res = unsigned'(maximum(res, fp_width(fp_format_e'(i))));
    return res;
  endfunction

  // Returns the narrowest FP format present
  function automatic int unsigned min_fp_width(fmt_logic_t cfg);
    automatic int unsigned res = max_fp_width(cfg);
    for (int unsigned i = 0; i < NUM_FP_FORMATS; i++)
      if (cfg[i])
        res = unsigned'(minimum(res, fp_width(fp_format_e'(i))));
    return res;
  endfunction

  // Returns the number of expoent bits for a format
  function automatic int unsigned exp_bits(fp_format_e fmt);
    return FP_ENCODINGS[fmt].exp_bits;
  endfunction

  // Returns the number of mantissa bits for a format
  function automatic int unsigned man_bits(fp_format_e fmt);
    return FP_ENCODINGS[fmt].man_bits;
  endfunction

  // Returns the bias value for a given format (as per IEEE 754-2008)
  function automatic int unsigned bias(fp_format_e fmt);
    return unsigned'(2**(FP_ENCODINGS[fmt].exp_bits-1)-1); // symmetrical bias
  endfunction

  function automatic fp_encoding_t super_format(fmt_logic_t cfg);
    automatic fp_encoding_t res;
    res = '0;
    for (int unsigned fmt = 0; fmt < NUM_FP_FORMATS; fmt++)
      if (cfg[fmt]) begin // only active format
        res.exp_bits = unsigned'(maximum(res.exp_bits, exp_bits(fp_format_e'(fmt))));
        res.man_bits = unsigned'(maximum(res.man_bits, man_bits(fp_format_e'(fmt))));
      end
    return res;
  endfunction

  // -------------------------------------------
  // Helper functions for INT formats and values
  // -------------------------------------------
  // Returns the widest INT format present
  function automatic int unsigned max_int_width(ifmt_logic_t cfg);
    automatic int unsigned res = 0;
    for (int ifmt = 0; ifmt < NUM_INT_FORMATS; ifmt++) begin
      if (cfg[ifmt]) res = maximum(res, int_width(int_format_e'(ifmt)));
    end
    return res;
  endfunction

  // --------------------------------------------------
  // Helper functions for operations and FPU structure
  // --------------------------------------------------
  // Returns the operation group of the given operation
  function automatic opgroup_e get_opgroup(operation_e op);
    unique case (op)
      FMADD, FNMSUB, ADD, MUL:     return ADDMUL;
      DIV, SQRT:                   return DIVSQRT;
      SGNJ, MINMAX, CMP, CLASSIFY: return NONCOMP;
      F2F, F2I, I2F, CPKAB, CPKCD: return CONV;
      default:                     return NONCOMP;
    endcase
  endfunction

  // Returns the number of operands by operation group
  function automatic int unsigned num_operands(opgroup_e grp);
    unique case (grp)
      ADDMUL:  return 3;
      DIVSQRT: return 2;
      NONCOMP: return 2;
      CONV:    return 3; // vectorial casts use 3 operands
      default: return 0;
    endcase
  endfunction

  // Returns the number of lanes according to width, format and vectors
  function automatic int unsigned num_lanes(int unsigned width, fp_format_e fmt, logic vec);
    return vec ? width / fp_width(fmt) : 1; // if no vectors, only one lane
  endfunction

  // Returns the maximum number of lanes in the FPU according to width, format config and vectors
  function automatic int unsigned max_num_lanes(int unsigned width, fmt_logic_t cfg, logic vec);
    return vec ? width / min_fp_width(cfg) : 1; // if no vectors, only one lane
  endfunction

  // Returns a mask of active FP formats that are present in lane lane_no of a multiformat slice
  function automatic fmt_logic_t get_lane_formats(int unsigned width,
                                                  fmt_logic_t cfg,
                                                  int unsigned lane_no);
    automatic fmt_logic_t res;
    for (int unsigned fmt = 0; fmt < NUM_FP_FORMATS; fmt++)
      // Mask active formats with the number of lanes for that format
      res[fmt] = cfg[fmt] & (width / fp_width(fp_format_e'(fmt)) > lane_no);
    return res;
  endfunction

  // Returns a mask of active INT formats that are present in lane lane_no of a multiformat slice
  function automatic ifmt_logic_t get_lane_int_formats(int unsigned width,
                                                       fmt_logic_t cfg,
                                                       ifmt_logic_t icfg,
                                                       int unsigned lane_no);
    automatic ifmt_logic_t res;
    automatic fmt_logic_t lanefmts;
    res = '0;
    lanefmts = get_lane_formats(width, cfg, lane_no);

    for (int unsigned ifmt = 0; ifmt < NUM_INT_FORMATS; ifmt++)
      for (int unsigned fmt = 0; fmt < NUM_FP_FORMATS; fmt++)
        // Mask active int formats with the width of the float formats
        if ((fp_width(fp_format_e'(fmt)) == int_width(int_format_e'(ifmt))))
          res[ifmt] |= icfg[ifmt] && lanefmts[fmt];
    return res;
  endfunction

  // Returns a mask of active FP formats that are present in lane lane_no of a CONV slice
  function automatic fmt_logic_t get_conv_lane_formats(int unsigned width,
                                                       fmt_logic_t cfg,
                                                       int unsigned lane_no);
    automatic fmt_logic_t res;
    for (int unsigned fmt = 0; fmt < NUM_FP_FORMATS; fmt++)
      // Mask active formats with the number of lanes for that format, CPK at least twice
      res[fmt] = cfg[fmt] && ((width / fp_width(fp_format_e'(fmt)) > lane_no) ||
                             (CPK_FORMATS[fmt] && (lane_no < 2)));
    return res;
  endfunction

  // Returns a mask of active INT formats that are present in lane lane_no of a CONV slice
  function automatic ifmt_logic_t get_conv_lane_int_formats(int unsigned width,
                                                            fmt_logic_t cfg,
                                                            ifmt_logic_t icfg,
                                                            int unsigned lane_no);
    automatic ifmt_logic_t res;
    automatic fmt_logic_t lanefmts;
    res = '0;
    lanefmts = get_conv_lane_formats(width, cfg, lane_no);

    for (int unsigned ifmt = 0; ifmt < NUM_INT_FORMATS; ifmt++)
      for (int unsigned fmt = 0; fmt < NUM_FP_FORMATS; fmt++)
        // Mask active int formats with the width of the float formats
        res[ifmt] |= icfg[ifmt] && lanefmts[fmt] &&
                     (fp_width(fp_format_e'(fmt)) == int_width(int_format_e'(ifmt)));
    return res;
  endfunction

  // Return whether any active format is set as MERGED
  function automatic logic any_enabled_multi(fmt_unit_types_t types, fmt_logic_t cfg);
    for (int unsigned i = 0; i < NUM_FP_FORMATS; i++)
      if (cfg[i] && types[i] == MERGED)
        return 1'b1;
      return 1'b0;
  endfunction

  // Return whether the given format is the first active one set as MERGED
  function automatic logic is_first_enabled_multi(fp_format_e fmt,
                                                  fmt_unit_types_t types,
                                                  fmt_logic_t cfg);
    for (int unsigned i = 0; i < NUM_FP_FORMATS; i++) begin
      if (cfg[i] && types[i] == MERGED) return (fp_format_e'(i) == fmt);
    end
    return 1'b0;
  endfunction

  // Returns the first format that is active and is set as MERGED
  function automatic fp_format_e get_first_enabled_multi(fmt_unit_types_t types, fmt_logic_t cfg);
    for (int unsigned i = 0; i < NUM_FP_FORMATS; i++)
      if (cfg[i] && types[i] == MERGED)
        return fp_format_e'(i);
      return fp_format_e'(0);
  endfunction

  // Returns the largest number of regs that is active and is set as MERGED
  function automatic int unsigned get_num_regs_multi(fmt_unsigned_t regs,
                                                     fmt_unit_types_t types,
                                                     fmt_logic_t cfg);
    automatic int unsigned res = 0;
    for (int unsigned i = 0; i < NUM_FP_FORMATS; i++) begin
      if (cfg[i] && types[i] == MERGED) res = maximum(res, regs[i]);
    end
    return res;
  endfunction

endpackage
// Copyright 2016 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

/// cf_math_pkg: Constant Function Implementations of Mathematical Functions for HDL Elaboration
///
/// This package contains a collection of mathematical functions that are commonly used when defining
/// the value of constants in HDL code.  These functions are implemented as Verilog constants
/// functions.  Introduced in Verilog 2001 (IEEE Std 1364-2001), a constant function (§ 10.3.5) is a
/// function whose value can be evaluated at compile time or during elaboration.  A constant function
/// must be called with arguments that are constants.
package cf_math_pkg;

    /// Ceiled Division of Two Natural Numbers
    ///
    /// Returns the quotient of two natural numbers, rounded towards plus infinity.
    function automatic integer ceil_div (input longint dividend, input longint divisor);
        automatic longint remainder;

        // pragma translate_off
        `ifndef VERILATOR
        if (dividend < 0) begin
            $fatal(1, "Dividend %0d is not a natural number!", dividend);
        end

        if (divisor < 0) begin
            $fatal(1, "Divisor %0d is not a natural number!", divisor);
        end

        if (divisor == 0) begin
            $fatal(1, "Division by zero!");
        end
        `endif
        // pragma translate_on

        remainder = dividend;
        for (ceil_div = 0; remainder > 0; ceil_div++) begin
            remainder = remainder - divisor;
        end
    endfunction

    /// Index width required to be able to represent up to `num_idx` indices as a binary
    /// encoded signal.
    /// Ensures that the minimum width if an index signal is `1`, regardless of parametrization.
    ///
    /// Sample usage in type definition:
    /// As parameter:
    ///   `parameter type idx_t = logic[cf_math_pkg::idx_width(NumIdx)-1:0]`
    /// As typedef:
    ///   `typedef logic [cf_math_pkg::idx_width(NumIdx)-1:0] idx_t`
    function automatic integer unsigned idx_width (input integer unsigned num_idx);
        return (num_idx > 32'd1) ? unsigned'($clog2(num_idx)) : 32'd1;
    endfunction

endpackage
// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Company:        IIS @ ETHZ - Federal Institute of Technology               //
//                                                                            //
// Engineers:      Lei Li                    lile@iis.ee.ethz.ch              //
//                                                                            //
// Additional contributions by:                                               //
//                                                                            //
//                                                                            //
//                                                                            //
// Create Date:    04/03/2018                                                 //
// Design Name:    FPU                                                        //
// Module Name:    control_mvp.sv                                             //
// Project Name:   Private FPU                                                //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    the control logic  of div and sqrt                         //
//                                                                            //
// Revision Date:  12/04/2018                                                 //
//                 Lei Li                                                     //
//                 To address some requirements by Stefan and add low power   //
//                 control for special cases                                  //
// Revision Date:  13/04/2018                                                 //
//                 Lei Li                                                     //
//                 To fix some bug found in Control FSM                       //
//                 when Iteration_unit_num_S  = 2'b10                         //
//                                                                            //
//                                                                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

//import defs_div_sqrt_mvp::*;

module control_mvp

  (//Input
   input logic                                        Clk_CI,
   input logic                                        Rst_RBI,
   input logic                                        Div_start_SI ,
   input logic                                        Sqrt_start_SI,
   input logic                                        Start_SI,
   input logic                                        Kill_SI,
   input logic                                        Special_case_SBI,
   input logic                                        Special_case_dly_SBI,
   input logic [C_PC-1:0]                             Precision_ctl_SI,
   input logic [1:0]                                  Format_sel_SI,
   input logic [C_MANT_FP64:0]                        Numerator_DI,
   input logic [C_EXP_FP64:0]                         Exp_num_DI,
   input logic [C_MANT_FP64:0]                        Denominator_DI,
   input logic [C_EXP_FP64:0]                         Exp_den_DI,


   output logic                                       Div_start_dly_SO ,
   output logic                                       Sqrt_start_dly_SO,
   output logic                                       Div_enable_SO,
   output logic                                       Sqrt_enable_SO,


   //To next stage
   output logic                                       Full_precision_SO,
   output logic                                       FP32_SO,
   output logic                                       FP64_SO,
   output logic                                       FP16_SO,
   output logic                                       FP16ALT_SO,

   output logic                                       Ready_SO,
   output logic                                       Done_SO,

   output logic [C_MANT_FP64+4:0]                     Mant_result_prenorm_DO,
 //  output logic [3:0]                                 Round_bit_DO,
   output logic [C_EXP_FP64+1:0]                      Exp_result_prenorm_DO
 );

   logic  [C_MANT_FP64+1+4:0]                         Partial_remainder_DN,Partial_remainder_DP; //58bits,r=q+2
   logic  [C_MANT_FP64+4:0]                           Quotient_DP; //57bits
   /////////////////////////////////////////////////////////////////////////////
   // Assign Inputs                                                          //
   /////////////////////////////////////////////////////////////////////////////
   logic [C_MANT_FP64+1:0]                            Numerator_se_D;  //sign extension and hidden bit
   logic [C_MANT_FP64+1:0]                            Denominator_se_D; //signa extension and hidden bit
   logic [C_MANT_FP64+1:0]                            Denominator_se_DB;  //1's complement

   assign  Numerator_se_D={1'b0,Numerator_DI};

   assign  Denominator_se_D={1'b0,Denominator_DI};

  always_comb
   begin
     if(FP32_SO)
       begin
         Denominator_se_DB={~Denominator_se_D[C_MANT_FP64+1:C_MANT_FP64-C_MANT_FP32], {(C_MANT_FP64-C_MANT_FP32){1'b0}} };
       end
     else if(FP64_SO) begin
         Denominator_se_DB=~Denominator_se_D;
     end
     else if(FP16_SO) begin
         Denominator_se_DB={~Denominator_se_D[C_MANT_FP64+1:C_MANT_FP64-C_MANT_FP16], {(C_MANT_FP64-C_MANT_FP16){1'b0}} };
     end
     else begin
         Denominator_se_DB={~Denominator_se_D[C_MANT_FP64+1:C_MANT_FP64-C_MANT_FP16ALT], {(C_MANT_FP64-C_MANT_FP16ALT){1'b0}} };
     end
   end


   logic [C_MANT_FP64+1:0]                            Mant_D_sqrt_Norm;

   assign Mant_D_sqrt_Norm=Exp_num_DI[0]?{1'b0,Numerator_DI}:{Numerator_DI,1'b0}; //for sqrt

   /////////////////////////////////////////////////////////////////////////////
   // Format Selection                                                       //
   /////////////////////////////////////////////////////////////////////////////
   logic [1:0]                                      Format_sel_S;

   always_ff @(posedge Clk_CI, negedge Rst_RBI)
     begin
        if(~Rst_RBI)
          begin
            Format_sel_S<='b0;
          end
        else if(Start_SI&&Ready_SO)
          begin
            Format_sel_S<=Format_sel_SI;
          end
        else
          begin
            Format_sel_S<=Format_sel_S;
          end
    end

   assign FP32_SO = (Format_sel_S==2'b00);
   assign FP64_SO = (Format_sel_S==2'b01);
   assign FP16_SO = (Format_sel_S==2'b10);
   assign FP16ALT_SO = (Format_sel_S==2'b11);



   /////////////////////////////////////////////////////////////////////////////
   // Precision Control                                                       //
   /////////////////////////////////////////////////////////////////////////////

   logic [C_PC-1:0]                                   Precision_ctl_S;
   always_ff @(posedge Clk_CI, negedge Rst_RBI)
     begin
        if(~Rst_RBI)
          begin
            Precision_ctl_S<='b0;
          end
        else if(Start_SI&&Ready_SO)
          begin
            Precision_ctl_S<=Precision_ctl_SI;
          end
        else
          begin
            Precision_ctl_S<=Precision_ctl_S;
          end
    end
  assign Full_precision_SO = (Precision_ctl_S==6'h00);



     logic [5:0]                                     State_ctl_S;
     logic [5:0]                                     State_Two_iteration_unit_S;
     logic [5:0]                                     State_Four_iteration_unit_S;

    assign State_Two_iteration_unit_S = Precision_ctl_S[C_PC-1:1];  //Two iteration units
    assign State_Four_iteration_unit_S = Precision_ctl_S[C_PC-1:2];  //Four iteration units
     always_comb
       begin
         case(Iteration_unit_num_S)
//////////////////////one iteration unit, start///////////////////////////////////////
           2'b00:  //one iteration unit
             begin
               case(Format_sel_S)
                 2'b00: //FP32
                   begin
                     if(Full_precision_SO)
                       begin
                         State_ctl_S = 6'h1b;  //24+4 more iterations for rounding bits
                       end
                     else
                       begin
                         State_ctl_S = Precision_ctl_S;
                       end
                   end
                 2'b01: //FP64
                   begin
                     if(Full_precision_SO)
                       begin
                         State_ctl_S = 6'h38;  //53+4 more iterations for rounding bits
                       end
                     else
                       begin
                         State_ctl_S = Precision_ctl_S;
                       end
                   end
                 2'b10: //FP16
                   begin
                     if(Full_precision_SO)
                       begin
                         State_ctl_S = 6'h0e;  //11+4 more iterations for rounding bits
                       end
                     else
                       begin
                         State_ctl_S = Precision_ctl_S;
                       end
                   end
                 2'b11: //FP16ALT
                   begin
                     if(Full_precision_SO)
                       begin
                         State_ctl_S = 6'h0b;  //8+4 more iterations for rounding bits
                       end
                     else
                       begin
                         State_ctl_S = Precision_ctl_S;
                       end
                  end
                endcase
              end
//////////////////////one iteration unit, end///////////////////////////////////////

//////////////////////two iteration units, start///////////////////////////////////////
           2'b01:  //two iteration units
             begin
               case(Format_sel_S)
                 2'b00: //FP32
                   begin
                     if(Full_precision_SO)
                       begin
                         State_ctl_S = 6'h0d;  //24+4 more iterations for rounding bits
                       end
                     else
                       begin
                         State_ctl_S = State_Two_iteration_unit_S;
                       end
                   end
                 2'b01: //FP64
                   begin
                     if(Full_precision_SO)
                       begin
                         State_ctl_S = 6'h1b;  //53+3 more iterations for rounding bits
                       end
                     else
                       begin
                         State_ctl_S = State_Two_iteration_unit_S;
                       end
                   end
                 2'b10: //FP16
                   begin
                     if(Full_precision_SO)
                       begin
                         State_ctl_S = 6'h06;  //11+3 more iterations for rounding bits
                       end
                     else
                       begin
                         State_ctl_S = State_Two_iteration_unit_S;
                       end
                   end
                 2'b11: //FP16ALT
                   begin
                     if(Full_precision_SO)
                       begin
                         State_ctl_S = 6'h05;  //8+4 more iterations for rounding bits
                       end
                     else
                       begin
                         State_ctl_S = State_Two_iteration_unit_S;
                       end
                  end
                endcase
              end
//////////////////////two iteration units, end///////////////////////////////////////

//////////////////////three iteration units, start///////////////////////////////////////
           2'b10:  //three iteration units
             begin
               case(Format_sel_S)
                 2'b00: //FP32
                   begin
                     case(Precision_ctl_S)
                       6'h00:
                         begin
                           State_ctl_S = 6'h08;  //24+3 more iterations for rounding bits
                         end
                       6'h06,6'h07,6'h08:
                         begin
                           State_ctl_S = 6'h02;
                         end
                       6'h09,6'h0a,6'h0b:
                         begin
                           State_ctl_S = 6'h03;
                         end
                       6'h0c,6'h0d,6'h0e:
                         begin
                           State_ctl_S = 6'h04;
                         end
                       6'h0f,6'h10,6'h11:
                         begin
                           State_ctl_S = 6'h05;
                         end
                       6'h12,6'h13,6'h14:
                         begin
                           State_ctl_S = 6'h06;
                         end
                       6'h15,6'h16,6'h17:
                         begin
                           State_ctl_S = 6'h07;
                         end
                       default:
                         begin
                           State_ctl_S = 6'h08;  //24+3 more iterations for rounding bits
                         end
                     endcase
                   end
                 2'b01: //FP64
                   begin
                     case(Precision_ctl_S)
                       6'h00:
                         begin
                           State_ctl_S = 6'h12;  //53+4 more iterations for rounding bits
                         end
                       6'h06,6'h07,6'h08:
                         begin
                           State_ctl_S = 6'h02;
                         end
                       6'h09,6'h0a,6'h0b:
                         begin
                           State_ctl_S = 6'h03;
                         end
                       6'h0c,6'h0d,6'h0e:
                         begin
                           State_ctl_S = 6'h04;
                         end
                       6'h0f,6'h10,6'h11:
                         begin
                           State_ctl_S = 6'h05;
                         end
                       6'h12,6'h13,6'h14:
                         begin
                           State_ctl_S = 6'h06;
                         end
                       6'h15,6'h16,6'h17:
                         begin
                           State_ctl_S = 6'h07;
                         end
                       6'h18,6'h19,6'h1a:
                         begin
                           State_ctl_S = 6'h08;
                         end
                       6'h1b,6'h1c,6'h1d:
                         begin
                           State_ctl_S = 6'h09;
                         end
                       6'h1e,6'h1f,6'h20:
                         begin
                           State_ctl_S = 6'h0a;
                         end
                       6'h21,6'h22,6'h23:
                         begin
                           State_ctl_S = 6'h0b;
                         end
                       6'h24,6'h25,6'h26:
                         begin
                           State_ctl_S = 6'h0c;
                         end
                       6'h27,6'h28,6'h29:
                         begin
                           State_ctl_S = 6'h0d;
                         end
                       6'h2a,6'h2b,6'h2c:
                         begin
                           State_ctl_S = 6'h0e;
                         end
                       6'h2d,6'h2e,6'h2f:
                         begin
                           State_ctl_S = 6'h0f;
                         end
                       6'h30,6'h31,6'h32:
                         begin
                           State_ctl_S = 6'h10;
                         end
                       6'h33,6'h34,6'h35:
                         begin
                           State_ctl_S = 6'h11;
                         end
                       default:
                         begin
                           State_ctl_S = 6'h12;  //53+4 more iterations for rounding bits
                         end
                     endcase
                   end
                 2'b10: //FP16
                   begin
                     case(Precision_ctl_S)
                       6'h00:
                         begin
                           State_ctl_S = 6'h04;  //12+3 more iterations for rounding bits
                         end
                       6'h06,6'h07,6'h08:
                         begin
                           State_ctl_S = 6'h02;
                         end
                       6'h09,6'h0a,6'h0b:
                         begin
                           State_ctl_S = 6'h03;
                         end
                       default:
                         begin
                           State_ctl_S = 6'h04;  //12+3 more iterations for rounding bits
                         end
                     endcase
                   end
                 2'b11: //FP16ALT
                   begin
                     case(Precision_ctl_S)
                       6'h00:
                         begin
                           State_ctl_S = 6'h03;  //8+4 more iterations for rounding bits
                         end
                       6'h06,6'h07,6'h08:
                         begin
                           State_ctl_S = 6'h02;
                         end
                       default:
                         begin
                           State_ctl_S = 6'h03;  //8+4 more iterations for rounding bits
                         end
                     endcase
                  end
                endcase
              end
//////////////////////three iteration units, end///////////////////////////////////////

//////////////////////four iteration units, start///////////////////////////////////////
           2'b11:  //four iteration units
             begin
               case(Format_sel_S)
                 2'b00: //FP32
                   begin
                     if(Full_precision_SO)
                       begin
                         State_ctl_S = 6'h06;  //24+4 more iterations for rounding bits
                       end
                     else
                       begin
                         State_ctl_S = State_Four_iteration_unit_S;
                       end
                   end
                 2'b01: //FP64
                   begin
                     if(Full_precision_SO)
                       begin
                         State_ctl_S = 6'h0d;  //53+3 more iterations for rounding bits
                       end
                     else
                       begin
                         State_ctl_S = State_Four_iteration_unit_S;
                       end
                   end
                 2'b10: //FP16
                   begin
                     if(Full_precision_SO)
                       begin
                         State_ctl_S = 6'h03;  //11+4 more iterations for rounding bits
                       end
                     else
                       begin
                         State_ctl_S = State_Four_iteration_unit_S;
                       end
                   end
                 2'b11: //FP16ALT
                   begin
                     if(Full_precision_SO)
                       begin
                         State_ctl_S = 6'h02;  //8+4 more iterations for rounding bits
                       end
                     else
                       begin
                         State_ctl_S = State_Four_iteration_unit_S;
                       end
                  end
                endcase
              end
//////////////////////four iteration units, end///////////////////////////////////////

           endcase
        end


   /////////////////////////////////////////////////////////////////////////////
   // control logic                                                           //
   /////////////////////////////////////////////////////////////////////////////

   logic                                               Div_start_dly_S;

   always_ff @(posedge Clk_CI, negedge Rst_RBI)   //  generate Div_start_dly_S signal
     begin
        if(~Rst_RBI)
          begin
            Div_start_dly_S<=1'b0;
          end
        else if(Div_start_SI&&Ready_SO)
         begin
           Div_start_dly_S<=1'b1;
         end
        else
          begin
            Div_start_dly_S<=1'b0;
          end
    end

   assign Div_start_dly_SO=Div_start_dly_S;

  always_ff @(posedge Clk_CI, negedge Rst_RBI) begin  //  generate Div_enable_SO signal
    if(~Rst_RBI)
      Div_enable_SO<=1'b0;
    // Synchronous reset with Flush
    else if (Kill_SI)
      Div_enable_SO <= 1'b0;
    else if(Div_start_SI&&Ready_SO)
      Div_enable_SO<=1'b1;
    else if(Done_SO)
      Div_enable_SO<=1'b0;
    else
      Div_enable_SO<=Div_enable_SO;
  end

   logic                                                Sqrt_start_dly_S;

   always_ff @(posedge Clk_CI, negedge Rst_RBI)   //  generate Sqrt_start_dly_SI signal
     begin
        if(~Rst_RBI)
          begin
            Sqrt_start_dly_S<=1'b0;
          end
        else if(Sqrt_start_SI&&Ready_SO)
         begin
           Sqrt_start_dly_S<=1'b1;
         end
        else
          begin
            Sqrt_start_dly_S<=1'b0;
          end
      end
    assign Sqrt_start_dly_SO=Sqrt_start_dly_S;

   always_ff @(posedge Clk_CI, negedge Rst_RBI) begin   //  generate Sqrt_enable_SO signal
    if(~Rst_RBI)
      Sqrt_enable_SO<=1'b0;
    else if (Kill_SI)
      Sqrt_enable_SO <= 1'b0;
    else if(Sqrt_start_SI&&Ready_SO)
      Sqrt_enable_SO<=1'b1;
    else if(Done_SO)
      Sqrt_enable_SO<=1'b0;
    else
      Sqrt_enable_SO<=Sqrt_enable_SO;
  end

   logic [5:0]                                                  Crtl_cnt_S;
   logic                                                        Start_dly_S;

   assign   Start_dly_S=Div_start_dly_S |Sqrt_start_dly_S;

   logic       Fsm_enable_S;
   assign      Fsm_enable_S=( (Start_dly_S | (| Crtl_cnt_S)) && (~Kill_SI) && Special_case_dly_SBI);

   logic                                                        Final_state_S;
   assign     Final_state_S= (Crtl_cnt_S==State_ctl_S);


   always_ff @(posedge Clk_CI, negedge Rst_RBI) //control_FSM
     begin
        if (~Rst_RBI)
          begin
             Crtl_cnt_S    <= '0;
          end
          else if (Final_state_S | Kill_SI)
            begin
              Crtl_cnt_S    <= '0;
            end
          else if(Fsm_enable_S) // one cycle Start_SI
            begin
              Crtl_cnt_S    <= Crtl_cnt_S+1;
            end
          else
            begin
              Crtl_cnt_S    <= '0;
            end
     end // always_ff



    always_ff @(posedge Clk_CI, negedge Rst_RBI) //Generate  Done_SO,  they can share this Done_SO.
      begin
        if(~Rst_RBI)
          begin
            Done_SO<=1'b0;
          end
        else if(Start_SI&&Ready_SO)
          begin
            if(~Special_case_SBI)
              begin
                Done_SO<=1'b1;
              end
            else
              begin
                Done_SO<=1'b0;
              end
          end
        else if(Final_state_S)
          begin
            Done_SO<=1'b1;
          end
        else
          begin
            Done_SO<=1'b0;
          end
       end




   always_ff @(posedge Clk_CI, negedge Rst_RBI) //Generate  Ready_SO
     begin
       if(~Rst_RBI)
         begin
           Ready_SO<=1'b1;
         end

       else if(Start_SI&&Ready_SO)
         begin
            if(~Special_case_SBI)
              begin
                Ready_SO<=1'b1;
              end
            else
              begin
                Ready_SO<=1'b0;
              end
         end
       else if(Final_state_S | Kill_SI)
         begin
           Ready_SO<=1'b1;
         end
       else
         begin
           Ready_SO<=Ready_SO;
         end
     end


  /////////////////////////////////////////////////////////////////////////////
   // Declarations for square root when Iteration_unit_num_S = 2'b00, start  //
   ////////////////////////////////////////////////////////////////////////////

  logic                                    Qcnt_one_0;
  logic                                    Qcnt_one_1;
  logic [1:0]                              Qcnt_one_2;
  logic [2:0]                              Qcnt_one_3;
  logic [3:0]                              Qcnt_one_4;
  logic [4:0]                              Qcnt_one_5;
  logic [5:0]                              Qcnt_one_6;
  logic [6:0]                              Qcnt_one_7;
  logic [7:0]                              Qcnt_one_8;
  logic [8:0]                              Qcnt_one_9;
  logic [9:0]                              Qcnt_one_10;
  logic [10:0]                             Qcnt_one_11;
  logic [11:0]                             Qcnt_one_12;
  logic [12:0]                             Qcnt_one_13;
  logic [13:0]                             Qcnt_one_14;
  logic [14:0]                             Qcnt_one_15;
  logic [15:0]                             Qcnt_one_16;
  logic [16:0]                             Qcnt_one_17;
  logic [17:0]                             Qcnt_one_18;
  logic [18:0]                             Qcnt_one_19;
  logic [19:0]                             Qcnt_one_20;
  logic [20:0]                             Qcnt_one_21;
  logic [21:0]                             Qcnt_one_22;
  logic [22:0]                             Qcnt_one_23;
  logic [23:0]                             Qcnt_one_24;
  logic [24:0]                             Qcnt_one_25;
  logic [25:0]                             Qcnt_one_26;
  logic [26:0]                             Qcnt_one_27;
  logic [27:0]                             Qcnt_one_28;
  logic [28:0]                             Qcnt_one_29;
  logic [29:0]                             Qcnt_one_30;
  logic [30:0]                             Qcnt_one_31;
  logic [31:0]                             Qcnt_one_32;
  logic [32:0]                             Qcnt_one_33;
  logic [33:0]                             Qcnt_one_34;
  logic [34:0]                             Qcnt_one_35;
  logic [35:0]                             Qcnt_one_36;
  logic [36:0]                             Qcnt_one_37;
  logic [37:0]                             Qcnt_one_38;
  logic [38:0]                             Qcnt_one_39;
  logic [39:0]                             Qcnt_one_40;
  logic [40:0]                             Qcnt_one_41;
  logic [41:0]                             Qcnt_one_42;
  logic [42:0]                             Qcnt_one_43;
  logic [43:0]                             Qcnt_one_44;
  logic [44:0]                             Qcnt_one_45;
  logic [45:0]                             Qcnt_one_46;
  logic [46:0]                             Qcnt_one_47;
  logic [47:0]                             Qcnt_one_48;
  logic [48:0]                             Qcnt_one_49;
  logic [49:0]                             Qcnt_one_50;
  logic [50:0]                             Qcnt_one_51;
  logic [51:0]                             Qcnt_one_52;
  logic [52:0]                             Qcnt_one_53;
  logic [53:0]                             Qcnt_one_54;
  logic [54:0]                             Qcnt_one_55;
  logic [55:0]                             Qcnt_one_56;
  logic [56:0]                             Qcnt_one_57;
  logic [57:0]                             Qcnt_one_58;
  logic [58:0]                             Qcnt_one_59;
  logic [59:0]                             Qcnt_one_60;

  /////////////////////////////////////////////////////////////////////////////
   // Declarations for square root when Iteration_unit_num_S = 2'b00, end    //
   ////////////////////////////////////////////////////////////////////////////



  /////////////////////////////////////////////////////////////////////////////
   // Declarations for square root when Iteration_unit_num_S = 2'b01, start  //
   ////////////////////////////////////////////////////////////////////////////
  logic [1:0]                              Qcnt_two_0;
  logic [2:0]                              Qcnt_two_1;
  logic [4:0]                              Qcnt_two_2;
  logic [6:0]                              Qcnt_two_3;
  logic [8:0]                              Qcnt_two_4;
  logic [10:0]                             Qcnt_two_5;
  logic [12:0]                             Qcnt_two_6;
  logic [14:0]                             Qcnt_two_7;
  logic [16:0]                             Qcnt_two_8;
  logic [18:0]                             Qcnt_two_9;
  logic [20:0]                             Qcnt_two_10;
  logic [22:0]                             Qcnt_two_11;
  logic [24:0]                             Qcnt_two_12;
  logic [26:0]                             Qcnt_two_13;
  logic [28:0]                             Qcnt_two_14;
  logic [30:0]                             Qcnt_two_15;
  logic [32:0]                             Qcnt_two_16;
  logic [34:0]                             Qcnt_two_17;
  logic [36:0]                             Qcnt_two_18;
  logic [38:0]                             Qcnt_two_19;
  logic [40:0]                             Qcnt_two_20;
  logic [42:0]                             Qcnt_two_21;
  logic [44:0]                             Qcnt_two_22;
  logic [46:0]                             Qcnt_two_23;
  logic [48:0]                             Qcnt_two_24;
  logic [50:0]                             Qcnt_two_25;
  logic [52:0]                             Qcnt_two_26;
  logic [54:0]                             Qcnt_two_27;
  logic [56:0]                             Qcnt_two_28;
  /////////////////////////////////////////////////////////////////////////////
   // Declarations for square root when Iteration_unit_num_S = 2'b01, end    //
   ////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////////
   // Declarations for square root when Iteration_unit_num_S = 2'b10, start  //
   ////////////////////////////////////////////////////////////////////////////
  logic [2:0]                              Qcnt_three_0;
  logic [4:0]                              Qcnt_three_1;
  logic [7:0]                              Qcnt_three_2;
  logic [10:0]                             Qcnt_three_3;
  logic [13:0]                             Qcnt_three_4;
  logic [16:0]                             Qcnt_three_5;
  logic [19:0]                             Qcnt_three_6;
  logic [22:0]                             Qcnt_three_7;
  logic [25:0]                             Qcnt_three_8;
  logic [28:0]                             Qcnt_three_9;
  logic [31:0]                             Qcnt_three_10;
  logic [34:0]                             Qcnt_three_11;
  logic [37:0]                             Qcnt_three_12;
  logic [40:0]                             Qcnt_three_13;
  logic [43:0]                             Qcnt_three_14;
  logic [46:0]                             Qcnt_three_15;
  logic [49:0]                             Qcnt_three_16;
  logic [52:0]                             Qcnt_three_17;
  logic [55:0]                             Qcnt_three_18;
  logic [58:0]                             Qcnt_three_19;
  logic [61:0]                             Qcnt_three_20;
  /////////////////////////////////////////////////////////////////////////////
   // Declarations for square root when Iteration_unit_num_S = 2'b10, end    //
   ////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////////
   // Declarations for square root when Iteration_unit_num_S = 2'b11, start  //
   ////////////////////////////////////////////////////////////////////////////
  logic [3:0]                              Qcnt_four_0;
  logic [6:0]                              Qcnt_four_1;
  logic [10:0]                             Qcnt_four_2;
  logic [14:0]                             Qcnt_four_3;
  logic [18:0]                             Qcnt_four_4;
  logic [22:0]                             Qcnt_four_5;
  logic [26:0]                             Qcnt_four_6;
  logic [30:0]                             Qcnt_four_7;
  logic [34:0]                             Qcnt_four_8;
  logic [38:0]                             Qcnt_four_9;
  logic [42:0]                             Qcnt_four_10;
  logic [46:0]                             Qcnt_four_11;
  logic [50:0]                             Qcnt_four_12;
  logic [54:0]                             Qcnt_four_13;
  logic [58:0]                             Qcnt_four_14;

  /////////////////////////////////////////////////////////////////////////////
   // Declarations for square root when Iteration_unit_num_S = 2'b11, end    //
   ////////////////////////////////////////////////////////////////////////////



   logic [C_MANT_FP64+1+4:0]                                      Sqrt_R0,Sqrt_Q0,Q_sqrt0,Q_sqrt_com_0;
   logic [C_MANT_FP64+1+4:0]                                      Sqrt_R1,Sqrt_Q1,Q_sqrt1,Q_sqrt_com_1;
   logic [C_MANT_FP64+1+4:0]                                      Sqrt_R2,Sqrt_Q2,Q_sqrt2,Q_sqrt_com_2;
   logic [C_MANT_FP64+1+4:0]                                      Sqrt_R3,Sqrt_Q3,Q_sqrt3,Q_sqrt_com_3,Sqrt_R4; //Sqrt_Q4;


   logic [1:0]                                                    Sqrt_DI  [3:0];
   logic [1:0]                                                    Sqrt_DO  [3:0];
   logic                                                          Sqrt_carry_DO;


  logic  [C_MANT_FP64+1+4:0]                                      Iteration_cell_a_D [3:0];
  logic  [C_MANT_FP64+1+4:0]                                      Iteration_cell_b_D [3:0];
  logic  [C_MANT_FP64+1+4:0]                                      Iteration_cell_a_BMASK_D [3:0];
  logic  [C_MANT_FP64+1+4:0]                                      Iteration_cell_b_BMASK_D [3:0];
  logic                                                           Iteration_cell_carry_D [3:0];
  logic  [C_MANT_FP64+1+4:0]                                      Iteration_cell_sum_D [3:0];
  logic  [C_MANT_FP64+1+4:0]                                      Iteration_cell_sum_AMASK_D [3:0];


  logic [3:0]                                                     Sqrt_quotinent_S;


   always_comb
    begin  //
      case (Format_sel_S)
        2'b00:
          begin
            Sqrt_quotinent_S = {(~Iteration_cell_sum_AMASK_D[0][C_MANT_FP32+5]),(~Iteration_cell_sum_AMASK_D[1][C_MANT_FP32+5]),(~Iteration_cell_sum_AMASK_D[2][C_MANT_FP32+5]),(~Iteration_cell_sum_AMASK_D[3][C_MANT_FP32+5])};
            Q_sqrt_com_0 ={ {(C_MANT_FP64-C_MANT_FP32){1'b0}},~Q_sqrt0[C_MANT_FP32+5:0] };
            Q_sqrt_com_1 ={ {(C_MANT_FP64-C_MANT_FP32){1'b0}},~Q_sqrt1[C_MANT_FP32+5:0] };
            Q_sqrt_com_2 ={ {(C_MANT_FP64-C_MANT_FP32){1'b0}},~Q_sqrt2[C_MANT_FP32+5:0] };
            Q_sqrt_com_3 ={ {(C_MANT_FP64-C_MANT_FP32){1'b0}},~Q_sqrt3[C_MANT_FP32+5:0] };
          end
        2'b01:
          begin
            Sqrt_quotinent_S = {Iteration_cell_carry_D[0],Iteration_cell_carry_D[1],Iteration_cell_carry_D[2],Iteration_cell_carry_D[3]};
            Q_sqrt_com_0=~Q_sqrt0;
            Q_sqrt_com_1=~Q_sqrt1;
            Q_sqrt_com_2=~Q_sqrt2;
            Q_sqrt_com_3=~Q_sqrt3;
          end
        2'b10:
          begin
            Sqrt_quotinent_S = {(~Iteration_cell_sum_AMASK_D[0][C_MANT_FP16+5]),(~Iteration_cell_sum_AMASK_D[1][C_MANT_FP16+5]),(~Iteration_cell_sum_AMASK_D[2][C_MANT_FP16+5]),(~Iteration_cell_sum_AMASK_D[3][C_MANT_FP16+5])};
            Q_sqrt_com_0 ={ {(C_MANT_FP64-C_MANT_FP16){1'b0}},~Q_sqrt0[C_MANT_FP16+5:0] };
            Q_sqrt_com_1 ={ {(C_MANT_FP64-C_MANT_FP16){1'b0}},~Q_sqrt1[C_MANT_FP16+5:0] };
            Q_sqrt_com_2 ={ {(C_MANT_FP64-C_MANT_FP16){1'b0}},~Q_sqrt2[C_MANT_FP16+5:0] };
            Q_sqrt_com_3 ={ {(C_MANT_FP64-C_MANT_FP16){1'b0}},~Q_sqrt3[C_MANT_FP16+5:0] };
          end
        2'b11:
          begin
            Sqrt_quotinent_S = {(~Iteration_cell_sum_AMASK_D[0][C_MANT_FP16ALT+5]),(~Iteration_cell_sum_AMASK_D[1][C_MANT_FP16ALT+5]),(~Iteration_cell_sum_AMASK_D[2][C_MANT_FP16ALT+5]),(~Iteration_cell_sum_AMASK_D[3][C_MANT_FP16ALT+5])};
            Q_sqrt_com_0 ={ {(C_MANT_FP64-C_MANT_FP16ALT){1'b0}},~Q_sqrt0[C_MANT_FP16ALT+5:0] };
            Q_sqrt_com_1 ={ {(C_MANT_FP64-C_MANT_FP16ALT){1'b0}},~Q_sqrt1[C_MANT_FP16ALT+5:0] };
            Q_sqrt_com_2 ={ {(C_MANT_FP64-C_MANT_FP16ALT){1'b0}},~Q_sqrt2[C_MANT_FP16ALT+5:0] };
            Q_sqrt_com_3 ={ {(C_MANT_FP64-C_MANT_FP16ALT){1'b0}},~Q_sqrt3[C_MANT_FP16ALT+5:0] };
          end
        endcase
    end



  assign  Qcnt_one_0=    {1'b0};  //qk for each feedback
  assign  Qcnt_one_1=    {Quotient_DP[0]};
  assign  Qcnt_one_2=    {Quotient_DP[1:0]};
  assign  Qcnt_one_3=    {Quotient_DP[2:0]};
  assign  Qcnt_one_4=    {Quotient_DP[3:0]};
  assign  Qcnt_one_5=    {Quotient_DP[4:0]};
  assign  Qcnt_one_6=    {Quotient_DP[5:0]};
  assign  Qcnt_one_7=    {Quotient_DP[6:0]};
  assign  Qcnt_one_8=    {Quotient_DP[7:0]};
  assign  Qcnt_one_9=    {Quotient_DP[8:0]};
  assign  Qcnt_one_10=    {Quotient_DP[9:0]};
  assign  Qcnt_one_11=    {Quotient_DP[10:0]};
  assign  Qcnt_one_12=    {Quotient_DP[11:0]};
  assign  Qcnt_one_13=    {Quotient_DP[12:0]};
  assign  Qcnt_one_14=    {Quotient_DP[13:0]};
  assign  Qcnt_one_15=    {Quotient_DP[14:0]};
  assign  Qcnt_one_16=    {Quotient_DP[15:0]};
  assign  Qcnt_one_17=    {Quotient_DP[16:0]};
  assign  Qcnt_one_18=    {Quotient_DP[17:0]};
  assign  Qcnt_one_19=    {Quotient_DP[18:0]};
  assign  Qcnt_one_20=    {Quotient_DP[19:0]};
  assign  Qcnt_one_21=    {Quotient_DP[20:0]};
  assign  Qcnt_one_22=    {Quotient_DP[21:0]};
  assign  Qcnt_one_23=    {Quotient_DP[22:0]};
  assign  Qcnt_one_24=    {Quotient_DP[23:0]};
  assign  Qcnt_one_25=    {Quotient_DP[24:0]};
  assign  Qcnt_one_26=    {Quotient_DP[25:0]};
  assign  Qcnt_one_27=    {Quotient_DP[26:0]};
  assign  Qcnt_one_28=    {Quotient_DP[27:0]};
  assign  Qcnt_one_29=    {Quotient_DP[28:0]};
  assign  Qcnt_one_30=    {Quotient_DP[29:0]};
  assign  Qcnt_one_31=    {Quotient_DP[30:0]};
  assign  Qcnt_one_32=    {Quotient_DP[31:0]};
  assign  Qcnt_one_33=    {Quotient_DP[32:0]};
  assign  Qcnt_one_34=    {Quotient_DP[33:0]};
  assign  Qcnt_one_35=    {Quotient_DP[34:0]};
  assign  Qcnt_one_36=    {Quotient_DP[35:0]};
  assign  Qcnt_one_37=    {Quotient_DP[36:0]};
  assign  Qcnt_one_38=    {Quotient_DP[37:0]};
  assign  Qcnt_one_39=    {Quotient_DP[38:0]};
  assign  Qcnt_one_40=    {Quotient_DP[39:0]};
  assign  Qcnt_one_41=    {Quotient_DP[40:0]};
  assign  Qcnt_one_42=    {Quotient_DP[41:0]};
  assign  Qcnt_one_43=    {Quotient_DP[42:0]};
  assign  Qcnt_one_44=    {Quotient_DP[43:0]};
  assign  Qcnt_one_45=    {Quotient_DP[44:0]};
  assign  Qcnt_one_46=    {Quotient_DP[45:0]};
  assign  Qcnt_one_47=    {Quotient_DP[46:0]};
  assign  Qcnt_one_48=    {Quotient_DP[47:0]};
  assign  Qcnt_one_49=    {Quotient_DP[48:0]};
  assign  Qcnt_one_50=    {Quotient_DP[49:0]};
  assign  Qcnt_one_51=    {Quotient_DP[50:0]};
  assign  Qcnt_one_52=    {Quotient_DP[51:0]};
  assign  Qcnt_one_53=    {Quotient_DP[52:0]};
  assign  Qcnt_one_54=    {Quotient_DP[53:0]};
  assign  Qcnt_one_55=    {Quotient_DP[54:0]};
  assign  Qcnt_one_56=    {Quotient_DP[55:0]};
  assign  Qcnt_one_57=    {Quotient_DP[56:0]};


  assign  Qcnt_two_0 =    {1'b0,            Sqrt_quotinent_S[3]};  //qk for each feedback
  assign  Qcnt_two_1 =    {Quotient_DP[1:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_2 =    {Quotient_DP[3:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_3 =    {Quotient_DP[5:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_4 =    {Quotient_DP[7:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_5 =    {Quotient_DP[9:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_6 =    {Quotient_DP[11:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_7 =    {Quotient_DP[13:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_8 =    {Quotient_DP[15:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_9 =    {Quotient_DP[17:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_10 =    {Quotient_DP[19:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_11 =    {Quotient_DP[21:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_12 =    {Quotient_DP[23:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_13 =    {Quotient_DP[25:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_14 =    {Quotient_DP[27:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_15 =    {Quotient_DP[29:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_16 =    {Quotient_DP[31:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_17 =    {Quotient_DP[33:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_18 =    {Quotient_DP[35:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_19 =    {Quotient_DP[37:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_20 =    {Quotient_DP[39:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_21 =    {Quotient_DP[41:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_22 =    {Quotient_DP[43:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_23 =    {Quotient_DP[45:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_24 =    {Quotient_DP[47:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_25 =    {Quotient_DP[49:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_26 =    {Quotient_DP[51:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_27 =    {Quotient_DP[53:0],Sqrt_quotinent_S[3]};
  assign  Qcnt_two_28 =    {Quotient_DP[55:0],Sqrt_quotinent_S[3]};


  assign  Qcnt_three_0 =    {1'b0,            Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};  //qk for each feedback
  assign  Qcnt_three_1 =    {Quotient_DP[2:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_2 =    {Quotient_DP[5:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_3 =    {Quotient_DP[8:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_4 =    {Quotient_DP[11:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_5 =    {Quotient_DP[14:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_6 =    {Quotient_DP[17:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_7 =    {Quotient_DP[20:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_8 =    {Quotient_DP[23:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_9 =    {Quotient_DP[26:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_10 =    {Quotient_DP[29:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_11 =    {Quotient_DP[32:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_12 =    {Quotient_DP[35:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_13 =    {Quotient_DP[38:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_14 =    {Quotient_DP[41:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_15 =    {Quotient_DP[44:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_16 =    {Quotient_DP[47:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_17 =    {Quotient_DP[50:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_18 =    {Quotient_DP[53:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};
  assign  Qcnt_three_19 =    {Quotient_DP[56:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2]};


  assign      Qcnt_four_0 =    {1'b0,            Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_1 =    {Quotient_DP[3:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_2 =    {Quotient_DP[7:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_3 =    {Quotient_DP[11:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_4 =    {Quotient_DP[15:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_5 =    {Quotient_DP[19:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_6 =    {Quotient_DP[23:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_7 =    {Quotient_DP[27:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_8 =    {Quotient_DP[31:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_9 =    {Quotient_DP[35:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_10 =    {Quotient_DP[39:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_11 =    {Quotient_DP[43:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_12 =    {Quotient_DP[47:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_13 =    {Quotient_DP[51:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};
  assign      Qcnt_four_14 =    {Quotient_DP[55:0],Sqrt_quotinent_S[3],Sqrt_quotinent_S[2],Sqrt_quotinent_S[1]};




  always_comb begin  // the intermediate operands for sqrt

  case(Iteration_unit_num_S)
    2'b00:
      begin

  /////////////////////////////////////////////////////////////////////////////
   // Operands for square root when Iteration_unit_num_S = 2'b00, start       //
   /////////////////////////////////////////////////////////////////////////////




        case(Crtl_cnt_S)

          6'b000000:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64+1:C_MANT_FP64];
              Q_sqrt0={{(C_MANT_FP64+5){1'b0}},Qcnt_one_0};
              Sqrt_Q0=Q_sqrt_com_0;
            end
          6'b000001:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-1:C_MANT_FP64-2];
              Q_sqrt0={{(C_MANT_FP64+5){1'b0}},Qcnt_one_1};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b000010:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-3:C_MANT_FP64-4];
              Q_sqrt0={{(C_MANT_FP64+4){1'b0}},Qcnt_one_2};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b000011:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-5:C_MANT_FP64-6];
              Q_sqrt0={{(C_MANT_FP64+3){1'b0}},Qcnt_one_3};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b000100:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-7:C_MANT_FP64-8];
              Q_sqrt0={{(C_MANT_FP64+2){1'b0}},Qcnt_one_4};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b000101:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-9:C_MANT_FP64-10];
              Q_sqrt0={{(C_MANT_FP64+1){1'b0}},Qcnt_one_5};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b000110:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-11:C_MANT_FP64-12];
              Q_sqrt0={{(C_MANT_FP64){1'b0}},Qcnt_one_6};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b000111:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-13:C_MANT_FP64-14];
              Q_sqrt0={{(C_MANT_FP64-1){1'b0}},Qcnt_one_7};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b001000:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-15:C_MANT_FP64-16];
              Q_sqrt0={{(C_MANT_FP64-2){1'b0}},Qcnt_one_8};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b001001:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-17:C_MANT_FP64-18];
              Q_sqrt0={{(C_MANT_FP64-3){1'b0}},Qcnt_one_9};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b001010:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-19:C_MANT_FP64-20];
              Q_sqrt0={{(C_MANT_FP64-4){1'b0}},Qcnt_one_10};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b001011:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-21:C_MANT_FP64-22];
              Q_sqrt0={{(C_MANT_FP64-5){1'b0}},Qcnt_one_11};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b001100:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-23:C_MANT_FP64-24];
              Q_sqrt0={{(C_MANT_FP64-6){1'b0}},Qcnt_one_12};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b001101:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-25:C_MANT_FP64-26];
              Q_sqrt0={{(C_MANT_FP64-7){1'b0}},Qcnt_one_13};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b001110:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-27:C_MANT_FP64-28];
              Q_sqrt0={{(C_MANT_FP64-8){1'b0}},Qcnt_one_14};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b001111:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-29:C_MANT_FP64-30];
              Q_sqrt0={{(C_MANT_FP64-9){1'b0}},Qcnt_one_15};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b010000:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-31:C_MANT_FP64-32];
              Q_sqrt0={{(C_MANT_FP64-10){1'b0}},Qcnt_one_16};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b010001:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-33:C_MANT_FP64-34];
              Q_sqrt0={{(C_MANT_FP64-11){1'b0}},Qcnt_one_17};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b010010:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-35:C_MANT_FP64-36];
              Q_sqrt0={{(C_MANT_FP64-12){1'b0}},Qcnt_one_18};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b010011:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-37:C_MANT_FP64-38];
              Q_sqrt0={{(C_MANT_FP64-13){1'b0}},Qcnt_one_19};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b010100:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-39:C_MANT_FP64-40];
              Q_sqrt0={{(C_MANT_FP64-14){1'b0}},Qcnt_one_20};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b010101:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-41:C_MANT_FP64-42];
              Q_sqrt0={{(C_MANT_FP64-15){1'b0}},Qcnt_one_21};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b010110:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-43:C_MANT_FP64-44];
              Q_sqrt0={{(C_MANT_FP64-16){1'b0}},Qcnt_one_22};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b010111:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-45:C_MANT_FP64-46];
              Q_sqrt0={{(C_MANT_FP64-17){1'b0}},Qcnt_one_23};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b011000:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-47:C_MANT_FP64-48];
              Q_sqrt0={{(C_MANT_FP64-18){1'b0}},Qcnt_one_24};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b011001:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-49:C_MANT_FP64-50];
              Q_sqrt0={{(C_MANT_FP64-19){1'b0}},Qcnt_one_25};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b011010:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-51:C_MANT_FP64-52];
              Q_sqrt0={{(C_MANT_FP64-20){1'b0}},Qcnt_one_26};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b011011:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-21){1'b0}},Qcnt_one_27};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b011100:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-22){1'b0}},Qcnt_one_28};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b011101:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-23){1'b0}},Qcnt_one_29};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b011110:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-24){1'b0}},Qcnt_one_30};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b011111:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-25){1'b0}},Qcnt_one_31};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b100000:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-26){1'b0}},Qcnt_one_32};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b100001:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-27){1'b0}},Qcnt_one_33};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b100010:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-28){1'b0}},Qcnt_one_34};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b100011:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-29){1'b0}},Qcnt_one_35};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b100100:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-30){1'b0}},Qcnt_one_36};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b100101:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-31){1'b0}},Qcnt_one_37};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b100110:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-32){1'b0}},Qcnt_one_38};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b100111:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-33){1'b0}},Qcnt_one_39};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b101000:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-34){1'b0}},Qcnt_one_40};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b101001:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-35){1'b0}},Qcnt_one_41};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b101010:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-36){1'b0}},Qcnt_one_42};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b101011:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-37){1'b0}},Qcnt_one_43};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b101100:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-38){1'b0}},Qcnt_one_44};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b101101:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-39){1'b0}},Qcnt_one_45};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b101110:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-40){1'b0}},Qcnt_one_46};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b101111:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-41){1'b0}},Qcnt_one_47};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b110000:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-42){1'b0}},Qcnt_one_48};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b110001:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-43){1'b0}},Qcnt_one_49};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b110010:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-44){1'b0}},Qcnt_one_50};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b110011:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-45){1'b0}},Qcnt_one_51};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b110100:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-46){1'b0}},Qcnt_one_52};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b110101:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-47){1'b0}},Qcnt_one_53};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b110110:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-48){1'b0}},Qcnt_one_54};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b110111:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-49){1'b0}},Qcnt_one_55};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end
          6'b111000:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-50){1'b0}},Qcnt_one_56};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
            end

          default:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0='0;
              Sqrt_Q0='0;
            end
        endcase
      end


   /////////////////////////////////////////////////////////////////////////////
   // Operands for square root when Iteration_unit_num_S = 2'b00, end         //
   /////////////////////////////////////////////////////////////////////////////


    2'b01:
      begin
   /////////////////////////////////////////////////////////////////////////////
   // Operands for square root when Iteration_unit_num_S = 2'b01, start       //
   /////////////////////////////////////////////////////////////////////////////
        case(Crtl_cnt_S)

          6'b000000:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64+1:C_MANT_FP64];
              Q_sqrt0={{(C_MANT_FP64+5){1'b0}},Qcnt_two_0[1]};
              Sqrt_Q0=Q_sqrt_com_0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-1:C_MANT_FP64-2];
              Q_sqrt1={{(C_MANT_FP64+4){1'b0}},Qcnt_two_0[1:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b000001:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-3:C_MANT_FP64-4];
              Q_sqrt0={{(C_MANT_FP64+4){1'b0}},Qcnt_two_1[2:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-5:C_MANT_FP64-6];
              Q_sqrt1={{(C_MANT_FP64+3){1'b0}},Qcnt_two_1[2:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b000010:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-7:C_MANT_FP64-8];
              Q_sqrt0={{(C_MANT_FP64+2){1'b0}},Qcnt_two_2[4:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-9:C_MANT_FP64-10];
              Q_sqrt1={{(C_MANT_FP64+1){1'b0}},Qcnt_two_2[4:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b000011:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-11:C_MANT_FP64-12];
              Q_sqrt0={{(C_MANT_FP64){1'b0}},Qcnt_two_3[6:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-13:C_MANT_FP64-14];
              Q_sqrt1={{(C_MANT_FP64-1){1'b0}},Qcnt_two_3[6:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b000100:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-15:C_MANT_FP64-16];
              Q_sqrt0={{(C_MANT_FP64-2){1'b0}},Qcnt_two_4[8:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-17:C_MANT_FP64-18];
              Q_sqrt1={{(C_MANT_FP64-3){1'b0}},Qcnt_two_4[8:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

            6'b000101:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-19:C_MANT_FP64-20];
              Q_sqrt0={{(C_MANT_FP64-4){1'b0}},Qcnt_two_5[10:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-21:C_MANT_FP64-22];
              Q_sqrt1={{(C_MANT_FP64-5){1'b0}},Qcnt_two_5[10:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b000110:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-23:C_MANT_FP64-24];
              Q_sqrt0={{(C_MANT_FP64-6){1'b0}},Qcnt_two_6[12:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-25:C_MANT_FP64-26];
              Q_sqrt1={{(C_MANT_FP64-7){1'b0}},Qcnt_two_6[12:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b000111:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-27:C_MANT_FP64-28];
              Q_sqrt0={{(C_MANT_FP64-8){1'b0}},Qcnt_two_7[14:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-29:C_MANT_FP64-30];
              Q_sqrt1={{(C_MANT_FP64-9){1'b0}},Qcnt_two_7[14:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b001000:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-31:C_MANT_FP64-32];
              Q_sqrt0={{(C_MANT_FP64-10){1'b0}},Qcnt_two_8[16:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-33:C_MANT_FP64-34];
              Q_sqrt1={{(C_MANT_FP64-11){1'b0}},Qcnt_two_8[16:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b001001:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-35:C_MANT_FP64-36];
              Q_sqrt0={{(C_MANT_FP64-12){1'b0}},Qcnt_two_9[18:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-37:C_MANT_FP64-38];
              Q_sqrt1={{(C_MANT_FP64-13){1'b0}},Qcnt_two_9[18:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b001010:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-39:C_MANT_FP64-40];
              Q_sqrt0={{(C_MANT_FP64-14){1'b0}},Qcnt_two_10[20:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-41:C_MANT_FP64-42];
              Q_sqrt1={{(C_MANT_FP64-15){1'b0}},Qcnt_two_10[20:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b001011:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-43:C_MANT_FP64-44];
              Q_sqrt0={{(C_MANT_FP64-16){1'b0}},Qcnt_two_11[22:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-45:C_MANT_FP64-46];
              Q_sqrt1={{(C_MANT_FP64-17){1'b0}},Qcnt_two_11[22:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b001100:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-47:C_MANT_FP64-48];
              Q_sqrt0={{(C_MANT_FP64-18){1'b0}},Qcnt_two_12[24:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-49:C_MANT_FP64-50];
              Q_sqrt1={{(C_MANT_FP64-19){1'b0}},Qcnt_two_12[24:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b001101:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-51:C_MANT_FP64-52];
              Q_sqrt0={{(C_MANT_FP64-20){1'b0}},Qcnt_two_13[26:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-21){1'b0}},Qcnt_two_13[26:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b001110:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-22){1'b0}},Qcnt_two_14[28:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-23){1'b0}},Qcnt_two_14[28:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b001111:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-24){1'b0}},Qcnt_two_15[30:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-25){1'b0}},Qcnt_two_15[30:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b010000:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-26){1'b0}},Qcnt_two_16[32:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-27){1'b0}},Qcnt_two_16[32:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b010001:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-28){1'b0}},Qcnt_two_17[34:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-29){1'b0}},Qcnt_two_17[34:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b010010:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-30){1'b0}},Qcnt_two_18[36:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-31){1'b0}},Qcnt_two_18[36:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b010011:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-32){1'b0}},Qcnt_two_19[38:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-33){1'b0}},Qcnt_two_19[38:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b010100:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-34){1'b0}},Qcnt_two_20[40:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-35){1'b0}},Qcnt_two_20[40:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b010101:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-36){1'b0}},Qcnt_two_21[42:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-37){1'b0}},Qcnt_two_21[42:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b010110:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-38){1'b0}},Qcnt_two_22[44:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-39){1'b0}},Qcnt_two_22[44:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b010111:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-40){1'b0}},Qcnt_two_23[46:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-41){1'b0}},Qcnt_two_23[46:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b011000:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-42){1'b0}},Qcnt_two_24[48:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-43){1'b0}},Qcnt_two_24[48:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b011001:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-44){1'b0}},Qcnt_two_25[50:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-45){1'b0}},Qcnt_two_25[50:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b011010:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-46){1'b0}},Qcnt_two_26[52:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-47){1'b0}},Qcnt_two_26[52:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b011011:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-48){1'b0}},Qcnt_two_27[54:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-49){1'b0}},Qcnt_two_27[54:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          6'b011100:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-50){1'b0}},Qcnt_two_28[56:1]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-51){1'b0}},Qcnt_two_28[56:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

          default:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64+1:C_MANT_FP64];
              Q_sqrt0={{(C_MANT_FP64+5){1'b0}},Qcnt_two_0[1]};
              Sqrt_Q0=Q_sqrt_com_0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-1:C_MANT_FP64-2];
              Q_sqrt1={{(C_MANT_FP64+4){1'b0}},Qcnt_two_0[1:0]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
            end

        endcase
      end

   /////////////////////////////////////////////////////////////////////////////
   // Operands for square root when Iteration_unit_num_S = 2'b01, end       //
   /////////////////////////////////////////////////////////////////////////////


    2'b10:
      begin
   /////////////////////////////////////////////////////////////////////////////
   // Operands for square root when Iteration_unit_num_S = 2'b10, start       //
   /////////////////////////////////////////////////////////////////////////////

        case(Crtl_cnt_S)
          6'b000000:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64+1:C_MANT_FP64];
              Q_sqrt0={{(C_MANT_FP64+5){1'b0}},Qcnt_three_0[2]};
              Sqrt_Q0=Q_sqrt_com_0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-1:C_MANT_FP64-2];
              Q_sqrt1={{(C_MANT_FP64+4){1'b0}},Qcnt_three_0[2:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-3:C_MANT_FP64-4];
              Q_sqrt2={{(C_MANT_FP64+3){1'b0}},Qcnt_three_0[2:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b000001:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-5:C_MANT_FP64-6];
              Q_sqrt0={{(C_MANT_FP64+2){1'b0}},Qcnt_three_1[4:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-7:C_MANT_FP64-8];
              Q_sqrt1={{(C_MANT_FP64+1){1'b0}},Qcnt_three_1[4:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-9:C_MANT_FP64-10];
              Q_sqrt2={{(C_MANT_FP64){1'b0}},Qcnt_three_1[4:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b000010:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-11:C_MANT_FP64-12];
              Q_sqrt0={{(C_MANT_FP64-1){1'b0}},Qcnt_three_2[7:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-13:C_MANT_FP64-14];
              Q_sqrt1={{(C_MANT_FP64-2){1'b0}},Qcnt_three_2[7:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-15:C_MANT_FP64-16];
              Q_sqrt2={{(C_MANT_FP64-3){1'b0}},Qcnt_three_2[7:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b000011:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-17:C_MANT_FP64-18];
              Q_sqrt0={{(C_MANT_FP64-4){1'b0}},Qcnt_three_3[10:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-19:C_MANT_FP64-20];
              Q_sqrt1={{(C_MANT_FP64-5){1'b0}},Qcnt_three_3[10:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-21:C_MANT_FP64-22];
              Q_sqrt2={{(C_MANT_FP64-6){1'b0}},Qcnt_three_3[10:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b000100:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-23:C_MANT_FP64-24];
              Q_sqrt0={{(C_MANT_FP64-7){1'b0}},Qcnt_three_4[13:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-25:C_MANT_FP64-26];
              Q_sqrt1={{(C_MANT_FP64-8){1'b0}},Qcnt_three_4[13:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-27:C_MANT_FP64-28];
              Q_sqrt2={{(C_MANT_FP64-9){1'b0}},Qcnt_three_4[13:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b000101:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-29:C_MANT_FP64-30];
              Q_sqrt0={{(C_MANT_FP64-10){1'b0}},Qcnt_three_5[16:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-31:C_MANT_FP64-32];
              Q_sqrt1={{(C_MANT_FP64-11){1'b0}},Qcnt_three_5[16:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-33:C_MANT_FP64-34];
              Q_sqrt2={{(C_MANT_FP64-12){1'b0}},Qcnt_three_5[16:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b000110:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-35:C_MANT_FP64-36];
              Q_sqrt0={{(C_MANT_FP64-13){1'b0}},Qcnt_three_6[19:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-37:C_MANT_FP64-38];
              Q_sqrt1={{(C_MANT_FP64-14){1'b0}},Qcnt_three_6[19:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-39:C_MANT_FP64-40];
              Q_sqrt2={{(C_MANT_FP64-15){1'b0}},Qcnt_three_6[19:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b000111:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-41:C_MANT_FP64-42];
              Q_sqrt0={{(C_MANT_FP64-16){1'b0}},Qcnt_three_7[22:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-43:C_MANT_FP64-44];
              Q_sqrt1={{(C_MANT_FP64-17){1'b0}},Qcnt_three_7[22:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-45:C_MANT_FP64-46];
              Q_sqrt2={{(C_MANT_FP64-18){1'b0}},Qcnt_three_7[22:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b001000:
            begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-47:C_MANT_FP64-48];
              Q_sqrt0={{(C_MANT_FP64-19){1'b0}},Qcnt_three_8[25:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-49:C_MANT_FP64-50];
              Q_sqrt1={{(C_MANT_FP64-20){1'b0}},Qcnt_three_8[25:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-51:C_MANT_FP64-52];
              Q_sqrt2={{(C_MANT_FP64-21){1'b0}},Qcnt_three_8[25:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b001001:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-22){1'b0}},Qcnt_three_9[28:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-23){1'b0}},Qcnt_three_9[28:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=2'b00;
              Q_sqrt2={{(C_MANT_FP64-24){1'b0}},Qcnt_three_9[28:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b001010:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-25){1'b0}},Qcnt_three_10[31:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-26){1'b0}},Qcnt_three_10[31:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=2'b00;
              Q_sqrt2={{(C_MANT_FP64-27){1'b0}},Qcnt_three_10[31:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b001011:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-28){1'b0}},Qcnt_three_11[34:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-29){1'b0}},Qcnt_three_11[34:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=2'b00;
              Q_sqrt2={{(C_MANT_FP64-30){1'b0}},Qcnt_three_11[34:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b001100:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-31){1'b0}},Qcnt_three_12[37:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-32){1'b0}},Qcnt_three_12[37:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=2'b00;
              Q_sqrt2={{(C_MANT_FP64-33){1'b0}},Qcnt_three_12[37:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b001101:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-34){1'b0}},Qcnt_three_13[40:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-35){1'b0}},Qcnt_three_13[40:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=2'b00;
              Q_sqrt2={{(C_MANT_FP64-36){1'b0}},Qcnt_three_13[40:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b001110:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-37){1'b0}},Qcnt_three_14[43:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-38){1'b0}},Qcnt_three_14[43:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=2'b00;
              Q_sqrt2={{(C_MANT_FP64-39){1'b0}},Qcnt_three_14[43:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b001111:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-40){1'b0}},Qcnt_three_15[46:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-41){1'b0}},Qcnt_three_15[46:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=2'b00;
              Q_sqrt2={{(C_MANT_FP64-42){1'b0}},Qcnt_three_15[46:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b010000:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-43){1'b0}},Qcnt_three_16[49:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-44){1'b0}},Qcnt_three_16[49:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=2'b00;
              Q_sqrt2={{(C_MANT_FP64-45){1'b0}},Qcnt_three_16[49:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b010001:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-46){1'b0}},Qcnt_three_17[52:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-47){1'b0}},Qcnt_three_17[52:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=2'b00;
              Q_sqrt2={{(C_MANT_FP64-48){1'b0}},Qcnt_three_17[52:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          6'b010010:
            begin
              Sqrt_DI[0]=2'b00;
              Q_sqrt0={{(C_MANT_FP64-49){1'b0}},Qcnt_three_18[55:2]};
              Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
              Sqrt_DI[1]=2'b00;
              Q_sqrt1={{(C_MANT_FP64-50){1'b0}},Qcnt_three_18[55:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=2'b00;
              Q_sqrt2={{(C_MANT_FP64-51){1'b0}},Qcnt_three_18[55:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end

          default :
              begin
              Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64+1:C_MANT_FP64];
              Q_sqrt0={{(C_MANT_FP64+5){1'b0}},Qcnt_three_0[2]};
              Sqrt_Q0=Q_sqrt_com_0;
              Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-1:C_MANT_FP64-2];
              Q_sqrt1={{(C_MANT_FP64+4){1'b0}},Qcnt_three_0[2:1]};
              Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
              Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-3:C_MANT_FP64-4];
              Q_sqrt2={{(C_MANT_FP64+3){1'b0}},Qcnt_three_0[2:0]};
              Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
            end
        endcase

      end
   /////////////////////////////////////////////////////////////////////////////
   // Operands for square root when Iteration_unit_num_S = 2'b10, end       //
   /////////////////////////////////////////////////////////////////////////////


    2'b11:
      begin
   /////////////////////////////////////////////////////////////////////////////
   // Operands for square root when Iteration_unit_num_S = 2'b11, start       //
   /////////////////////////////////////////////////////////////////////////////

              case(Crtl_cnt_S)

                6'b000000:
                  begin
                    Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64+1:C_MANT_FP64];
                    Q_sqrt0={{(C_MANT_FP64+5){1'b0}},Qcnt_four_0[3]};
                    Sqrt_Q0=Q_sqrt_com_0;
                    Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-1:C_MANT_FP64-2];
                    Q_sqrt1={{(C_MANT_FP64+4){1'b0}},Qcnt_four_0[3:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-3:C_MANT_FP64-4];
                    Q_sqrt2={{(C_MANT_FP64+3){1'b0}},Qcnt_four_0[3:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=Mant_D_sqrt_Norm[C_MANT_FP64-5:C_MANT_FP64-6];
                    Q_sqrt3={{(C_MANT_FP64+2){1'b0}},Qcnt_four_0[3:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b000001:
                  begin
                    Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-7:C_MANT_FP64-8];
                    Q_sqrt0={{(C_MANT_FP64+1){1'b0}},Qcnt_four_1[6:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-9:C_MANT_FP64-10];
                    Q_sqrt1={{(C_MANT_FP64){1'b0}},Qcnt_four_1[6:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-11:C_MANT_FP64-12];
                    Q_sqrt2={{(C_MANT_FP64-1){1'b0}},Qcnt_four_1[6:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=Mant_D_sqrt_Norm[C_MANT_FP64-13:C_MANT_FP64-14];
                    Q_sqrt3={{(C_MANT_FP64-2){1'b0}},Qcnt_four_1[6:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b000010:
                  begin
                    Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-15:C_MANT_FP64-16];
                    Q_sqrt0={{(C_MANT_FP64-3){1'b0}},Qcnt_four_2[10:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-17:C_MANT_FP64-18];
                    Q_sqrt1={{(C_MANT_FP64-4){1'b0}},Qcnt_four_2[10:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-19:C_MANT_FP64-20];
                    Q_sqrt2={{(C_MANT_FP64-5){1'b0}},Qcnt_four_2[10:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=Mant_D_sqrt_Norm[C_MANT_FP64-21:C_MANT_FP64-22];
                    Q_sqrt3={{(C_MANT_FP64-6){1'b0}},Qcnt_four_2[10:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b000011:
                  begin
                    Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-23:C_MANT_FP64-24];
                    Q_sqrt0={{(C_MANT_FP64-7){1'b0}},Qcnt_four_3[14:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-25:C_MANT_FP64-26];
                    Q_sqrt1={{(C_MANT_FP64-8){1'b0}},Qcnt_four_3[14:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-27:C_MANT_FP64-28];
                    Q_sqrt2={{(C_MANT_FP64-9){1'b0}},Qcnt_four_3[14:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=Mant_D_sqrt_Norm[C_MANT_FP64-29:C_MANT_FP64-30];
                    Q_sqrt3={{(C_MANT_FP64-10){1'b0}},Qcnt_four_3[14:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b000100:
                  begin
                    Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-31:C_MANT_FP64-32];
                    Q_sqrt0={{(C_MANT_FP64-11){1'b0}},Qcnt_four_4[18:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-33:C_MANT_FP64-34];
                    Q_sqrt1={{(C_MANT_FP64-12){1'b0}},Qcnt_four_4[18:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-35:C_MANT_FP64-36];
                    Q_sqrt2={{(C_MANT_FP64-13){1'b0}},Qcnt_four_4[18:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=Mant_D_sqrt_Norm[C_MANT_FP64-37:C_MANT_FP64-38];
                    Q_sqrt3={{(C_MANT_FP64-14){1'b0}},Qcnt_four_4[18:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b000101:
                  begin
                    Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-39:C_MANT_FP64-40];
                    Q_sqrt0={{(C_MANT_FP64-15){1'b0}},Qcnt_four_5[22:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-41:C_MANT_FP64-42];
                    Q_sqrt1={{(C_MANT_FP64-16){1'b0}},Qcnt_four_5[22:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-43:C_MANT_FP64-44];
                    Q_sqrt2={{(C_MANT_FP64-17){1'b0}},Qcnt_four_5[22:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=Mant_D_sqrt_Norm[C_MANT_FP64-45:C_MANT_FP64-46];
                    Q_sqrt3={{(C_MANT_FP64-18){1'b0}},Qcnt_four_5[22:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b000110:
                  begin
                    Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64-47:C_MANT_FP64-48];
                    Q_sqrt0={{(C_MANT_FP64-19){1'b0}},Qcnt_four_6[26:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-49:C_MANT_FP64-50];
                    Q_sqrt1={{(C_MANT_FP64-20){1'b0}},Qcnt_four_6[26:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-51:C_MANT_FP64-52];
                    Q_sqrt2={{(C_MANT_FP64-21){1'b0}},Qcnt_four_6[26:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=2'b00;
                    Q_sqrt3={{(C_MANT_FP64-22){1'b0}},Qcnt_four_6[26:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b000111:
                  begin
                    Sqrt_DI[0]=2'b00;
                    Q_sqrt0={{(C_MANT_FP64-23){1'b0}},Qcnt_four_7[30:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=2'b00;
                    Q_sqrt1={{(C_MANT_FP64-24){1'b0}},Qcnt_four_7[30:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=2'b00;
                    Q_sqrt2={{(C_MANT_FP64-25){1'b0}},Qcnt_four_7[30:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=2'b00;
                    Q_sqrt3={{(C_MANT_FP64-26){1'b0}},Qcnt_four_7[30:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b001000:
                  begin
                    Sqrt_DI[0]=2'b00;
                    Q_sqrt0={{(C_MANT_FP64-27){1'b0}},Qcnt_four_8[34:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=2'b00;
                    Q_sqrt1={{(C_MANT_FP64-28){1'b0}},Qcnt_four_8[34:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=2'b00;
                    Q_sqrt2={{(C_MANT_FP64-29){1'b0}},Qcnt_four_8[34:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=2'b00;
                    Q_sqrt3={{(C_MANT_FP64-30){1'b0}},Qcnt_four_8[34:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b001001:
                  begin
                    Sqrt_DI[0]=2'b00;
                    Q_sqrt0={{(C_MANT_FP64-31){1'b0}},Qcnt_four_9[38:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=2'b00;
                    Q_sqrt1={{(C_MANT_FP64-32){1'b0}},Qcnt_four_9[38:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=2'b00;
                    Q_sqrt2={{(C_MANT_FP64-33){1'b0}},Qcnt_four_9[38:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=2'b00;
                    Q_sqrt3={{(C_MANT_FP64-34){1'b0}},Qcnt_four_9[38:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b001010:
                  begin
                    Sqrt_DI[0]=2'b00;
                    Q_sqrt0={{(C_MANT_FP64-35){1'b0}},Qcnt_four_10[42:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=2'b00;
                    Q_sqrt1={{(C_MANT_FP64-36){1'b0}},Qcnt_four_10[42:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=2'b00;
                    Q_sqrt2={{(C_MANT_FP64-37){1'b0}},Qcnt_four_10[42:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=2'b00;
                    Q_sqrt3={{(C_MANT_FP64-38){1'b0}},Qcnt_four_10[42:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b001011:
                  begin
                    Sqrt_DI[0]=2'b00;
                    Q_sqrt0={{(C_MANT_FP64-39){1'b0}},Qcnt_four_11[46:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=2'b00;
                    Q_sqrt1={{(C_MANT_FP64-40){1'b0}},Qcnt_four_11[46:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=2'b00;
                    Q_sqrt2={{(C_MANT_FP64-41){1'b0}},Qcnt_four_11[46:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=2'b00;
                    Q_sqrt3={{(C_MANT_FP64-42){1'b0}},Qcnt_four_11[46:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b001100:
                  begin
                    Sqrt_DI[0]=2'b00;
                    Q_sqrt0={{(C_MANT_FP64-43){1'b0}},Qcnt_four_12[50:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=2'b00;
                    Q_sqrt1={{(C_MANT_FP64-44){1'b0}},Qcnt_four_12[50:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=2'b00;
                    Q_sqrt2={{(C_MANT_FP64-45){1'b0}},Qcnt_four_12[50:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=2'b00;
                    Q_sqrt3={{(C_MANT_FP64-46){1'b0}},Qcnt_four_12[50:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                6'b001101:
                  begin
                    Sqrt_DI[0]=2'b00;
                    Q_sqrt0={{(C_MANT_FP64-47){1'b0}},Qcnt_four_13[54:3]};
                    Sqrt_Q0=Quotient_DP[0]?Q_sqrt_com_0:Q_sqrt0;
                    Sqrt_DI[1]=2'b00;
                    Q_sqrt1={{(C_MANT_FP64-48){1'b0}},Qcnt_four_13[54:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=2'b00;
                    Q_sqrt2={{(C_MANT_FP64-49){1'b0}},Qcnt_four_13[54:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=2'b00;
                    Q_sqrt3={{(C_MANT_FP64-50){1'b0}},Qcnt_four_13[54:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end

                default:
                  begin
                    Sqrt_DI[0]=Mant_D_sqrt_Norm[C_MANT_FP64+1:C_MANT_FP64];
                    Q_sqrt0={{(C_MANT_FP64+5){1'b0}},Qcnt_four_0[3]};
                    Sqrt_Q0=Q_sqrt_com_0;
                    Sqrt_DI[1]=Mant_D_sqrt_Norm[C_MANT_FP64-1:C_MANT_FP64-2];
                    Q_sqrt1={{(C_MANT_FP64+4){1'b0}},Qcnt_four_0[3:2]};
                    Sqrt_Q1=Sqrt_quotinent_S[3]?Q_sqrt_com_1:Q_sqrt1;
                    Sqrt_DI[2]=Mant_D_sqrt_Norm[C_MANT_FP64-3:C_MANT_FP64-4];
                    Q_sqrt2={{(C_MANT_FP64+3){1'b0}},Qcnt_four_0[3:1]};
                    Sqrt_Q2=Sqrt_quotinent_S[2]?Q_sqrt_com_2:Q_sqrt2;
                    Sqrt_DI[3]=Mant_D_sqrt_Norm[C_MANT_FP64-5:C_MANT_FP64-6];
                    Q_sqrt3={{(C_MANT_FP64+2){1'b0}},Qcnt_four_0[3:0]};
                    Sqrt_Q3=Sqrt_quotinent_S[1]?Q_sqrt_com_3:Q_sqrt3;
                  end
              endcase
            end
      endcase
   /////////////////////////////////////////////////////////////////////////////
   // Operands for square root when Iteration_unit_num_S = 2'b11, end         //
   /////////////////////////////////////////////////////////////////////////////
 end



  assign Sqrt_R0= ((Sqrt_start_dly_S)?'0:{Partial_remainder_DP[C_MANT_FP64+5:0]});
  assign Sqrt_R1= {Iteration_cell_sum_AMASK_D[0][C_MANT_FP64+5],Iteration_cell_sum_AMASK_D[0][C_MANT_FP64+2:0],Sqrt_DO[0]} ;
  assign Sqrt_R2= {Iteration_cell_sum_AMASK_D[1][C_MANT_FP64+5],Iteration_cell_sum_AMASK_D[1][C_MANT_FP64+2:0],Sqrt_DO[1]};
  assign Sqrt_R3= {Iteration_cell_sum_AMASK_D[2][C_MANT_FP64+5],Iteration_cell_sum_AMASK_D[2][C_MANT_FP64+2:0],Sqrt_DO[2]};
  assign Sqrt_R4= {Iteration_cell_sum_AMASK_D[3][C_MANT_FP64+5],Iteration_cell_sum_AMASK_D[3][C_MANT_FP64+2:0],Sqrt_DO[3]};

  logic [C_MANT_FP64+5:0]                               Denominator_se_format_DB;  //

  assign Denominator_se_format_DB={Denominator_se_DB[C_MANT_FP64+1:C_MANT_FP64-C_MANT_FP16ALT],{FP16ALT_SO?FP16ALT_SO:Denominator_se_DB[C_MANT_FP64-C_MANT_FP16ALT-1]},
                                                         Denominator_se_DB[C_MANT_FP64-C_MANT_FP16ALT-2:C_MANT_FP64-C_MANT_FP16],{FP16_SO?FP16_SO:Denominator_se_DB[C_MANT_FP64-C_MANT_FP16-1]},
                                                         Denominator_se_DB[C_MANT_FP64-C_MANT_FP16-2:C_MANT_FP64-C_MANT_FP32],{FP32_SO?FP32_SO:Denominator_se_DB[C_MANT_FP64-C_MANT_FP32-1]},
                                                         Denominator_se_DB[C_MANT_FP64-C_MANT_FP32-2:C_MANT_FP64-C_MANT_FP64],FP64_SO,3'b0} ;
  //                   for           iteration cell_U0
  logic [C_MANT_FP64+5:0]                           First_iteration_cell_div_a_D,First_iteration_cell_div_b_D;
  logic                                             Sel_b_for_first_S;


  assign First_iteration_cell_div_a_D=(Div_start_dly_S)?{Numerator_se_D[C_MANT_FP64+1:C_MANT_FP64-C_MANT_FP16ALT],{FP16ALT_SO?FP16ALT_SO:Numerator_se_D[C_MANT_FP64-C_MANT_FP16ALT-1]},
                                                         Numerator_se_D[C_MANT_FP64-C_MANT_FP16ALT-2:C_MANT_FP64-C_MANT_FP16],{FP16_SO?FP16_SO:Numerator_se_D[C_MANT_FP64-C_MANT_FP16-1]},
                                                         Numerator_se_D[C_MANT_FP64-C_MANT_FP16-2:C_MANT_FP64-C_MANT_FP32],{FP32_SO?FP32_SO:Numerator_se_D[C_MANT_FP64-C_MANT_FP32-1]},
                                                         Numerator_se_D[C_MANT_FP64-C_MANT_FP32-2:C_MANT_FP64-C_MANT_FP64],FP64_SO,3'b0}
                                                        :{Partial_remainder_DP[C_MANT_FP64+4:C_MANT_FP64-C_MANT_FP16ALT+3],{FP16ALT_SO?Quotient_DP[0]:Partial_remainder_DP[C_MANT_FP64-C_MANT_FP16ALT+2]},
                                                         Partial_remainder_DP[C_MANT_FP64-C_MANT_FP16ALT+1:C_MANT_FP64-C_MANT_FP16+3],{FP16_SO?Quotient_DP[0]:Partial_remainder_DP[C_MANT_FP64-C_MANT_FP16+2]},
                                                         Partial_remainder_DP[C_MANT_FP64-C_MANT_FP16+1:C_MANT_FP64-C_MANT_FP32+3],{FP32_SO?Quotient_DP[0]:Partial_remainder_DP[C_MANT_FP64-C_MANT_FP32+2]},
                                                         Partial_remainder_DP[C_MANT_FP64-C_MANT_FP32+1:C_MANT_FP64-C_MANT_FP64+3],FP64_SO&&Quotient_DP[0],3'b0};
  assign Sel_b_for_first_S=(Div_start_dly_S)?1:Quotient_DP[0];
  assign First_iteration_cell_div_b_D=Sel_b_for_first_S?Denominator_se_format_DB:{Denominator_se_D,4'b0};
  assign Iteration_cell_a_BMASK_D[0]=Sqrt_enable_SO?Sqrt_R0:{First_iteration_cell_div_a_D};
  assign Iteration_cell_b_BMASK_D[0]=Sqrt_enable_SO?Sqrt_Q0:{First_iteration_cell_div_b_D};



  //                   for           iteration cell_U1
  logic [C_MANT_FP64+5:0]                          Sec_iteration_cell_div_a_D,Sec_iteration_cell_div_b_D;
  logic                                            Sel_b_for_sec_S;
  generate
    if(|Iteration_unit_num_S)
      begin
        assign Sel_b_for_sec_S=~Iteration_cell_sum_AMASK_D[0][C_MANT_FP64+5];
        assign Sec_iteration_cell_div_a_D={Iteration_cell_sum_AMASK_D[0][C_MANT_FP64+4:C_MANT_FP64-C_MANT_FP16ALT+3],{FP16ALT_SO?Sel_b_for_sec_S:Iteration_cell_sum_AMASK_D[0][C_MANT_FP64-C_MANT_FP16ALT+2]},
                                           Iteration_cell_sum_AMASK_D[0][C_MANT_FP64-C_MANT_FP16ALT+1:C_MANT_FP64-C_MANT_FP16+3],{FP16_SO?Sel_b_for_sec_S:Iteration_cell_sum_AMASK_D[0][C_MANT_FP64-C_MANT_FP16+2]},
                                           Iteration_cell_sum_AMASK_D[0][C_MANT_FP64-C_MANT_FP16+1:C_MANT_FP64-C_MANT_FP32+3],{FP32_SO?Sel_b_for_sec_S:Iteration_cell_sum_AMASK_D[0][C_MANT_FP64-C_MANT_FP32+2]},
                                           Iteration_cell_sum_AMASK_D[0][C_MANT_FP64-C_MANT_FP32+1:C_MANT_FP64-C_MANT_FP64+3],FP64_SO&&Sel_b_for_sec_S,3'b0};
        assign Sec_iteration_cell_div_b_D=Sel_b_for_sec_S?Denominator_se_format_DB:{Denominator_se_D,4'b0};
        assign Iteration_cell_a_BMASK_D[1]=Sqrt_enable_SO?Sqrt_R1:{Sec_iteration_cell_div_a_D};
        assign Iteration_cell_b_BMASK_D[1]=Sqrt_enable_SO?Sqrt_Q1:{Sec_iteration_cell_div_b_D};
      end
    endgenerate

  //                   for           iteration cell_U2
  logic [C_MANT_FP64+5:0]                          Thi_iteration_cell_div_a_D,Thi_iteration_cell_div_b_D;
  logic                                            Sel_b_for_thi_S;
  generate
    if((Iteration_unit_num_S==2'b10) | (Iteration_unit_num_S==2'b11))
      begin
        assign Sel_b_for_thi_S=~Iteration_cell_sum_AMASK_D[1][C_MANT_FP64+5];
        assign Thi_iteration_cell_div_a_D={Iteration_cell_sum_AMASK_D[1][C_MANT_FP64+4:C_MANT_FP64-C_MANT_FP16ALT+3],{FP16ALT_SO?Sel_b_for_thi_S:Iteration_cell_sum_AMASK_D[1][C_MANT_FP64-C_MANT_FP16ALT+2]},
                                           Iteration_cell_sum_AMASK_D[1][C_MANT_FP64-C_MANT_FP16ALT+1:C_MANT_FP64-C_MANT_FP16+3],{FP16_SO?Sel_b_for_thi_S:Iteration_cell_sum_AMASK_D[1][C_MANT_FP64-C_MANT_FP16+2]},
                                           Iteration_cell_sum_AMASK_D[1][C_MANT_FP64-C_MANT_FP16+1:C_MANT_FP64-C_MANT_FP32+3],{FP32_SO?Sel_b_for_thi_S:Iteration_cell_sum_AMASK_D[1][C_MANT_FP64-C_MANT_FP32+2]},
                                           Iteration_cell_sum_AMASK_D[1][C_MANT_FP64-C_MANT_FP32+1:C_MANT_FP64-C_MANT_FP64+3],FP64_SO&&Sel_b_for_thi_S,3'b0};
        assign Thi_iteration_cell_div_b_D=Sel_b_for_thi_S?Denominator_se_format_DB:{Denominator_se_D,4'b0};
        assign Iteration_cell_a_BMASK_D[2]=Sqrt_enable_SO?Sqrt_R2:{Thi_iteration_cell_div_a_D};
        assign Iteration_cell_b_BMASK_D[2]=Sqrt_enable_SO?Sqrt_Q2:{Thi_iteration_cell_div_b_D};
      end
  endgenerate

  //                   for           iteration cell_U3
  logic [C_MANT_FP64+5:0]                          Fou_iteration_cell_div_a_D,Fou_iteration_cell_div_b_D;
  logic                                            Sel_b_for_fou_S;

  generate
    if(Iteration_unit_num_S==2'b11)
      begin
        assign Sel_b_for_fou_S=~Iteration_cell_sum_AMASK_D[2][C_MANT_FP64+5];
        assign Fou_iteration_cell_div_a_D={Iteration_cell_sum_AMASK_D[2][C_MANT_FP64+4:C_MANT_FP64-C_MANT_FP16ALT+3],{FP16ALT_SO?Sel_b_for_fou_S:Iteration_cell_sum_AMASK_D[2][C_MANT_FP64-C_MANT_FP16ALT+2]},
                                           Iteration_cell_sum_AMASK_D[2][C_MANT_FP64-C_MANT_FP16ALT+1:C_MANT_FP64-C_MANT_FP16+3],{FP16_SO?Sel_b_for_fou_S:Iteration_cell_sum_AMASK_D[2][C_MANT_FP64-C_MANT_FP16+2]},
                                           Iteration_cell_sum_AMASK_D[2][C_MANT_FP64-C_MANT_FP16+1:C_MANT_FP64-C_MANT_FP32+3],{FP32_SO?Sel_b_for_fou_S:Iteration_cell_sum_AMASK_D[2][C_MANT_FP64-C_MANT_FP32+2]},
                                           Iteration_cell_sum_AMASK_D[2][C_MANT_FP64-C_MANT_FP32+1:C_MANT_FP64-C_MANT_FP64+3],FP64_SO&&Sel_b_for_fou_S,3'b0};
        assign Fou_iteration_cell_div_b_D=Sel_b_for_fou_S?Denominator_se_format_DB:{Denominator_se_D,4'b0};
        assign Iteration_cell_a_BMASK_D[3]=Sqrt_enable_SO?Sqrt_R3:{Fou_iteration_cell_div_a_D};
        assign Iteration_cell_b_BMASK_D[3]=Sqrt_enable_SO?Sqrt_Q3:{Fou_iteration_cell_div_b_D};
      end
  endgenerate

   /////////////////////////////////////////////////////////////////////////////
   // Masking Contrl                                                          //
   /////////////////////////////////////////////////////////////////////////////


  logic [C_MANT_FP64+1+4:0]                          Mask_bits_ctl_S;  //For extension

  assign Mask_bits_ctl_S =58'h3ff_ffff_ffff_ffff;   //It is not needed. The corresponding process is handled the above codes

   /////////////////////////////////////////////////////////////////////////////
   // Iteration Instances  with masking control                               //
   /////////////////////////////////////////////////////////////////////////////


  logic                                             Div_enable_SI   [3:0];
  logic                                             Div_start_dly_SI   [3:0];
  logic                                             Sqrt_enable_SI   [3:0];
  generate
    genvar i,j;
      for (i=0; i <= Iteration_unit_num_S ; i++)
        begin
          for (j = 0; j <= C_MANT_FP64+5; j++) begin
              assign Iteration_cell_a_D[i][j] = Mask_bits_ctl_S[j] && Iteration_cell_a_BMASK_D[i][j];
              assign Iteration_cell_b_D[i][j] = Mask_bits_ctl_S[j] && Iteration_cell_b_BMASK_D[i][j];
              assign Iteration_cell_sum_AMASK_D[i][j] = Mask_bits_ctl_S[j] && Iteration_cell_sum_D[i][j];
          end

          assign  Div_enable_SI[i] = Div_enable_SO;
          assign  Div_start_dly_SI[i] = Div_start_dly_S;
          assign  Sqrt_enable_SI[i] = Sqrt_enable_SO;
          iteration_div_sqrt_mvp #(C_MANT_FP64+6) iteration_div_sqrt
          (
          .A_DI                                    (Iteration_cell_a_D[i]            ),
          .B_DI                                    (Iteration_cell_b_D[i]            ),
          .Div_enable_SI                           (Div_enable_SI[i]                 ),
          .Div_start_dly_SI                        (Div_start_dly_SI[i]              ),
          .Sqrt_enable_SI                          (Sqrt_enable_SI[i]                ),
          .D_DI                                    (Sqrt_DI[i]                       ),
          .D_DO                                    (Sqrt_DO[i]                       ),
          .Sum_DO                                  (Iteration_cell_sum_D[i]          ),
          .Carry_out_DO                            (Iteration_cell_carry_D[i]        )
         );

        end

  endgenerate



  always_comb
    begin
      case (Iteration_unit_num_S)
        2'b00:
          begin
            if(Fsm_enable_S)
               Partial_remainder_DN = Sqrt_enable_SO?Sqrt_R1:Iteration_cell_sum_AMASK_D[0];
            else
               Partial_remainder_DN = Partial_remainder_DP;
          end
        2'b01:
          begin
            if(Fsm_enable_S)
               Partial_remainder_DN = Sqrt_enable_SO?Sqrt_R2:Iteration_cell_sum_AMASK_D[1];
            else
               Partial_remainder_DN = Partial_remainder_DP;
          end
        2'b10:
          begin
            if(Fsm_enable_S)
               Partial_remainder_DN = Sqrt_enable_SO?Sqrt_R3:Iteration_cell_sum_AMASK_D[2];
            else
               Partial_remainder_DN = Partial_remainder_DP;
          end
        2'b11:
          begin
            if(Fsm_enable_S)
               Partial_remainder_DN = Sqrt_enable_SO?Sqrt_R4:Iteration_cell_sum_AMASK_D[3];
            else
               Partial_remainder_DN = Partial_remainder_DP;
          end
        endcase
     end



   always_ff @(posedge Clk_CI, negedge Rst_RBI)   // partial_remainder
     begin
        if(~Rst_RBI)
          begin
             Partial_remainder_DP <= '0;
          end
        else
          begin
             Partial_remainder_DP <= Partial_remainder_DN;
          end
    end

   logic [C_MANT_FP64+4:0] Quotient_DN;

  always_comb                                                      // Can choosen the different carry-outs based on different operations
    begin
      case (Iteration_unit_num_S)
        2'b00:
          begin
            if(Fsm_enable_S)
               Quotient_DN= Sqrt_enable_SO ? {Quotient_DP[C_MANT_FP64+3:0],Sqrt_quotinent_S[3]} :{Quotient_DP[C_MANT_FP64+3:0],Iteration_cell_carry_D[0]};
            else
               Quotient_DN= Quotient_DP;
          end
        2'b01:
          begin
            if(Fsm_enable_S)
               Quotient_DN= Sqrt_enable_SO ? {Quotient_DP[C_MANT_FP64+2:0],Sqrt_quotinent_S[3:2]} :{Quotient_DP[C_MANT_FP64+2:0],Iteration_cell_carry_D[0],Iteration_cell_carry_D[1]};
            else
               Quotient_DN= Quotient_DP;
          end
        2'b10:
          begin
            if(Fsm_enable_S)
               Quotient_DN= Sqrt_enable_SO ? {Quotient_DP[C_MANT_FP64+1:0],Sqrt_quotinent_S[3:1]} : {Quotient_DP[C_MANT_FP64+1:0],Iteration_cell_carry_D[0],Iteration_cell_carry_D[1],Iteration_cell_carry_D[2]};
            else
               Quotient_DN= Quotient_DP;
          end
        2'b11:
          begin
            if(Fsm_enable_S)
               Quotient_DN= Sqrt_enable_SO ? {Quotient_DP[C_MANT_FP64:0],Sqrt_quotinent_S } : {Quotient_DP[C_MANT_FP64:0],Iteration_cell_carry_D[0],Iteration_cell_carry_D[1],Iteration_cell_carry_D[2],Iteration_cell_carry_D[3]};
            else
               Quotient_DN= Quotient_DP;
          end
        endcase
     end

   always_ff @(posedge Clk_CI, negedge Rst_RBI)   // Quotient
     begin
        if(~Rst_RBI)
          begin
          Quotient_DP <= '0;
          end
        else
          Quotient_DP <= Quotient_DN;
    end


   /////////////////////////////////////////////////////////////////////////////
   // Precision Control for outputs                                          //
   /////////////////////////////////////////////////////////////////////////////


//////////////////////one iteration unit, start///////////////////////////////////////
   generate
     if(Iteration_unit_num_S==2'b00)
       begin
        always_comb
          begin
            case (Format_sel_S)
              2'b00:
                begin
                  case (Precision_ctl_S)
                    6'h00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32+4:0],{(C_MANT_FP64-C_MANT_FP32){1'b0}}}; //+4
                      end
                    6'h17:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32:0],{(C_MANT_FP64-C_MANT_FP32+4){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h16:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-1:0],{(C_MANT_FP64-C_MANT_FP32+4+1){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h15:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-2:0],{(C_MANT_FP64-C_MANT_FP32+4+2){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h14:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-3:0],{(C_MANT_FP64-C_MANT_FP32+4+3){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h13:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-4:0],{(C_MANT_FP64-C_MANT_FP32+4+4){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h12:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-5:0],{(C_MANT_FP64-C_MANT_FP32+4+5){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h11:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-6:0],{(C_MANT_FP64-C_MANT_FP32+4+6){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h10:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-7:0],{(C_MANT_FP64-C_MANT_FP32+4+7){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0f:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-8:0],{(C_MANT_FP64-C_MANT_FP32+4+8){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0e:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-9:0],{(C_MANT_FP64-C_MANT_FP32+4+9){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0d:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-10:0],{(C_MANT_FP64-C_MANT_FP32+4+10){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-11:0],{(C_MANT_FP64-C_MANT_FP32+4+11){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0b:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-12:0],{(C_MANT_FP64-C_MANT_FP32+4+12){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0a:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-13:0],{(C_MANT_FP64-C_MANT_FP32+4+13){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h09:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-14:0],{(C_MANT_FP64-C_MANT_FP32+4+14){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h08:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-15:0],{(C_MANT_FP64-C_MANT_FP32+4+15){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h07:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-16:0],{(C_MANT_FP64-C_MANT_FP32+4+16){1'b0}}}; //Precision_ctl_S+1
                      end
                    default :
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32+4:0],{(C_MANT_FP64-C_MANT_FP32){1'b0}}}; //+4
                      end
                  endcase
                end

              2'b01:
                begin
                  case (Precision_ctl_S)
                    6'h00:
                      begin
                        Mant_result_prenorm_DO = Quotient_DP[C_MANT_FP64+4:0]; //+4
                      end
                    6'h34:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64:0],{(4){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h33:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-1:0],{(4+1){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h32:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-2:0],{(4+2){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h31:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-3:0],{(4+3){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h30:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-4:0],{(4+4){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h2f:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-5:0],{(4+5){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h2e:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-6:0],{(4+6){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h2d:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-7:0],{(4+7){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h2c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-8:0],{(4+8){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h2b:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-9:0],{(4+9){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h2a:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-10:0],{(4+10){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h29:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-11:0],{(4+11){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h28:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-12:0],{(4+12){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h27:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-13:0],{(4+13){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h26:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-14:0],{(4+14){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h25:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-15:0],{(4+15){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h24:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-16:0],{(4+16){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h23:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-17:0],{(4+17){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h22:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-18:0],{(4+18){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h21:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-19:0],{(4+19){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h20:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-20:0],{(4+20){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h1f:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-21:0],{(4+21){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h1e:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-22:0],{(4+22){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h1d:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-23:0],{(4+23){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h1c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-24:0],{(4+24){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h1b:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-25:0],{(4+25){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h1a:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-26:0],{(4+26){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h19:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-27:0],{(4+27){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h18:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-28:0],{(4+28){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h17:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-29:0],{(4+29){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h16:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-30:0],{(4+30){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h15:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-31:0],{(4+31){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h14:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-32:0],{(4+32){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h13:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-33:0],{(4+33){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h12:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-34:0],{(4+34){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h11:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-35:0],{(4+35){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h10:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-36:0],{(4+36){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0f:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-37:0],{(4+37){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0e:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-38:0],{(4+38){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0d:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-39:0],{(4+39){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-40:0],{(4+40){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0b:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-41:0],{(4+41){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0a:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-42:0],{(4+42){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h09:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-43:0],{(4+43){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h08:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-44:0],{(4+44){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h07:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-45:0],{(4+45){1'b0}}}; //Precision_ctl_S+1
                      end
                    default:
                      begin
                        Mant_result_prenorm_DO = Quotient_DP[C_MANT_FP64+4:0]; //+4
                      end
                  endcase
                end

              2'b10:
                begin
                  case (Precision_ctl_S)
                    6'b00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16+4:0],{(C_MANT_FP64-C_MANT_FP16){1'b0}}}; //+4
                      end
                    6'h0a:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16:0],{(C_MANT_FP64-C_MANT_FP16+4){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h09:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16-1:0],{(C_MANT_FP64-C_MANT_FP16+4+1){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h08:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16-2:0],{(C_MANT_FP64-C_MANT_FP16+4+2){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h07:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16-3:0],{(C_MANT_FP64-C_MANT_FP16+4+3){1'b0}}}; //Precision_ctl_S+1
                      end
                    default :
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16+4:0],{(C_MANT_FP64-C_MANT_FP16){1'b0}}}; //+4
                      end
                  endcase
                end

              2'b11:
                begin

                  case (Precision_ctl_S)
                    6'b00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT+4:0],{(C_MANT_FP64-C_MANT_FP16ALT){1'b0}}}; //+4
                      end
                    6'h07:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT:0],{(C_MANT_FP64-C_MANT_FP16ALT+4){1'b0}}}; //Precision_ctl_S+1
                      end
                    default :
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT+4:0],{(C_MANT_FP64-C_MANT_FP16ALT){1'b0}}}; //+4
                      end
                  endcase
                end
            endcase
          end
        end
      endgenerate
//////////////////////one iteration unit, end//////////////////////////////////////////

//////////////////////two iteration units, start///////////////////////////////////////
   generate
     if(Iteration_unit_num_S==2'b01)
       begin
        always_comb
          begin
            case (Format_sel_S)
              2'b00:
                begin
                  case (Precision_ctl_S)
                    6'h00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32+4:0],{(C_MANT_FP64-C_MANT_FP32){1'b0}}}; //+4
                      end
                    6'h17,6'h16:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32:0],{(C_MANT_FP64-C_MANT_FP32+4){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h15,6'h14:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-2:0],{(C_MANT_FP64-C_MANT_FP32+4+2){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h13,6'h12:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-4:0],{(C_MANT_FP64-C_MANT_FP32+4+4){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h11,6'h10:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-6:0],{(C_MANT_FP64-C_MANT_FP32+4+6){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0f,6'h0e:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-8:0],{(C_MANT_FP64-C_MANT_FP32+4+8){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0d,6'h0c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-10:0],{(C_MANT_FP64-C_MANT_FP32+4+10){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0b,6'h0a:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-12:0],{(C_MANT_FP64-C_MANT_FP32+4+12){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h09,6'h08:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-14:0],{(C_MANT_FP64-C_MANT_FP32+4+14){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h07,6'h06:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-16:0],{(C_MANT_FP64-C_MANT_FP32+4+16){1'b0}}}; //Precision_ctl_S+1
                      end
                    default:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32+4:0],{(C_MANT_FP64-C_MANT_FP32){1'b0}}}; //+4
                      end
                  endcase
                end
              2'b01:
                begin
                  case (Precision_ctl_S)
                    6'h00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64+3:0],1'b0}; //+3
                      end
                    6'h34:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64+1:1],{(4){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h33,6'h32:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-1:0],{(4+1){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h31,6'h30:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-3:0],{(4+3){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h2f,6'h2e:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-5:0],{(4+5){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h2d,6'h2c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-7:0],{(4+7){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h2b,6'h2a:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-9:0],{(4+9){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h29,6'h28:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-11:0],{(4+11){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h27,6'h26:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-13:0],{(4+13){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h25,6'h24:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-15:0],{(4+15){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h23,6'h22:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-17:0],{(4+17){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h21,6'h20:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-19:0],{(4+19){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h1f,6'h1e:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-21:0],{(4+21){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h1d,6'h1c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-23:0],{(4+23){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h1b,6'h1a:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-25:0],{(4+25){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h19,6'h18:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-27:0],{(4+27){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h17,6'h16:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-29:0],{(4+29){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h15,6'h14:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-31:0],{(4+31){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h13,6'h12:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-33:0],{(4+33){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h11,6'h10:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-35:0],{(4+35){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h0f,6'h0e:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-37:0],{(4+37){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h0d,6'h0c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-39:0],{(4+39){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h0b,6'h0a:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-41:0],{(4+41){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h09,6'h08:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-43:0],{(4+43){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h07:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-45:0],{(4+45){1'b0}} }; //Precision_ctl_S+1
                      end
                    default:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64+3:0],1'b0}; //+3
                      end
                  endcase
                end

              2'b10:
                begin
                  case (Precision_ctl_S)
                    6'b00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16+3:0],{(C_MANT_FP64-C_MANT_FP16+1){1'b0}} }; //+3
                      end
                    6'h0a:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16+1:1],{(C_MANT_FP64-C_MANT_FP16+4){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h09,6'h08:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16-1:0],{(C_MANT_FP64-C_MANT_FP16+4+1){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h07:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16-3:0],{(C_MANT_FP64-C_MANT_FP16+4+3){1'b0}} }; //Precision_ctl_S+1
                      end
                    default :
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16+4:0],{(C_MANT_FP64-C_MANT_FP16){1'b0}} }; //+4
                      end
                  endcase
                end

              2'b11:
                begin

                  case (Precision_ctl_S)
                    6'b00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT+4:0],{(C_MANT_FP64-C_MANT_FP16ALT){1'b0}} }; //+4
                      end
                    6'h07:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT:0],{(C_MANT_FP64-C_MANT_FP16ALT+4){1'b0}} }; //Precision_ctl_S+1
                      end
                    default :
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT+4:0],{(C_MANT_FP64-C_MANT_FP16ALT){1'b0}} }; //+4
                      end
                  endcase
                end
            endcase
          end
       end
     endgenerate
//////////////////////two iteration units, end//////////////////////////////////////////

//////////////////////three iteration units, start///////////////////////////////////////
   generate
     if(Iteration_unit_num_S==2'b10)
       begin
        always_comb
          begin
            case (Format_sel_S)
              2'b00:
                begin
                  case (Precision_ctl_S)
                    6'h00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32+3:0],{(C_MANT_FP64-C_MANT_FP32+1){1'b0}}}; //+3
                      end
                    6'h17,6'h16,6'h15:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32:0],{(C_MANT_FP64-C_MANT_FP32+4){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h14,6'h13,6'h12:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-3:0],{(C_MANT_FP64-C_MANT_FP32+4+3){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h11,6'h10,6'h0f:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-6:0],{(C_MANT_FP64-C_MANT_FP32+4+6){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0e,6'h0d,6'h0c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-9:0],{(C_MANT_FP64-C_MANT_FP32+4+9){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0b,6'h0a,6'h09:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-12:0],{(C_MANT_FP64-C_MANT_FP32+4+12){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h08,6'h07,6'h06:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-15:0],{(C_MANT_FP64-C_MANT_FP32+4+15){1'b0}}}; //Precision_ctl_S+1
                      end
                    default:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32+3:0],{(C_MANT_FP64-C_MANT_FP32+1){1'b0}}}; //+3
                      end
                  endcase
                end

              2'b01:
                begin
                  case (Precision_ctl_S)
                    6'h00:
                      begin
                        Mant_result_prenorm_DO = Quotient_DP[C_MANT_FP64+4:0]; //+4
                      end
                    6'h34,6'h33:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64+1:1],{(4){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h32,6'h31,6'h30:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-2:0],{(4+2){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h2f,6'h2e,6'h2d:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-5:0],{(4+5){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h2c,6'h2b,6'h2a:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-8:0],{(4+8){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h29,6'h28,6'h27:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-11:0],{(4+11){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h26,6'h25,6'h24:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-14:0],{(4+14){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h23,6'h22,6'h21:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-17:0],{(4+17){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h20,6'h1f,6'h1e:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-20:0],{(4+20){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h1d,6'h1c,6'h1b:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-23:0],{(4+23){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h1a,6'h19,6'h18:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-26:0],{(4+26){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h17,6'h16,6'h15:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-29:0],{(4+29){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h14,6'h13,6'h12:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-32:0],{(4+32){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h11,6'h10,6'h0f:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-35:0],{(4+35){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h0e,6'h0d,6'h0c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-38:0],{(4+38){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h0b,6'h0a,6'h09:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-41:0],{(4+41){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h08,6'h07,6'h06:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-44:0],{(4+44){1'b0}} }; //Precision_ctl_S+1
                      end
                    default:
                      begin
                        Mant_result_prenorm_DO = Quotient_DP[C_MANT_FP64+4:0]; //+4
                      end
                  endcase
                end

              2'b10:
                begin
                  case (Precision_ctl_S)
                    6'b00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16+4:0],{(C_MANT_FP64-C_MANT_FP16){1'b0}} }; //+4
                      end
                    6'h0a,6'h09:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16+1:1],{(C_MANT_FP64-C_MANT_FP16+4){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h08,6'h07,6'h06:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16-2:0],{(C_MANT_FP64-C_MANT_FP16+4+2){1'b0}} }; //Precision_ctl_S+1
                      end
                    default :
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16+4:0],{(C_MANT_FP64-C_MANT_FP16){1'b0}} }; //+4
                      end
                  endcase
                end

              2'b11:
                begin

                  case (Precision_ctl_S)
                    6'b00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT+4:0],{(C_MANT_FP64-C_MANT_FP16ALT){1'b0}} }; //+4
                      end
                    6'h07,6'h06:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT+1:1],{(C_MANT_FP64-C_MANT_FP16ALT+4){1'b0}} }; //Precision_ctl_S+1
                      end
                    default :
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT+4:0],{(C_MANT_FP64-C_MANT_FP16ALT){1'b0}} }; //+4
                      end
                  endcase
                end
            endcase
          end
        end
      endgenerate
//////////////////////three iteration units, end//////////////////////////////////////////

//////////////////////four iteration units, start///////////////////////////////////////
   generate
     if(Iteration_unit_num_S==2'b11)
       begin
        always_comb
          begin
            case (Format_sel_S)
              2'b00:
                begin
                  case (Precision_ctl_S)
                    6'h00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32+4:0],{(C_MANT_FP64-C_MANT_FP32){1'b0}}}; //+4
                      end
                    6'h17,6'h16,6'h15,6'h14:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32:0],{(C_MANT_FP64-C_MANT_FP32+4){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h13,6'h12,6'h11,6'h10:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-4:0],{(C_MANT_FP64-C_MANT_FP32+4+4){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0f,6'h0e,6'h0d,6'h0c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-8:0],{(C_MANT_FP64-C_MANT_FP32+4+8){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h0b,6'h0a,6'h09,6'h08:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-12:0],{(C_MANT_FP64-C_MANT_FP32+4+12){1'b0}}}; //Precision_ctl_S+1
                      end
                    6'h07,6'h06:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32-16:0],{(C_MANT_FP64-C_MANT_FP32+4+16){1'b0}}}; //Precision_ctl_S+1
                      end
                    default:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP32+4:0],{(C_MANT_FP64-C_MANT_FP32){1'b0}}}; //+4
                      end
                  endcase
                end

              2'b01:
                begin
                  case (Precision_ctl_S)
                    6'h00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64+3:0],{(1){1'b0}}}; //+3
                      end
                    6'h34:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64+3:0],{(1){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h33,6'h32,6'h31,6'h30:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-1:0],{(5){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h2f,6'h2e,6'h2d,6'h2c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-5:0],{(9){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h2b,6'h2a,6'h29,6'h28:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-9:0],{(13){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h27,6'h26,6'h25,6'h24:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-13:0],{(17){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h23,6'h22,6'h21,6'h20:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-17:0],{(21){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h1f,6'h1e,6'h1d,6'h1c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-21:0],{(25){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h1b,6'h1a,6'h19,6'h18:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-25:0],{(29){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h17,6'h16,6'h15,6'h14:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-29:0],{(33){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h13,6'h12,6'h11,6'h10:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-33:0],{(37){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h0f,6'h0e,6'h0d,6'h0c:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-37:0],{(41){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h0b,6'h0a,6'h09,6'h08:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-41:0],{(45){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h07,6'h06:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64-45:0],{(49){1'b0}} }; //Precision_ctl_S+1
                      end
                    default:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP64+3:0],{(1){1'b0}}}; //+3
                      end
                  endcase
                end

              2'b10:
                begin
                  case (Precision_ctl_S)
                    6'b00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16+5:0],{(C_MANT_FP64-C_MANT_FP16-1){1'b0}} }; //+5
                      end
                    6'h0a,6'h09,6'h08:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16+1:1],{(C_MANT_FP64-C_MANT_FP16+4){1'b0}} }; //Precision_ctl_S+1
                      end
                    6'h07,6'h06:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16+1-4:0],{(C_MANT_FP64-C_MANT_FP16+4+3){1'b0}} }; //Precision_ctl_S+1
                      end
                    default :
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16+5:0],{(C_MANT_FP64-C_MANT_FP16-1){1'b0}} }; //+5
                      end
                  endcase
                end

              2'b11:
                begin

                  case (Precision_ctl_S)
                    6'b00:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT+4:0],{(C_MANT_FP64-C_MANT_FP16ALT){1'b0}} }; //+4
                      end
                    6'h07,6'h06:
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT:0],{(C_MANT_FP64-C_MANT_FP16ALT+4){1'b0}} }; //Precision_ctl_S+1
                      end
                    default :
                      begin
                        Mant_result_prenorm_DO = {Quotient_DP[C_MANT_FP16ALT+4:0],{(C_MANT_FP64-C_MANT_FP16ALT){1'b0}} }; //+4
                      end
                  endcase
                end
            endcase
          end
        end
      endgenerate
//////////////////////four iteration units, end///////////////////////////////////////





// resultant exponent
   logic   [C_EXP_FP64+1:0]    Exp_result_prenorm_DN,Exp_result_prenorm_DP;

   logic   [C_EXP_FP64+1:0]                                Exp_add_a_D;
   logic   [C_EXP_FP64+1:0]                                Exp_add_b_D;
   logic   [C_EXP_FP64+1:0]                                Exp_add_c_D;

  integer                                                 C_BIAS_AONE, C_HALF_BIAS;
  always_comb
    begin  //
      case (Format_sel_S)
        2'b00:
          begin
            C_BIAS_AONE =C_BIAS_AONE_FP32;
            C_HALF_BIAS =C_HALF_BIAS_FP32;
          end
        2'b01:
          begin
            C_BIAS_AONE =C_BIAS_AONE_FP64;
            C_HALF_BIAS =C_HALF_BIAS_FP64;
          end
        2'b10:
          begin
            C_BIAS_AONE =C_BIAS_AONE_FP16;
            C_HALF_BIAS =C_HALF_BIAS_FP16;
          end
        2'b11:
          begin
            C_BIAS_AONE =C_BIAS_AONE_FP16ALT;
            C_HALF_BIAS =C_HALF_BIAS_FP16ALT;
          end
        endcase
    end

//For division, exponent=(Exp_a_D-LZ1)-(Exp_b_D-LZ2)+BIAS
//For square root, exponent=(Exp_a_D-LZ1)/2+(Exp_a_D-LZ1)%2+C_HALF_BIAS
//For exponent, in preprorces module, (Exp_a_D-LZ1) and (Exp_b_D-LZ2) have been processed with the corresponding process for denormal numbers.

  assign Exp_add_a_D = {Sqrt_start_dly_S?{Exp_num_DI[C_EXP_FP64],Exp_num_DI[C_EXP_FP64],Exp_num_DI[C_EXP_FP64],Exp_num_DI[C_EXP_FP64:1]}:{Exp_num_DI[C_EXP_FP64],Exp_num_DI[C_EXP_FP64],Exp_num_DI}};
  assign Exp_add_b_D = {Sqrt_start_dly_S?{1'b0,{C_EXP_ZERO_FP64},Exp_num_DI[0]}:{~Exp_den_DI[C_EXP_FP64],~Exp_den_DI[C_EXP_FP64],~Exp_den_DI}};
  assign Exp_add_c_D = {Div_start_dly_S?{{C_BIAS_AONE}}:{{C_HALF_BIAS}}};
  assign Exp_result_prenorm_DN  = (Start_dly_S)?{Exp_add_a_D + Exp_add_b_D + Exp_add_c_D}:Exp_result_prenorm_DP;


  always_ff @(posedge Clk_CI, negedge Rst_RBI)
   begin
      if(~Rst_RBI)
        begin
          Exp_result_prenorm_DP <= '0;
        end
      else
        begin
          Exp_result_prenorm_DP<=  Exp_result_prenorm_DN;
        end
   end

  assign Exp_result_prenorm_DO = Exp_result_prenorm_DP;

endmodule
// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// This file contains all div_sqrt_top_mvp parameters
// Authors    : Lei Li  (lile@iis.ee.ethz.ch)

package defs_div_sqrt_mvp;

   // op command
   localparam C_RM                  = 3;
   localparam C_RM_NEAREST          = 3'h0;
   localparam C_RM_TRUNC            = 3'h1;
   localparam C_RM_PLUSINF          = 3'h2;
   localparam C_RM_MINUSINF         = 3'h3;
   localparam C_PC                  = 6; // Precision Control
   localparam C_FS                  = 2; // Format Selection
   localparam C_IUNC                = 2; // Iteration Unit Number Control
   localparam Iteration_unit_num_S  = 2'b10;

   // FP64
   localparam C_OP_FP64             = 64;
   localparam C_MANT_FP64           = 52;
   localparam C_EXP_FP64            = 11;
   localparam C_BIAS_FP64           = 1023;
   localparam C_BIAS_AONE_FP64      = 11'h400;
   localparam C_HALF_BIAS_FP64      = 511;
   localparam C_EXP_ZERO_FP64       = 11'h000;
   localparam C_EXP_ONE_FP64        = 13'h001; // Bit width is in agreement with in norm
   localparam C_EXP_INF_FP64        = 11'h7FF;
   localparam C_MANT_ZERO_FP64      = 52'h0;
   localparam C_MANT_NAN_FP64       = 52'h8_0000_0000_0000;
   localparam C_PZERO_FP64          = 64'h0000_0000_0000_0000;
   localparam C_MZERO_FP64          = 64'h8000_0000_0000_0000;
   localparam C_QNAN_FP64           = 64'h7FF8_0000_0000_0000;

   // FP32
   localparam C_OP_FP32             = 32;
   localparam C_MANT_FP32           = 23;
   localparam C_EXP_FP32            = 8;
   localparam C_BIAS_FP32           = 127;
   localparam C_BIAS_AONE_FP32      = 8'h80;
   localparam C_HALF_BIAS_FP32      = 63;
   localparam C_EXP_ZERO_FP32       = 8'h00;
   localparam C_EXP_INF_FP32        = 8'hFF;
   localparam C_MANT_ZERO_FP32      = 23'h0;
   localparam C_PZERO_FP32          = 32'h0000_0000;
   localparam C_MZERO_FP32          = 32'h8000_0000;
   localparam C_QNAN_FP32           = 32'h7FC0_0000;

   // FP16
   localparam C_OP_FP16             = 16;
   localparam C_MANT_FP16           = 10;
   localparam C_EXP_FP16            = 5;
   localparam C_BIAS_FP16           = 15;
   localparam C_BIAS_AONE_FP16      = 5'h10;
   localparam C_HALF_BIAS_FP16      = 7;
   localparam C_EXP_ZERO_FP16       = 5'h00;
   localparam C_EXP_INF_FP16        = 5'h1F;
   localparam C_MANT_ZERO_FP16      = 10'h0;
   localparam C_PZERO_FP16          = 16'h0000;
   localparam C_MZERO_FP16          = 16'h8000;
   localparam C_QNAN_FP16           = 16'h7E00;

   // FP16alt
   localparam C_OP_FP16ALT           = 16;
   localparam C_MANT_FP16ALT         = 7;
   localparam C_EXP_FP16ALT          = 8;
   localparam C_BIAS_FP16ALT         = 127;
   localparam C_BIAS_AONE_FP16ALT    = 8'h80;
   localparam C_HALF_BIAS_FP16ALT    = 63;
   localparam C_EXP_ZERO_FP16ALT     = 8'h00;
   localparam C_EXP_INF_FP16ALT      = 8'hFF;
   localparam C_MANT_ZERO_FP16ALT    = 7'h0;
   localparam C_QNAN_FP16ALT         = 16'h7FC0;

endpackage : defs_div_sqrt_mvp
// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
////////////////////////////////////////////////////////////////////////////////
// Company:        IIS @ ETHZ - Federal Institute of Technology               //
//                                                                            //
// Engineers:      Lei Li -- lile@iis.ee.ethz.ch                              //
//                                                                            //
// Additional contributions by:                                               //
//                                                                            //
//                                                                            //
//                                                                            //
// Create Date:    20/04/2018                                                 //
// Design Name:    FPU                                                        //
// Module Name:    div_sqrt_mvp_wrapper.sv                                    //
// Project Name:   The shared divisor and square root                         //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    The wrapper of  div_sqrt_top_mvp                           //
//                                                                            //
//                                                                            //
//                                                                            //
//                                                                            //
//                                                                            //
//                                                                            //
//                                                                            //
//                                                                            //
//                                                                            //
//                                                                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

//import defs_div_sqrt_mvp::*;

module div_sqrt_mvp_wrapper
#(
   parameter   PrePipeline_depth_S             =        0,  // If you want to add a flip/flop stage before preprocess, set it to 1.
   parameter   PostPipeline_depth_S            =        2  // The output delay stages
)
  (//Input
   input logic                            Clk_CI,
   input logic                            Rst_RBI,
   input logic                            Div_start_SI,
   input logic                            Sqrt_start_SI,

   //Input Operands
   input logic [C_OP_FP64-1:0]            Operand_a_DI,
   input logic [C_OP_FP64-1:0]            Operand_b_DI,

   // Input Control
   input logic [C_RM-1:0]                 RM_SI,    //Rounding Mode
   input logic [C_PC-1:0]                 Precision_ctl_SI, // Precision Control
   input logic [C_FS-1:0]                 Format_sel_SI,  // Format Selection,
   input logic                            Kill_SI,

   //Output Result
   output logic [C_OP_FP64-1:0]           Result_DO,

   //Output-Flags
   output logic [4:0]                     Fflags_SO,
   output logic                           Ready_SO,
   output logic                           Done_SO
 );


   logic                                 Div_start_S_S,Sqrt_start_S_S;
   logic [C_OP_FP64-1:0]                 Operand_a_S_D;
   logic [C_OP_FP64-1:0]                 Operand_b_S_D;

   // Input Control
   logic [C_RM-1:0]                      RM_S_S;    //Rounding Mode
   logic [C_PC-1:0]                      Precision_ctl_S_S; // Precision Control
   logic [C_FS-1:0]                      Format_sel_S_S;  // Format Selection,
   logic                                 Kill_S_S;


  logic [C_OP_FP64-1:0]                  Result_D;
  logic                                  Ready_S;
  logic                                  Done_S;
  logic [4:0]                            Fflags_S;


  generate
    if(PrePipeline_depth_S==1)
      begin

         div_sqrt_top_mvp  div_top_U0  //for RTL

          (//Input
           .Clk_CI                 (Clk_CI),
           .Rst_RBI                (Rst_RBI),
           .Div_start_SI           (Div_start_S_S),
           .Sqrt_start_SI          (Sqrt_start_S_S),
           //Input Operands
           .Operand_a_DI          (Operand_a_S_D),
           .Operand_b_DI          (Operand_b_S_D),
           .RM_SI                 (RM_S_S),    //Rounding Mode
           .Precision_ctl_SI      (Precision_ctl_S_S),
           .Format_sel_SI         (Format_sel_S_S),
           .Kill_SI               (Kill_S_S),
           .Result_DO             (Result_D),
           .Fflags_SO             (Fflags_S),
           .Ready_SO              (Ready_S),
           .Done_SO               (Done_S)
         );

           always_ff @(posedge Clk_CI, negedge Rst_RBI)
             begin
                if(~Rst_RBI)
                  begin
                    Div_start_S_S<='0;
                    Sqrt_start_S_S<=1'b0;
                    Operand_a_S_D<='0;
                    Operand_b_S_D<='0;
                    RM_S_S <=1'b0;
                    Precision_ctl_S_S<='0;
                    Format_sel_S_S<='0;
                    Kill_S_S<='0;
                  end
                else
                  begin
                    Div_start_S_S<=Div_start_SI;
                    Sqrt_start_S_S<=Sqrt_start_SI;
                    Operand_a_S_D<=Operand_a_DI;
                    Operand_b_S_D<=Operand_b_DI;
                    RM_S_S <=RM_SI;
                    Precision_ctl_S_S<=Precision_ctl_SI;
                    Format_sel_S_S<=Format_sel_SI;
                    Kill_S_S<=Kill_SI;
                  end
            end
     end

     else
      begin
          div_sqrt_top_mvp  div_top_U0  //for RTL
          (//Input
           .Clk_CI                 (Clk_CI),
           .Rst_RBI                (Rst_RBI),
           .Div_start_SI           (Div_start_SI),
           .Sqrt_start_SI          (Sqrt_start_SI),
           //Input Operands
           .Operand_a_DI          (Operand_a_DI),
           .Operand_b_DI          (Operand_b_DI),
           .RM_SI                 (RM_SI),    //Rounding Mode
           .Precision_ctl_SI      (Precision_ctl_SI),
           .Format_sel_SI         (Format_sel_SI),
           .Kill_SI               (Kill_SI),
           .Result_DO             (Result_D),
           .Fflags_SO             (Fflags_S),
           .Ready_SO              (Ready_S),
           .Done_SO               (Done_S)
         );
      end
  endgenerate

   /////////////////////////////////////////////////////////////////////////////
   // First Stage of Outputs
   /////////////////////////////////////////////////////////////////////////////
  logic [C_OP_FP64-1:0]         Result_dly_S_D;
  logic                         Ready_dly_S_S;
  logic                         Done_dly_S_S;
  logic [4:0]                   Fflags_dly_S_S;
   always_ff @(posedge Clk_CI, negedge Rst_RBI)
     begin
        if(~Rst_RBI)
          begin
            Result_dly_S_D<='0;
            Ready_dly_S_S<=1'b0;
            Done_dly_S_S<=1'b0;
            Fflags_dly_S_S<=1'b0;
          end
        else
          begin
            Result_dly_S_D<=Result_D;
            Ready_dly_S_S<=Ready_S;
            Done_dly_S_S<=Done_S;
            Fflags_dly_S_S<=Fflags_S;
          end
    end

   /////////////////////////////////////////////////////////////////////////////
   // Second Stage of Outputs
   /////////////////////////////////////////////////////////////////////////////

  logic [C_OP_FP64-1:0]         Result_dly_D_D;
  logic                         Ready_dly_D_S;
  logic                         Done_dly_D_S;
  logic [4:0]                   Fflags_dly_D_S;
  generate
    if(PostPipeline_depth_S==2)
      begin
        always_ff @(posedge Clk_CI, negedge Rst_RBI)
          begin
            if(~Rst_RBI)
              begin
                Result_dly_D_D<='0;
                Ready_dly_D_S<=1'b0;
                Done_dly_D_S<=1'b0;
                Fflags_dly_D_S<=1'b0;
              end
           else
             begin
               Result_dly_D_D<=Result_dly_S_D;
               Ready_dly_D_S<=Ready_dly_S_S;
               Done_dly_D_S<=Done_dly_S_S;
               Fflags_dly_D_S<=Fflags_dly_S_S;
             end
          end
        assign  Result_DO = Result_dly_D_D;
        assign  Ready_SO  = Ready_dly_D_S;
        assign  Done_SO  = Done_dly_D_S;
        assign  Fflags_SO=Fflags_dly_D_S;
       end

     else
       begin
         assign  Result_DO = Result_dly_S_D;
         assign  Ready_SO  = Ready_dly_S_S;
         assign  Done_SO   = Done_dly_S_S;
         assign  Fflags_SO  = Fflags_dly_S_S;
       end

   endgenerate

endmodule //
// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Company:        IIS @ ETHZ - Federal Institute of Technology               //
//                                                                            //
// Engineers:      Lei Li -- lile@iis.ee.ethz.ch                              //
//                                                                            //
// Additional contributions by:                                               //
//                                                                            //
//                                                                            //
//                                                                            //
// Create Date:    03/03/2018                                                 //
// Design Name:    div_sqrt_top_mvp                                           //
// Module Name:    div_sqrt_top_mvp.sv                                        //
// Project Name:   The shared divisor and square root                         //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    The top of div and sqrt                                    //
//                                                                            //
//                                                                            //
// Revision Date:  12/04/2018                                                 //
//                 Lei Li                                                     //
//                 To address some requirements by Stefan and add low power   //
//                 control for special cases                                  //
////////////////////////////////////////////////////////////////////////////////

//import defs_div_sqrt_mvp::*;

module div_sqrt_top_mvp

  (//Input
   input logic                            Clk_CI,
   input logic                            Rst_RBI,
   input logic                            Div_start_SI,
   input logic                            Sqrt_start_SI,

   //Input Operands
   input logic [C_OP_FP64-1:0]            Operand_a_DI,
   input logic [C_OP_FP64-1:0]            Operand_b_DI,

   // Input Control
   input logic [C_RM-1:0]                 RM_SI,    //Rounding Mode
   input logic [C_PC-1:0]                 Precision_ctl_SI, // Precision Control
   input logic [C_FS-1:0]                 Format_sel_SI,  // Format Selection,
   input logic                            Kill_SI,

   //Output Result
   output logic [C_OP_FP64-1:0]           Result_DO,

   //Output-Flags
   output logic [4:0]                     Fflags_SO,
   output logic                           Ready_SO,
   output logic                           Done_SO
 );





   //Operand components
   logic [C_EXP_FP64:0]                 Exp_a_D;
   logic [C_EXP_FP64:0]                 Exp_b_D;
   logic [C_MANT_FP64:0]                Mant_a_D;
   logic [C_MANT_FP64:0]                Mant_b_D;

   logic [C_EXP_FP64+1:0]               Exp_z_D;
   logic [C_MANT_FP64+4:0]              Mant_z_D;
   logic                                Sign_z_D;
   logic                                Start_S;
   logic [C_RM-1:0]                     RM_dly_S;
   logic                                Div_enable_S;
   logic                                Sqrt_enable_S;
   logic                                Inf_a_S;
   logic                                Inf_b_S;
   logic                                Zero_a_S;
   logic                                Zero_b_S;
   logic                                NaN_a_S;
   logic                                NaN_b_S;
   logic                                SNaN_S;
   logic                                Special_case_SB,Special_case_dly_SB;

   logic Full_precision_S;
   logic FP32_S;
   logic FP64_S;
   logic FP16_S;
   logic FP16ALT_S;


 preprocess_mvp  preprocess_U0
 (
   .Clk_CI                (Clk_CI             ),
   .Rst_RBI               (Rst_RBI            ),
   .Div_start_SI          (Div_start_SI       ),
   .Sqrt_start_SI         (Sqrt_start_SI      ),
   .Ready_SI              (Ready_SO           ),
   .Operand_a_DI          (Operand_a_DI       ),
   .Operand_b_DI          (Operand_b_DI       ),
   .RM_SI                 (RM_SI              ),
   .Format_sel_SI         (Format_sel_SI      ),
   .Start_SO              (Start_S            ),
   .Exp_a_DO_norm         (Exp_a_D            ),
   .Exp_b_DO_norm         (Exp_b_D            ),
   .Mant_a_DO_norm        (Mant_a_D           ),
   .Mant_b_DO_norm        (Mant_b_D           ),
   .RM_dly_SO             (RM_dly_S           ),
   .Sign_z_DO             (Sign_z_D           ),
   .Inf_a_SO              (Inf_a_S            ),
   .Inf_b_SO              (Inf_b_S            ),
   .Zero_a_SO             (Zero_a_S           ),
   .Zero_b_SO             (Zero_b_S           ),
   .NaN_a_SO              (NaN_a_S            ),
   .NaN_b_SO              (NaN_b_S            ),
   .SNaN_SO               (SNaN_S             ),
   .Special_case_SBO      (Special_case_SB    ),
   .Special_case_dly_SBO  (Special_case_dly_SB)
   );

 nrbd_nrsc_mvp   nrbd_nrsc_U0
  (
   .Clk_CI                (Clk_CI             ),
   .Rst_RBI               (Rst_RBI            ),
   .Div_start_SI          (Div_start_SI       ) ,
   .Sqrt_start_SI         (Sqrt_start_SI      ),
   .Start_SI              (Start_S            ),
   .Kill_SI               (Kill_SI            ),
   .Special_case_SBI      (Special_case_SB    ),
   .Special_case_dly_SBI  (Special_case_dly_SB),
   .Div_enable_SO         (Div_enable_S       ),
   .Sqrt_enable_SO        (Sqrt_enable_S      ),
   .Precision_ctl_SI      (Precision_ctl_SI   ),
   .Format_sel_SI         (Format_sel_SI      ),
   .Exp_a_DI              (Exp_a_D            ),
   .Exp_b_DI              (Exp_b_D            ),
   .Mant_a_DI             (Mant_a_D           ),
   .Mant_b_DI             (Mant_b_D           ),
   .Full_precision_SO     (Full_precision_S   ),
   .FP32_SO               (FP32_S             ),
   .FP64_SO               (FP64_S             ),
   .FP16_SO               (FP16_S             ),
   .FP16ALT_SO            (FP16ALT_S          ),
   .Ready_SO              (Ready_SO           ),
   .Done_SO               (Done_SO            ),
   .Exp_z_DO              (Exp_z_D            ),
   .Mant_z_DO             (Mant_z_D           )
    );


 norm_div_sqrt_mvp  fpu_norm_U0
  (
   .Mant_in_DI            (Mant_z_D           ),
   .Exp_in_DI             (Exp_z_D            ),
   .Sign_in_DI            (Sign_z_D           ),
   .Div_enable_SI         (Div_enable_S       ),
   .Sqrt_enable_SI        (Sqrt_enable_S      ),
   .Inf_a_SI              (Inf_a_S            ),
   .Inf_b_SI              (Inf_b_S            ),
   .Zero_a_SI             (Zero_a_S           ),
   .Zero_b_SI             (Zero_b_S           ),
   .NaN_a_SI              (NaN_a_S            ),
   .NaN_b_SI              (NaN_b_S            ),
   .SNaN_SI               (SNaN_S             ),
   .RM_SI                 (RM_dly_S           ),
   .Full_precision_SI     (Full_precision_S   ),
   .FP32_SI               (FP32_S             ),
   .FP64_SI               (FP64_S             ),
   .FP16_SI               (FP16_S             ),
   .FP16ALT_SI            (FP16ALT_S          ),
   .Result_DO             (Result_DO          ),
   .Fflags_SO             (Fflags_SO          ) //{NV,DZ,OF,UF,NX}
   );

endmodule
// Copyright 2019 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Stefan Mach <smach@iis.ee.ethz.ch>

//`include "common_cells/registers.svh"

module fpnew_cast_multi #(
  parameter fpnew_pkg::fmt_logic_t   FpFmtConfig  = '1,
  parameter fpnew_pkg::ifmt_logic_t  IntFmtConfig = '1,
  // FPU configuration
  parameter int unsigned             NumPipeRegs = 0,
  parameter fpnew_pkg::pipe_config_t PipeConfig  = fpnew_pkg::BEFORE,
  parameter type                     TagType     = logic,
  parameter type                     AuxType     = logic,
  // Do not change
  localparam int unsigned WIDTH = fpnew_pkg::maximum(fpnew_pkg::max_fp_width(FpFmtConfig),
                                                     fpnew_pkg::max_int_width(IntFmtConfig)),
  localparam int unsigned NUM_FORMATS = fpnew_pkg::NUM_FP_FORMATS
) (
  input  logic                   clk_i,
  input  logic                   rst_ni,
  // Input signals
  input  logic [WIDTH-1:0]       operands_i, // 1 operand
  input  logic [NUM_FORMATS-1:0] is_boxed_i, // 1 operand
  input  fpnew_pkg::roundmode_e  rnd_mode_i,
  input  fpnew_pkg::operation_e  op_i,
  input  logic                   op_mod_i,
  input  fpnew_pkg::fp_format_e  src_fmt_i,
  input  fpnew_pkg::fp_format_e  dst_fmt_i,
  input  fpnew_pkg::int_format_e int_fmt_i,
  input  TagType                 tag_i,
  input  AuxType                 aux_i,
  // Input Handshake
  input  logic                   in_valid_i,
  output logic                   in_ready_o,
  input  logic                   flush_i,
  // Output signals
  output logic [WIDTH-1:0]       result_o,
  output fpnew_pkg::status_t     status_o,
  output logic                   extension_bit_o,
  output TagType                 tag_o,
  output AuxType                 aux_o,
  // Output handshake
  output logic                   out_valid_o,
  input  logic                   out_ready_i,
  // Indication of valid data in flight
  output logic                   busy_o
);
    `define FFLARNC(__q, __d, __load, __clear, __reset_value, __clk, __arst_n)
    always_ff @(posedge (__clk) or negedge (__arst_n)) begin
      if (!__arst_n) begin
        __q <= (__reset_value);
      end else begin
        __q <= (__clear) ? (__reset_value) : (__load) ? (__d) : (__q);
      end
    end

  `define FFL(__q, __d, __load, __reset_value)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      __q <= (__reset_value);
    end else begin
      __q <= (__load) ? (__d) : (__q);
    end
  end
  // ----------
  // Constants
  // ----------
  localparam int unsigned NUM_INT_FORMATS = fpnew_pkg::NUM_INT_FORMATS;
  localparam int unsigned MAX_INT_WIDTH   = fpnew_pkg::max_int_width(IntFmtConfig);

  localparam fpnew_pkg::fp_encoding_t SUPER_FORMAT = fpnew_pkg::super_format(FpFmtConfig);

  localparam int unsigned SUPER_EXP_BITS = SUPER_FORMAT.exp_bits;
  localparam int unsigned SUPER_MAN_BITS = SUPER_FORMAT.man_bits;
  localparam int unsigned SUPER_BIAS     = 2**(SUPER_EXP_BITS - 1) - 1;

  // The internal mantissa includes normal bit or an entire integer
  localparam int unsigned INT_MAN_WIDTH = fpnew_pkg::maximum(SUPER_MAN_BITS + 1, MAX_INT_WIDTH);
  // If needed, there will be a LZC for renormalization
  localparam int unsigned LZC_RESULT_WIDTH = $clog2(INT_MAN_WIDTH);
  // The internal exponent must be able to represent the smallest denormal input value as signed
  // or the number of bits in an integer
  localparam int unsigned INT_EXP_WIDTH = fpnew_pkg::maximum($clog2(MAX_INT_WIDTH),
      fpnew_pkg::maximum(SUPER_EXP_BITS, $clog2(SUPER_BIAS + SUPER_MAN_BITS))) + 1;
  // Pipelines
  localparam NUM_INP_REGS = PipeConfig == fpnew_pkg::BEFORE
                            ? NumPipeRegs
                            : (PipeConfig == fpnew_pkg::DISTRIBUTED
                               ? ((NumPipeRegs + 1) / 3) // Second to get distributed regs
                               : 0); // no regs here otherwise
  localparam NUM_MID_REGS = PipeConfig == fpnew_pkg::INSIDE
                          ? NumPipeRegs
                          : (PipeConfig == fpnew_pkg::DISTRIBUTED
                             ? ((NumPipeRegs + 2) / 3) // First to get distributed regs
                             : 0); // no regs here otherwise
  localparam NUM_OUT_REGS = PipeConfig == fpnew_pkg::AFTER
                            ? NumPipeRegs
                            : (PipeConfig == fpnew_pkg::DISTRIBUTED
                               ? (NumPipeRegs / 3) // Last to get distributed regs
                               : 0); // no regs here otherwise

  // ---------------
  // Input pipeline
  // ---------------
  // Selected pipeline output signals as non-arrays
  logic [WIDTH-1:0]       operands_q;
  logic [NUM_FORMATS-1:0] is_boxed_q;
  logic                   op_mod_q;
  fpnew_pkg::fp_format_e  src_fmt_q;
  fpnew_pkg::fp_format_e  dst_fmt_q;
  fpnew_pkg::int_format_e int_fmt_q;

  // Input pipeline signals, index i holds signal after i register stages
  logic                   [0:NUM_INP_REGS][WIDTH-1:0]       inp_pipe_operands_q;
  logic                   [0:NUM_INP_REGS][NUM_FORMATS-1:0] inp_pipe_is_boxed_q;
  fpnew_pkg::roundmode_e  [0:NUM_INP_REGS]                  inp_pipe_rnd_mode_q;
  fpnew_pkg::operation_e  [0:NUM_INP_REGS]                  inp_pipe_op_q;
  logic                   [0:NUM_INP_REGS]                  inp_pipe_op_mod_q;
  fpnew_pkg::fp_format_e  [0:NUM_INP_REGS]                  inp_pipe_src_fmt_q;
  fpnew_pkg::fp_format_e  [0:NUM_INP_REGS]                  inp_pipe_dst_fmt_q;
  fpnew_pkg::int_format_e [0:NUM_INP_REGS]                  inp_pipe_int_fmt_q;
  TagType                 [0:NUM_INP_REGS]                  inp_pipe_tag_q;
  AuxType                 [0:NUM_INP_REGS]                  inp_pipe_aux_q;
  logic                   [0:NUM_INP_REGS]                  inp_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_INP_REGS] inp_pipe_ready;

  // Input stage: First element of pipeline is taken from inputs
  assign inp_pipe_operands_q[0] = operands_i;
  assign inp_pipe_is_boxed_q[0] = is_boxed_i;
  assign inp_pipe_rnd_mode_q[0] = rnd_mode_i;
  assign inp_pipe_op_q[0]       = op_i;
  assign inp_pipe_op_mod_q[0]   = op_mod_i;
  assign inp_pipe_src_fmt_q[0]  = src_fmt_i;
  assign inp_pipe_dst_fmt_q[0]  = dst_fmt_i;
  assign inp_pipe_int_fmt_q[0]  = int_fmt_i;
  assign inp_pipe_tag_q[0]      = tag_i;
  assign inp_pipe_aux_q[0]      = aux_i;
  assign inp_pipe_valid_q[0]    = in_valid_i;
  // Input stage: Propagate pipeline ready signal to updtream circuitry
  assign in_ready_o = inp_pipe_ready[0];
  // Generate the register stages
  for (genvar i = 0; i < NUM_INP_REGS; i++) begin : gen_input_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign inp_pipe_ready[i] = inp_pipe_ready[i+1] | ~inp_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(inp_pipe_valid_q[i+1], inp_pipe_valid_q[i], inp_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = inp_pipe_ready[i] & inp_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(inp_pipe_operands_q[i+1], inp_pipe_operands_q[i], reg_ena, '0)
    `FFL(inp_pipe_is_boxed_q[i+1], inp_pipe_is_boxed_q[i], reg_ena, '0)
    `FFL(inp_pipe_rnd_mode_q[i+1], inp_pipe_rnd_mode_q[i], reg_ena, fpnew_pkg::RNE)
    `FFL(inp_pipe_op_q[i+1],       inp_pipe_op_q[i],       reg_ena, fpnew_pkg::FMADD)
    `FFL(inp_pipe_op_mod_q[i+1],   inp_pipe_op_mod_q[i],   reg_ena, '0)
    `FFL(inp_pipe_src_fmt_q[i+1],  inp_pipe_src_fmt_q[i],  reg_ena, fpnew_pkg::fp_format_e'(0))
    `FFL(inp_pipe_dst_fmt_q[i+1],  inp_pipe_dst_fmt_q[i],  reg_ena, fpnew_pkg::fp_format_e'(0))
    `FFL(inp_pipe_int_fmt_q[i+1],  inp_pipe_int_fmt_q[i],  reg_ena, fpnew_pkg::int_format_e'(0))
    `FFL(inp_pipe_tag_q[i+1],      inp_pipe_tag_q[i],      reg_ena, TagType'('0))
    `FFL(inp_pipe_aux_q[i+1],      inp_pipe_aux_q[i],      reg_ena, AuxType'('0))
  end
  // Output stage: assign selected pipe outputs to signals for later use
  assign operands_q = inp_pipe_operands_q[NUM_INP_REGS];
  assign is_boxed_q = inp_pipe_is_boxed_q[NUM_INP_REGS];
  assign op_mod_q   = inp_pipe_op_mod_q[NUM_INP_REGS];
  assign src_fmt_q  = inp_pipe_src_fmt_q[NUM_INP_REGS];
  assign dst_fmt_q  = inp_pipe_dst_fmt_q[NUM_INP_REGS];
  assign int_fmt_q  = inp_pipe_int_fmt_q[NUM_INP_REGS];

  // -----------------
  // Input processing
  // -----------------
  logic src_is_int, dst_is_int; // if 0, it's a float

  assign src_is_int = (inp_pipe_op_q[NUM_INP_REGS] == fpnew_pkg::I2F);
  assign dst_is_int = (inp_pipe_op_q[NUM_INP_REGS] == fpnew_pkg::F2I);

  logic [INT_MAN_WIDTH-1:0] encoded_mant; // input mantissa with implicit bit

  logic        [NUM_FORMATS-1:0]                    fmt_sign;
  logic signed [NUM_FORMATS-1:0][INT_EXP_WIDTH-1:0] fmt_exponent;
  logic        [NUM_FORMATS-1:0][INT_MAN_WIDTH-1:0] fmt_mantissa;
  logic signed [NUM_FORMATS-1:0][INT_EXP_WIDTH-1:0] fmt_shift_compensation; // for LZC

  fpnew_pkg::fp_info_t [NUM_FORMATS-1:0] info;

  logic [NUM_INT_FORMATS-1:0][INT_MAN_WIDTH-1:0] ifmt_input_val;
  logic                                          int_sign;
  logic [INT_MAN_WIDTH-1:0]                      int_value, int_mantissa;

  // FP Input initialization
  for (genvar fmt = 0; fmt < int'(NUM_FORMATS); fmt++) begin : fmt_init_inputs
    // Set up some constants
    localparam int unsigned FP_WIDTH = fpnew_pkg::fp_width(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned EXP_BITS = fpnew_pkg::exp_bits(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned MAN_BITS = fpnew_pkg::man_bits(fpnew_pkg::fp_format_e'(fmt));

    if (FpFmtConfig[fmt]) begin : active_format
      // Classify input
      fpnew_classifier #(
        .FpFormat    ( fpnew_pkg::fp_format_e'(fmt) ),
        .NumOperands ( 1                            )
      ) i_fpnew_classifier (
        .operands_i ( operands_q[FP_WIDTH-1:0] ),
        .is_boxed_i ( is_boxed_q[fmt]          ),
        .info_o     ( info[fmt]                )
      );

      assign fmt_sign[fmt]     = operands_q[FP_WIDTH-1];
      assign fmt_exponent[fmt] = signed'({1'b0, operands_q[MAN_BITS+:EXP_BITS]});
      assign fmt_mantissa[fmt] = {info[fmt].is_normal, operands_q[MAN_BITS-1:0]}; // zero pad
      // Compensation for the difference in mantissa widths used for leading-zero count
      assign fmt_shift_compensation[fmt] = signed'(INT_MAN_WIDTH - 1 - MAN_BITS);
    end else begin : inactive_format
      assign info[fmt]                   = '{default: fpnew_pkg::DONT_CARE}; // format disabled
      assign fmt_sign[fmt]               = fpnew_pkg::DONT_CARE;             // format disabled
      assign fmt_exponent[fmt]           = '{default: fpnew_pkg::DONT_CARE}; // format disabled
      assign fmt_mantissa[fmt]           = '{default: fpnew_pkg::DONT_CARE}; // format disabled
      assign fmt_shift_compensation[fmt] = '{default: fpnew_pkg::DONT_CARE}; // format disabled
    end
  end

  // Sign-extend INT input
  for (genvar ifmt = 0; ifmt < int'(NUM_INT_FORMATS); ifmt++) begin : gen_sign_extend_int
    // Set up some constants
    localparam int unsigned INT_WIDTH = fpnew_pkg::int_width(fpnew_pkg::int_format_e'(ifmt));

    if (IntFmtConfig[ifmt]) begin : active_format // only active formats
      always_comb begin : sign_ext_input
        // sign-extend value only if it's signed
        ifmt_input_val[ifmt]                = '{default: operands_q[INT_WIDTH-1] & ~op_mod_q};
        ifmt_input_val[ifmt][INT_WIDTH-1:0] = operands_q[INT_WIDTH-1:0];
      end
    end else begin : inactive_format
      assign ifmt_input_val[ifmt] = '{default: fpnew_pkg::DONT_CARE}; // format disabled
    end
  end

  // Construct input mantissa from integer
  assign int_value    = ifmt_input_val[int_fmt_q];
  assign int_sign     = int_value[INT_MAN_WIDTH-1] & ~op_mod_q; // only signed ints are negative
  assign int_mantissa = int_sign ? unsigned'(-int_value) : int_value; // get magnitude of negative

  // select mantissa with source format
  assign encoded_mant = src_is_int ? int_mantissa : fmt_mantissa[src_fmt_q];

  // --------------
  // Normalization
  // --------------
  logic signed [INT_EXP_WIDTH-1:0] src_bias;      // src format bias
  logic signed [INT_EXP_WIDTH-1:0] src_exp;       // src format exponent (biased)
  logic signed [INT_EXP_WIDTH-1:0] src_subnormal; // src is subnormal
  logic signed [INT_EXP_WIDTH-1:0] src_offset;    // src offset within mantissa

  assign src_bias      = signed'(fpnew_pkg::bias(src_fmt_q));
  assign src_exp       = fmt_exponent[src_fmt_q];
  assign src_subnormal = signed'({1'b0, info[src_fmt_q].is_subnormal});
  assign src_offset    = fmt_shift_compensation[src_fmt_q];

  logic                            input_sign;   // input sign
  logic signed [INT_EXP_WIDTH-1:0] input_exp;    // unbiased true exponent
  logic        [INT_MAN_WIDTH-1:0] input_mant;   // normalized input mantissa
  logic                            mant_is_zero; // for integer zeroes

  logic signed [INT_EXP_WIDTH-1:0] fp_input_exp;
  logic signed [INT_EXP_WIDTH-1:0] int_input_exp;

  // Input mantissa needs to be normalized
  logic [LZC_RESULT_WIDTH-1:0] renorm_shamt;     // renormalization shift amount
  logic [LZC_RESULT_WIDTH:0]   renorm_shamt_sgn; // signed form for calculations

  // Leading-zero counter is needed for renormalization
  lzc #(
    .WIDTH ( INT_MAN_WIDTH ),
    .MODE  ( 1             ) // MODE = 1 counts leading zeroes
  ) i_lzc (
    .in_i    ( encoded_mant ),
    .cnt_o   ( renorm_shamt ),
    .empty_o ( mant_is_zero )
  );
  assign renorm_shamt_sgn = signed'({1'b0, renorm_shamt});

  // Get the sign from the proper source
  assign input_sign = src_is_int ? int_sign : fmt_sign[src_fmt_q];
  // Realign input mantissa, append zeroes if destination is wider
  assign input_mant = encoded_mant << renorm_shamt;
  // Unbias exponent and compensate for shift
  assign fp_input_exp  = signed'(src_exp + src_subnormal - src_bias -
                                 renorm_shamt_sgn + src_offset); // compensate for shift
  assign int_input_exp = signed'(INT_MAN_WIDTH - 1 - renorm_shamt_sgn);

  assign input_exp     = src_is_int ? int_input_exp : fp_input_exp;

  logic signed [INT_EXP_WIDTH-1:0] destination_exp;  // re-biased exponent for destination

  // Rebias the exponent
  assign destination_exp = input_exp + signed'(fpnew_pkg::bias(dst_fmt_q));

  // ---------------
  // Internal pipeline
  // ---------------
  // Pipeline output signals as non-arrays
  logic                            input_sign_q;
  logic signed [INT_EXP_WIDTH-1:0] input_exp_q;
  logic [INT_MAN_WIDTH-1:0]        input_mant_q;
  logic signed [INT_EXP_WIDTH-1:0] destination_exp_q;
  logic                            src_is_int_q;
  logic                            dst_is_int_q;
  fpnew_pkg::fp_info_t             info_q;
  logic                            mant_is_zero_q;
  logic                            op_mod_q2;
  fpnew_pkg::roundmode_e           rnd_mode_q;
  fpnew_pkg::fp_format_e           src_fmt_q2;
  fpnew_pkg::fp_format_e           dst_fmt_q2;
  fpnew_pkg::int_format_e          int_fmt_q2;
  // Internal pipeline signals, index i holds signal after i register stages


  logic                   [0:NUM_MID_REGS]                    mid_pipe_input_sign_q;
  logic signed            [0:NUM_MID_REGS][INT_EXP_WIDTH-1:0] mid_pipe_input_exp_q;
  logic                   [0:NUM_MID_REGS][INT_MAN_WIDTH-1:0] mid_pipe_input_mant_q;
  logic signed            [0:NUM_MID_REGS][INT_EXP_WIDTH-1:0] mid_pipe_dest_exp_q;
  logic                   [0:NUM_MID_REGS]                    mid_pipe_src_is_int_q;
  logic                   [0:NUM_MID_REGS]                    mid_pipe_dst_is_int_q;
  fpnew_pkg::fp_info_t    [0:NUM_MID_REGS]                    mid_pipe_info_q;
  logic                   [0:NUM_MID_REGS]                    mid_pipe_mant_zero_q;
  logic                   [0:NUM_MID_REGS]                    mid_pipe_op_mod_q;
  fpnew_pkg::roundmode_e  [0:NUM_MID_REGS]                    mid_pipe_rnd_mode_q;
  fpnew_pkg::fp_format_e  [0:NUM_MID_REGS]                    mid_pipe_src_fmt_q;
  fpnew_pkg::fp_format_e  [0:NUM_MID_REGS]                    mid_pipe_dst_fmt_q;
  fpnew_pkg::int_format_e [0:NUM_MID_REGS]                    mid_pipe_int_fmt_q;
  TagType                 [0:NUM_MID_REGS]                    mid_pipe_tag_q;
  AuxType                 [0:NUM_MID_REGS]                    mid_pipe_aux_q;
  logic                   [0:NUM_MID_REGS]                    mid_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_MID_REGS] mid_pipe_ready;

  // Input stage: First element of pipeline is taken from upstream logic
  assign mid_pipe_input_sign_q[0] = input_sign;
  assign mid_pipe_input_exp_q[0]  = input_exp;
  assign mid_pipe_input_mant_q[0] = input_mant;
  assign mid_pipe_dest_exp_q[0]   = destination_exp;
  assign mid_pipe_src_is_int_q[0] = src_is_int;
  assign mid_pipe_dst_is_int_q[0] = dst_is_int;
  assign mid_pipe_info_q[0]       = info[src_fmt_q];
  assign mid_pipe_mant_zero_q[0]  = mant_is_zero;
  assign mid_pipe_op_mod_q[0]     = op_mod_q;
  assign mid_pipe_rnd_mode_q[0]   = inp_pipe_rnd_mode_q[NUM_INP_REGS];
  assign mid_pipe_src_fmt_q[0]    = src_fmt_q;
  assign mid_pipe_dst_fmt_q[0]    = dst_fmt_q;
  assign mid_pipe_int_fmt_q[0]    = int_fmt_q;
  assign mid_pipe_tag_q[0]        = inp_pipe_tag_q[NUM_INP_REGS];
  assign mid_pipe_aux_q[0]        = inp_pipe_aux_q[NUM_INP_REGS];
  assign mid_pipe_valid_q[0]      = inp_pipe_valid_q[NUM_INP_REGS];
  // Input stage: Propagate pipeline ready signal to input pipe
  assign inp_pipe_ready[NUM_INP_REGS] = mid_pipe_ready[0];

  // Generate the register stages
  for (genvar i = 0; i < NUM_MID_REGS; i++) begin : gen_inside_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign mid_pipe_ready[i] = mid_pipe_ready[i+1] | ~mid_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(mid_pipe_valid_q[i+1], mid_pipe_valid_q[i], mid_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = mid_pipe_ready[i] & mid_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(mid_pipe_input_sign_q[i+1], mid_pipe_input_sign_q[i], reg_ena, '0)
    `FFL(mid_pipe_input_exp_q[i+1],  mid_pipe_input_exp_q[i],  reg_ena, '0)
    `FFL(mid_pipe_input_mant_q[i+1], mid_pipe_input_mant_q[i], reg_ena, '0)
    `FFL(mid_pipe_dest_exp_q[i+1],   mid_pipe_dest_exp_q[i],   reg_ena, '0)
    `FFL(mid_pipe_src_is_int_q[i+1], mid_pipe_src_is_int_q[i], reg_ena, '0)
    `FFL(mid_pipe_dst_is_int_q[i+1], mid_pipe_dst_is_int_q[i], reg_ena, '0)
    `FFL(mid_pipe_info_q[i+1],       mid_pipe_info_q[i],       reg_ena, '0)
    `FFL(mid_pipe_mant_zero_q[i+1],  mid_pipe_mant_zero_q[i],  reg_ena, '0)
    `FFL(mid_pipe_op_mod_q[i+1],     mid_pipe_op_mod_q[i],     reg_ena, '0)
    `FFL(mid_pipe_rnd_mode_q[i+1],   mid_pipe_rnd_mode_q[i],   reg_ena, fpnew_pkg::RNE)
    `FFL(mid_pipe_src_fmt_q[i+1],    mid_pipe_src_fmt_q[i],    reg_ena, fpnew_pkg::fp_format_e'(0))
    `FFL(mid_pipe_dst_fmt_q[i+1],    mid_pipe_dst_fmt_q[i],    reg_ena, fpnew_pkg::fp_format_e'(0))
    `FFL(mid_pipe_int_fmt_q[i+1],    mid_pipe_int_fmt_q[i],    reg_ena, fpnew_pkg::int_format_e'(0))
    `FFL(mid_pipe_tag_q[i+1],        mid_pipe_tag_q[i],        reg_ena, TagType'('0))
    `FFL(mid_pipe_aux_q[i+1],        mid_pipe_aux_q[i],        reg_ena, AuxType'('0))
  end
  // Output stage: assign selected pipe outputs to signals for later use
  assign input_sign_q      = mid_pipe_input_sign_q[NUM_MID_REGS];
  assign input_exp_q       = mid_pipe_input_exp_q[NUM_MID_REGS];
  assign input_mant_q      = mid_pipe_input_mant_q[NUM_MID_REGS];
  assign destination_exp_q = mid_pipe_dest_exp_q[NUM_MID_REGS];
  assign src_is_int_q      = mid_pipe_src_is_int_q[NUM_MID_REGS];
  assign dst_is_int_q      = mid_pipe_dst_is_int_q[NUM_MID_REGS];
  assign info_q            = mid_pipe_info_q[NUM_MID_REGS];
  assign mant_is_zero_q    = mid_pipe_mant_zero_q[NUM_MID_REGS];
  assign op_mod_q2         = mid_pipe_op_mod_q[NUM_MID_REGS];
  assign rnd_mode_q        = mid_pipe_rnd_mode_q[NUM_MID_REGS];
  assign src_fmt_q2        = mid_pipe_src_fmt_q[NUM_MID_REGS];
  assign dst_fmt_q2        = mid_pipe_dst_fmt_q[NUM_MID_REGS];
  assign int_fmt_q2        = mid_pipe_int_fmt_q[NUM_MID_REGS];

  // --------
  // Casting
  // --------
  logic [INT_EXP_WIDTH-1:0] final_exp;        // after eventual adjustments

  logic [2*INT_MAN_WIDTH:0]  preshift_mant;    // mantissa before final shift
  logic [2*INT_MAN_WIDTH:0]  destination_mant; // mantissa from shifter, with rnd bit
  logic [SUPER_MAN_BITS-1:0] final_mant;       // mantissa after adjustments
  logic [MAX_INT_WIDTH-1:0]  final_int;        // integer shifted in position

  logic [$clog2(INT_MAN_WIDTH+1)-1:0] denorm_shamt; // shift amount for denormalization

  logic [1:0] fp_round_sticky_bits, int_round_sticky_bits, round_sticky_bits;
  logic       of_before_round, uf_before_round;


  // Perform adjustments to mantissa and exponent
  always_comb begin : cast_value
    // Default assignment
    final_exp       = unsigned'(destination_exp_q); // take exponent as is, only look at lower bits
    preshift_mant   = '0;  // initialize mantissa container with zeroes
    denorm_shamt    = SUPER_MAN_BITS - fpnew_pkg::man_bits(dst_fmt_q2); // right of mantissa
    of_before_round = 1'b0;
    uf_before_round = 1'b0;

    // Place mantissa to the left of the shifter
    preshift_mant = input_mant_q << (INT_MAN_WIDTH + 1);

    // Handle INT casts
    if (dst_is_int_q) begin
      // By default right shift mantissa to be an integer
      denorm_shamt = unsigned'(MAX_INT_WIDTH - 1 - input_exp_q);
      // overflow: when converting to unsigned the range is larger by one
      if (input_exp_q >= signed'(fpnew_pkg::int_width(int_fmt_q2) - 1 + op_mod_q2)) begin
        denorm_shamt    = '0; // prevent shifting
        of_before_round = 1'b1;
      // underflow
      end else if (input_exp_q < -1) begin
        denorm_shamt    = MAX_INT_WIDTH + 1; // all bits go to the sticky
        uf_before_round = 1'b1;
      end
    // Handle FP over-/underflows
    end else begin
      // Overflow or infinities (for proper rounding)
      if ((destination_exp_q >= signed'(2**fpnew_pkg::exp_bits(dst_fmt_q2))-1) ||
          (~src_is_int_q && info_q.is_inf)) begin
        final_exp       = unsigned'(2**fpnew_pkg::exp_bits(dst_fmt_q2)-2); // largest normal value
        preshift_mant   = '1;                           // largest normal value and RS bits set
        of_before_round = 1'b1;
      // Denormalize underflowing values
      end else if (destination_exp_q < 1 &&
                   destination_exp_q >= -signed'(fpnew_pkg::man_bits(dst_fmt_q2))) begin
        final_exp       = '0; // denormal result
        denorm_shamt    = unsigned'(denorm_shamt + 1 - destination_exp_q); // adjust right shifting
        uf_before_round = 1'b1;
      // Limit the shift to retain sticky bits
      end else if (destination_exp_q < -signed'(fpnew_pkg::man_bits(dst_fmt_q2))) begin
        final_exp       = '0; // denormal result
        denorm_shamt    = unsigned'(denorm_shamt + 2 + fpnew_pkg::man_bits(dst_fmt_q2)); // to sticky
        uf_before_round = 1'b1;
      end
    end
  end

  localparam NUM_FP_STICKY  = 2 * INT_MAN_WIDTH - SUPER_MAN_BITS - 1; // removed mantissa, 1. and R
  localparam NUM_INT_STICKY = 2 * INT_MAN_WIDTH - MAX_INT_WIDTH; // removed int and R

  // Mantissa adjustment shift
  assign destination_mant = preshift_mant >> denorm_shamt;
  // Extract final mantissa and round bit, discard the normal bit (for FP)
  assign {final_mant, fp_round_sticky_bits[1]} =
      destination_mant[2*INT_MAN_WIDTH-1-:SUPER_MAN_BITS+1];
  assign {final_int, int_round_sticky_bits[1]} = destination_mant[2*INT_MAN_WIDTH-:MAX_INT_WIDTH+1];
  // Collapse sticky bits
  assign fp_round_sticky_bits[0]  = (| {destination_mant[NUM_FP_STICKY-1:0]});
  assign int_round_sticky_bits[0] = (| {destination_mant[NUM_INT_STICKY-1:0]});

  // select RS bits for destination operation
  assign round_sticky_bits = dst_is_int_q ? int_round_sticky_bits : fp_round_sticky_bits;

  // ----------------------------
  // Rounding and classification
  // ----------------------------
  logic [WIDTH-1:0] pre_round_abs;  // absolute value of result before rnd
  logic             of_after_round; // overflow
  logic             uf_after_round; // underflow

  logic [NUM_FORMATS-1:0][WIDTH-1:0] fmt_pre_round_abs; // per format
  logic [NUM_FORMATS-1:0]            fmt_of_after_round;
  logic [NUM_FORMATS-1:0]            fmt_uf_after_round;

  logic [NUM_INT_FORMATS-1:0][WIDTH-1:0] ifmt_pre_round_abs; // per format

  logic             rounded_sign;
  logic [WIDTH-1:0] rounded_abs; // absolute value of result after rounding
  logic             result_true_zero;

  logic [WIDTH-1:0] rounded_int_res; // after possible inversion
  logic             rounded_int_res_zero; // after rounding


  // Pack exponent and mantissa into proper rounding form
  for (genvar fmt = 0; fmt < int'(NUM_FORMATS); fmt++) begin : gen_res_assemble
    // Set up some constants
    localparam int unsigned EXP_BITS = fpnew_pkg::exp_bits(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned MAN_BITS = fpnew_pkg::man_bits(fpnew_pkg::fp_format_e'(fmt));

    if (FpFmtConfig[fmt]) begin : active_format
      always_comb begin : assemble_result
        fmt_pre_round_abs[fmt] = {final_exp[EXP_BITS-1:0], final_mant[MAN_BITS-1:0]}; // 0-extend
      end
    end else begin : inactive_format
      assign fmt_pre_round_abs[fmt] = '{default: fpnew_pkg::DONT_CARE};
    end
  end

  // Sign-extend integer result
  for (genvar ifmt = 0; ifmt < int'(NUM_INT_FORMATS); ifmt++) begin : gen_int_res_sign_ext
    // Set up some constants
    localparam int unsigned INT_WIDTH = fpnew_pkg::int_width(fpnew_pkg::int_format_e'(ifmt));

    if (IntFmtConfig[ifmt]) begin : active_format
      always_comb begin : assemble_result
        // sign-extend reusult
        ifmt_pre_round_abs[ifmt]                = '{default: final_int[INT_WIDTH-1]};
        ifmt_pre_round_abs[ifmt][INT_WIDTH-1:0] = final_int[INT_WIDTH-1:0];
      end
    end else begin : inactive_format
      assign ifmt_pre_round_abs[ifmt] = '{default: fpnew_pkg::DONT_CARE};
    end
  end

  // Select output with destination format and operation
  assign pre_round_abs = dst_is_int_q ? ifmt_pre_round_abs[int_fmt_q2] : fmt_pre_round_abs[dst_fmt_q2];

  fpnew_rounding #(
    .AbsWidth ( WIDTH )
  ) i_fpnew_rounding (
    .abs_value_i             ( pre_round_abs     ),
    .sign_i                  ( input_sign_q      ), // source format
    .round_sticky_bits_i     ( round_sticky_bits ),
    .rnd_mode_i              ( rnd_mode_q        ),
    .effective_subtraction_i ( 1'b0              ), // no operation happened
    .abs_rounded_o           ( rounded_abs       ),
    .sign_o                  ( rounded_sign      ),
    .exact_zero_o            ( result_true_zero  )
  );

  logic [NUM_FORMATS-1:0][WIDTH-1:0] fmt_result;

  // Detect overflows and inject sign
  for (genvar fmt = 0; fmt < int'(NUM_FORMATS); fmt++) begin : gen_sign_inject
    // Set up some constants
    localparam int unsigned FP_WIDTH = fpnew_pkg::fp_width(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned EXP_BITS = fpnew_pkg::exp_bits(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned MAN_BITS = fpnew_pkg::man_bits(fpnew_pkg::fp_format_e'(fmt));

    if (FpFmtConfig[fmt]) begin : active_format
      always_comb begin : post_process
        // detect of / uf
        fmt_uf_after_round[fmt] = rounded_abs[EXP_BITS+MAN_BITS-1:MAN_BITS] == '0; // denormal
        fmt_of_after_round[fmt] = rounded_abs[EXP_BITS+MAN_BITS-1:MAN_BITS] == '1; // inf exp.

        // Assemble regular result, nan box short ones. Int zeroes need to be detected`
        fmt_result[fmt]               = '1;
        fmt_result[fmt][FP_WIDTH-1:0] = src_is_int_q & mant_is_zero_q
                                        ? '0
                                        : {rounded_sign, rounded_abs[EXP_BITS+MAN_BITS-1:0]};
      end
    end else begin : inactive_format
      assign fmt_uf_after_round[fmt] = fpnew_pkg::DONT_CARE;
      assign fmt_of_after_round[fmt] = fpnew_pkg::DONT_CARE;
      assign fmt_result[fmt]         = '{default: fpnew_pkg::DONT_CARE};
    end
  end

  // Classification after rounding select by destination format
  assign uf_after_round = fmt_uf_after_round[dst_fmt_q2];
  assign of_after_round = fmt_of_after_round[dst_fmt_q2];

  // Negative integer result needs to be brought into two's complement
  assign rounded_int_res      = rounded_sign ? unsigned'(-rounded_abs) : rounded_abs;
  assign rounded_int_res_zero = (rounded_int_res == '0);

  // -------------------------
  // FP Special case handling
  // -------------------------
  logic [WIDTH-1:0]   fp_special_result;
  fpnew_pkg::status_t fp_special_status;
  logic               fp_result_is_special;

  logic [NUM_FORMATS-1:0][WIDTH-1:0] fmt_special_result;

  // Special result construction
  for (genvar fmt = 0; fmt < int'(NUM_FORMATS); fmt++) begin : gen_special_results
    // Set up some constants
    localparam int unsigned FP_WIDTH = fpnew_pkg::fp_width(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned EXP_BITS = fpnew_pkg::exp_bits(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned MAN_BITS = fpnew_pkg::man_bits(fpnew_pkg::fp_format_e'(fmt));

    localparam logic [EXP_BITS-1:0] QNAN_EXPONENT = '1;
    localparam logic [MAN_BITS-1:0] QNAN_MANTISSA = 2**(MAN_BITS-1);

    if (FpFmtConfig[fmt]) begin : active_format
      always_comb begin : special_results
        logic [FP_WIDTH-1:0] special_res;
        special_res = info_q.is_zero
                      ? input_sign_q << FP_WIDTH-1 // signed zero
                      : {1'b0, QNAN_EXPONENT, QNAN_MANTISSA}; // qNaN

        // Initialize special result with ones (NaN-box)
        fmt_special_result[fmt]               = '1;
        fmt_special_result[fmt][FP_WIDTH-1:0] = special_res;
      end
    end else begin : inactive_format
      assign fmt_special_result[fmt] = '{default: fpnew_pkg::DONT_CARE};
    end
  end

  // Detect special case from source format, I2F casts don't produce a special result
  assign fp_result_is_special = ~src_is_int_q & (info_q.is_zero |
                                                 info_q.is_nan |
                                                 ~info_q.is_boxed);

  // Signalling input NaNs raise invalid flag, otherwise no flags set
  assign fp_special_status = '{NV: info_q.is_signalling, default: 1'b0};

  // Assemble result according to destination format
  assign fp_special_result = fmt_special_result[dst_fmt_q2]; // destination format

  // --------------------------
  // INT Special case handling
  // --------------------------
  logic [WIDTH-1:0]   int_special_result;
  fpnew_pkg::status_t int_special_status;
  logic               int_result_is_special;

  logic [NUM_INT_FORMATS-1:0][WIDTH-1:0] ifmt_special_result;

  // Special result construction
  for (genvar ifmt = 0; ifmt < int'(NUM_INT_FORMATS); ifmt++) begin : gen_special_results_int
    // Set up some constants
    localparam int unsigned INT_WIDTH = fpnew_pkg::int_width(fpnew_pkg::int_format_e'(ifmt));

    if (IntFmtConfig[ifmt]) begin : active_format
      always_comb begin : special_results
        automatic logic [INT_WIDTH-1:0] special_res;

        // Default is overflow to positive max, which is 2**INT_WIDTH-1 or 2**(INT_WIDTH-1)-1
        special_res[INT_WIDTH-2:0] = '1;       // alone yields 2**(INT_WIDTH-1)-1
        special_res[INT_WIDTH-1]   = op_mod_q2; // for unsigned casts yields 2**INT_WIDTH-1

        // Negative special case (except for nans) tie to -max or 0
        if (input_sign_q && !info_q.is_nan)
          special_res = ~special_res;

        // Initialize special result with sign-extension
        ifmt_special_result[ifmt]                = '{default: special_res[INT_WIDTH-1]};
        ifmt_special_result[ifmt][INT_WIDTH-1:0] = special_res;
      end
    end else begin : inactive_format
      assign ifmt_special_result[ifmt] = '{default: fpnew_pkg::DONT_CARE};
    end
  end

  // Detect special case from source format (inf, nan, overflow, nan-boxing or negative unsigned)
  assign int_result_is_special = info_q.is_nan | info_q.is_inf |
                                 of_before_round | ~info_q.is_boxed |
                                 (input_sign_q & op_mod_q2 & ~rounded_int_res_zero);

  // All integer special cases are invalid
  assign int_special_status = '{NV: 1'b1, default: 1'b0};

  // Assemble result according to destination format
  assign int_special_result = ifmt_special_result[int_fmt_q2]; // destination format

  // -----------------
  // Result selection
  // -----------------
  fpnew_pkg::status_t int_regular_status, fp_regular_status;

  logic [WIDTH-1:0]   fp_result, int_result;
  fpnew_pkg::status_t fp_status, int_status;

  assign fp_regular_status.NV = src_is_int_q & (of_before_round | of_after_round); // overflow is invalid for I2F casts
  assign fp_regular_status.DZ = 1'b0; // no divisions
  assign fp_regular_status.OF = ~src_is_int_q & (~info_q.is_inf & (of_before_round | of_after_round)); // inf casts no OF
  assign fp_regular_status.UF = uf_after_round & fp_regular_status.NX;
  assign fp_regular_status.NX = src_is_int_q ? (| fp_round_sticky_bits) // overflow is invalid in i2f
            : (| fp_round_sticky_bits) | (~info_q.is_inf & (of_before_round | of_after_round));
  assign int_regular_status = '{NX: (| int_round_sticky_bits), default: 1'b0};

  assign fp_result  = fp_result_is_special  ? fp_special_result  : fmt_result[dst_fmt_q2];
  assign fp_status  = fp_result_is_special  ? fp_special_status  : fp_regular_status;
  assign int_result = int_result_is_special ? int_special_result : rounded_int_res;
  assign int_status = int_result_is_special ? int_special_status : int_regular_status;

  // Final results for output pipeline
  logic [WIDTH-1:0]   result_d;
  fpnew_pkg::status_t status_d;
  logic               extension_bit;

  // Select output depending on special case detection
  assign result_d = dst_is_int_q ? int_result : fp_result;
  assign status_d = dst_is_int_q ? int_status : fp_status;

  // MSB of int result decides extension, otherwise NaN box
  assign extension_bit = dst_is_int_q ? int_result[WIDTH-1] : 1'b1;

  // ----------------
  // Output Pipeline
  // ----------------
  // Output pipeline signals, index i holds signal after i register stages
  logic               [0:NUM_OUT_REGS][WIDTH-1:0] out_pipe_result_q;
  fpnew_pkg::status_t [0:NUM_OUT_REGS]            out_pipe_status_q;
  logic               [0:NUM_OUT_REGS]            out_pipe_ext_bit_q;
  TagType             [0:NUM_OUT_REGS]            out_pipe_tag_q;
  AuxType             [0:NUM_OUT_REGS]            out_pipe_aux_q;
  logic               [0:NUM_OUT_REGS]            out_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_OUT_REGS] out_pipe_ready;

  // Input stage: First element of pipeline is taken from inputs
  assign out_pipe_result_q[0]  = result_d;
  assign out_pipe_status_q[0]  = status_d;
  assign out_pipe_ext_bit_q[0] = extension_bit;
  assign out_pipe_tag_q[0]     = mid_pipe_tag_q[NUM_MID_REGS];
  assign out_pipe_aux_q[0]     = mid_pipe_aux_q[NUM_MID_REGS];
  assign out_pipe_valid_q[0]   = mid_pipe_valid_q[NUM_MID_REGS];
  // Input stage: Propagate pipeline ready signal to inside pipe
  assign mid_pipe_ready[NUM_MID_REGS] = out_pipe_ready[0];
  // Generate the register stages
  for (genvar i = 0; i < NUM_OUT_REGS; i++) begin : gen_output_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign out_pipe_ready[i] = out_pipe_ready[i+1] | ~out_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(out_pipe_valid_q[i+1], out_pipe_valid_q[i], out_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = out_pipe_ready[i] & out_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(out_pipe_result_q[i+1],  out_pipe_result_q[i],  reg_ena, '0)
    `FFL(out_pipe_status_q[i+1],  out_pipe_status_q[i],  reg_ena, '0)
    `FFL(out_pipe_ext_bit_q[i+1], out_pipe_ext_bit_q[i], reg_ena, '0)
    `FFL(out_pipe_tag_q[i+1],     out_pipe_tag_q[i],     reg_ena, TagType'('0))
    `FFL(out_pipe_aux_q[i+1],     out_pipe_aux_q[i],     reg_ena, AuxType'('0))
  end
  // Output stage: Ready travels backwards from output side, driven by downstream circuitry
  assign out_pipe_ready[NUM_OUT_REGS] = out_ready_i;
  // Output stage: assign module outputs
  assign result_o        = out_pipe_result_q[NUM_OUT_REGS];
  assign status_o        = out_pipe_status_q[NUM_OUT_REGS];
  assign extension_bit_o = out_pipe_ext_bit_q[NUM_OUT_REGS];
  assign tag_o           = out_pipe_tag_q[NUM_OUT_REGS];
  assign aux_o           = out_pipe_aux_q[NUM_OUT_REGS];
  assign out_valid_o     = out_pipe_valid_q[NUM_OUT_REGS];
  assign busy_o          = (| {inp_pipe_valid_q, mid_pipe_valid_q, out_pipe_valid_q});
endmodule
// Copyright 2019 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Stefan Mach <smach@iis.ee.ethz.ch>

module fpnew_classifier #(
  parameter fpnew_pkg::fp_format_e   FpFormat = fpnew_pkg::fp_format_e'(0),
  parameter int unsigned             NumOperands = 1,
  // Do not change
  localparam int unsigned WIDTH = fpnew_pkg::fp_width(FpFormat)
) (
  input  logic                [NumOperands-1:0][WIDTH-1:0] operands_i,
  input  logic                [NumOperands-1:0]            is_boxed_i,
  output fpnew_pkg::fp_info_t [NumOperands-1:0]            info_o
);

  localparam int unsigned EXP_BITS = fpnew_pkg::exp_bits(FpFormat);
  localparam int unsigned MAN_BITS = fpnew_pkg::man_bits(FpFormat);

  // Type definition
  typedef struct packed {
    logic                sign;
    logic [EXP_BITS-1:0] exponent;
    logic [MAN_BITS-1:0] mantissa;
  } fp_t;

  // Iterate through all operands
  for (genvar op = 0; op < int'(NumOperands); op++) begin : gen_num_values

    fp_t value;
    logic is_boxed;
    logic is_normal;
    logic is_inf;
    logic is_nan;
    logic is_signalling;
    logic is_quiet;
    logic is_zero;
    logic is_subnormal;

    // ---------------
    // Classify Input
    // ---------------
    always_comb begin : classify_input
      value         = operands_i[op];
      is_boxed      = is_boxed_i[op];
      is_normal     = is_boxed && (value.exponent != '0) && (value.exponent != '1);
      is_zero       = is_boxed && (value.exponent == '0) && (value.mantissa == '0);
      is_subnormal  = is_boxed && (value.exponent == '0) && !is_zero;
      is_inf        = is_boxed && ((value.exponent == '1) && (value.mantissa == '0));
      is_nan        = !is_boxed || ((value.exponent == '1) && (value.mantissa != '0));
      is_signalling = is_boxed && is_nan && (value.mantissa[MAN_BITS-1] == 1'b0);
      is_quiet      = is_nan && !is_signalling;
      // Assign output for current input
      info_o[op].is_normal     = is_normal;
      info_o[op].is_subnormal  = is_subnormal;
      info_o[op].is_zero       = is_zero;
      info_o[op].is_inf        = is_inf;
      info_o[op].is_nan        = is_nan;
      info_o[op].is_signalling = is_signalling;
      info_o[op].is_quiet      = is_quiet;
      info_o[op].is_boxed      = is_boxed;
    end
  end
endmodule
// Copyright 2019 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Stefan Mach <smach@iis.ee.ethz.ch>

//`include "common_cells/registers.svh"

module fpnew_divsqrt_multi #(
  parameter fpnew_pkg::fmt_logic_t   FpFmtConfig  = '1,
  // FPU configuration
  parameter int unsigned             NumPipeRegs = 0,
  parameter fpnew_pkg::pipe_config_t PipeConfig  = fpnew_pkg::AFTER,
  parameter type                     TagType     = logic,
  parameter type                     AuxType     = logic,
  // Do not change
  localparam int unsigned WIDTH       = fpnew_pkg::max_fp_width(FpFmtConfig),
  localparam int unsigned NUM_FORMATS = fpnew_pkg::NUM_FP_FORMATS
) (
  input  logic                        clk_i,
  input  logic                        rst_ni,
  // Input signals
  input  logic [1:0][WIDTH-1:0]       operands_i, // 2 operands
  input  logic [NUM_FORMATS-1:0][1:0] is_boxed_i, // 2 operands
  input  fpnew_pkg::roundmode_e       rnd_mode_i,
  input  fpnew_pkg::operation_e       op_i,
  input  fpnew_pkg::fp_format_e       dst_fmt_i,
  input  TagType                      tag_i,
  input  AuxType                      aux_i,
  // Input Handshake
  input  logic                        in_valid_i,
  output logic                        in_ready_o,
  input  logic                        flush_i,
  // Output signals
  output logic [WIDTH-1:0]            result_o,
  output fpnew_pkg::status_t          status_o,
  output logic                        extension_bit_o,
  output TagType                      tag_o,
  output AuxType                      aux_o,
  // Output handshake
  output logic                        out_valid_o,
  input  logic                        out_ready_i,
  // Indication of valid data in flight
  output logic                        busy_o
);

  // ----------
  // Constants
  // ----------
  // Pipelines
  localparam NUM_INP_REGS = (PipeConfig == fpnew_pkg::BEFORE)
                            ? NumPipeRegs
                            : (PipeConfig == fpnew_pkg::DISTRIBUTED
                               ? (NumPipeRegs / 2) // Last to get distributed regs
                               : 0); // no regs here otherwise
  localparam NUM_OUT_REGS = (PipeConfig == fpnew_pkg::AFTER || PipeConfig == fpnew_pkg::INSIDE)
                            ? NumPipeRegs
                            : (PipeConfig == fpnew_pkg::DISTRIBUTED
                               ? ((NumPipeRegs + 1) / 2) // First to get distributed regs
                               : 0); // no regs here otherwise

  // ---------------
  // Input pipeline
  // ---------------
  // Selected pipeline output signals as non-arrays
  logic [1:0][WIDTH-1:0] operands_q;
  fpnew_pkg::roundmode_e rnd_mode_q;
  fpnew_pkg::operation_e op_q;
  fpnew_pkg::fp_format_e dst_fmt_q;
  logic                  in_valid_q;

  // Input pipeline signals, index i holds signal after i register stages
  logic                  [0:NUM_INP_REGS][1:0][WIDTH-1:0]       inp_pipe_operands_q;
  fpnew_pkg::roundmode_e [0:NUM_INP_REGS]                       inp_pipe_rnd_mode_q;
  fpnew_pkg::operation_e [0:NUM_INP_REGS]                       inp_pipe_op_q;
  fpnew_pkg::fp_format_e [0:NUM_INP_REGS]                       inp_pipe_dst_fmt_q;
  TagType                [0:NUM_INP_REGS]                       inp_pipe_tag_q;
  AuxType                [0:NUM_INP_REGS]                       inp_pipe_aux_q;
  logic                  [0:NUM_INP_REGS]                       inp_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_INP_REGS] inp_pipe_ready;

  // Input stage: First element of pipeline is taken from inputs
  assign inp_pipe_operands_q[0] = operands_i;
  assign inp_pipe_rnd_mode_q[0] = rnd_mode_i;
  assign inp_pipe_op_q[0]       = op_i;
  assign inp_pipe_dst_fmt_q[0]  = dst_fmt_i;
  assign inp_pipe_tag_q[0]      = tag_i;
  assign inp_pipe_aux_q[0]      = aux_i;
  assign inp_pipe_valid_q[0]    = in_valid_i;
  // Input stage: Propagate pipeline ready signal to updtream circuitry
  assign in_ready_o = inp_pipe_ready[0];
  // Generate the register stages
  for (genvar i = 0; i < NUM_INP_REGS; i++) begin : gen_input_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign inp_pipe_ready[i] = inp_pipe_ready[i+1] | ~inp_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(inp_pipe_valid_q[i+1], inp_pipe_valid_q[i], inp_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = inp_pipe_ready[i] & inp_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(inp_pipe_operands_q[i+1], inp_pipe_operands_q[i], reg_ena, '0)
    `FFL(inp_pipe_rnd_mode_q[i+1], inp_pipe_rnd_mode_q[i], reg_ena, fpnew_pkg::RNE)
    `FFL(inp_pipe_op_q[i+1],       inp_pipe_op_q[i],       reg_ena, fpnew_pkg::FMADD)
    `FFL(inp_pipe_dst_fmt_q[i+1],  inp_pipe_dst_fmt_q[i],  reg_ena, fpnew_pkg::fp_format_e'(0))
    `FFL(inp_pipe_tag_q[i+1],      inp_pipe_tag_q[i],      reg_ena, TagType'('0))
    `FFL(inp_pipe_aux_q[i+1],      inp_pipe_aux_q[i],      reg_ena, AuxType'('0))
  end
  // Output stage: assign selected pipe outputs to signals for later use
  assign operands_q = inp_pipe_operands_q[NUM_INP_REGS];
  assign rnd_mode_q = inp_pipe_rnd_mode_q[NUM_INP_REGS];
  assign op_q       = inp_pipe_op_q[NUM_INP_REGS];
  assign dst_fmt_q  = inp_pipe_dst_fmt_q[NUM_INP_REGS];
  assign in_valid_q = inp_pipe_valid_q[NUM_INP_REGS];

  // -----------------
  // Input processing
  // -----------------
  logic [1:0]       divsqrt_fmt;
  logic [1:0][63:0] divsqrt_operands; // those are fixed to 64bit
  logic             input_is_fp8;

  // Translate fpnew formats into divsqrt formats
  always_comb begin : translate_fmt
    unique case (dst_fmt_q)
      fpnew_pkg::FP32:    divsqrt_fmt = 2'b00;
      fpnew_pkg::FP64:    divsqrt_fmt = 2'b01;
      fpnew_pkg::FP16:    divsqrt_fmt = 2'b10;
      fpnew_pkg::FP16ALT: divsqrt_fmt = 2'b11;
      default:            divsqrt_fmt = 2'b10; // maps also FP8 to FP16
    endcase

    // Only if FP8 is enabled
    input_is_fp8 = FpFmtConfig[fpnew_pkg::FP8] & (dst_fmt_q == fpnew_pkg::FP8);

    // If FP8 is supported, map it to an FP16 value
    divsqrt_operands[0] = input_is_fp8 ? operands_q[0] << 8 : operands_q[0];
    divsqrt_operands[1] = input_is_fp8 ? operands_q[1] << 8 : operands_q[1];
  end

  // ------------
  // Control FSM
  // ------------
  logic in_ready;               // input handshake with upstream
  logic div_valid, sqrt_valid;  // input signalling with unit
  logic unit_ready, unit_done;  // status signals from unit instance
  logic op_starting;            // high in the cycle a new operation starts
  logic out_valid, out_ready;   // output handshake with downstream
  logic hold_result;            // whether to put result into hold register
  logic data_is_held;           // data in hold register is valid
  logic unit_busy;              // valid data in flight
  // FSM states
  typedef enum logic [1:0] {IDLE, BUSY, HOLD} fsm_state_e;
  fsm_state_e state_q, state_d;

  // Upstream ready comes from sanitization FSM
  assign inp_pipe_ready[NUM_INP_REGS] = in_ready;

  // Valids are gated by the FSM ready. Invalid input ops run a sqrt to not lose illegal instr.
  assign div_valid   = in_valid_q & (op_q == fpnew_pkg::DIV) & in_ready & ~flush_i;
  assign sqrt_valid  = in_valid_q & (op_q != fpnew_pkg::DIV) & in_ready & ~flush_i;
  assign op_starting = div_valid | sqrt_valid;

  // FSM to safely apply and receive data from DIVSQRT unit
  always_comb begin : flag_fsm
    // Default assignments
    in_ready     = 1'b0;
    out_valid    = 1'b0;
    hold_result  = 1'b0;
    data_is_held = 1'b0;
    unit_busy    = 1'b0;
    state_d      = state_q;

    unique case (state_q)
      // Waiting for work
      IDLE: begin
        in_ready = 1'b1; // we're ready
        if (in_valid_q && unit_ready) begin // New work arrives
          state_d = BUSY; // go into processing state
        end
      end
      // Operation in progress
      BUSY: begin
        unit_busy = 1'b1; // data in flight
        // If the unit is done with processing
        if (unit_done) begin
          out_valid = 1'b1; // try to commit result downstream
          // If downstream accepts our result
          if (out_ready) begin
            state_d = IDLE; // we anticipate going back to idling..
            if (in_valid_q && unit_ready) begin // ..unless new work comes in
              in_ready = 1'b1; // we acknowledge the instruction
              state_d  = BUSY; // and stay busy with it
            end
          // Otherwise if downstream is not ready for the result
          end else begin
            hold_result = 1'b1; // activate the hold register
            state_d     = HOLD; // wait for the pipeline to take the data
          end
        end
      end
      // Waiting with valid result for downstream
      HOLD: begin
        unit_busy    = 1'b1; // data in flight
        data_is_held = 1'b1; // data in hold register is valid
        out_valid    = 1'b1; // try to commit result downstream
        // If the result is accepted by downstream
        if (out_ready) begin
          state_d = IDLE; // go back to idle..
          if (in_valid_q && unit_ready) begin // ..unless new work comes in
            in_ready = 1'b1; // acknowledge the new transaction
            state_d  = BUSY; // will be busy with the next instruction
          end
        end
      end
      // fall into idle state otherwise
      default: state_d = IDLE;
    endcase

    // Flushing overrides the other actions
    if (flush_i) begin
      unit_busy = 1'b0; // data is invalidated
      out_valid = 1'b0; // cancel any valid data
      state_d   = IDLE; // go to default state
    end
  end
  `define FF(__q, __d, __reset_value)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      __q <= (__reset_value);
    end else begin
      __q <= (__d);
    end
  end
  // FSM status register (asynch active low reset)
  `FF(state_q, state_d, IDLE)

  // Hold additional information while the operation is in progress
  logic result_is_fp8_q;
  TagType result_tag_q;
  AuxType result_aux_q;

  // Fill the registers everytime a valid operation arrives (load FF, active low asynch rst)
  `FFL(result_is_fp8_q, input_is_fp8,                 op_starting, '0)
  `FFL(result_tag_q,    inp_pipe_tag_q[NUM_INP_REGS], op_starting, '0)
  `FFL(result_aux_q,    inp_pipe_aux_q[NUM_INP_REGS], op_starting, '0)

  // -----------------
  // DIVSQRT instance
  // -----------------
  logic [63:0]        unit_result;
  logic [WIDTH-1:0]   adjusted_result, held_result_q;
  fpnew_pkg::status_t unit_status, held_status_q;

  div_sqrt_top_mvp i_divsqrt_lei (
   .Clk_CI           ( clk_i               ),
   .Rst_RBI          ( rst_ni              ),
   .Div_start_SI     ( div_valid           ),
   .Sqrt_start_SI    ( sqrt_valid          ),
   .Operand_a_DI     ( divsqrt_operands[0] ),
   .Operand_b_DI     ( divsqrt_operands[1] ),
   .RM_SI            ( rnd_mode_q          ),
   .Precision_ctl_SI ( '0                  ),
   .Format_sel_SI    ( divsqrt_fmt         ),
   .Kill_SI          ( flush_i             ),
   .Result_DO        ( unit_result         ),
   .Fflags_SO        ( unit_status         ),
   .Ready_SO         ( unit_ready          ),
   .Done_SO          ( unit_done           )
  );

  // Adjust result width and fix FP8
  assign adjusted_result = result_is_fp8_q ? unit_result >> 8 : unit_result;
  
  `define FFLNR(__q, __d, __load, __clk)
  always_ff @(posedge (__clk)) begin
    __q <= (__load) ? (__d) : (__q);
  end
  // The Hold register (load, no reset)
  `FFLNR(held_result_q, adjusted_result, hold_result, clk_i)
  `FFLNR(held_status_q, unit_status,     hold_result, clk_i)

  // --------------
  // Output Select
  // --------------
  logic [WIDTH-1:0]   result_d;
  fpnew_pkg::status_t status_d;
  // Prioritize hold register data
  assign result_d = data_is_held ? held_result_q : adjusted_result;
  assign status_d = data_is_held ? held_status_q : unit_status;

  // ----------------
  // Output Pipeline
  // ----------------
  // Output pipeline signals, index i holds signal after i register stages
  logic               [0:NUM_OUT_REGS][WIDTH-1:0] out_pipe_result_q;
  fpnew_pkg::status_t [0:NUM_OUT_REGS]            out_pipe_status_q;
  TagType             [0:NUM_OUT_REGS]            out_pipe_tag_q;
  AuxType             [0:NUM_OUT_REGS]            out_pipe_aux_q;
  logic               [0:NUM_OUT_REGS]            out_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_OUT_REGS] out_pipe_ready;

  // Input stage: First element of pipeline is taken from inputs
  assign out_pipe_result_q[0] = result_d;
  assign out_pipe_status_q[0] = status_d;
  assign out_pipe_tag_q[0]    = result_tag_q;
  assign out_pipe_aux_q[0]    = result_aux_q;
  assign out_pipe_valid_q[0]  = out_valid;
  // Input stage: Propagate pipeline ready signal to inside pipe
  assign out_ready = out_pipe_ready[0];
  // Generate the register stages
  for (genvar i = 0; i < NUM_OUT_REGS; i++) begin : gen_output_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign out_pipe_ready[i] = out_pipe_ready[i+1] | ~out_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(out_pipe_valid_q[i+1], out_pipe_valid_q[i], out_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = out_pipe_ready[i] & out_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(out_pipe_result_q[i+1], out_pipe_result_q[i], reg_ena, '0)
    `FFL(out_pipe_status_q[i+1], out_pipe_status_q[i], reg_ena, '0)
    `FFL(out_pipe_tag_q[i+1],    out_pipe_tag_q[i],    reg_ena, TagType'('0))
    `FFL(out_pipe_aux_q[i+1],    out_pipe_aux_q[i],    reg_ena, AuxType'('0))
  end
  // Output stage: Ready travels backwards from output side, driven by downstream circuitry
  assign out_pipe_ready[NUM_OUT_REGS] = out_ready_i;
  // Output stage: assign module outputs
  assign result_o        = out_pipe_result_q[NUM_OUT_REGS];
  assign status_o        = out_pipe_status_q[NUM_OUT_REGS];
  assign extension_bit_o = 1'b1; // always NaN-Box result
  assign tag_o           = out_pipe_tag_q[NUM_OUT_REGS];
  assign aux_o           = out_pipe_aux_q[NUM_OUT_REGS];
  assign out_valid_o     = out_pipe_valid_q[NUM_OUT_REGS];
  assign busy_o          = (| {inp_pipe_valid_q, unit_busy, out_pipe_valid_q});
endmodule
// Copyright 2019 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Stefan Mach <smach@iis.ee.ethz.ch>

//`include "common_cells/registers.svh"

module fpnew_fma_multi #(
  parameter fpnew_pkg::fmt_logic_t   FpFmtConfig = '1,
  parameter int unsigned             NumPipeRegs = 0,
  parameter fpnew_pkg::pipe_config_t PipeConfig  = fpnew_pkg::BEFORE,
  parameter type                     TagType     = logic,
  parameter type                     AuxType     = logic,
  // Do not change
  localparam int unsigned WIDTH       = fpnew_pkg::max_fp_width(FpFmtConfig),
  localparam int unsigned NUM_FORMATS = fpnew_pkg::NUM_FP_FORMATS
) (
  input  logic                        clk_i,
  input  logic                        rst_ni,
  // Input signals
  input  logic [2:0][WIDTH-1:0]       operands_i, // 3 operands
  input  logic [NUM_FORMATS-1:0][2:0] is_boxed_i, // 3 operands
  input  fpnew_pkg::roundmode_e       rnd_mode_i,
  input  fpnew_pkg::operation_e       op_i,
  input  logic                        op_mod_i,
  input  fpnew_pkg::fp_format_e       src_fmt_i, // format of the multiplicands
  input  fpnew_pkg::fp_format_e       dst_fmt_i, // format of the addend and result
  input  TagType                      tag_i,
  input  AuxType                      aux_i,
  // Input Handshake
  input  logic                        in_valid_i,
  output logic                        in_ready_o,
  input  logic                        flush_i,
  // Output signals
  output logic [WIDTH-1:0]            result_o,
  output fpnew_pkg::status_t          status_o,
  output logic                        extension_bit_o,
  output TagType                      tag_o,
  output AuxType                      aux_o,
  // Output handshake
  output logic                        out_valid_o,
  input  logic                        out_ready_i,
  // Indication of valid data in flight
  output logic                        busy_o
);

  // ----------
  // Constants
  // ----------
  // The super-format that can hold all formats
  localparam fpnew_pkg::fp_encoding_t SUPER_FORMAT = fpnew_pkg::super_format(FpFmtConfig);

  localparam int unsigned SUPER_EXP_BITS = SUPER_FORMAT.exp_bits;
  localparam int unsigned SUPER_MAN_BITS = SUPER_FORMAT.man_bits;

  // Precision bits 'p' include the implicit bit
  localparam int unsigned PRECISION_BITS = SUPER_MAN_BITS + 1;
  // The lower 2p+3 bits of the internal FMA result will be needed for leading-zero detection
  localparam int unsigned LOWER_SUM_WIDTH  = 2 * PRECISION_BITS + 3;
  localparam int unsigned LZC_RESULT_WIDTH = $clog2(LOWER_SUM_WIDTH);
  // Internal exponent width of FMA must accomodate all meaningful exponent values in order to avoid
  // datapath leakage. This is either given by the exponent bits or the width of the LZC result.
  // In most reasonable FP formats the internal exponent will be wider than the LZC result.
  localparam int unsigned EXP_WIDTH = fpnew_pkg::maximum(SUPER_EXP_BITS + 2, LZC_RESULT_WIDTH);
  // Shift amount width: maximum internal mantissa size is 3p+3 bits
  localparam int unsigned SHIFT_AMOUNT_WIDTH = $clog2(3 * PRECISION_BITS + 3);
  // Pipelines
  localparam NUM_INP_REGS = PipeConfig == fpnew_pkg::BEFORE
                            ? NumPipeRegs
                            : (PipeConfig == fpnew_pkg::DISTRIBUTED
                               ? ((NumPipeRegs + 1) / 3) // Second to get distributed regs
                               : 0); // no regs here otherwise
  localparam NUM_MID_REGS = PipeConfig == fpnew_pkg::INSIDE
                          ? NumPipeRegs
                          : (PipeConfig == fpnew_pkg::DISTRIBUTED
                             ? ((NumPipeRegs + 2) / 3) // First to get distributed regs
                             : 0); // no regs here otherwise
  localparam NUM_OUT_REGS = PipeConfig == fpnew_pkg::AFTER
                            ? NumPipeRegs
                            : (PipeConfig == fpnew_pkg::DISTRIBUTED
                               ? (NumPipeRegs / 3) // Last to get distributed regs
                               : 0); // no regs here otherwise

  // ----------------
  // Type definition
  // ----------------
  typedef struct packed {
    logic                      sign;
    logic [SUPER_EXP_BITS-1:0] exponent;
    logic [SUPER_MAN_BITS-1:0] mantissa;
  } fp_t;

  // ---------------
  // Input pipeline
  // ---------------
  // Selected pipeline output signals as non-arrays
  logic [2:0][WIDTH-1:0] operands_q;
  fpnew_pkg::fp_format_e src_fmt_q;
  fpnew_pkg::fp_format_e dst_fmt_q;

  // Input pipeline signals, index i holds signal after i register stages
  logic                  [0:NUM_INP_REGS][2:0][WIDTH-1:0]       inp_pipe_operands_q;
  logic                  [0:NUM_INP_REGS][NUM_FORMATS-1:0][2:0] inp_pipe_is_boxed_q;
  fpnew_pkg::roundmode_e [0:NUM_INP_REGS]                       inp_pipe_rnd_mode_q;
  fpnew_pkg::operation_e [0:NUM_INP_REGS]                       inp_pipe_op_q;
  logic                  [0:NUM_INP_REGS]                       inp_pipe_op_mod_q;
  fpnew_pkg::fp_format_e [0:NUM_INP_REGS]                       inp_pipe_src_fmt_q;
  fpnew_pkg::fp_format_e [0:NUM_INP_REGS]                       inp_pipe_dst_fmt_q;
  TagType                [0:NUM_INP_REGS]                       inp_pipe_tag_q;
  AuxType                [0:NUM_INP_REGS]                       inp_pipe_aux_q;
  logic                  [0:NUM_INP_REGS]                       inp_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_INP_REGS] inp_pipe_ready;

  // Input stage: First element of pipeline is taken from inputs
  assign inp_pipe_operands_q[0] = operands_i;
  assign inp_pipe_is_boxed_q[0] = is_boxed_i;
  assign inp_pipe_rnd_mode_q[0] = rnd_mode_i;
  assign inp_pipe_op_q[0]       = op_i;
  assign inp_pipe_op_mod_q[0]   = op_mod_i;
  assign inp_pipe_src_fmt_q[0]  = src_fmt_i;
  assign inp_pipe_dst_fmt_q[0]  = dst_fmt_i;
  assign inp_pipe_tag_q[0]      = tag_i;
  assign inp_pipe_aux_q[0]      = aux_i;
  assign inp_pipe_valid_q[0]    = in_valid_i;
  // Input stage: Propagate pipeline ready signal to updtream circuitry
  assign in_ready_o = inp_pipe_ready[0];
  // Generate the register stages
  for (genvar i = 0; i < NUM_INP_REGS; i++) begin : gen_input_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign inp_pipe_ready[i] = inp_pipe_ready[i+1] | ~inp_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(inp_pipe_valid_q[i+1], inp_pipe_valid_q[i], inp_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = inp_pipe_ready[i] & inp_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(inp_pipe_operands_q[i+1], inp_pipe_operands_q[i], reg_ena, '0)
    `FFL(inp_pipe_is_boxed_q[i+1], inp_pipe_is_boxed_q[i], reg_ena, '0)
    `FFL(inp_pipe_rnd_mode_q[i+1], inp_pipe_rnd_mode_q[i], reg_ena, fpnew_pkg::RNE)
    `FFL(inp_pipe_op_q[i+1],       inp_pipe_op_q[i],       reg_ena, fpnew_pkg::FMADD)
    `FFL(inp_pipe_op_mod_q[i+1],   inp_pipe_op_mod_q[i],   reg_ena, '0)
    `FFL(inp_pipe_src_fmt_q[i+1],  inp_pipe_src_fmt_q[i],  reg_ena, fpnew_pkg::fp_format_e'(0))
    `FFL(inp_pipe_dst_fmt_q[i+1],  inp_pipe_dst_fmt_q[i],  reg_ena, fpnew_pkg::fp_format_e'(0))
    `FFL(inp_pipe_tag_q[i+1],      inp_pipe_tag_q[i],      reg_ena, TagType'('0))
    `FFL(inp_pipe_aux_q[i+1],      inp_pipe_aux_q[i],      reg_ena, AuxType'('0))
  end
  // Output stage: assign selected pipe outputs to signals for later use
  assign operands_q = inp_pipe_operands_q[NUM_INP_REGS];
  assign src_fmt_q  = inp_pipe_src_fmt_q[NUM_INP_REGS];
  assign dst_fmt_q  = inp_pipe_dst_fmt_q[NUM_INP_REGS];

  // -----------------
  // Input processing
  // -----------------
  logic        [NUM_FORMATS-1:0][2:0]                     fmt_sign;
  logic signed [NUM_FORMATS-1:0][2:0][SUPER_EXP_BITS-1:0] fmt_exponent;
  logic        [NUM_FORMATS-1:0][2:0][SUPER_MAN_BITS-1:0] fmt_mantissa;

  fpnew_pkg::fp_info_t [NUM_FORMATS-1:0][2:0] info_q;

  // FP Input initialization
  for (genvar fmt = 0; fmt < int'(NUM_FORMATS); fmt++) begin : fmt_init_inputs
    // Set up some constants
    localparam int unsigned FP_WIDTH = fpnew_pkg::fp_width(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned EXP_BITS = fpnew_pkg::exp_bits(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned MAN_BITS = fpnew_pkg::man_bits(fpnew_pkg::fp_format_e'(fmt));

    if (FpFmtConfig[fmt]) begin : active_format
      logic [2:0][FP_WIDTH-1:0] trimmed_ops;

      // Classify input
      fpnew_classifier #(
        .FpFormat    ( fpnew_pkg::fp_format_e'(fmt) ),
        .NumOperands ( 3                            )
      ) i_fpnew_classifier (
        .operands_i ( trimmed_ops                            ),
        .is_boxed_i ( inp_pipe_is_boxed_q[NUM_INP_REGS][fmt] ),
        .info_o     ( info_q[fmt]                            )
      );
      for (genvar op = 0; op < 3; op++) begin : gen_operands
        assign trimmed_ops[op]       = operands_q[op][FP_WIDTH-1:0];
        assign fmt_sign[fmt][op]     = operands_q[op][FP_WIDTH-1];
        assign fmt_exponent[fmt][op] = signed'({1'b0, operands_q[op][MAN_BITS+:EXP_BITS]});
        assign fmt_mantissa[fmt][op] = {info_q[fmt][op].is_normal, operands_q[op][MAN_BITS-1:0]} <<
                                       (SUPER_MAN_BITS - MAN_BITS); // move to left of mantissa
      end
    end else begin : inactive_format
      assign info_q[fmt]                 = '{default: fpnew_pkg::DONT_CARE}; // format disabled
      assign fmt_sign[fmt]               = fpnew_pkg::DONT_CARE;             // format disabled
      assign fmt_exponent[fmt]           = '{default: fpnew_pkg::DONT_CARE}; // format disabled
      assign fmt_mantissa[fmt]           = '{default: fpnew_pkg::DONT_CARE}; // format disabled
    end
  end

  fp_t                 operand_a, operand_b, operand_c;
  fpnew_pkg::fp_info_t info_a,    info_b,    info_c;

  // Operation selection and operand adjustment
  // | \c op_q  | \c op_mod_q | Operation Adjustment
  // |:--------:|:-----------:|---------------------
  // | FMADD    | \c 0        | FMADD: none
  // | FMADD    | \c 1        | FMSUB: Invert sign of operand C
  // | FNMSUB   | \c 0        | FNMSUB: Invert sign of operand A
  // | FNMSUB   | \c 1        | FNMADD: Invert sign of operands A and C
  // | ADD      | \c 0        | ADD: Set operand A to +1.0
  // | ADD      | \c 1        | SUB: Set operand A to +1.0, invert sign of operand C
  // | MUL      | \c 0        | MUL: Set operand C to +0.0
  // | *others* | \c -        | *invalid*
  // \note \c op_mod_q always inverts the sign of the addend.
  always_comb begin : op_select

    // Default assignments - packing-order-agnostic
    operand_a = {fmt_sign[src_fmt_q][0], fmt_exponent[src_fmt_q][0], fmt_mantissa[src_fmt_q][0]};
    operand_b = {fmt_sign[src_fmt_q][1], fmt_exponent[src_fmt_q][1], fmt_mantissa[src_fmt_q][1]};
    operand_c = {fmt_sign[dst_fmt_q][2], fmt_exponent[dst_fmt_q][2], fmt_mantissa[dst_fmt_q][2]};
    info_a    = info_q[src_fmt_q][0];
    info_b    = info_q[src_fmt_q][1];
    info_c    = info_q[dst_fmt_q][2];

    // op_mod_q inverts sign of operand C
    operand_c.sign = operand_c.sign ^ inp_pipe_op_mod_q[NUM_INP_REGS];

    unique case (inp_pipe_op_q[NUM_INP_REGS])
      fpnew_pkg::FMADD:  ; // do nothing
      fpnew_pkg::FNMSUB: operand_a.sign = ~operand_a.sign; // invert sign of product
      fpnew_pkg::ADD: begin // Set multiplicand to +1
        operand_a = '{sign: 1'b0, exponent: fpnew_pkg::bias(src_fmt_q), mantissa: '0};
        info_a    = '{is_normal: 1'b1, is_boxed: 1'b1, default: 1'b0}; //normal, boxed value.
      end
      fpnew_pkg::MUL: begin // Set addend to -0 (for proper rounding with RDN)
        operand_c = '{sign: 1'b1, exponent: '0, mantissa: '0};
        info_c    = '{is_zero: 1'b1, is_boxed: 1'b1, default: 1'b0}; //zero, boxed value.
      end
      default: begin // propagate don't cares
        operand_a  = '{default: fpnew_pkg::DONT_CARE};
        operand_b  = '{default: fpnew_pkg::DONT_CARE};
        operand_c  = '{default: fpnew_pkg::DONT_CARE};
        info_a     = '{default: fpnew_pkg::DONT_CARE};
        info_b     = '{default: fpnew_pkg::DONT_CARE};
        info_c     = '{default: fpnew_pkg::DONT_CARE};
      end
    endcase
  end

  // ---------------------
  // Input classification
  // ---------------------
  logic any_operand_inf;
  logic any_operand_nan;
  logic signalling_nan;
  logic effective_subtraction;
  logic tentative_sign;

  // Reduction for special case handling
  assign any_operand_inf = (| {info_a.is_inf,        info_b.is_inf,        info_c.is_inf});
  assign any_operand_nan = (| {info_a.is_nan,        info_b.is_nan,        info_c.is_nan});
  assign signalling_nan  = (| {info_a.is_signalling, info_b.is_signalling, info_c.is_signalling});
  // Effective subtraction in FMA occurs when product and addend signs differ
  assign effective_subtraction = operand_a.sign ^ operand_b.sign ^ operand_c.sign;
  // The tentative sign of the FMA shall be the sign of the product
  assign tentative_sign = operand_a.sign ^ operand_b.sign;

  // ----------------------
  // Special case handling
  // ----------------------
  logic [WIDTH-1:0]   special_result;
  fpnew_pkg::status_t special_status;
  logic               result_is_special;

  logic [NUM_FORMATS-1:0][WIDTH-1:0]    fmt_special_result;
  fpnew_pkg::status_t [NUM_FORMATS-1:0] fmt_special_status;
  logic [NUM_FORMATS-1:0]               fmt_result_is_special;


  for (genvar fmt = 0; fmt < int'(NUM_FORMATS); fmt++) begin : gen_special_results
    // Set up some constants
    localparam int unsigned FP_WIDTH = fpnew_pkg::fp_width(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned EXP_BITS = fpnew_pkg::exp_bits(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned MAN_BITS = fpnew_pkg::man_bits(fpnew_pkg::fp_format_e'(fmt));

    localparam logic [EXP_BITS-1:0] QNAN_EXPONENT = '1;
    localparam logic [MAN_BITS-1:0] QNAN_MANTISSA = 2**(MAN_BITS-1);
    localparam logic [MAN_BITS-1:0] ZERO_MANTISSA = '0;

    if (FpFmtConfig[fmt]) begin : active_format
      always_comb begin : special_results
        logic [FP_WIDTH-1:0] special_res;

        // Default assignment
        special_res                = {1'b0, QNAN_EXPONENT, QNAN_MANTISSA}; // qNaN
        fmt_special_status[fmt]    = '0;
        fmt_result_is_special[fmt] = 1'b0;

        // Handle potentially mixed nan & infinity input => important for the case where infinity and
        // zero are multiplied and added to a qnan.
        // RISC-V mandates raising the NV exception in these cases:
        // (inf * 0) + c or (0 * inf) + c INVALID, no matter c (even quiet NaNs)
        if ((info_a.is_inf && info_b.is_zero) || (info_a.is_zero && info_b.is_inf)) begin
          fmt_result_is_special[fmt] = 1'b1; // bypass FMA, output is the canonical qNaN
          fmt_special_status[fmt].NV = 1'b1; // invalid operation
        // NaN Inputs cause canonical quiet NaN at the output and maybe invalid OP
        end else if (any_operand_nan) begin
          fmt_result_is_special[fmt] = 1'b1;           // bypass FMA, output is the canonical qNaN
          fmt_special_status[fmt].NV = signalling_nan; // raise the invalid operation flag if signalling
        // Special cases involving infinity
        end else if (any_operand_inf) begin
          fmt_result_is_special[fmt] = 1'b1; // bypass FMA
          // Effective addition of opposite infinities (±inf - ±inf) is invalid!
          if ((info_a.is_inf || info_b.is_inf) && info_c.is_inf && effective_subtraction)
            fmt_special_status[fmt].NV = 1'b1; // invalid operation
          // Handle cases where output will be inf because of inf product input
          else if (info_a.is_inf || info_b.is_inf) begin
            // Result is infinity with the sign of the product
            special_res = {operand_a.sign ^ operand_b.sign, QNAN_EXPONENT, ZERO_MANTISSA};
          // Handle cases where the addend is inf
          end else if (info_c.is_inf) begin
            // Result is inifinity with sign of the addend (= operand_c)
            special_res = {operand_c.sign, QNAN_EXPONENT, ZERO_MANTISSA};
          end
        end
        // Initialize special result with ones (NaN-box)
        fmt_special_result[fmt]               = '1;
        fmt_special_result[fmt][FP_WIDTH-1:0] = special_res;
      end
    end else begin : inactive_format
      assign fmt_special_result[fmt] = '{default: fpnew_pkg::DONT_CARE};
      assign fmt_special_status[fmt] = '0;
      assign fmt_result_is_special[fmt] = 1'b0;
    end
  end

  // Detect special case from source format, I2F casts don't produce a special result
  assign result_is_special = fmt_result_is_special[dst_fmt_q]; // they're all the same
  // Signalling input NaNs raise invalid flag, otherwise no flags set
  assign special_status = fmt_special_status[dst_fmt_q];
  // Assemble result according to destination format
  assign special_result = fmt_special_result[dst_fmt_q]; // destination format

  // ---------------------------
  // Initial exponent data path
  // ---------------------------
  logic signed [EXP_WIDTH-1:0] exponent_a, exponent_b, exponent_c;
  logic signed [EXP_WIDTH-1:0] exponent_addend, exponent_product, exponent_difference;
  logic signed [EXP_WIDTH-1:0] tentative_exponent;

  // Zero-extend exponents into signed container - implicit width extension
  assign exponent_a = signed'({1'b0, operand_a.exponent});
  assign exponent_b = signed'({1'b0, operand_b.exponent});
  assign exponent_c = signed'({1'b0, operand_c.exponent});

  // Calculate internal exponents from encoded values. Real exponents are (ex = Ex - bias + 1 - nx)
  // with Ex the encoded exponent and nx the implicit bit. Internal exponents are biased to dst fmt.
  assign exponent_addend = signed'(exponent_c + $signed({1'b0, ~info_c.is_normal})); // 0 as subnorm
  // Biased product exponent is the sum of encoded exponents minus the bias.
  assign exponent_product = (info_a.is_zero || info_b.is_zero) // in case the product is zero, set minimum exp.
                            ? 2 - signed'(fpnew_pkg::bias(dst_fmt_q))
                            : signed'(exponent_a + info_a.is_subnormal
                                      + exponent_b + info_b.is_subnormal
                                      - 2*signed'(fpnew_pkg::bias(src_fmt_q))
                                      + signed'(fpnew_pkg::bias(dst_fmt_q))); // rebias for dst fmt
  // Exponent difference is the addend exponent minus the product exponent
  assign exponent_difference = exponent_addend - exponent_product;
  // The tentative exponent will be the larger of the product or addend exponent
  assign tentative_exponent = (exponent_difference > 0) ? exponent_addend : exponent_product;

  // Shift amount for addend based on exponents (unsigned as only right shifts)
  logic [SHIFT_AMOUNT_WIDTH-1:0] addend_shamt;

  always_comb begin : addend_shift_amount
    // Product-anchored case, saturated shift (addend is only in the sticky bit)
    if (exponent_difference <= signed'(-2 * PRECISION_BITS - 1))
      addend_shamt = 3 * PRECISION_BITS + 4;
    // Addend and product will have mutual bits to add
    else if (exponent_difference <= signed'(PRECISION_BITS + 2))
      addend_shamt = unsigned'(signed'(PRECISION_BITS) + 3 - exponent_difference);
    // Addend-anchored case, saturated shift (product is only in the sticky bit)
    else
      addend_shamt = 0;
  end

  // ------------------
  // Product data path
  // ------------------
  logic [PRECISION_BITS-1:0]   mantissa_a, mantissa_b, mantissa_c;
  logic [2*PRECISION_BITS-1:0] product;             // the p*p product is 2p bits wide
  logic [3*PRECISION_BITS+3:0] product_shifted;     // addends are 3p+4 bit wide (including G/R)

  // Add implicit bits to mantissae
  assign mantissa_a = {info_a.is_normal, operand_a.mantissa};
  assign mantissa_b = {info_b.is_normal, operand_b.mantissa};
  assign mantissa_c = {info_c.is_normal, operand_c.mantissa};

  // Mantissa multiplier (a*b)
  assign product = mantissa_a * mantissa_b;

  // Product is placed into a 3p+4 bit wide vector, padded with 2 bits for round and sticky:
  // | 000...000 | product | RS |
  //  <-  p+2  -> <-  2p -> < 2>
  assign product_shifted = product << 2; // constant shift

  // -----------------
  // Addend data path
  // -----------------
  logic [3*PRECISION_BITS+3:0] addend_after_shift;  // upper 3p+4 bits are needed to go on
  logic [PRECISION_BITS-1:0]   addend_sticky_bits;  // up to p bit of shifted addend are sticky
  logic                        sticky_before_add;   // they are compressed into a single sticky bit
  logic [3*PRECISION_BITS+3:0] addend_shifted;      // addends are 3p+4 bit wide (including G/R)
  logic                        inject_carry_in;     // inject carry for subtractions if needed

  // In parallel, the addend is right-shifted according to the exponent difference. Up to p bits are
  // shifted out and compressed into a sticky bit.
  // BEFORE THE SHIFT:
  // | mantissa_c | 000..000 |
  //  <-    p   -> <- 3p+4 ->
  // AFTER THE SHIFT:
  // | 000..........000 | mantissa_c | 000...............0GR |  sticky bits  |
  //  <- addend_shamt -> <-    p   -> <- 2p+4-addend_shamt -> <-  up to p  ->
  assign {addend_after_shift, addend_sticky_bits} =
      (mantissa_c << (3 * PRECISION_BITS + 4)) >> addend_shamt;

  assign sticky_before_add     = (| addend_sticky_bits);

  // In case of a subtraction, the addend is inverted
  assign addend_shifted = (effective_subtraction) ? ~addend_after_shift : addend_after_shift;
  assign inject_carry_in = effective_subtraction & ~sticky_before_add;

  // ------
  // Adder
  // ------
  logic [3*PRECISION_BITS+4:0] sum_raw;   // added one bit for the carry
  logic                        sum_carry; // observe carry bit from sum for sign fixing
  logic [3*PRECISION_BITS+3:0] sum;       // discard carry as sum won't overflow
  logic                        final_sign;

  //Mantissa adder (ab+c). In normal addition, it cannot overflow.
  assign sum_raw = product_shifted + addend_shifted + inject_carry_in;
  assign sum_carry = sum_raw[3*PRECISION_BITS+4];

  // Complement negative sum (can only happen in subtraction -> overflows for positive results)
  assign sum        = (effective_subtraction && ~sum_carry) ? -sum_raw : sum_raw;

  // In case of a mispredicted subtraction result, do a sign flip
  assign final_sign = (effective_subtraction && (sum_carry == tentative_sign))
                      ? 1'b1
                      : (effective_subtraction ? 1'b0 : tentative_sign);

  // ---------------
  // Internal pipeline
  // ---------------
  // Pipeline output signals as non-arrays
  logic                          effective_subtraction_q;
  logic signed [EXP_WIDTH-1:0]   exponent_product_q;
  logic signed [EXP_WIDTH-1:0]   exponent_difference_q;
  logic signed [EXP_WIDTH-1:0]   tentative_exponent_q;
  logic [SHIFT_AMOUNT_WIDTH-1:0] addend_shamt_q;
  logic                          sticky_before_add_q;
  logic [3*PRECISION_BITS+3:0]   sum_q;
  logic                          final_sign_q;
  fpnew_pkg::fp_format_e         dst_fmt_q2;
  fpnew_pkg::roundmode_e         rnd_mode_q;
  logic                          result_is_special_q;
  fp_t                           special_result_q;
  fpnew_pkg::status_t            special_status_q;
  // Internal pipeline signals, index i holds signal after i register stages
  logic                  [0:NUM_MID_REGS]                         mid_pipe_eff_sub_q;
  logic signed           [0:NUM_MID_REGS][EXP_WIDTH-1:0]          mid_pipe_exp_prod_q;
  logic signed           [0:NUM_MID_REGS][EXP_WIDTH-1:0]          mid_pipe_exp_diff_q;
  logic signed           [0:NUM_MID_REGS][EXP_WIDTH-1:0]          mid_pipe_tent_exp_q;
  logic                  [0:NUM_MID_REGS][SHIFT_AMOUNT_WIDTH-1:0] mid_pipe_add_shamt_q;
  logic                  [0:NUM_MID_REGS]                         mid_pipe_sticky_q;
  logic                  [0:NUM_MID_REGS][3*PRECISION_BITS+3:0]   mid_pipe_sum_q;
  logic                  [0:NUM_MID_REGS]                         mid_pipe_final_sign_q;
  fpnew_pkg::roundmode_e [0:NUM_MID_REGS]                         mid_pipe_rnd_mode_q;
  fpnew_pkg::fp_format_e [0:NUM_MID_REGS]                         mid_pipe_dst_fmt_q;
  logic                  [0:NUM_MID_REGS]                         mid_pipe_res_is_spec_q;
  fp_t                   [0:NUM_MID_REGS]                         mid_pipe_spec_res_q;
  fpnew_pkg::status_t    [0:NUM_MID_REGS]                         mid_pipe_spec_stat_q;
  TagType                [0:NUM_MID_REGS]                         mid_pipe_tag_q;
  AuxType                [0:NUM_MID_REGS]                         mid_pipe_aux_q;
  logic                  [0:NUM_MID_REGS]                         mid_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_MID_REGS] mid_pipe_ready;

  // Input stage: First element of pipeline is taken from upstream logic
  assign mid_pipe_eff_sub_q[0]     = effective_subtraction;
  assign mid_pipe_exp_prod_q[0]    = exponent_product;
  assign mid_pipe_exp_diff_q[0]    = exponent_difference;
  assign mid_pipe_tent_exp_q[0]    = tentative_exponent;
  assign mid_pipe_add_shamt_q[0]   = addend_shamt;
  assign mid_pipe_sticky_q[0]      = sticky_before_add;
  assign mid_pipe_sum_q[0]         = sum;
  assign mid_pipe_final_sign_q[0]  = final_sign;
  assign mid_pipe_rnd_mode_q[0]    = inp_pipe_rnd_mode_q[NUM_INP_REGS];
  assign mid_pipe_dst_fmt_q[0]     = dst_fmt_q;
  assign mid_pipe_res_is_spec_q[0] = result_is_special;
  assign mid_pipe_spec_res_q[0]    = special_result;
  assign mid_pipe_spec_stat_q[0]   = special_status;
  assign mid_pipe_tag_q[0]         = inp_pipe_tag_q[NUM_INP_REGS];
  assign mid_pipe_aux_q[0]         = inp_pipe_aux_q[NUM_INP_REGS];
  assign mid_pipe_valid_q[0]       = inp_pipe_valid_q[NUM_INP_REGS];
  // Input stage: Propagate pipeline ready signal to input pipe
  assign inp_pipe_ready[NUM_INP_REGS] = mid_pipe_ready[0];

  // Generate the register stages
  for (genvar i = 0; i < NUM_MID_REGS; i++) begin : gen_inside_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign mid_pipe_ready[i] = mid_pipe_ready[i+1] | ~mid_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(mid_pipe_valid_q[i+1], mid_pipe_valid_q[i], mid_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = mid_pipe_ready[i] & mid_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(mid_pipe_eff_sub_q[i+1],     mid_pipe_eff_sub_q[i],     reg_ena, '0)
    `FFL(mid_pipe_exp_prod_q[i+1],    mid_pipe_exp_prod_q[i],    reg_ena, '0)
    `FFL(mid_pipe_exp_diff_q[i+1],    mid_pipe_exp_diff_q[i],    reg_ena, '0)
    `FFL(mid_pipe_tent_exp_q[i+1],    mid_pipe_tent_exp_q[i],    reg_ena, '0)
    `FFL(mid_pipe_add_shamt_q[i+1],   mid_pipe_add_shamt_q[i],   reg_ena, '0)
    `FFL(mid_pipe_sticky_q[i+1],      mid_pipe_sticky_q[i],      reg_ena, '0)
    `FFL(mid_pipe_sum_q[i+1],         mid_pipe_sum_q[i],         reg_ena, '0)
    `FFL(mid_pipe_final_sign_q[i+1],  mid_pipe_final_sign_q[i],  reg_ena, '0)
    `FFL(mid_pipe_rnd_mode_q[i+1],    mid_pipe_rnd_mode_q[i],    reg_ena, fpnew_pkg::RNE)
    `FFL(mid_pipe_dst_fmt_q[i+1],     mid_pipe_dst_fmt_q[i],     reg_ena, fpnew_pkg::fp_format_e'(0))
    `FFL(mid_pipe_res_is_spec_q[i+1], mid_pipe_res_is_spec_q[i], reg_ena, '0)
    `FFL(mid_pipe_spec_res_q[i+1],    mid_pipe_spec_res_q[i],    reg_ena, '0)
    `FFL(mid_pipe_spec_stat_q[i+1],   mid_pipe_spec_stat_q[i],   reg_ena, '0)
    `FFL(mid_pipe_tag_q[i+1],         mid_pipe_tag_q[i],         reg_ena, TagType'('0))
    `FFL(mid_pipe_aux_q[i+1],         mid_pipe_aux_q[i],         reg_ena, AuxType'('0))
  end
  // Output stage: assign selected pipe outputs to signals for later use
  assign effective_subtraction_q = mid_pipe_eff_sub_q[NUM_MID_REGS];
  assign exponent_product_q      = mid_pipe_exp_prod_q[NUM_MID_REGS];
  assign exponent_difference_q   = mid_pipe_exp_diff_q[NUM_MID_REGS];
  assign tentative_exponent_q    = mid_pipe_tent_exp_q[NUM_MID_REGS];
  assign addend_shamt_q          = mid_pipe_add_shamt_q[NUM_MID_REGS];
  assign sticky_before_add_q     = mid_pipe_sticky_q[NUM_MID_REGS];
  assign sum_q                   = mid_pipe_sum_q[NUM_MID_REGS];
  assign final_sign_q            = mid_pipe_final_sign_q[NUM_MID_REGS];
  assign rnd_mode_q              = mid_pipe_rnd_mode_q[NUM_MID_REGS];
  assign dst_fmt_q2              = mid_pipe_dst_fmt_q[NUM_MID_REGS];
  assign result_is_special_q     = mid_pipe_res_is_spec_q[NUM_MID_REGS];
  assign special_result_q        = mid_pipe_spec_res_q[NUM_MID_REGS];
  assign special_status_q        = mid_pipe_spec_stat_q[NUM_MID_REGS];

  // --------------
  // Normalization
  // --------------
  logic        [LOWER_SUM_WIDTH-1:0]  sum_lower;              // lower 2p+3 bits of sum are searched
  logic        [LZC_RESULT_WIDTH-1:0] leading_zero_count;     // the number of leading zeroes
  logic signed [LZC_RESULT_WIDTH:0]   leading_zero_count_sgn; // signed leading-zero count
  logic                               lzc_zeroes;             // in case only zeroes found

  logic        [SHIFT_AMOUNT_WIDTH-1:0] norm_shamt; // Normalization shift amount
  logic signed [EXP_WIDTH-1:0]          normalized_exponent;

  logic [3*PRECISION_BITS+4:0] sum_shifted;       // result after first normalization shift
  logic [PRECISION_BITS:0]     final_mantissa;    // final mantissa before rounding with round bit
  logic [2*PRECISION_BITS+2:0] sum_sticky_bits;   // remaining 2p+3 sticky bits after normalization
  logic                        sticky_after_norm; // sticky bit after normalization

  logic signed [EXP_WIDTH-1:0] final_exponent;

  assign sum_lower = sum_q[LOWER_SUM_WIDTH-1:0];

  // Leading zero counter for cancellations
  lzc #(
    .WIDTH ( LOWER_SUM_WIDTH ),
    .MODE  ( 1               ) // MODE = 1 counts leading zeroes
  ) i_lzc (
    .in_i    ( sum_lower          ),
    .cnt_o   ( leading_zero_count ),
    .empty_o ( lzc_zeroes         )
  );

  assign leading_zero_count_sgn = signed'({1'b0, leading_zero_count});

  // Normalization shift amount based on exponents and LZC (unsigned as only left shifts)
  always_comb begin : norm_shift_amount
    // Product-anchored case or cancellations require LZC
    if ((exponent_difference_q <= 0) || (effective_subtraction_q && (exponent_difference_q <= 2))) begin
      // Normal result (biased exponent > 0 and not a zero)
      if ((exponent_product_q - leading_zero_count_sgn + 1 >= 0) && !lzc_zeroes) begin
        // Undo initial product shift, remove the counted zeroes
        norm_shamt          = PRECISION_BITS + 2 + leading_zero_count;
        normalized_exponent = exponent_product_q - leading_zero_count_sgn + 1; // account for shift
      // Subnormal result
      end else begin
        // Cap the shift distance to align mantissa with minimum exponent
        norm_shamt          = unsigned'(signed'(PRECISION_BITS + 2 + exponent_product_q));
        normalized_exponent = 0; // subnormals encoded as 0
      end
    // Addend-anchored case
    end else begin
      norm_shamt          = addend_shamt_q; // Undo the initial shift
      normalized_exponent = tentative_exponent_q;
    end
  end

  // Do the large normalization shift
  assign sum_shifted       = sum_q << norm_shamt;

  // The addend-anchored case needs a 1-bit normalization since the leading-one can be to the left
  // or right of the (non-carry) MSB of the sum.
  always_comb begin : small_norm
    // Default assignment, discarding carry bit
    {final_mantissa, sum_sticky_bits} = sum_shifted;
    final_exponent                    = normalized_exponent;

    // The normalized sum has overflown, align right and fix exponent
    if (sum_shifted[3*PRECISION_BITS+4]) begin // check the carry bit
      {final_mantissa, sum_sticky_bits} = sum_shifted >> 1;
      final_exponent                    = normalized_exponent + 1;
    // The normalized sum is normal, nothing to do
    end else if (sum_shifted[3*PRECISION_BITS+3]) begin // check the sum MSB
      // do nothing
    // The normalized sum is still denormal, align left - unless the result is not already subnormal
    end else if (normalized_exponent > 1) begin
      {final_mantissa, sum_sticky_bits} = sum_shifted << 1;
      final_exponent                    = normalized_exponent - 1;
    // Otherwise we're denormal
    end else begin
      final_exponent = '0;
    end
  end

  // Update the sticky bit with the shifted-out bits
  assign sticky_after_norm = (| {sum_sticky_bits}) | sticky_before_add_q;

  // ----------------------------
  // Rounding and classification
  // ----------------------------
  logic                                     pre_round_sign;
  logic [SUPER_EXP_BITS+SUPER_MAN_BITS-1:0] pre_round_abs; // absolute value of result before rounding
  logic [1:0]                               round_sticky_bits;

  logic of_before_round, of_after_round; // overflow
  logic uf_before_round, uf_after_round; // underflow

  logic [NUM_FORMATS-1:0][SUPER_EXP_BITS+SUPER_MAN_BITS-1:0] fmt_pre_round_abs; // per format
  logic [NUM_FORMATS-1:0][1:0]                               fmt_round_sticky_bits;

  logic [NUM_FORMATS-1:0]                                    fmt_of_after_round;
  logic [NUM_FORMATS-1:0]                                    fmt_uf_after_round;

  logic                                     rounded_sign;
  logic [SUPER_EXP_BITS+SUPER_MAN_BITS-1:0] rounded_abs; // absolute value of result after rounding
  logic                                     result_zero;

  // Classification before round. RISC-V mandates checking underflow AFTER rounding!
  assign of_before_round = final_exponent >= 2**(fpnew_pkg::exp_bits(dst_fmt_q2))-1; // infinity exponent is all ones
  assign uf_before_round = final_exponent == 0;               // exponent for subnormals capped to 0

  // Pack exponent and mantissa into proper rounding form
  for (genvar fmt = 0; fmt < int'(NUM_FORMATS); fmt++) begin : gen_res_assemble
    // Set up some constants
    localparam int unsigned EXP_BITS = fpnew_pkg::exp_bits(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned MAN_BITS = fpnew_pkg::man_bits(fpnew_pkg::fp_format_e'(fmt));

    logic [EXP_BITS-1:0] pre_round_exponent;
    logic [MAN_BITS-1:0] pre_round_mantissa;

    if (FpFmtConfig[fmt]) begin : active_format

      assign pre_round_exponent = (of_before_round) ? 2**EXP_BITS-2 : final_exponent[EXP_BITS-1:0];
      assign pre_round_mantissa = (of_before_round) ? '1 : final_mantissa[SUPER_MAN_BITS-:MAN_BITS];
      // Assemble result before rounding. In case of overflow, the largest normal value is set.
      assign fmt_pre_round_abs[fmt] = {pre_round_exponent, pre_round_mantissa}; // 0-extend

      // Round bit is after mantissa (1 in case of overflow for rounding)
      assign fmt_round_sticky_bits[fmt][1] = final_mantissa[SUPER_MAN_BITS-MAN_BITS] |
                                             of_before_round;

      // remaining bits in mantissa to sticky (1 in case of overflow for rounding)
      if (MAN_BITS < SUPER_MAN_BITS) begin : narrow_sticky
        assign fmt_round_sticky_bits[fmt][0] = (| final_mantissa[SUPER_MAN_BITS-MAN_BITS-1:0]) |
                                               sticky_after_norm | of_before_round;
      end else begin : normal_sticky
        assign fmt_round_sticky_bits[fmt][0] = sticky_after_norm | of_before_round;
      end
    end else begin : inactive_format
      assign fmt_pre_round_abs[fmt] = '{default: fpnew_pkg::DONT_CARE};
      assign fmt_round_sticky_bits[fmt] = '{default: fpnew_pkg::DONT_CARE};
    end
  end

  // Assemble result before rounding. In case of overflow, the largest normal value is set.
  assign pre_round_sign     = final_sign_q;
  assign pre_round_abs      = fmt_pre_round_abs[dst_fmt_q2];

  // In case of overflow, the round and sticky bits are set for proper rounding
  assign round_sticky_bits  = fmt_round_sticky_bits[dst_fmt_q2];

  // Perform the rounding
  fpnew_rounding #(
    .AbsWidth ( SUPER_EXP_BITS + SUPER_MAN_BITS )
  ) i_fpnew_rounding (
    .abs_value_i             ( pre_round_abs           ),
    .sign_i                  ( pre_round_sign          ),
    .round_sticky_bits_i     ( round_sticky_bits       ),
    .rnd_mode_i              ( rnd_mode_q              ),
    .effective_subtraction_i ( effective_subtraction_q ),
    .abs_rounded_o           ( rounded_abs             ),
    .sign_o                  ( rounded_sign            ),
    .exact_zero_o            ( result_zero             )
  );

  logic [NUM_FORMATS-1:0][WIDTH-1:0] fmt_result;

  for (genvar fmt = 0; fmt < int'(NUM_FORMATS); fmt++) begin : gen_sign_inject
    // Set up some constants
    localparam int unsigned FP_WIDTH = fpnew_pkg::fp_width(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned EXP_BITS = fpnew_pkg::exp_bits(fpnew_pkg::fp_format_e'(fmt));
    localparam int unsigned MAN_BITS = fpnew_pkg::man_bits(fpnew_pkg::fp_format_e'(fmt));

    if (FpFmtConfig[fmt]) begin : active_format
      always_comb begin : post_process
        // detect of / uf
        fmt_uf_after_round[fmt] = rounded_abs[EXP_BITS+MAN_BITS-1:MAN_BITS] == '0; // denormal
        fmt_of_after_round[fmt] = rounded_abs[EXP_BITS+MAN_BITS-1:MAN_BITS] == '1; // inf exp.

        // Assemble regular result, nan box short ones.
        fmt_result[fmt]               = '1;
        fmt_result[fmt][FP_WIDTH-1:0] = {rounded_sign, rounded_abs[EXP_BITS+MAN_BITS-1:0]};
      end
    end else begin : inactive_format
      assign fmt_uf_after_round[fmt] = fpnew_pkg::DONT_CARE;
      assign fmt_of_after_round[fmt] = fpnew_pkg::DONT_CARE;
      assign fmt_result[fmt]         = '{default: fpnew_pkg::DONT_CARE};
    end
  end

  // Classification after rounding select by destination format
  assign uf_after_round = fmt_uf_after_round[dst_fmt_q2];
  assign of_after_round = fmt_of_after_round[dst_fmt_q2];


  // -----------------
  // Result selection
  // -----------------
  logic [WIDTH-1:0]     regular_result;
  fpnew_pkg::status_t   regular_status;

  // Assemble regular result
  assign regular_result = fmt_result[dst_fmt_q2];
  assign regular_status.NV = 1'b0; // only valid cases are handled in regular path
  assign regular_status.DZ = 1'b0; // no divisions
  assign regular_status.OF = of_before_round | of_after_round;   // rounding can introduce overflow
  assign regular_status.UF = uf_after_round & regular_status.NX; // only inexact results raise UF
  assign regular_status.NX = (| round_sticky_bits) | of_before_round | of_after_round;

  // Final results for output pipeline
  logic [WIDTH-1:0]   result_d;
  fpnew_pkg::status_t status_d;

  // Select output depending on special case detection
  assign result_d = result_is_special_q ? special_result_q : regular_result;
  assign status_d = result_is_special_q ? special_status_q : regular_status;

  // ----------------
  // Output Pipeline
  // ----------------
  // Output pipeline signals, index i holds signal after i register stages
  logic               [0:NUM_OUT_REGS][WIDTH-1:0] out_pipe_result_q;
  fpnew_pkg::status_t [0:NUM_OUT_REGS]            out_pipe_status_q;
  TagType             [0:NUM_OUT_REGS]            out_pipe_tag_q;
  AuxType             [0:NUM_OUT_REGS]            out_pipe_aux_q;
  logic               [0:NUM_OUT_REGS]            out_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_OUT_REGS] out_pipe_ready;

  // Input stage: First element of pipeline is taken from inputs
  assign out_pipe_result_q[0] = result_d;
  assign out_pipe_status_q[0] = status_d;
  assign out_pipe_tag_q[0]    = mid_pipe_tag_q[NUM_MID_REGS];
  assign out_pipe_aux_q[0]    = mid_pipe_aux_q[NUM_MID_REGS];
  assign out_pipe_valid_q[0]  = mid_pipe_valid_q[NUM_MID_REGS];
  // Input stage: Propagate pipeline ready signal to inside pipe
  assign mid_pipe_ready[NUM_MID_REGS] = out_pipe_ready[0];
  // Generate the register stages
  for (genvar i = 0; i < NUM_OUT_REGS; i++) begin : gen_output_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign out_pipe_ready[i] = out_pipe_ready[i+1] | ~out_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(out_pipe_valid_q[i+1], out_pipe_valid_q[i], out_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = out_pipe_ready[i] & out_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(out_pipe_result_q[i+1], out_pipe_result_q[i], reg_ena, '0)
    `FFL(out_pipe_status_q[i+1], out_pipe_status_q[i], reg_ena, '0)
    `FFL(out_pipe_tag_q[i+1],    out_pipe_tag_q[i],    reg_ena, TagType'('0))
    `FFL(out_pipe_aux_q[i+1],    out_pipe_aux_q[i],    reg_ena, AuxType'('0))
  end
  // Output stage: Ready travels backwards from output side, driven by downstream circuitry
  assign out_pipe_ready[NUM_OUT_REGS] = out_ready_i;
  // Output stage: assign module outputs
  assign result_o        = out_pipe_result_q[NUM_OUT_REGS];
  assign status_o        = out_pipe_status_q[NUM_OUT_REGS];
  assign extension_bit_o = 1'b1; // always NaN-Box result
  assign tag_o           = out_pipe_tag_q[NUM_OUT_REGS];
  assign aux_o           = out_pipe_aux_q[NUM_OUT_REGS];
  assign out_valid_o     = out_pipe_valid_q[NUM_OUT_REGS];
  assign busy_o          = (| {inp_pipe_valid_q, mid_pipe_valid_q, out_pipe_valid_q});
endmodule
// Copyright 2019 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Stefan Mach <smach@iis.ee.ethz.ch>

//`include "common_cells/registers.svh"

module fpnew_fma #(
  parameter fpnew_pkg::fp_format_e   FpFormat    = fpnew_pkg::fp_format_e'(0),
  parameter int unsigned             NumPipeRegs = 0,
  parameter fpnew_pkg::pipe_config_t PipeConfig  = fpnew_pkg::BEFORE,
  parameter type                     TagType     = logic,
  parameter type                     AuxType     = logic,

  localparam int unsigned WIDTH = fpnew_pkg::fp_width(FpFormat) // do not change
) (
  input logic                      clk_i,
  input logic                      rst_ni,
  // Input signals
  input logic [2:0][WIDTH-1:0]     operands_i, // 3 operands
  input logic [2:0]                is_boxed_i, // 3 operands
  input fpnew_pkg::roundmode_e     rnd_mode_i,
  input fpnew_pkg::operation_e     op_i,
  input logic                      op_mod_i,
  input TagType                    tag_i,
  input AuxType                    aux_i,
  // Input Handshake
  input  logic                     in_valid_i,
  output logic                     in_ready_o,
  input  logic                     flush_i,
  // Output signals
  output logic [WIDTH-1:0]         result_o,
  output fpnew_pkg::status_t       status_o,
  output logic                     extension_bit_o,
  output TagType                   tag_o,
  output AuxType                   aux_o,
  // Output handshake
  output logic                     out_valid_o,
  input  logic                     out_ready_i,
  // Indication of valid data in flight
  output logic                     busy_o
);

  // ----------
  // Constants
  // ----------
  localparam int unsigned EXP_BITS = fpnew_pkg::exp_bits(FpFormat);
  localparam int unsigned MAN_BITS = fpnew_pkg::man_bits(FpFormat);
  localparam int unsigned BIAS     = fpnew_pkg::bias(FpFormat);
  // Precision bits 'p' include the implicit bit
  localparam int unsigned PRECISION_BITS = MAN_BITS + 1;
  // The lower 2p+3 bits of the internal FMA result will be needed for leading-zero detection
  localparam int unsigned LOWER_SUM_WIDTH  = 2 * PRECISION_BITS + 3;
  localparam int unsigned LZC_RESULT_WIDTH = $clog2(LOWER_SUM_WIDTH);
  // Internal exponent width of FMA must accomodate all meaningful exponent values in order to avoid
  // datapath leakage. This is either given by the exponent bits or the width of the LZC result.
  // In most reasonable FP formats the internal exponent will be wider than the LZC result.
  localparam int unsigned EXP_WIDTH = unsigned'(fpnew_pkg::maximum(EXP_BITS + 2, LZC_RESULT_WIDTH));
  // Shift amount width: maximum internal mantissa size is 3p+3 bits
  localparam int unsigned SHIFT_AMOUNT_WIDTH = $clog2(3 * PRECISION_BITS + 3);
  // Pipelines
  localparam NUM_INP_REGS = PipeConfig == fpnew_pkg::BEFORE
                            ? NumPipeRegs
                            : (PipeConfig == fpnew_pkg::DISTRIBUTED
                               ? ((NumPipeRegs + 1) / 3) // Second to get distributed regs
                               : 0); // no regs here otherwise
  localparam NUM_MID_REGS = PipeConfig == fpnew_pkg::INSIDE
                          ? NumPipeRegs
                          : (PipeConfig == fpnew_pkg::DISTRIBUTED
                             ? ((NumPipeRegs + 2) / 3) // First to get distributed regs
                             : 0); // no regs here otherwise
  localparam NUM_OUT_REGS = PipeConfig == fpnew_pkg::AFTER
                            ? NumPipeRegs
                            : (PipeConfig == fpnew_pkg::DISTRIBUTED
                               ? (NumPipeRegs / 3) // Last to get distributed regs
                               : 0); // no regs here otherwise

  // ----------------
  // Type definition
  // ----------------
  typedef struct packed {
    logic                sign;
    logic [EXP_BITS-1:0] exponent;
    logic [MAN_BITS-1:0] mantissa;
  } fp_t;

  // ---------------
  // Input pipeline
  // ---------------
  // Input pipeline signals, index i holds signal after i register stages
  logic                  [0:NUM_INP_REGS][2:0][WIDTH-1:0] inp_pipe_operands_q;
  logic                  [0:NUM_INP_REGS][2:0]            inp_pipe_is_boxed_q;
  fpnew_pkg::roundmode_e [0:NUM_INP_REGS]                 inp_pipe_rnd_mode_q;
  fpnew_pkg::operation_e [0:NUM_INP_REGS]                 inp_pipe_op_q;
  logic                  [0:NUM_INP_REGS]                 inp_pipe_op_mod_q;
  TagType                [0:NUM_INP_REGS]                 inp_pipe_tag_q;
  AuxType                [0:NUM_INP_REGS]                 inp_pipe_aux_q;
  logic                  [0:NUM_INP_REGS]                 inp_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_INP_REGS] inp_pipe_ready;

  // Input stage: First element of pipeline is taken from inputs
  assign inp_pipe_operands_q[0] = operands_i;
  assign inp_pipe_is_boxed_q[0] = is_boxed_i;
  assign inp_pipe_rnd_mode_q[0] = rnd_mode_i;
  assign inp_pipe_op_q[0]       = op_i;
  assign inp_pipe_op_mod_q[0]   = op_mod_i;
  assign inp_pipe_tag_q[0]      = tag_i;
  assign inp_pipe_aux_q[0]      = aux_i;
  assign inp_pipe_valid_q[0]    = in_valid_i;
  // Input stage: Propagate pipeline ready signal to updtream circuitry
  assign in_ready_o = inp_pipe_ready[0];
  // Generate the register stages
  for (genvar i = 0; i < NUM_INP_REGS; i++) begin : gen_input_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign inp_pipe_ready[i] = inp_pipe_ready[i+1] | ~inp_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(inp_pipe_valid_q[i+1], inp_pipe_valid_q[i], inp_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = inp_pipe_ready[i] & inp_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(inp_pipe_operands_q[i+1], inp_pipe_operands_q[i], reg_ena, '0)
    `FFL(inp_pipe_is_boxed_q[i+1], inp_pipe_is_boxed_q[i], reg_ena, '0)
    `FFL(inp_pipe_rnd_mode_q[i+1], inp_pipe_rnd_mode_q[i], reg_ena, fpnew_pkg::RNE)
    `FFL(inp_pipe_op_q[i+1],       inp_pipe_op_q[i],       reg_ena, fpnew_pkg::FMADD)
    `FFL(inp_pipe_op_mod_q[i+1],   inp_pipe_op_mod_q[i],   reg_ena, '0)
    `FFL(inp_pipe_tag_q[i+1],      inp_pipe_tag_q[i],      reg_ena, TagType'('0))
    `FFL(inp_pipe_aux_q[i+1],      inp_pipe_aux_q[i],      reg_ena, AuxType'('0))
  end

  // -----------------
  // Input processing
  // -----------------
  fpnew_pkg::fp_info_t [2:0] info_q;

  // Classify input
  fpnew_classifier #(
    .FpFormat    ( FpFormat ),
    .NumOperands ( 3        )
    ) i_class_inputs (
    .operands_i ( inp_pipe_operands_q[NUM_INP_REGS] ),
    .is_boxed_i ( inp_pipe_is_boxed_q[NUM_INP_REGS] ),
    .info_o     ( info_q                            )
  );

  fp_t                 operand_a, operand_b, operand_c;
  fpnew_pkg::fp_info_t info_a,    info_b,    info_c;

  // Operation selection and operand adjustment
  // | \c op_q  | \c op_mod_q | Operation Adjustment
  // |:--------:|:-----------:|---------------------
  // | FMADD    | \c 0        | FMADD: none
  // | FMADD    | \c 1        | FMSUB: Invert sign of operand C
  // | FNMSUB   | \c 0        | FNMSUB: Invert sign of operand A
  // | FNMSUB   | \c 1        | FNMADD: Invert sign of operands A and C
  // | ADD      | \c 0        | ADD: Set operand A to +1.0
  // | ADD      | \c 1        | SUB: Set operand A to +1.0, invert sign of operand C
  // | MUL      | \c 0        | MUL: Set operand C to +0.0
  // | *others* | \c -        | *invalid*
  // \note \c op_mod_q always inverts the sign of the addend.
  always_comb begin : op_select

    // Default assignments - packing-order-agnostic
    operand_a = inp_pipe_operands_q[NUM_INP_REGS][0];
    operand_b = inp_pipe_operands_q[NUM_INP_REGS][1];
    operand_c = inp_pipe_operands_q[NUM_INP_REGS][2];
    info_a    = info_q[0];
    info_b    = info_q[1];
    info_c    = info_q[2];

    // op_mod_q inverts sign of operand C
    operand_c.sign = operand_c.sign ^ inp_pipe_op_mod_q[NUM_INP_REGS];

    unique case (inp_pipe_op_q[NUM_INP_REGS])
      fpnew_pkg::FMADD:  ; // do nothing
      fpnew_pkg::FNMSUB: operand_a.sign = ~operand_a.sign; // invert sign of product
      fpnew_pkg::ADD: begin // Set multiplicand to +1
        operand_a = '{sign: 1'b0, exponent: BIAS, mantissa: '0};
        info_a    = '{is_normal: 1'b1, is_boxed: 1'b1, default: 1'b0}; //normal, boxed value.
      end
      fpnew_pkg::MUL: begin // Set addend to -0 (for proper rounding with RDN)
        operand_c = '{sign: 1'b1, exponent: '0, mantissa: '0};
        info_c    = '{is_zero: 1'b1, is_boxed: 1'b1, default: 1'b0}; //zero, boxed value.
      end
      default: begin // propagate don't cares
        operand_a  = '{default: fpnew_pkg::DONT_CARE};
        operand_b  = '{default: fpnew_pkg::DONT_CARE};
        operand_c  = '{default: fpnew_pkg::DONT_CARE};
        info_a     = '{default: fpnew_pkg::DONT_CARE};
        info_b     = '{default: fpnew_pkg::DONT_CARE};
        info_c     = '{default: fpnew_pkg::DONT_CARE};
      end
    endcase
  end

  // ---------------------
  // Input classification
  // ---------------------
  logic any_operand_inf;
  logic any_operand_nan;
  logic signalling_nan;
  logic effective_subtraction;
  logic tentative_sign;

  // Reduction for special case handling
  assign any_operand_inf = (| {info_a.is_inf,        info_b.is_inf,        info_c.is_inf});
  assign any_operand_nan = (| {info_a.is_nan,        info_b.is_nan,        info_c.is_nan});
  assign signalling_nan  = (| {info_a.is_signalling, info_b.is_signalling, info_c.is_signalling});
  // Effective subtraction in FMA occurs when product and addend signs differ
  assign effective_subtraction = operand_a.sign ^ operand_b.sign ^ operand_c.sign;
  // The tentative sign of the FMA shall be the sign of the product
  assign tentative_sign = operand_a.sign ^ operand_b.sign;

  // ----------------------
  // Special case handling
  // ----------------------
  fp_t                special_result;
  fpnew_pkg::status_t special_status;
  logic               result_is_special;

  always_comb begin : special_cases
    // Default assignments
    special_result    = '{sign: 1'b0, exponent: '1, mantissa: 2**(MAN_BITS-1)}; // canonical qNaN
    special_status    = '0;
    result_is_special = 1'b0;

    // Handle potentially mixed nan & infinity input => important for the case where infinity and
    // zero are multiplied and added to a qnan.
    // RISC-V mandates raising the NV exception in these cases:
    // (inf * 0) + c or (0 * inf) + c INVALID, no matter c (even quiet NaNs)
    if ((info_a.is_inf && info_b.is_zero) || (info_a.is_zero && info_b.is_inf)) begin
      result_is_special = 1'b1; // bypass FMA, output is the canonical qNaN
      special_status.NV = 1'b1; // invalid operation
    // NaN Inputs cause canonical quiet NaN at the output and maybe invalid OP
    end else if (any_operand_nan) begin
      result_is_special = 1'b1;           // bypass FMA, output is the canonical qNaN
      special_status.NV = signalling_nan; // raise the invalid operation flag if signalling
    // Special cases involving infinity
    end else if (any_operand_inf) begin
      result_is_special = 1'b1; // bypass FMA
      // Effective addition of opposite infinities (±inf - ±inf) is invalid!
      if ((info_a.is_inf || info_b.is_inf) && info_c.is_inf && effective_subtraction)
        special_status.NV = 1'b1; // invalid operation
      // Handle cases where output will be inf because of inf product input
      else if (info_a.is_inf || info_b.is_inf) begin
        // Result is infinity with the sign of the product
        special_result    = '{sign: operand_a.sign ^ operand_b.sign, exponent: '1, mantissa: '0};
      // Handle cases where the addend is inf
      end else if (info_c.is_inf) begin
        // Result is inifinity with sign of the addend (= operand_c)
        special_result    = '{sign: operand_c.sign, exponent: '1, mantissa: '0};
      end
    end
  end

  // ---------------------------
  // Initial exponent data path
  // ---------------------------
  logic signed [EXP_WIDTH-1:0] exponent_a, exponent_b, exponent_c;
  logic signed [EXP_WIDTH-1:0] exponent_addend, exponent_product, exponent_difference;
  logic signed [EXP_WIDTH-1:0] tentative_exponent;

  // Zero-extend exponents into signed container - implicit width extension
  assign exponent_a = signed'({1'b0, operand_a.exponent});
  assign exponent_b = signed'({1'b0, operand_b.exponent});
  assign exponent_c = signed'({1'b0, operand_c.exponent});

  // Calculate internal exponents from encoded values. Real exponents are (ex = Ex - bias + 1 - nx)
  // with Ex the encoded exponent and nx the implicit bit. Internal exponents stay biased.
  assign exponent_addend = signed'(exponent_c + $signed({1'b0, ~info_c.is_normal})); // 0 as subnorm
  // Biased product exponent is the sum of encoded exponents minus the bias.
  assign exponent_product = (info_a.is_zero || info_b.is_zero)
                            ? 2 - signed'(BIAS) // in case the product is zero, set minimum exp.
                            : signed'(exponent_a + info_a.is_subnormal
                                      + exponent_b + info_b.is_subnormal
                                      - signed'(BIAS));
  // Exponent difference is the addend exponent minus the product exponent
  assign exponent_difference = exponent_addend - exponent_product;
  // The tentative exponent will be the larger of the product or addend exponent
  assign tentative_exponent = (exponent_difference > 0) ? exponent_addend : exponent_product;

  // Shift amount for addend based on exponents (unsigned as only right shifts)
  logic [SHIFT_AMOUNT_WIDTH-1:0] addend_shamt;

  always_comb begin : addend_shift_amount
    // Product-anchored case, saturated shift (addend is only in the sticky bit)
    if (exponent_difference <= signed'(-2 * PRECISION_BITS - 1))
      addend_shamt = 3 * PRECISION_BITS + 4;
    // Addend and product will have mutual bits to add
    else if (exponent_difference <= signed'(PRECISION_BITS + 2))
      addend_shamt = unsigned'(signed'(PRECISION_BITS) + 3 - exponent_difference);
    // Addend-anchored case, saturated shift (product is only in the sticky bit)
    else
      addend_shamt = 0;
  end

  // ------------------
  // Product data path
  // ------------------
  logic [PRECISION_BITS-1:0]   mantissa_a, mantissa_b, mantissa_c;
  logic [2*PRECISION_BITS-1:0] product;             // the p*p product is 2p bits wide
  logic [3*PRECISION_BITS+3:0] product_shifted;     // addends are 3p+4 bit wide (including G/R)

  // Add implicit bits to mantissae
  assign mantissa_a = {info_a.is_normal, operand_a.mantissa};
  assign mantissa_b = {info_b.is_normal, operand_b.mantissa};
  assign mantissa_c = {info_c.is_normal, operand_c.mantissa};

  // Mantissa multiplier (a*b)
  assign product = mantissa_a * mantissa_b;

  // Product is placed into a 3p+4 bit wide vector, padded with 2 bits for round and sticky:
  // | 000...000 | product | RS |
  //  <-  p+2  -> <-  2p -> < 2>
  assign product_shifted = product << 2; // constant shift

  // -----------------
  // Addend data path
  // -----------------
  logic [3*PRECISION_BITS+3:0] addend_after_shift;  // upper 3p+4 bits are needed to go on
  logic [PRECISION_BITS-1:0]   addend_sticky_bits;  // up to p bit of shifted addend are sticky
  logic                        sticky_before_add;   // they are compressed into a single sticky bit
  logic [3*PRECISION_BITS+3:0] addend_shifted;      // addends are 3p+4 bit wide (including G/R)
  logic                        inject_carry_in;     // inject carry for subtractions if needed

  // In parallel, the addend is right-shifted according to the exponent difference. Up to p bits
  // are shifted out and compressed into a sticky bit.
  // BEFORE THE SHIFT:
  // | mantissa_c | 000..000 |
  //  <-    p   -> <- 3p+4 ->
  // AFTER THE SHIFT:
  // | 000..........000 | mantissa_c | 000...............0GR |  sticky bits  |
  //  <- addend_shamt -> <-    p   -> <- 2p+4-addend_shamt -> <-  up to p  ->
  assign {addend_after_shift, addend_sticky_bits} =
      (mantissa_c << (3 * PRECISION_BITS + 4)) >> addend_shamt;

  assign sticky_before_add     = (| addend_sticky_bits);
  // assign addend_after_shift[0] = sticky_before_add;

  // In case of a subtraction, the addend is inverted
  assign addend_shifted  = (effective_subtraction) ? ~addend_after_shift : addend_after_shift;
  assign inject_carry_in = effective_subtraction & ~sticky_before_add;

  // ------
  // Adder
  // ------
  logic [3*PRECISION_BITS+4:0] sum_raw;   // added one bit for the carry
  logic                        sum_carry; // observe carry bit from sum for sign fixing
  logic [3*PRECISION_BITS+3:0] sum;       // discard carry as sum won't overflow
  logic                        final_sign;

  //Mantissa adder (ab+c). In normal addition, it cannot overflow.
  assign sum_raw = product_shifted + addend_shifted + inject_carry_in;
  assign sum_carry = sum_raw[3*PRECISION_BITS+4];

  // Complement negative sum (can only happen in subtraction -> overflows for positive results)
  assign sum        = (effective_subtraction && ~sum_carry) ? -sum_raw : sum_raw;

  // In case of a mispredicted subtraction result, do a sign flip
  assign final_sign = (effective_subtraction && (sum_carry == tentative_sign))
                      ? 1'b1
                      : (effective_subtraction ? 1'b0 : tentative_sign);

  // ---------------
  // Internal pipeline
  // ---------------
  // Pipeline output signals as non-arrays
  logic                          effective_subtraction_q;
  logic signed [EXP_WIDTH-1:0]   exponent_product_q;
  logic signed [EXP_WIDTH-1:0]   exponent_difference_q;
  logic signed [EXP_WIDTH-1:0]   tentative_exponent_q;
  logic [SHIFT_AMOUNT_WIDTH-1:0] addend_shamt_q;
  logic                          sticky_before_add_q;
  logic [3*PRECISION_BITS+3:0]   sum_q;
  logic                          final_sign_q;
  fpnew_pkg::roundmode_e         rnd_mode_q;
  logic                          result_is_special_q;
  fp_t                           special_result_q;
  fpnew_pkg::status_t            special_status_q;
  // Internal pipeline signals, index i holds signal after i register stages
  logic                  [0:NUM_MID_REGS]                         mid_pipe_eff_sub_q;
  logic signed           [0:NUM_MID_REGS][EXP_WIDTH-1:0]          mid_pipe_exp_prod_q;
  logic signed           [0:NUM_MID_REGS][EXP_WIDTH-1:0]          mid_pipe_exp_diff_q;
  logic signed           [0:NUM_MID_REGS][EXP_WIDTH-1:0]          mid_pipe_tent_exp_q;
  logic                  [0:NUM_MID_REGS][SHIFT_AMOUNT_WIDTH-1:0] mid_pipe_add_shamt_q;
  logic                  [0:NUM_MID_REGS]                         mid_pipe_sticky_q;
  logic                  [0:NUM_MID_REGS][3*PRECISION_BITS+3:0]   mid_pipe_sum_q;
  logic                  [0:NUM_MID_REGS]                         mid_pipe_final_sign_q;
  fpnew_pkg::roundmode_e [0:NUM_MID_REGS]                         mid_pipe_rnd_mode_q;
  logic                  [0:NUM_MID_REGS]                         mid_pipe_res_is_spec_q;
  fp_t                   [0:NUM_MID_REGS]                         mid_pipe_spec_res_q;
  fpnew_pkg::status_t    [0:NUM_MID_REGS]                         mid_pipe_spec_stat_q;
  TagType                [0:NUM_MID_REGS]                         mid_pipe_tag_q;
  AuxType                [0:NUM_MID_REGS]                         mid_pipe_aux_q;
  logic                  [0:NUM_MID_REGS]                         mid_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_MID_REGS] mid_pipe_ready;

  // Input stage: First element of pipeline is taken from upstream logic
  assign mid_pipe_eff_sub_q[0]     = effective_subtraction;
  assign mid_pipe_exp_prod_q[0]    = exponent_product;
  assign mid_pipe_exp_diff_q[0]    = exponent_difference;
  assign mid_pipe_tent_exp_q[0]    = tentative_exponent;
  assign mid_pipe_add_shamt_q[0]   = addend_shamt;
  assign mid_pipe_sticky_q[0]      = sticky_before_add;
  assign mid_pipe_sum_q[0]         = sum;
  assign mid_pipe_final_sign_q[0]  = final_sign;
  assign mid_pipe_rnd_mode_q[0]    = inp_pipe_rnd_mode_q[NUM_INP_REGS];
  assign mid_pipe_res_is_spec_q[0] = result_is_special;
  assign mid_pipe_spec_res_q[0]    = special_result;
  assign mid_pipe_spec_stat_q[0]   = special_status;
  assign mid_pipe_tag_q[0]         = inp_pipe_tag_q[NUM_INP_REGS];
  assign mid_pipe_aux_q[0]         = inp_pipe_aux_q[NUM_INP_REGS];
  assign mid_pipe_valid_q[0]       = inp_pipe_valid_q[NUM_INP_REGS];
  // Input stage: Propagate pipeline ready signal to input pipe
  assign inp_pipe_ready[NUM_INP_REGS] = mid_pipe_ready[0];

  // Generate the register stages
  for (genvar i = 0; i < NUM_MID_REGS; i++) begin : gen_inside_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign mid_pipe_ready[i] = mid_pipe_ready[i+1] | ~mid_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(mid_pipe_valid_q[i+1], mid_pipe_valid_q[i], mid_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = mid_pipe_ready[i] & mid_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(mid_pipe_eff_sub_q[i+1],     mid_pipe_eff_sub_q[i],     reg_ena, '0)
    `FFL(mid_pipe_exp_prod_q[i+1],    mid_pipe_exp_prod_q[i],    reg_ena, '0)
    `FFL(mid_pipe_exp_diff_q[i+1],    mid_pipe_exp_diff_q[i],    reg_ena, '0)
    `FFL(mid_pipe_tent_exp_q[i+1],    mid_pipe_tent_exp_q[i],    reg_ena, '0)
    `FFL(mid_pipe_add_shamt_q[i+1],   mid_pipe_add_shamt_q[i],   reg_ena, '0)
    `FFL(mid_pipe_sticky_q[i+1],      mid_pipe_sticky_q[i],      reg_ena, '0)
    `FFL(mid_pipe_sum_q[i+1],         mid_pipe_sum_q[i],         reg_ena, '0)
    `FFL(mid_pipe_final_sign_q[i+1],  mid_pipe_final_sign_q[i],  reg_ena, '0)
    `FFL(mid_pipe_rnd_mode_q[i+1],    mid_pipe_rnd_mode_q[i],    reg_ena, fpnew_pkg::RNE)
    `FFL(mid_pipe_res_is_spec_q[i+1], mid_pipe_res_is_spec_q[i], reg_ena, '0)
    `FFL(mid_pipe_spec_res_q[i+1],    mid_pipe_spec_res_q[i],    reg_ena, '0)
    `FFL(mid_pipe_spec_stat_q[i+1],   mid_pipe_spec_stat_q[i],   reg_ena, '0)
    `FFL(mid_pipe_tag_q[i+1],         mid_pipe_tag_q[i],         reg_ena, TagType'('0))
    `FFL(mid_pipe_aux_q[i+1],         mid_pipe_aux_q[i],         reg_ena, AuxType'('0))
  end
  // Output stage: assign selected pipe outputs to signals for later use
  assign effective_subtraction_q = mid_pipe_eff_sub_q[NUM_MID_REGS];
  assign exponent_product_q      = mid_pipe_exp_prod_q[NUM_MID_REGS];
  assign exponent_difference_q   = mid_pipe_exp_diff_q[NUM_MID_REGS];
  assign tentative_exponent_q    = mid_pipe_tent_exp_q[NUM_MID_REGS];
  assign addend_shamt_q          = mid_pipe_add_shamt_q[NUM_MID_REGS];
  assign sticky_before_add_q     = mid_pipe_sticky_q[NUM_MID_REGS];
  assign sum_q                   = mid_pipe_sum_q[NUM_MID_REGS];
  assign final_sign_q            = mid_pipe_final_sign_q[NUM_MID_REGS];
  assign rnd_mode_q              = mid_pipe_rnd_mode_q[NUM_MID_REGS];
  assign result_is_special_q     = mid_pipe_res_is_spec_q[NUM_MID_REGS];
  assign special_result_q        = mid_pipe_spec_res_q[NUM_MID_REGS];
  assign special_status_q        = mid_pipe_spec_stat_q[NUM_MID_REGS];

  // --------------
  // Normalization
  // --------------
  logic        [LOWER_SUM_WIDTH-1:0]  sum_lower;              // lower 2p+3 bits of sum are searched
  logic        [LZC_RESULT_WIDTH-1:0] leading_zero_count;     // the number of leading zeroes
  logic signed [LZC_RESULT_WIDTH:0]   leading_zero_count_sgn; // signed leading-zero count
  logic                               lzc_zeroes;             // in case only zeroes found

  logic        [SHIFT_AMOUNT_WIDTH-1:0] norm_shamt; // Normalization shift amount
  logic signed [EXP_WIDTH-1:0]          normalized_exponent;

  logic [3*PRECISION_BITS+4:0] sum_shifted;       // result after first normalization shift
  logic [PRECISION_BITS:0]     final_mantissa;    // final mantissa before rounding with round bit
  logic [2*PRECISION_BITS+2:0] sum_sticky_bits;   // remaining 2p+3 sticky bits after normalization
  logic                        sticky_after_norm; // sticky bit after normalization

  logic signed [EXP_WIDTH-1:0] final_exponent;

  assign sum_lower = sum_q[LOWER_SUM_WIDTH-1:0];

  // Leading zero counter for cancellations
  lzc #(
    .WIDTH ( LOWER_SUM_WIDTH ),
    .MODE  ( 1               ) // MODE = 1 counts leading zeroes
  ) i_lzc (
    .in_i    ( sum_lower          ),
    .cnt_o   ( leading_zero_count ),
    .empty_o ( lzc_zeroes         )
  );

  assign leading_zero_count_sgn = signed'({1'b0, leading_zero_count});

  // Normalization shift amount based on exponents and LZC (unsigned as only left shifts)
  always_comb begin : norm_shift_amount
    // Product-anchored case or cancellations require LZC
    if ((exponent_difference_q <= 0) || (effective_subtraction_q && (exponent_difference_q <= 2))) begin
      // Normal result (biased exponent > 0 and not a zero)
      if ((exponent_product_q - leading_zero_count_sgn + 1 >= 0) && !lzc_zeroes) begin
        // Undo initial product shift, remove the counted zeroes
        norm_shamt          = PRECISION_BITS + 2 + leading_zero_count;
        normalized_exponent = exponent_product_q - leading_zero_count_sgn + 1; // account for shift
      // Subnormal result
      end else begin
        // Cap the shift distance to align mantissa with minimum exponent
        norm_shamt          = unsigned'(signed'(PRECISION_BITS) + 2 + exponent_product_q);
        normalized_exponent = 0; // subnormals encoded as 0
      end
    // Addend-anchored case
    end else begin
      norm_shamt          = addend_shamt_q; // Undo the initial shift
      normalized_exponent = tentative_exponent_q;
    end
  end

  // Do the large normalization shift
  assign sum_shifted       = sum_q << norm_shamt;

  // The addend-anchored case needs a 1-bit normalization since the leading-one can be to the left
  // or right of the (non-carry) MSB of the sum.
  always_comb begin : small_norm
    // Default assignment, discarding carry bit
    {final_mantissa, sum_sticky_bits} = sum_shifted;
    final_exponent                    = normalized_exponent;

    // The normalized sum has overflown, align right and fix exponent
    if (sum_shifted[3*PRECISION_BITS+4]) begin // check the carry bit
      {final_mantissa, sum_sticky_bits} = sum_shifted >> 1;
      final_exponent                    = normalized_exponent + 1;
    // The normalized sum is normal, nothing to do
    end else if (sum_shifted[3*PRECISION_BITS+3]) begin // check the sum MSB
      // do nothing
    // The normalized sum is still denormal, align left - unless the result is not already subnormal
    end else if (normalized_exponent > 1) begin
      {final_mantissa, sum_sticky_bits} = sum_shifted << 1;
      final_exponent                    = normalized_exponent - 1;
    // Otherwise we're denormal
    end else begin
      final_exponent = '0;
    end
  end

  // Update the sticky bit with the shifted-out bits
  assign sticky_after_norm = (| {sum_sticky_bits}) | sticky_before_add_q;

  // ----------------------------
  // Rounding and classification
  // ----------------------------
  logic                         pre_round_sign;
  logic [EXP_BITS-1:0]          pre_round_exponent;
  logic [MAN_BITS-1:0]          pre_round_mantissa;
  logic [EXP_BITS+MAN_BITS-1:0] pre_round_abs; // absolute value of result before rounding
  logic [1:0]                   round_sticky_bits;

  logic of_before_round, of_after_round; // overflow
  logic uf_before_round, uf_after_round; // underflow
  logic result_zero;

  logic                         rounded_sign;
  logic [EXP_BITS+MAN_BITS-1:0] rounded_abs; // absolute value of result after rounding

  // Classification before round. RISC-V mandates checking underflow AFTER rounding!
  assign of_before_round = final_exponent >= 2**(EXP_BITS)-1; // infinity exponent is all ones
  assign uf_before_round = final_exponent == 0;               // exponent for subnormals capped to 0

  // Assemble result before rounding. In case of overflow, the largest normal value is set.
  assign pre_round_sign     = final_sign_q;
  assign pre_round_exponent = (of_before_round) ? 2**EXP_BITS-2 : unsigned'(final_exponent[EXP_BITS-1:0]);
  assign pre_round_mantissa = (of_before_round) ? '1 : final_mantissa[MAN_BITS:1]; // bit 0 is R bit
  assign pre_round_abs      = {pre_round_exponent, pre_round_mantissa};

  // In case of overflow, the round and sticky bits are set for proper rounding
  assign round_sticky_bits  = (of_before_round) ? 2'b11 : {final_mantissa[0], sticky_after_norm};

  // Perform the rounding
  fpnew_rounding #(
    .AbsWidth ( EXP_BITS + MAN_BITS )
  ) i_fpnew_rounding (
    .abs_value_i             ( pre_round_abs           ),
    .sign_i                  ( pre_round_sign          ),
    .round_sticky_bits_i     ( round_sticky_bits       ),
    .rnd_mode_i              ( rnd_mode_q              ),
    .effective_subtraction_i ( effective_subtraction_q ),
    .abs_rounded_o           ( rounded_abs             ),
    .sign_o                  ( rounded_sign            ),
    .exact_zero_o            ( result_zero             )
  );

  // Classification after rounding
  assign uf_after_round = rounded_abs[EXP_BITS+MAN_BITS-1:MAN_BITS] == '0; // exponent = 0
  assign of_after_round = rounded_abs[EXP_BITS+MAN_BITS-1:MAN_BITS] == '1; // exponent all ones

  // -----------------
  // Result selection
  // -----------------
  logic [WIDTH-1:0]     regular_result;
  fpnew_pkg::status_t   regular_status;

  // Assemble regular result
  assign regular_result    = {rounded_sign, rounded_abs};
  assign regular_status.NV = 1'b0; // only valid cases are handled in regular path
  assign regular_status.DZ = 1'b0; // no divisions
  assign regular_status.OF = of_before_round | of_after_round;   // rounding can introduce overflow
  assign regular_status.UF = uf_after_round & regular_status.NX; // only inexact results raise UF
  assign regular_status.NX = (| round_sticky_bits) | of_before_round | of_after_round;

  // Final results for output pipeline
  fp_t                result_d;
  fpnew_pkg::status_t status_d;

  // Select output depending on special case detection
  assign result_d = result_is_special_q ? special_result_q : regular_result;
  assign status_d = result_is_special_q ? special_status_q : regular_status;

  // ----------------
  // Output Pipeline
  // ----------------
  // Output pipeline signals, index i holds signal after i register stages
  fp_t                [0:NUM_OUT_REGS] out_pipe_result_q;
  fpnew_pkg::status_t [0:NUM_OUT_REGS] out_pipe_status_q;
  TagType             [0:NUM_OUT_REGS] out_pipe_tag_q;
  AuxType             [0:NUM_OUT_REGS] out_pipe_aux_q;
  logic               [0:NUM_OUT_REGS] out_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_OUT_REGS] out_pipe_ready;

  // Input stage: First element of pipeline is taken from inputs
  assign out_pipe_result_q[0] = result_d;
  assign out_pipe_status_q[0] = status_d;
  assign out_pipe_tag_q[0]    = mid_pipe_tag_q[NUM_MID_REGS];
  assign out_pipe_aux_q[0]    = mid_pipe_aux_q[NUM_MID_REGS];
  assign out_pipe_valid_q[0]  = mid_pipe_valid_q[NUM_MID_REGS];
  // Input stage: Propagate pipeline ready signal to inside pipe
  assign mid_pipe_ready[NUM_MID_REGS] = out_pipe_ready[0];
  // Generate the register stages
  for (genvar i = 0; i < NUM_OUT_REGS; i++) begin : gen_output_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign out_pipe_ready[i] = out_pipe_ready[i+1] | ~out_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(out_pipe_valid_q[i+1], out_pipe_valid_q[i], out_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = out_pipe_ready[i] & out_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(out_pipe_result_q[i+1], out_pipe_result_q[i], reg_ena, '0)
    `FFL(out_pipe_status_q[i+1], out_pipe_status_q[i], reg_ena, '0)
    `FFL(out_pipe_tag_q[i+1],    out_pipe_tag_q[i],    reg_ena, TagType'('0))
    `FFL(out_pipe_aux_q[i+1],    out_pipe_aux_q[i],    reg_ena, AuxType'('0))
  end
  // Output stage: Ready travels backwards from output side, driven by downstream circuitry
  assign out_pipe_ready[NUM_OUT_REGS] = out_ready_i;
  // Output stage: assign module outputs
  assign result_o        = out_pipe_result_q[NUM_OUT_REGS];
  assign status_o        = out_pipe_status_q[NUM_OUT_REGS];
  assign extension_bit_o = 1'b1; // always NaN-Box result
  assign tag_o           = out_pipe_tag_q[NUM_OUT_REGS];
  assign aux_o           = out_pipe_aux_q[NUM_OUT_REGS];
  assign out_valid_o     = out_pipe_valid_q[NUM_OUT_REGS];
  assign busy_o          = (| {inp_pipe_valid_q, mid_pipe_valid_q, out_pipe_valid_q});
endmodule
// Copyright 2019 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Stefan Mach <smach@iis.ee.ethz.ch>

//`include "common_cells/registers.svh"

module fpnew_noncomp #(
  parameter fpnew_pkg::fp_format_e   FpFormat    = fpnew_pkg::fp_format_e'(0),
  parameter int unsigned             NumPipeRegs = 0,
  parameter fpnew_pkg::pipe_config_t PipeConfig  = fpnew_pkg::BEFORE,
  parameter type                     TagType     = logic,
  parameter type                     AuxType     = logic,

  localparam int unsigned WIDTH = fpnew_pkg::fp_width(FpFormat) // do not change
) (
  input logic                  clk_i,
  input logic                  rst_ni,
  // Input signals
  input logic [1:0][WIDTH-1:0]     operands_i, // 2 operands
  input logic [1:0]                is_boxed_i, // 2 operands
  input fpnew_pkg::roundmode_e     rnd_mode_i,
  input fpnew_pkg::operation_e     op_i,
  input logic                      op_mod_i,
  input TagType                    tag_i,
  input AuxType                    aux_i,
  // Input Handshake
  input  logic                     in_valid_i,
  output logic                     in_ready_o,
  input  logic                     flush_i,
  // Output signals
  output logic [WIDTH-1:0]         result_o,
  output fpnew_pkg::status_t       status_o,
  output logic                     extension_bit_o,
  output fpnew_pkg::classmask_e    class_mask_o,
  output logic                     is_class_o,
  output TagType                   tag_o,
  output AuxType                   aux_o,
  // Output handshake
  output logic                     out_valid_o,
  input  logic                     out_ready_i,
  // Indication of valid data in flight
  output logic                     busy_o
);

  // ----------
  // Constants
  // ----------
  localparam int unsigned EXP_BITS = fpnew_pkg::exp_bits(FpFormat);
  localparam int unsigned MAN_BITS = fpnew_pkg::man_bits(FpFormat);
  // Pipelines
  localparam NUM_INP_REGS = (PipeConfig == fpnew_pkg::BEFORE || PipeConfig == fpnew_pkg::INSIDE)
                            ? NumPipeRegs
                            : (PipeConfig == fpnew_pkg::DISTRIBUTED
                               ? ((NumPipeRegs + 1) / 2) // First to get distributed regs
                               : 0); // no regs here otherwise
  localparam NUM_OUT_REGS = PipeConfig == fpnew_pkg::AFTER
                            ? NumPipeRegs
                            : (PipeConfig == fpnew_pkg::DISTRIBUTED
                               ? (NumPipeRegs / 2) // Last to get distributed regs
                               : 0); // no regs here otherwise

  // ----------------
  // Type definition
  // ----------------
  typedef struct packed {
    logic                sign;
    logic [EXP_BITS-1:0] exponent;
    logic [MAN_BITS-1:0] mantissa;
  } fp_t;

  // ---------------
  // Input pipeline
  // ---------------
  // Input pipeline signals, index i holds signal after i register stages
  logic                  [0:NUM_INP_REGS][1:0][WIDTH-1:0] inp_pipe_operands_q;
  logic                  [0:NUM_INP_REGS][1:0]            inp_pipe_is_boxed_q;
  fpnew_pkg::roundmode_e [0:NUM_INP_REGS]                 inp_pipe_rnd_mode_q;
  fpnew_pkg::operation_e [0:NUM_INP_REGS]                 inp_pipe_op_q;
  logic                  [0:NUM_INP_REGS]                 inp_pipe_op_mod_q;
  TagType                [0:NUM_INP_REGS]                 inp_pipe_tag_q;
  AuxType                [0:NUM_INP_REGS]                 inp_pipe_aux_q;
  logic                  [0:NUM_INP_REGS]                 inp_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_INP_REGS] inp_pipe_ready;

  // Input stage: First element of pipeline is taken from inputs
  assign inp_pipe_operands_q[0] = operands_i;
  assign inp_pipe_is_boxed_q[0] = is_boxed_i;
  assign inp_pipe_rnd_mode_q[0] = rnd_mode_i;
  assign inp_pipe_op_q[0]       = op_i;
  assign inp_pipe_op_mod_q[0]   = op_mod_i;
  assign inp_pipe_tag_q[0]      = tag_i;
  assign inp_pipe_aux_q[0]      = aux_i;
  assign inp_pipe_valid_q[0]    = in_valid_i;
  // Input stage: Propagate pipeline ready signal to updtream circuitry
  assign in_ready_o = inp_pipe_ready[0];
  // Generate the register stages
  for (genvar i = 0; i < NUM_INP_REGS; i++) begin : gen_input_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign inp_pipe_ready[i] = inp_pipe_ready[i+1] | ~inp_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(inp_pipe_valid_q[i+1], inp_pipe_valid_q[i], inp_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = inp_pipe_ready[i] & inp_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(inp_pipe_operands_q[i+1], inp_pipe_operands_q[i], reg_ena, '0)
    `FFL(inp_pipe_is_boxed_q[i+1], inp_pipe_is_boxed_q[i], reg_ena, '0)
    `FFL(inp_pipe_rnd_mode_q[i+1], inp_pipe_rnd_mode_q[i], reg_ena, fpnew_pkg::RNE)
    `FFL(inp_pipe_op_q[i+1],       inp_pipe_op_q[i],       reg_ena, fpnew_pkg::FMADD)
    `FFL(inp_pipe_op_mod_q[i+1],   inp_pipe_op_mod_q[i],   reg_ena, '0)
    `FFL(inp_pipe_tag_q[i+1],      inp_pipe_tag_q[i],      reg_ena, TagType'('0))
    `FFL(inp_pipe_aux_q[i+1],      inp_pipe_aux_q[i],      reg_ena, AuxType'('0))
  end

  // ---------------------
  // Input classification
  // ---------------------
  fpnew_pkg::fp_info_t [1:0] info_q;

  // Classify input
  fpnew_classifier #(
    .FpFormat    ( FpFormat ),
    .NumOperands ( 2        )
    ) i_class_a (
    .operands_i ( inp_pipe_operands_q[NUM_INP_REGS] ),
    .is_boxed_i ( inp_pipe_is_boxed_q[NUM_INP_REGS] ),
    .info_o     ( info_q                            )
  );

  fp_t                 operand_a, operand_b;
  fpnew_pkg::fp_info_t info_a,    info_b;

  // Packing-order-agnostic assignments
  assign operand_a = inp_pipe_operands_q[NUM_INP_REGS][0];
  assign operand_b = inp_pipe_operands_q[NUM_INP_REGS][1];
  assign info_a    = info_q[0];
  assign info_b    = info_q[1];

  logic any_operand_inf;
  logic any_operand_nan;
  logic signalling_nan;

  // Reduction for special case handling
  assign any_operand_inf = (| {info_a.is_inf,        info_b.is_inf});
  assign any_operand_nan = (| {info_a.is_nan,        info_b.is_nan});
  assign signalling_nan  = (| {info_a.is_signalling, info_b.is_signalling});

  logic operands_equal, operand_a_smaller;

  // Equality checks for zeroes too
  assign operands_equal    = (operand_a == operand_b) || (info_a.is_zero && info_b.is_zero);
  // Invert result if non-zero signs involved (unsigned comparison)
  assign operand_a_smaller = (operand_a < operand_b) ^ (operand_a.sign || operand_b.sign);

  // ---------------
  // Sign Injection
  // ---------------
  fp_t                sgnj_result;
  fpnew_pkg::status_t sgnj_status;
  logic               sgnj_extension_bit;

  // Sign Injection - operation is encoded in rnd_mode_q:
  // RNE = SGNJ, RTZ = SGNJN, RDN = SGNJX, RUP = Passthrough (no NaN-box check)
  always_comb begin : sign_injections
    logic sign_a, sign_b; // internal signs
    // Default assignment
    sgnj_result = operand_a; // result based on operand a

    // NaN-boxing check will treat invalid inputs as canonical NaNs
    if (!info_a.is_boxed) sgnj_result = '{sign: 1'b0, exponent: '1, mantissa: 2**(MAN_BITS-1)};

    // Internal signs are treated as positive in case of non-NaN-boxed values
    sign_a = operand_a.sign & info_a.is_boxed;
    sign_b = operand_b.sign & info_b.is_boxed;

    // Do the sign injection based on rm field
    unique case (inp_pipe_rnd_mode_q[NUM_INP_REGS])
      fpnew_pkg::RNE: sgnj_result.sign = sign_b;          // SGNJ
      fpnew_pkg::RTZ: sgnj_result.sign = ~sign_b;         // SGNJN
      fpnew_pkg::RDN: sgnj_result.sign = sign_a ^ sign_b; // SGNJX
      fpnew_pkg::RUP: sgnj_result      = operand_a;       // passthrough
      default: sgnj_result = '{default: fpnew_pkg::DONT_CARE}; // don't care
    endcase
  end

  assign sgnj_status = '0;        // sign injections never raise exceptions

  // op_mod_q enables integer sign-extension of result (for storing to integer regfile)
  assign sgnj_extension_bit = inp_pipe_op_mod_q[NUM_INP_REGS] ? sgnj_result.sign : 1'b1;

  // ------------------
  // Minimum / Maximum
  // ------------------
  fp_t                minmax_result;
  fpnew_pkg::status_t minmax_status;
  logic               minmax_extension_bit;

  // Minimum/Maximum - operation is encoded in rnd_mode_q:
  // RNE = MIN, RTZ = MAX
  always_comb begin : min_max
    // Default assignment
    minmax_status = '0;

    // Min/Max use quiet comparisons - only sNaN are invalid
    minmax_status.NV = signalling_nan;

    // Both NaN inputs cause a NaN output
    if (info_a.is_nan && info_b.is_nan)
      minmax_result = '{sign: 1'b0, exponent: '1, mantissa: 2**(MAN_BITS-1)}; // canonical qNaN
    // If one operand is NaN, the non-NaN operand is returned
    else if (info_a.is_nan) minmax_result = operand_b;
    else if (info_b.is_nan) minmax_result = operand_a;
    // Otherwise decide according to the operation
    else begin
      unique case (inp_pipe_rnd_mode_q[NUM_INP_REGS])
        fpnew_pkg::RNE: minmax_result = operand_a_smaller ? operand_a : operand_b; // MIN
        fpnew_pkg::RTZ: minmax_result = operand_a_smaller ? operand_b : operand_a; // MAX
        default: minmax_result = '{default: fpnew_pkg::DONT_CARE}; // don't care
      endcase
    end
  end

  assign minmax_extension_bit = 1'b1; // NaN-box as result is always a float value

  // ------------
  // Comparisons
  // ------------
  fp_t                cmp_result;
  fpnew_pkg::status_t cmp_status;
  logic               cmp_extension_bit;

  // Comparisons - operation is encoded in rnd_mode_q:
  // RNE = LE, RTZ = LT, RDN = EQ
  // op_mod_q inverts boolean outputs
  always_comb begin : comparisons
    // Default assignment
    cmp_result = '0; // false
    cmp_status = '0; // no flags

    // Signalling NaNs always compare as false and are illegal
    if (signalling_nan) cmp_status.NV = 1'b1; // invalid operation
    // Otherwise do comparisons
    else begin
      unique case (inp_pipe_rnd_mode_q[NUM_INP_REGS])
        fpnew_pkg::RNE: begin // Less than or equal
          if (any_operand_nan) cmp_status.NV = 1'b1; // Signalling comparison: NaNs are invalid
          else cmp_result = (operand_a_smaller | operands_equal) ^ inp_pipe_op_mod_q[NUM_INP_REGS];
        end
        fpnew_pkg::RTZ: begin // Less than
          if (any_operand_nan) cmp_status.NV = 1'b1; // Signalling comparison: NaNs are invalid
          else cmp_result = (operand_a_smaller & ~operands_equal) ^ inp_pipe_op_mod_q[NUM_INP_REGS];
        end
        fpnew_pkg::RDN: begin // Equal
          if (any_operand_nan) cmp_result = inp_pipe_op_mod_q[NUM_INP_REGS]; // NaN always not equal
          else cmp_result = operands_equal ^ inp_pipe_op_mod_q[NUM_INP_REGS];
        end
        default: cmp_result = '{default: fpnew_pkg::DONT_CARE}; // don't care
      endcase
    end
  end

  assign cmp_extension_bit = 1'b0; // Comparisons always produce booleans in integer registers

  // ---------------
  // Classification
  // ---------------
  fpnew_pkg::status_t    class_status;
  logic                  class_extension_bit;
  fpnew_pkg::classmask_e class_mask_d; // the result is actually here

  // Classification - always return the classification mask on the dedicated port
  always_comb begin : classify
    if (info_a.is_normal) begin
      class_mask_d = operand_a.sign       ? fpnew_pkg::NEGNORM    : fpnew_pkg::POSNORM;
    end else if (info_a.is_subnormal) begin
      class_mask_d = operand_a.sign       ? fpnew_pkg::NEGSUBNORM : fpnew_pkg::POSSUBNORM;
    end else if (info_a.is_zero) begin
      class_mask_d = operand_a.sign       ? fpnew_pkg::NEGZERO    : fpnew_pkg::POSZERO;
    end else if (info_a.is_inf) begin
      class_mask_d = operand_a.sign       ? fpnew_pkg::NEGINF     : fpnew_pkg::POSINF;
    end else if (info_a.is_nan) begin
      class_mask_d = info_a.is_signalling ? fpnew_pkg::SNAN       : fpnew_pkg::QNAN;
    end else begin
      class_mask_d = fpnew_pkg::QNAN; // default value
    end
  end

  assign class_status        = '0;   // classification does not set flags
  assign class_extension_bit = 1'b0; // classification always produces results in integer registers

  // -----------------
  // Result selection
  // -----------------
  fp_t                   result_d;
  fpnew_pkg::status_t    status_d;
  logic                  extension_bit_d;
  logic                  is_class_d;

  // Select result
  always_comb begin : select_result
    unique case (inp_pipe_op_q[NUM_INP_REGS])
      fpnew_pkg::SGNJ: begin
        result_d        = sgnj_result;
        status_d        = sgnj_status;
        extension_bit_d = sgnj_extension_bit;
      end
      fpnew_pkg::MINMAX: begin
        result_d        = minmax_result;
        status_d        = minmax_status;
        extension_bit_d = minmax_extension_bit;
      end
      fpnew_pkg::CMP: begin
        result_d        = cmp_result;
        status_d        = cmp_status;
        extension_bit_d = cmp_extension_bit;
      end
      fpnew_pkg::CLASSIFY: begin
        result_d        = '{default: fpnew_pkg::DONT_CARE}; // unused
        status_d        = class_status;
        extension_bit_d = class_extension_bit;
      end
      default: begin
        result_d        = '{default: fpnew_pkg::DONT_CARE}; // dont care
        status_d        = '{default: fpnew_pkg::DONT_CARE}; // dont care
        extension_bit_d = fpnew_pkg::DONT_CARE;             // dont care
      end
    endcase
  end

  assign is_class_d = (inp_pipe_op_q[NUM_INP_REGS] == fpnew_pkg::CLASSIFY);

  // ----------------
  // Output Pipeline
  // ----------------
  // Output pipeline signals, index i holds signal after i register stages
  fp_t                   [0:NUM_OUT_REGS] out_pipe_result_q;
  fpnew_pkg::status_t    [0:NUM_OUT_REGS] out_pipe_status_q;
  logic                  [0:NUM_OUT_REGS] out_pipe_extension_bit_q;
  fpnew_pkg::classmask_e [0:NUM_OUT_REGS] out_pipe_class_mask_q;
  logic                  [0:NUM_OUT_REGS] out_pipe_is_class_q;
  TagType                [0:NUM_OUT_REGS] out_pipe_tag_q;
  AuxType                [0:NUM_OUT_REGS] out_pipe_aux_q;
  logic                  [0:NUM_OUT_REGS] out_pipe_valid_q;
  // Ready signal is combinatorial for all stages
  logic [0:NUM_OUT_REGS] out_pipe_ready;

  // Input stage: First element of pipeline is taken from inputs
  assign out_pipe_result_q[0]        = result_d;
  assign out_pipe_status_q[0]        = status_d;
  assign out_pipe_extension_bit_q[0] = extension_bit_d;
  assign out_pipe_class_mask_q[0]    = class_mask_d;
  assign out_pipe_is_class_q[0]      = is_class_d;
  assign out_pipe_tag_q[0]           = inp_pipe_tag_q[NUM_INP_REGS];
  assign out_pipe_aux_q[0]           = inp_pipe_aux_q[NUM_INP_REGS];
  assign out_pipe_valid_q[0]         = inp_pipe_valid_q[NUM_INP_REGS];
  // Input stage: Propagate pipeline ready signal to inside pipe
  assign inp_pipe_ready[NUM_INP_REGS] = out_pipe_ready[0];
  // Generate the register stages
  for (genvar i = 0; i < NUM_OUT_REGS; i++) begin : gen_output_pipeline
    // Internal register enable for this stage
    logic reg_ena;
    // Determine the ready signal of the current stage - advance the pipeline:
    // 1. if the next stage is ready for our data
    // 2. if the next stage only holds a bubble (not valid) -> we can pop it
    assign out_pipe_ready[i] = out_pipe_ready[i+1] | ~out_pipe_valid_q[i+1];
    // Valid: enabled by ready signal, synchronous clear with the flush signal
    `FFLARNC(out_pipe_valid_q[i+1], out_pipe_valid_q[i], out_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
    // Enable register if pipleine ready and a valid data item is present
    assign reg_ena = out_pipe_ready[i] & out_pipe_valid_q[i];
    // Generate the pipeline registers within the stages, use enable-registers
    `FFL(out_pipe_result_q[i+1],        out_pipe_result_q[i],        reg_ena, '0)
    `FFL(out_pipe_status_q[i+1],        out_pipe_status_q[i],        reg_ena, '0)
    `FFL(out_pipe_extension_bit_q[i+1], out_pipe_extension_bit_q[i], reg_ena, '0)
    `FFL(out_pipe_class_mask_q[i+1],    out_pipe_class_mask_q[i],    reg_ena, fpnew_pkg::QNAN)
    `FFL(out_pipe_is_class_q[i+1],      out_pipe_is_class_q[i],      reg_ena, '0)
    `FFL(out_pipe_tag_q[i+1],           out_pipe_tag_q[i],           reg_ena, TagType'('0))
    `FFL(out_pipe_aux_q[i+1],           out_pipe_aux_q[i],           reg_ena, AuxType'('0))
  end
  // Output stage: Ready travels backwards from output side, driven by downstream circuitry
  assign out_pipe_ready[NUM_OUT_REGS] = out_ready_i;
  // Output stage: assign module outputs
  assign result_o        = out_pipe_result_q[NUM_OUT_REGS];
  assign status_o        = out_pipe_status_q[NUM_OUT_REGS];
  assign extension_bit_o = out_pipe_extension_bit_q[NUM_OUT_REGS];
  assign class_mask_o    = out_pipe_class_mask_q[NUM_OUT_REGS];
  assign is_class_o      = out_pipe_is_class_q[NUM_OUT_REGS];
  assign tag_o           = out_pipe_tag_q[NUM_OUT_REGS];
  assign aux_o           = out_pipe_aux_q[NUM_OUT_REGS];
  assign out_valid_o     = out_pipe_valid_q[NUM_OUT_REGS];
  assign busy_o          = (| {inp_pipe_valid_q, out_pipe_valid_q});
endmodule
// Copyright 2019 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Stefan Mach <smach@iis.ee.ethz.ch>

module fpnew_opgroup_block #(
  parameter fpnew_pkg::opgroup_e        OpGroup       = fpnew_pkg::ADDMUL,
  // FPU configuration
  parameter int unsigned                Width         = 32,
  parameter logic                       EnableVectors = 1'b1,
  parameter fpnew_pkg::fmt_logic_t      FpFmtMask     = '1,
  parameter fpnew_pkg::ifmt_logic_t     IntFmtMask    = '1,
  parameter fpnew_pkg::fmt_unsigned_t   FmtPipeRegs   = '{default: 0},
  parameter fpnew_pkg::fmt_unit_types_t FmtUnitTypes  = '{default: fpnew_pkg::PARALLEL},
  parameter fpnew_pkg::pipe_config_t    PipeConfig    = fpnew_pkg::BEFORE,
  parameter type                        TagType       = logic,
  // Do not change
  localparam int unsigned NUM_FORMATS  = fpnew_pkg::NUM_FP_FORMATS,
  localparam int unsigned NUM_OPERANDS = fpnew_pkg::num_operands(OpGroup)
) (
  input logic                                     clk_i,
  input logic                                     rst_ni,
  // Input signals
  input logic [NUM_OPERANDS-1:0][Width-1:0]       operands_i,
  input logic [NUM_FORMATS-1:0][NUM_OPERANDS-1:0] is_boxed_i,
  input fpnew_pkg::roundmode_e                    rnd_mode_i,
  input fpnew_pkg::operation_e                    op_i,
  input logic                                     op_mod_i,
  input fpnew_pkg::fp_format_e                    src_fmt_i,
  input fpnew_pkg::fp_format_e                    dst_fmt_i,
  input fpnew_pkg::int_format_e                   int_fmt_i,
  input logic                                     vectorial_op_i,
  input TagType                                   tag_i,
  // Input Handshake
  input  logic                                    in_valid_i,
  output logic                                    in_ready_o,
  input  logic                                    flush_i,
  // Output signals
  output logic [Width-1:0]                        result_o,
  output fpnew_pkg::status_t                      status_o,
  output logic                                    extension_bit_o,
  output TagType                                  tag_o,
  // Output handshake
  output logic                                    out_valid_o,
  input  logic                                    out_ready_i,
  // Indication of valid data in flight
  output logic                                    busy_o
);

  // ----------------
  // Type Definition
  // ----------------
  typedef struct packed {
    logic [Width-1:0]   result;
    fpnew_pkg::status_t status;
    logic               ext_bit;
    TagType             tag;
  } output_t;

  // Handshake signals for the slices
  logic [NUM_FORMATS-1:0] fmt_in_ready, fmt_out_valid, fmt_out_ready, fmt_busy;
  output_t [NUM_FORMATS-1:0] fmt_outputs;

  // -----------
  // Input Side
  // -----------
  assign in_ready_o = in_valid_i & fmt_in_ready[dst_fmt_i]; // Ready is given by selected format

  // -------------------------
  // Generate Parallel Slices
  // -------------------------
  for (genvar fmt = 0; fmt < int'(NUM_FORMATS); fmt++) begin : gen_parallel_slices
    // Some constants for this format
    localparam logic ANY_MERGED = fpnew_pkg::any_enabled_multi(FmtUnitTypes, FpFmtMask);
    localparam logic IS_FIRST_MERGED =
        fpnew_pkg::is_first_enabled_multi(fpnew_pkg::fp_format_e'(fmt), FmtUnitTypes, FpFmtMask);

    // Generate slice only if format enabled
    if (FpFmtMask[fmt] && (FmtUnitTypes[fmt] == fpnew_pkg::PARALLEL)) begin : active_format

      logic in_valid;

      assign in_valid = in_valid_i & (dst_fmt_i == fmt); // enable selected format

      fpnew_opgroup_fmt_slice #(
        .OpGroup       ( OpGroup                      ),
        .FpFormat      ( fpnew_pkg::fp_format_e'(fmt) ),
        .Width         ( Width                        ),
        .EnableVectors ( EnableVectors                ),
        .NumPipeRegs   ( FmtPipeRegs[fmt]             ),
        .PipeConfig    ( PipeConfig                   ),
        .TagType       ( TagType                      )
      ) i_fmt_slice (
        .clk_i,
        .rst_ni,
        .operands_i     ( operands_i               ),
        .is_boxed_i     ( is_boxed_i[fmt]          ),
        .rnd_mode_i,
        .op_i,
        .op_mod_i,
        .vectorial_op_i,
        .tag_i,
        .in_valid_i     ( in_valid                 ),
        .in_ready_o     ( fmt_in_ready[fmt]        ),
        .flush_i,
        .result_o       ( fmt_outputs[fmt].result  ),
        .status_o       ( fmt_outputs[fmt].status  ),
        .extension_bit_o( fmt_outputs[fmt].ext_bit ),
        .tag_o          ( fmt_outputs[fmt].tag     ),
        .out_valid_o    ( fmt_out_valid[fmt]       ),
        .out_ready_i    ( fmt_out_ready[fmt]       ),
        .busy_o         ( fmt_busy[fmt]            )
      );
    // If the format wants to use merged ops, tie off the dangling ones not used here
    end else if (FpFmtMask[fmt] && ANY_MERGED && !IS_FIRST_MERGED) begin : merged_unused

      localparam FMT = fpnew_pkg::get_first_enabled_multi(FmtUnitTypes, FpFmtMask);
      // Ready is split up into formats
      assign fmt_in_ready[fmt]  = fmt_in_ready[int'(FMT)];

      assign fmt_out_valid[fmt] = 1'b0; // don't emit values
      assign fmt_busy[fmt]      = 1'b0; // never busy
      // Outputs are don't care
      assign fmt_outputs[fmt].result  = '{default: fpnew_pkg::DONT_CARE};
      assign fmt_outputs[fmt].status  = '{default: fpnew_pkg::DONT_CARE};
      assign fmt_outputs[fmt].ext_bit = fpnew_pkg::DONT_CARE;
      assign fmt_outputs[fmt].tag     = TagType'(fpnew_pkg::DONT_CARE);

    // Tie off disabled formats
    end else if (!FpFmtMask[fmt] || (FmtUnitTypes[fmt] == fpnew_pkg::DISABLED)) begin : disable_fmt
      assign fmt_in_ready[fmt]  = 1'b0; // don't accept operations
      assign fmt_out_valid[fmt] = 1'b0; // don't emit values
      assign fmt_busy[fmt]      = 1'b0; // never busy
      // Outputs are don't care
      assign fmt_outputs[fmt].result  = '{default: fpnew_pkg::DONT_CARE};
      assign fmt_outputs[fmt].status  = '{default: fpnew_pkg::DONT_CARE};
      assign fmt_outputs[fmt].ext_bit = fpnew_pkg::DONT_CARE;
      assign fmt_outputs[fmt].tag     = TagType'(fpnew_pkg::DONT_CARE);
    end
  end

  // ----------------------
  // Generate Merged Slice
  // ----------------------
  if (fpnew_pkg::any_enabled_multi(FmtUnitTypes, FpFmtMask)) begin : gen_merged_slice

    localparam FMT = fpnew_pkg::get_first_enabled_multi(FmtUnitTypes, FpFmtMask);
    localparam REG = fpnew_pkg::get_num_regs_multi(FmtPipeRegs, FmtUnitTypes, FpFmtMask);

    logic in_valid;

    assign in_valid = in_valid_i & (FmtUnitTypes[dst_fmt_i] == fpnew_pkg::MERGED);

    fpnew_opgroup_multifmt_slice #(
      .OpGroup       ( OpGroup          ),
      .Width         ( Width            ),
      .FpFmtConfig   ( FpFmtMask        ),
      .IntFmtConfig  ( IntFmtMask       ),
      .EnableVectors ( EnableVectors    ),
      .NumPipeRegs   ( REG              ),
      .PipeConfig    ( PipeConfig       ),
      .TagType       ( TagType          )
    ) i_multifmt_slice (
      .clk_i,
      .rst_ni,
      .operands_i,
      .is_boxed_i,
      .rnd_mode_i,
      .op_i,
      .op_mod_i,
      .src_fmt_i,
      .dst_fmt_i,
      .int_fmt_i,
      .vectorial_op_i,
      .tag_i,
      .in_valid_i      ( in_valid                 ),
      .in_ready_o      ( fmt_in_ready[FMT]        ),
      .flush_i,
      .result_o        ( fmt_outputs[FMT].result  ),
      .status_o        ( fmt_outputs[FMT].status  ),
      .extension_bit_o ( fmt_outputs[FMT].ext_bit ),
      .tag_o           ( fmt_outputs[FMT].tag     ),
      .out_valid_o     ( fmt_out_valid[FMT]       ),
      .out_ready_i     ( fmt_out_ready[FMT]       ),
      .busy_o          ( fmt_busy[FMT]            )
    );

  end

  // ------------------
  // Arbitrate Outputs
  // ------------------
  output_t arbiter_output;

  // Round-Robin arbiter to decide which result to use
  rr_arb_tree #(
    .NumIn     ( NUM_FORMATS ),
    .DataType  ( output_t    ),
    .AxiVldRdy ( 1'b1        )
  ) i_arbiter (
    .clk_i,
    .rst_ni,
    .flush_i,
    .rr_i   ( '0             ),
    .req_i  ( fmt_out_valid  ),
    .gnt_o  ( fmt_out_ready  ),
    .data_i ( fmt_outputs    ),
    .gnt_i  ( out_ready_i    ),
    .req_o  ( out_valid_o    ),
    .data_o ( arbiter_output ),
    .idx_o  ( /* unused */   )
  );

  // Unpack output
  assign result_o        = arbiter_output.result;
  assign status_o        = arbiter_output.status;
  assign extension_bit_o = arbiter_output.ext_bit;
  assign tag_o           = arbiter_output.tag;

  assign busy_o = (| fmt_busy);

endmodule
// Copyright 2019 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Stefan Mach <smach@iis.ee.ethz.ch>

module fpnew_opgroup_fmt_slice #(
  parameter fpnew_pkg::opgroup_e     OpGroup       = fpnew_pkg::ADDMUL,
  parameter fpnew_pkg::fp_format_e   FpFormat      = fpnew_pkg::fp_format_e'(0),
  // FPU configuration
  parameter int unsigned             Width         = 32,
  parameter logic                    EnableVectors = 1'b1,
  parameter int unsigned             NumPipeRegs   = 0,
  parameter fpnew_pkg::pipe_config_t PipeConfig    = fpnew_pkg::BEFORE,
  parameter type                     TagType       = logic,
  // Do not change
  localparam int unsigned NUM_OPERANDS = fpnew_pkg::num_operands(OpGroup)
) (
  input logic                               clk_i,
  input logic                               rst_ni,
  // Input signals
  input logic [NUM_OPERANDS-1:0][Width-1:0] operands_i,
  input logic [NUM_OPERANDS-1:0]            is_boxed_i,
  input fpnew_pkg::roundmode_e              rnd_mode_i,
  input fpnew_pkg::operation_e              op_i,
  input logic                               op_mod_i,
  input logic                               vectorial_op_i,
  input TagType                             tag_i,
  // Input Handshake
  input  logic                              in_valid_i,
  output logic                              in_ready_o,
  input  logic                              flush_i,
  // Output signals
  output logic [Width-1:0]                  result_o,
  output fpnew_pkg::status_t                status_o,
  output logic                              extension_bit_o,
  output TagType                            tag_o,
  // Output handshake
  output logic                              out_valid_o,
  input  logic                              out_ready_i,
  // Indication of valid data in flight
  output logic                              busy_o
);

  localparam int unsigned FP_WIDTH  = fpnew_pkg::fp_width(FpFormat);
  localparam int unsigned NUM_LANES = fpnew_pkg::num_lanes(Width, FpFormat, EnableVectors);


  logic [NUM_LANES-1:0] lane_in_ready, lane_out_valid; // Handshake signals for the lanes
  logic                 vectorial_op;

  logic [NUM_LANES*FP_WIDTH-1:0] slice_result;
  logic [Width-1:0]              slice_regular_result, slice_class_result, slice_vec_class_result;

  fpnew_pkg::status_t    [NUM_LANES-1:0] lane_status;
  logic                  [NUM_LANES-1:0] lane_ext_bit; // only the first one is actually used
  fpnew_pkg::classmask_e [NUM_LANES-1:0] lane_class_mask;
  TagType                [NUM_LANES-1:0] lane_tags; // only the first one is actually used
  logic                  [NUM_LANES-1:0] lane_vectorial, lane_busy, lane_is_class; // dito

  logic result_is_vector, result_is_class;

  // -----------
  // Input Side
  // -----------
  assign in_ready_o   = lane_in_ready[0]; // Upstream ready is given by first lane
  assign vectorial_op = vectorial_op_i & EnableVectors; // only do vectorial stuff if enabled

  // ---------------
  // Generate Lanes
  // ---------------
  for (genvar lane = 0; lane < int'(NUM_LANES); lane++) begin : gen_num_lanes
    logic [FP_WIDTH-1:0] local_result; // lane-local results
    logic                local_sign;

    // Generate instances only if needed, lane 0 always generated
    if ((lane == 0) || EnableVectors) begin : active_lane
      logic in_valid, out_valid, out_ready; // lane-local handshake

      logic [NUM_OPERANDS-1:0][FP_WIDTH-1:0] local_operands; // lane-local operands
      logic [FP_WIDTH-1:0]                   op_result;      // lane-local results
      fpnew_pkg::status_t                    op_status;

      assign in_valid = in_valid_i & ((lane == 0) | vectorial_op); // upper lanes only for vectors
      // Slice out the operands for this lane
      always_comb begin : prepare_input
        for (int i = 0; i < int'(NUM_OPERANDS); i++) begin
          local_operands[i] = operands_i[i][(unsigned'(lane)+1)*FP_WIDTH-1:unsigned'(lane)*FP_WIDTH];
        end
      end

      // Instantiate the operation from the selected opgroup
      if (OpGroup == fpnew_pkg::ADDMUL) begin : lane_instance
        fpnew_fma #(
          .FpFormat    ( FpFormat    ),
          .NumPipeRegs ( NumPipeRegs ),
          .PipeConfig  ( PipeConfig  ),
          .TagType     ( TagType     ),
          .AuxType     ( logic       )
        ) i_fma (
          .clk_i,
          .rst_ni,
          .operands_i      ( local_operands               ),
          .is_boxed_i      ( is_boxed_i[NUM_OPERANDS-1:0] ),
          .rnd_mode_i,
          .op_i,
          .op_mod_i,
          .tag_i,
          .aux_i           ( vectorial_op         ), // Remember whether operation was vectorial
          .in_valid_i      ( in_valid             ),
          .in_ready_o      ( lane_in_ready[lane]  ),
          .flush_i,
          .result_o        ( op_result            ),
          .status_o        ( op_status            ),
          .extension_bit_o ( lane_ext_bit[lane]   ),
          .tag_o           ( lane_tags[lane]      ),
          .aux_o           ( lane_vectorial[lane] ),
          .out_valid_o     ( out_valid            ),
          .out_ready_i     ( out_ready            ),
          .busy_o          ( lane_busy[lane]      )
        );
        assign lane_is_class[lane]   = 1'b0;
        assign lane_class_mask[lane] = fpnew_pkg::NEGINF;
      end else if (OpGroup == fpnew_pkg::DIVSQRT) begin : lane_instance
        // fpnew_divsqrt #(
        //   .FpFormat   (FpFormat),
        //   .NumPipeRegs(NumPipeRegs),
        //   .PipeConfig (PipeConfig),
        //   .TagType    (TagType),
        //   .AuxType    (logic)
        // ) i_divsqrt (
        //   .clk_i,
        //   .rst_ni,
        //   .operands_i      ( local_operands               ),
        //   .is_boxed_i      ( is_boxed_i[NUM_OPERANDS-1:0] ),
        //   .rnd_mode_i,
        //   .op_i,
        //   .op_mod_i,
        //   .tag_i,
        //   .aux_i           ( vectorial_op         ), // Remember whether operation was vectorial
        //   .in_valid_i      ( in_valid             ),
        //   .in_ready_o      ( lane_in_ready[lane]  ),
        //   .flush_i,
        //   .result_o        ( op_result            ),
        //   .status_o        ( op_status            ),
        //   .extension_bit_o ( lane_ext_bit[lane]   ),
        //   .tag_o           ( lane_tags[lane]      ),
        //   .aux_o           ( lane_vectorial[lane] ),
        //   .out_valid_o     ( out_valid            ),
        //   .out_ready_i     ( out_ready            ),
        //   .busy_o          ( lane_busy[lane]      )
        // );
        // assign lane_is_class[lane] = 1'b0;
      end else if (OpGroup == fpnew_pkg::NONCOMP) begin : lane_instance
        fpnew_noncomp #(
          .FpFormat   (FpFormat),
          .NumPipeRegs(NumPipeRegs),
          .PipeConfig (PipeConfig),
          .TagType    (TagType),
          .AuxType    (logic)
        ) i_noncomp (
          .clk_i,
          .rst_ni,
          .operands_i      ( local_operands               ),
          .is_boxed_i      ( is_boxed_i[NUM_OPERANDS-1:0] ),
          .rnd_mode_i,
          .op_i,
          .op_mod_i,
          .tag_i,
          .aux_i           ( vectorial_op          ), // Remember whether operation was vectorial
          .in_valid_i      ( in_valid              ),
          .in_ready_o      ( lane_in_ready[lane]   ),
          .flush_i,
          .result_o        ( op_result             ),
          .status_o        ( op_status             ),
          .extension_bit_o ( lane_ext_bit[lane]    ),
          .class_mask_o    ( lane_class_mask[lane] ),
          .is_class_o      ( lane_is_class[lane]   ),
          .tag_o           ( lane_tags[lane]       ),
          .aux_o           ( lane_vectorial[lane]  ),
          .out_valid_o     ( out_valid             ),
          .out_ready_i     ( out_ready             ),
          .busy_o          ( lane_busy[lane]       )
        );
      end // ADD OTHER OPTIONS HERE

      // Handshakes are only done if the lane is actually used
      assign out_ready            = out_ready_i & ((lane == 0) | result_is_vector);
      assign lane_out_valid[lane] = out_valid   & ((lane == 0) | result_is_vector);

      // Properly NaN-box or sign-extend the slice result if not in use
      assign local_result      = lane_out_valid[lane] ? op_result : '{default: lane_ext_bit[0]};
      assign lane_status[lane] = lane_out_valid[lane] ? op_status : '0;

    // Otherwise generate constant sign-extension
    end else begin
      assign lane_out_valid[lane] = 1'b0; // unused lane
      assign lane_in_ready[lane]  = 1'b0; // unused lane
      assign local_result         = '{default: lane_ext_bit[0]}; // sign-extend/nan box
      assign lane_status[lane]    = '0;
      assign lane_busy[lane]      = 1'b0;
      assign lane_is_class[lane]  = 1'b0;
    end

    // Insert lane result into slice result
    assign slice_result[(unsigned'(lane)+1)*FP_WIDTH-1:unsigned'(lane)*FP_WIDTH] = local_result;

    // Create Classification results
    if ((lane+1)*8 <= Width) begin : vectorial_class // vectorial class blocks are 8bits in size
      assign local_sign = (lane_class_mask[lane] == fpnew_pkg::NEGINF ||
                           lane_class_mask[lane] == fpnew_pkg::NEGNORM ||
                           lane_class_mask[lane] == fpnew_pkg::NEGSUBNORM ||
                           lane_class_mask[lane] == fpnew_pkg::NEGZERO);
      // Write the current block segment
      assign slice_vec_class_result[(lane+1)*8-1:lane*8] = {
        local_sign,  // BIT 7
        ~local_sign, // BIT 6
        lane_class_mask[lane] == fpnew_pkg::QNAN, // BIT 5
        lane_class_mask[lane] == fpnew_pkg::SNAN, // BIT 4
        lane_class_mask[lane] == fpnew_pkg::POSZERO
            || lane_class_mask[lane] == fpnew_pkg::NEGZERO, // BIT 3
        lane_class_mask[lane] == fpnew_pkg::POSSUBNORM
            || lane_class_mask[lane] == fpnew_pkg::NEGSUBNORM, // BIT 2
        lane_class_mask[lane] == fpnew_pkg::POSNORM
            || lane_class_mask[lane] == fpnew_pkg::NEGNORM, // BIT 1
        lane_class_mask[lane] == fpnew_pkg::POSINF
            || lane_class_mask[lane] == fpnew_pkg::NEGINF // BIT 0
      };
    end
  end

  // ------------
  // Output Side
  // ------------
  assign result_is_vector = lane_vectorial[0];
  assign result_is_class  = lane_is_class[0];

  assign slice_regular_result = $signed({extension_bit_o, slice_result});

  localparam int unsigned CLASS_VEC_BITS = (NUM_LANES*8 > Width) ? 8 * (Width / 8) : NUM_LANES*8;

  // Pad out unused vec_class bits
  if (CLASS_VEC_BITS < Width) begin : pad_vectorial_class
    assign slice_vec_class_result[Width-1:CLASS_VEC_BITS] = '0;
  end

  // localparam logic [Width-1:0] CLASS_VEC_MASK = 2**CLASS_VEC_BITS - 1;

  assign slice_class_result = result_is_vector ? slice_vec_class_result : lane_class_mask[0];

  // Select the proper result
  assign result_o = result_is_class ? slice_class_result : slice_regular_result;

  assign extension_bit_o                              = lane_ext_bit[0]; // upper lanes unused
  assign tag_o                                        = lane_tags[0];    // upper lanes unused
  assign busy_o                                       = (| lane_busy);
  assign out_valid_o                                  = lane_out_valid[0]; // upper lanes unused


  // Collapse the lane status
  always_comb begin : output_processing
    // Collapse the status
    automatic fpnew_pkg::status_t temp_status;
    temp_status = '0;
    for (int i = 0; i < int'(NUM_LANES); i++)
      temp_status |= lane_status[i];
    status_o = temp_status;
  end
endmodule
// Copyright 2019 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Stefan Mach <smach@iis.ee.ethz.ch>

//`include "common_cells/registers.svh"

module fpnew_opgroup_multifmt_slice #(
  parameter fpnew_pkg::opgroup_e     OpGroup       = fpnew_pkg::CONV,
  parameter int unsigned             Width         = 64,
  // FPU configuration
  parameter fpnew_pkg::fmt_logic_t   FpFmtConfig   = '1,
  parameter fpnew_pkg::ifmt_logic_t  IntFmtConfig  = '1,
  parameter logic                    EnableVectors = 1'b1,
  parameter int unsigned             NumPipeRegs   = 0,
  parameter fpnew_pkg::pipe_config_t PipeConfig    = fpnew_pkg::BEFORE,
  parameter type                     TagType       = logic,
  // Do not change
  localparam int unsigned NUM_OPERANDS = fpnew_pkg::num_operands(OpGroup),
  localparam int unsigned NUM_FORMATS  = fpnew_pkg::NUM_FP_FORMATS
) (
  input logic                                     clk_i,
  input logic                                     rst_ni,
  // Input signals
  input logic [NUM_OPERANDS-1:0][Width-1:0]       operands_i,
  input logic [NUM_FORMATS-1:0][NUM_OPERANDS-1:0] is_boxed_i,
  input fpnew_pkg::roundmode_e                    rnd_mode_i,
  input fpnew_pkg::operation_e                    op_i,
  input logic                                     op_mod_i,
  input fpnew_pkg::fp_format_e                    src_fmt_i,
  input fpnew_pkg::fp_format_e                    dst_fmt_i,
  input fpnew_pkg::int_format_e                   int_fmt_i,
  input logic                                     vectorial_op_i,
  input TagType                                   tag_i,
  // Input Handshake
  input  logic                                    in_valid_i,
  output logic                                    in_ready_o,
  input  logic                                    flush_i,
  // Output signals
  output logic [Width-1:0]                        result_o,
  output fpnew_pkg::status_t                      status_o,
  output logic                                    extension_bit_o,
  output TagType                                  tag_o,
  // Output handshake
  output logic                                    out_valid_o,
  input  logic                                    out_ready_i,
  // Indication of valid data in flight
  output logic                                    busy_o
);

  localparam int unsigned MAX_FP_WIDTH   = fpnew_pkg::max_fp_width(FpFmtConfig);
  localparam int unsigned MAX_INT_WIDTH  = fpnew_pkg::max_int_width(IntFmtConfig);
  localparam int unsigned NUM_LANES = fpnew_pkg::max_num_lanes(Width, FpFmtConfig, 1'b1);
  localparam int unsigned NUM_INT_FORMATS = fpnew_pkg::NUM_INT_FORMATS;
  // We will send the format information along with the data
  localparam int unsigned FMT_BITS =
      fpnew_pkg::maximum($clog2(NUM_FORMATS), $clog2(NUM_INT_FORMATS));
  localparam int unsigned AUX_BITS = FMT_BITS + 2; // also add vectorial and integer flags

  logic [NUM_LANES-1:0] lane_in_ready, lane_out_valid; // Handshake signals for the lanes
  logic                 vectorial_op;
  logic [FMT_BITS-1:0]  dst_fmt; // destination format to pass along with operation
  logic [AUX_BITS-1:0]  aux_data;

  // additional flags for CONV
  logic       dst_fmt_is_int, dst_is_cpk;
  logic [1:0] dst_vec_op; // info for vectorial results (for packing)
  logic [2:0] target_aux_d, target_aux_q;
  logic       is_up_cast, is_down_cast;

  logic [NUM_FORMATS-1:0][Width-1:0]     fmt_slice_result;
  logic [NUM_INT_FORMATS-1:0][Width-1:0] ifmt_slice_result;
  logic [Width-1:0]                      conv_slice_result;


  logic [Width-1:0] conv_target_d, conv_target_q; // vectorial conversions update a register

  fpnew_pkg::status_t [NUM_LANES-1:0]   lane_status;
  logic   [NUM_LANES-1:0]               lane_ext_bit; // only the first one is actually used
  TagType [NUM_LANES-1:0]               lane_tags; // only the first one is actually used
  logic   [NUM_LANES-1:0][AUX_BITS-1:0] lane_aux; // only the first one is actually used
  logic   [NUM_LANES-1:0]               lane_busy; // dito

  logic                result_is_vector;
  logic [FMT_BITS-1:0] result_fmt;
  logic                result_fmt_is_int, result_is_cpk;
  logic [1:0]          result_vec_op; // info for vectorial results (for packing)

  // -----------
  // Input Side
  // -----------
  assign in_ready_o   = lane_in_ready[0]; // Upstream ready is given by first lane
  assign vectorial_op = vectorial_op_i & EnableVectors; // only do vectorial stuff if enabled

  // Cast-and-Pack ops are encoded in operation and modifier
  assign dst_fmt_is_int = (OpGroup == fpnew_pkg::CONV) & (op_i == fpnew_pkg::F2I);
  assign dst_is_cpk     = (OpGroup == fpnew_pkg::CONV) & (op_i == fpnew_pkg::CPKAB ||
                                                          op_i == fpnew_pkg::CPKCD);
  assign dst_vec_op     = (OpGroup == fpnew_pkg::CONV) & {(op_i == fpnew_pkg::CPKCD), op_mod_i};

  assign is_up_cast   = (fpnew_pkg::fp_width(dst_fmt_i) > fpnew_pkg::fp_width(src_fmt_i));
  assign is_down_cast = (fpnew_pkg::fp_width(dst_fmt_i) < fpnew_pkg::fp_width(src_fmt_i));

  // The destination format is the int format for F2I casts
  assign dst_fmt    = dst_fmt_is_int ? int_fmt_i : dst_fmt_i;

  // The data sent along consists of the vectorial flag and format bits
  assign aux_data      = {dst_fmt_is_int, vectorial_op, dst_fmt};
  assign target_aux_d  = {dst_vec_op, dst_is_cpk};

  // CONV passes one operand for assembly after the unit: opC for cpk, opB for others
  if (OpGroup == fpnew_pkg::CONV) begin : conv_target
    assign conv_target_d = dst_is_cpk ? operands_i[2] : operands_i[1];
  end

  // For 2-operand units, prepare boxing info
  logic [NUM_FORMATS-1:0]      is_boxed_1op;
  logic [NUM_FORMATS-1:0][1:0] is_boxed_2op;

  always_comb begin : boxed_2op
    for (int fmt = 0; fmt < NUM_FORMATS; fmt++) begin
      is_boxed_1op[fmt] = is_boxed_i[fmt][0];
      is_boxed_2op[fmt] = is_boxed_i[fmt][1:0];
    end
  end

  // ---------------
  // Generate Lanes
  // ---------------
  for (genvar lane = 0; lane < int'(NUM_LANES); lane++) begin : gen_num_lanes
    localparam int unsigned LANE = unsigned'(lane); // unsigned to please the linter
    // Get a mask of active formats for this lane
    localparam fpnew_pkg::fmt_logic_t ACTIVE_FORMATS =
        fpnew_pkg::get_lane_formats(Width, FpFmtConfig, LANE);
    localparam fpnew_pkg::ifmt_logic_t ACTIVE_INT_FORMATS =
        fpnew_pkg::get_lane_int_formats(Width, FpFmtConfig, IntFmtConfig, LANE);
    localparam int unsigned MAX_WIDTH = fpnew_pkg::max_fp_width(ACTIVE_FORMATS);

    // Cast-specific parameters
    localparam fpnew_pkg::fmt_logic_t CONV_FORMATS =
        fpnew_pkg::get_conv_lane_formats(Width, FpFmtConfig, LANE);
    localparam fpnew_pkg::ifmt_logic_t CONV_INT_FORMATS =
        fpnew_pkg::get_conv_lane_int_formats(Width, FpFmtConfig, IntFmtConfig, LANE);
    localparam int unsigned CONV_WIDTH = fpnew_pkg::max_fp_width(CONV_FORMATS);

    // Lane parameters from Opgroup
    localparam fpnew_pkg::fmt_logic_t LANE_FORMATS = (OpGroup == fpnew_pkg::CONV)
                                                     ? CONV_FORMATS : ACTIVE_FORMATS;
    localparam int unsigned LANE_WIDTH = (OpGroup == fpnew_pkg::CONV) ? CONV_WIDTH : MAX_WIDTH;

    logic [LANE_WIDTH-1:0] local_result; // lane-local results

    // Generate instances only if needed, lane 0 always generated
    if ((lane == 0) || EnableVectors) begin : active_lane
      logic in_valid, out_valid, out_ready; // lane-local handshake

      logic [NUM_OPERANDS-1:0][LANE_WIDTH-1:0] local_operands;  // lane-local oprands
      logic [LANE_WIDTH-1:0]                   op_result;       // lane-local results
      fpnew_pkg::status_t                      op_status;

      assign in_valid = in_valid_i & ((lane == 0) | vectorial_op); // upper lanes only for vectors

      // Slice out the operands for this lane, upper bits are ignored in the unit
      always_comb begin : prepare_input
        for (int unsigned i = 0; i < NUM_OPERANDS; i++) begin
          local_operands[i] = operands_i[i] >> LANE*fpnew_pkg::fp_width(src_fmt_i);
        end

        // override operand 0 for some conversions
        if (OpGroup == fpnew_pkg::CONV) begin
          // Source is an integer
          if (op_i == fpnew_pkg::I2F) begin
            local_operands[0] = operands_i[0] >> LANE*fpnew_pkg::int_width(int_fmt_i);
          // vectorial F2F up casts
          end else if (op_i == fpnew_pkg::F2F) begin
            if (vectorial_op && op_mod_i && is_up_cast) begin // up cast with upper half
              local_operands[0] = operands_i[0] >> LANE*fpnew_pkg::fp_width(src_fmt_i) +
                                                   MAX_FP_WIDTH/2;
            end
          // CPK
          end else if (dst_is_cpk) begin
            if (lane == 1) begin
              local_operands[0] = operands_i[1][LANE_WIDTH-1:0]; // using opB as second argument
            end
          end
        end
      end

      // Instantiate the operation from the selected opgroup
      if (OpGroup == fpnew_pkg::ADDMUL) begin : lane_instance
        fpnew_fma_multi #(
          .FpFmtConfig ( LANE_FORMATS         ),
          .NumPipeRegs ( NumPipeRegs          ),
          .PipeConfig  ( PipeConfig           ),
          .TagType     ( TagType              ),
          .AuxType     ( logic [AUX_BITS-1:0] )
        ) i_fpnew_fma_multi (
          .clk_i,
          .rst_ni,
          .operands_i      ( local_operands  ),
          .is_boxed_i,
          .rnd_mode_i,
          .op_i,
          .op_mod_i,
          .src_fmt_i,
          .dst_fmt_i,
          .tag_i,
          .aux_i           ( aux_data            ),
          .in_valid_i      ( in_valid            ),
          .in_ready_o      ( lane_in_ready[lane] ),
          .flush_i,
          .result_o        ( op_result           ),
          .status_o        ( op_status           ),
          .extension_bit_o ( lane_ext_bit[lane]  ),
          .tag_o           ( lane_tags[lane]     ),
          .aux_o           ( lane_aux[lane]      ),
          .out_valid_o     ( out_valid           ),
          .out_ready_i     ( out_ready           ),
          .busy_o          ( lane_busy[lane]     )
        );

      end else if (OpGroup == fpnew_pkg::DIVSQRT) begin : lane_instance
        fpnew_divsqrt_multi #(
          .FpFmtConfig ( LANE_FORMATS         ),
          .NumPipeRegs ( NumPipeRegs          ),
          .PipeConfig  ( PipeConfig           ),
          .TagType     ( TagType              ),
          .AuxType     ( logic [AUX_BITS-1:0] )
        ) i_fpnew_divsqrt_multi (
          .clk_i,
          .rst_ni,
          .operands_i      ( local_operands[1:0] ), // 2 operands
          .is_boxed_i      ( is_boxed_2op        ), // 2 operands
          .rnd_mode_i,
          .op_i,
          .dst_fmt_i,
          .tag_i,
          .aux_i           ( aux_data            ),
          .in_valid_i      ( in_valid            ),
          .in_ready_o      ( lane_in_ready[lane] ),
          .flush_i,
          .result_o        ( op_result           ),
          .status_o        ( op_status           ),
          .extension_bit_o ( lane_ext_bit[lane]  ),
          .tag_o           ( lane_tags[lane]     ),
          .aux_o           ( lane_aux[lane]      ),
          .out_valid_o     ( out_valid           ),
          .out_ready_i     ( out_ready           ),
          .busy_o          ( lane_busy[lane]     )
        );
      end else if (OpGroup == fpnew_pkg::NONCOMP) begin : lane_instance

      end else if (OpGroup == fpnew_pkg::CONV) begin : lane_instance
        fpnew_cast_multi #(
          .FpFmtConfig  ( LANE_FORMATS         ),
          .IntFmtConfig ( CONV_INT_FORMATS     ),
          .NumPipeRegs  ( NumPipeRegs          ),
          .PipeConfig   ( PipeConfig           ),
          .TagType      ( TagType              ),
          .AuxType      ( logic [AUX_BITS-1:0] )
        ) i_fpnew_cast_multi (
          .clk_i,
          .rst_ni,
          .operands_i      ( local_operands[0]   ),
          .is_boxed_i      ( is_boxed_1op        ),
          .rnd_mode_i,
          .op_i,
          .op_mod_i,
          .src_fmt_i,
          .dst_fmt_i,
          .int_fmt_i,
          .tag_i,
          .aux_i           ( aux_data            ),
          .in_valid_i      ( in_valid            ),
          .in_ready_o      ( lane_in_ready[lane] ),
          .flush_i,
          .result_o        ( op_result           ),
          .status_o        ( op_status           ),
          .extension_bit_o ( lane_ext_bit[lane]  ),
          .tag_o           ( lane_tags[lane]     ),
          .aux_o           ( lane_aux[lane]      ),
          .out_valid_o     ( out_valid           ),
          .out_ready_i     ( out_ready           ),
          .busy_o          ( lane_busy[lane]     )
        );
      end // ADD OTHER OPTIONS HERE

      // Handshakes are only done if the lane is actually used
      assign out_ready            = out_ready_i & ((lane == 0) | result_is_vector);
      assign lane_out_valid[lane] = out_valid & ((lane == 0) | result_is_vector);

      // Properly NaN-box or sign-extend the slice result if not in use
      assign local_result      = lane_out_valid[lane] ? op_result : '{default: lane_ext_bit[0]};
      assign lane_status[lane] = lane_out_valid[lane] ? op_status : '0;

    // Otherwise generate constant sign-extension
    end else begin : inactive_lane
      assign lane_out_valid[lane] = 1'b0; // unused lane
      assign lane_in_ready[lane]  = 1'b0; // unused lane
      assign local_result         = '{default: lane_ext_bit[0]}; // sign-extend/nan box
      assign lane_status[lane]    = '0;
      assign lane_busy[lane]      = 1'b0;
    end

    // Generate result packing depending on float format
    for (genvar fmt = 0; fmt < NUM_FORMATS; fmt++) begin : pack_fp_result
      // Set up some constants
      localparam int unsigned FP_WIDTH = fpnew_pkg::fp_width(fpnew_pkg::fp_format_e'(fmt));
      // only for active formats within the lane
      if (ACTIVE_FORMATS[fmt]) begin
        assign fmt_slice_result[fmt][(LANE+1)*FP_WIDTH-1:LANE*FP_WIDTH] =
            local_result[FP_WIDTH-1:0];
      end else if ((LANE+1)*FP_WIDTH <= Width) begin
        assign fmt_slice_result[fmt][(LANE+1)*FP_WIDTH-1:LANE*FP_WIDTH] =
            '{default: lane_ext_bit[LANE]};
      end else if (LANE*FP_WIDTH < Width) begin
        assign fmt_slice_result[fmt][Width-1:LANE*FP_WIDTH] =
            '{default: lane_ext_bit[LANE]};
      end
    end

    // Generate result packing depending on integer format
    if (OpGroup == fpnew_pkg::CONV) begin : int_results_enabled
      for (genvar ifmt = 0; ifmt < NUM_INT_FORMATS; ifmt++) begin : pack_int_result
        // Set up some constants
        localparam int unsigned INT_WIDTH = fpnew_pkg::int_width(fpnew_pkg::int_format_e'(ifmt));
        if (ACTIVE_INT_FORMATS[ifmt]) begin
          assign ifmt_slice_result[ifmt][(LANE+1)*INT_WIDTH-1:LANE*INT_WIDTH] =
            local_result[INT_WIDTH-1:0];
        end else if ((LANE+1)*INT_WIDTH <= Width) begin
          assign ifmt_slice_result[ifmt][(LANE+1)*INT_WIDTH-1:LANE*INT_WIDTH] = '0;
        end else if (LANE*INT_WIDTH < Width) begin
          assign ifmt_slice_result[ifmt][Width-1:LANE*INT_WIDTH] = '0;
        end
      end
    end
  end

  // Extend slice result if needed
  for (genvar fmt = 0; fmt < NUM_FORMATS; fmt++) begin : extend_fp_result
    // Set up some constants
    localparam int unsigned FP_WIDTH = fpnew_pkg::fp_width(fpnew_pkg::fp_format_e'(fmt));
    if (NUM_LANES*FP_WIDTH < Width)
      assign fmt_slice_result[fmt][Width-1:NUM_LANES*FP_WIDTH] = '{default: lane_ext_bit[0]};
  end

  // Mute int results if unused
  for (genvar ifmt = 0; ifmt < NUM_INT_FORMATS; ifmt++) begin : int_results_disabled
    if (OpGroup != fpnew_pkg::CONV) begin : mute_int_result
      assign ifmt_slice_result[ifmt] = '0;
    end
  end

  // Bypass lanes with target operand for vectorial casts
  if (OpGroup == fpnew_pkg::CONV) begin : target_regs
    // Bypass pipeline signals, index i holds signal after i register stages
    logic [0:NumPipeRegs][Width-1:0] byp_pipe_target_q;
    logic [0:NumPipeRegs][2:0]       byp_pipe_aux_q;
    logic [0:NumPipeRegs]            byp_pipe_valid_q;
    // Ready signal is combinatorial for all stages
    logic [0:NumPipeRegs] byp_pipe_ready;

    // Input stage: First element of pipeline is taken from inputs
    assign byp_pipe_target_q[0]  = conv_target_d;
    assign byp_pipe_aux_q[0]     = target_aux_d;
    assign byp_pipe_valid_q[0]   = in_valid_i & vectorial_op;
    // Generate the register stages
    for (genvar i = 0; i < NumPipeRegs; i++) begin : gen_bypass_pipeline
      // Internal register enable for this stage
      logic reg_ena;
      // Determine the ready signal of the current stage - advance the pipeline:
      // 1. if the next stage is ready for our data
      // 2. if the next stage only holds a bubble (not valid) -> we can pop it
      assign byp_pipe_ready[i] = byp_pipe_ready[i+1] | ~byp_pipe_valid_q[i+1];
      // Valid: enabled by ready signal, synchronous clear with the flush signal
      `FFLARNC(byp_pipe_valid_q[i+1], byp_pipe_valid_q[i], byp_pipe_ready[i], flush_i, 1'b0, clk_i, rst_ni)
      // Enable register if pipleine ready and a valid data item is present
      assign reg_ena = byp_pipe_ready[i] & byp_pipe_valid_q[i];
      // Generate the pipeline registers within the stages, use enable-registers
      `FFL(byp_pipe_target_q[i+1],  byp_pipe_target_q[i],  reg_ena, '0)
      `FFL(byp_pipe_aux_q[i+1],     byp_pipe_aux_q[i],     reg_ena, '0)
    end
    // Output stage: Ready travels backwards from output side, driven by downstream circuitry
    assign byp_pipe_ready[NumPipeRegs] = out_ready_i & result_is_vector;
    // Output stage: assign module outputs
    assign conv_target_q = byp_pipe_target_q[NumPipeRegs];

    // decode the aux data
    assign {result_vec_op, result_is_cpk} = byp_pipe_aux_q[NumPipeRegs];
  end else begin : no_conv
    assign {result_vec_op, result_is_cpk} = '0;
  end

  // ------------
  // Output Side
  // ------------
  assign {result_fmt_is_int, result_is_vector, result_fmt} = lane_aux[0];

  assign result_o = result_fmt_is_int
                    ? ifmt_slice_result[result_fmt]
                    : fmt_slice_result[result_fmt];

  assign extension_bit_o = lane_ext_bit[0]; // don't care about upper ones
  assign tag_o           = lane_tags[0];    // don't care about upper ones
  assign busy_o          = (| lane_busy);

  assign out_valid_o     = lane_out_valid[0]; // don't care about upper ones

  // Collapse the status
  always_comb begin : output_processing
    // Collapse the status
    automatic fpnew_pkg::status_t temp_status;
    temp_status = '0;
    for (int i = 0; i < int'(NUM_LANES); i++)
      temp_status |= lane_status[i];
    status_o = temp_status;
  end
endmodule
// Copyright 2019 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Stefan Mach <smach@iis.ee.ethz.ch>

module fpnew_rounding #(
  parameter int unsigned AbsWidth=2 // Width of the abolute value, without sign bit
) (
  // Input value
  input logic [AbsWidth-1:0]   abs_value_i,             // absolute value without sign
  input logic                  sign_i,
  // Rounding information
  input logic [1:0]            round_sticky_bits_i,     // round and sticky bits {RS}
  input fpnew_pkg::roundmode_e rnd_mode_i,
  input logic                  effective_subtraction_i, // sign of inputs affects rounding of zeroes
  // Output value
  output logic [AbsWidth-1:0]  abs_rounded_o,           // absolute value without sign
  output logic                 sign_o,
  // Output classification
  output logic                 exact_zero_o             // output is an exact zero
);

  logic round_up; // Rounding decision

  // Take the rounding decision according to RISC-V spec
  // RoundMode | Mnemonic | Meaning
  // :--------:|:--------:|:-------
  //    000    |   RNE    | Round to Nearest, ties to Even
  //    001    |   RTZ    | Round towards Zero
  //    010    |   RDN    | Round Down (towards -\infty)
  //    011    |   RUP    | Round Up (towards \infty)
  //    100    |   RMM    | Round to Nearest, ties to Max Magnitude
  //  others   |          | *invalid*
  always_comb begin : rounding_decision
    unique case (rnd_mode_i)
      fpnew_pkg::RNE: // Decide accoring to round/sticky bits
        unique case (round_sticky_bits_i)
          2'b00,
          2'b01: round_up = 1'b0;           // < ulp/2 away, round down
          2'b10: round_up = abs_value_i[0]; // = ulp/2 away, round towards even result
          2'b11: round_up = 1'b1;           // > ulp/2 away, round up
          default: round_up = fpnew_pkg::DONT_CARE;
        endcase
      fpnew_pkg::RTZ: round_up = 1'b0; // always round down
      fpnew_pkg::RDN: round_up = (| round_sticky_bits_i) ? sign_i  : 1'b0; // to 0 if +, away if -
      fpnew_pkg::RUP: round_up = (| round_sticky_bits_i) ? ~sign_i : 1'b0; // to 0 if -, away if +
      fpnew_pkg::RMM: round_up = round_sticky_bits_i[1]; // round down if < ulp/2 away, else up
      default: round_up = fpnew_pkg::DONT_CARE; // propagate x
    endcase
  end

  // Perform the rounding, exponent change and overflow to inf happens automagically
  assign abs_rounded_o = abs_value_i + round_up;

  // True zero result is a zero result without dirty round/sticky bits
  assign exact_zero_o = (abs_value_i == '0) && (round_sticky_bits_i == '0);

  // In case of effective subtraction (thus signs of addition operands must have differed) and a
  // true zero result, the result sign is '-' in case of RDN and '+' for other modes.
  assign sign_o = (exact_zero_o && effective_subtraction_i)
                  ? (rnd_mode_i == fpnew_pkg::RDN)
                  : sign_i;

endmodule
// Copyright 2019 ETH Zurich and University of Bologna.
//
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Stefan Mach <smach@iis.ee.ethz.ch>

module fpnew_top #(
  // FPU configuration
  parameter fpnew_pkg::fpu_features_t       Features       = fpnew_pkg::RV64D_Xsflt,
  parameter fpnew_pkg::fpu_implementation_t Implementation = fpnew_pkg::DEFAULT_NOREGS,
  parameter type                            TagType        = logic,
  // Do not change
  localparam int unsigned WIDTH        = Features.Width,
  localparam int unsigned NUM_OPERANDS = 3
) (
  input logic                               clk_i,
  input logic                               rst_ni,
  // Input signals
  input logic [NUM_OPERANDS-1:0][WIDTH-1:0] operands_i,
  input fpnew_pkg::roundmode_e              rnd_mode_i,
  input fpnew_pkg::operation_e              op_i,
  input logic                               op_mod_i,
  input fpnew_pkg::fp_format_e              src_fmt_i,
  input fpnew_pkg::fp_format_e              dst_fmt_i,
  input fpnew_pkg::int_format_e             int_fmt_i,
  input logic                               vectorial_op_i,
  input TagType                             tag_i,
  // Input Handshake
  input  logic                              in_valid_i,
  output logic                              in_ready_o,
  input  logic                              flush_i,
  // Output signals
  output logic [WIDTH-1:0]                  result_o,
  output fpnew_pkg::status_t                status_o,
  output TagType                            tag_o,
  // Output handshake
  output logic                              out_valid_o,
  input  logic                              out_ready_i,
  // Indication of valid data in flight
  output logic                              busy_o
);

  localparam int unsigned NUM_OPGROUPS = fpnew_pkg::NUM_OPGROUPS;
  localparam int unsigned NUM_FORMATS  = fpnew_pkg::NUM_FP_FORMATS;

  // ----------------
  // Type Definition
  // ----------------
  typedef struct packed {
    logic [WIDTH-1:0]   result;
    fpnew_pkg::status_t status;
    TagType             tag;
  } output_t;

  // Handshake signals for the blocks
  logic [NUM_OPGROUPS-1:0] opgrp_in_ready, opgrp_out_valid, opgrp_out_ready, opgrp_ext, opgrp_busy;
  output_t [NUM_OPGROUPS-1:0] opgrp_outputs;

  logic [NUM_FORMATS-1:0][NUM_OPERANDS-1:0] is_boxed;

  // -----------
  // Input Side
  // -----------
  assign in_ready_o = in_valid_i & opgrp_in_ready[fpnew_pkg::get_opgroup(op_i)];

  // NaN-boxing check
  for (genvar fmt = 0; fmt < int'(NUM_FORMATS); fmt++) begin : gen_nanbox_check
    localparam int unsigned FP_WIDTH = fpnew_pkg::fp_width(fpnew_pkg::fp_format_e'(fmt));
    // NaN boxing is only generated if it's enabled and needed
    if (Features.EnableNanBox && (FP_WIDTH < WIDTH)) begin : check
      for (genvar op = 0; op < int'(NUM_OPERANDS); op++) begin : operands
        assign is_boxed[fmt][op] = (!vectorial_op_i)
                                   ? operands_i[op][WIDTH-1:FP_WIDTH] == '1
                                   : 1'b1;
      end
    end else begin : no_check
      assign is_boxed[fmt] = '1;
    end
  end

  // -------------------------
  // Generate Operation Blocks
  // -------------------------
  for (genvar opgrp = 0; opgrp < int'(NUM_OPGROUPS); opgrp++) begin : gen_operation_groups
    localparam int unsigned NUM_OPS = fpnew_pkg::num_operands(fpnew_pkg::opgroup_e'(opgrp));

    logic in_valid;
    logic [NUM_FORMATS-1:0][NUM_OPS-1:0] input_boxed;

    assign in_valid = in_valid_i & (fpnew_pkg::get_opgroup(op_i) == fpnew_pkg::opgroup_e'(opgrp));

    // slice out input boxing
    always_comb begin : slice_inputs
      for (int unsigned fmt = 0; fmt < NUM_FORMATS; fmt++)
        input_boxed[fmt] = is_boxed[fmt][NUM_OPS-1:0];
    end

    fpnew_opgroup_block #(
      .OpGroup       ( fpnew_pkg::opgroup_e'(opgrp)    ),
      .Width         ( WIDTH                           ),
      .EnableVectors ( Features.EnableVectors          ),
      .FpFmtMask     ( Features.FpFmtMask              ),
      .IntFmtMask    ( Features.IntFmtMask             ),
      .FmtPipeRegs   ( Implementation.PipeRegs[opgrp]  ),
      .FmtUnitTypes  ( Implementation.UnitTypes[opgrp] ),
      .PipeConfig    ( Implementation.PipeConfig       ),
      .TagType       ( TagType                         )
    ) i_opgroup_block (
      .clk_i,
      .rst_ni,
      .operands_i      ( operands_i[NUM_OPS-1:0] ),
      .is_boxed_i      ( input_boxed             ),
      .rnd_mode_i,
      .op_i,
      .op_mod_i,
      .src_fmt_i,
      .dst_fmt_i,
      .int_fmt_i,
      .vectorial_op_i,
      .tag_i,
      .in_valid_i      ( in_valid              ),
      .in_ready_o      ( opgrp_in_ready[opgrp] ),
      .flush_i,
      .result_o        ( opgrp_outputs[opgrp].result ),
      .status_o        ( opgrp_outputs[opgrp].status ),
      .extension_bit_o ( opgrp_ext[opgrp]            ),
      .tag_o           ( opgrp_outputs[opgrp].tag    ),
      .out_valid_o     ( opgrp_out_valid[opgrp]      ),
      .out_ready_i     ( opgrp_out_ready[opgrp]      ),
      .busy_o          ( opgrp_busy[opgrp]           )
    );
  end

  // ------------------
  // Arbitrate Outputs
  // ------------------
  output_t arbiter_output;

  // Round-Robin arbiter to decide which result to use
  rr_arb_tree #(
    .NumIn     ( NUM_OPGROUPS ),
    .DataType  ( output_t     ),
    .AxiVldRdy ( 1'b1         )
  ) i_arbiter (
    .clk_i,
    .rst_ni,
    .flush_i,
    .rr_i   ( '0             ),
    .req_i  ( opgrp_out_valid ),
    .gnt_o  ( opgrp_out_ready ),
    .data_i ( opgrp_outputs   ),
    .gnt_i  ( out_ready_i     ),
    .req_o  ( out_valid_o     ),
    .data_o ( arbiter_output  ),
    .idx_o  ( /* unused */    )
  );

  // Unpack output
  assign result_o        = arbiter_output.result;
  assign status_o        = arbiter_output.status;
  assign tag_o           = arbiter_output.tag;

  assign busy_o = (| opgrp_busy);

endmodule
// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
////////////////////////////////////////////////////////////////////////////////
// Company:        IIS @ ETHZ - Federal Institute of Technology               //
//                                                                            //
// Engineers:      Lei Li                  lile@iis.ee.ethz.ch                //
//                                                                            //
// Additional contributions by:                                               //
//                                                                            //
//                                                                            //
//                                                                            //
// Create Date:    12/01/2017                                                 //
// Design Name:    FPU                                                        //
// Module Name:    iteration_div_sqrt_mvp                                     //
// Project Name:   Private FPU                                                //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    iteration unit for div and sqrt                            //
//                                                                            //
//                                                                            //
// Revision:        03/14/2018                                                //
//                  For div_sqrt_mvp                                          //
////////////////////////////////////////////////////////////////////////////////

module iteration_div_sqrt_mvp
#(
   parameter   WIDTH=25
)
  (//Input

   input logic [WIDTH-1:0]      A_DI,
   input logic [WIDTH-1:0]      B_DI,
   input logic                  Div_enable_SI,
   input logic                  Div_start_dly_SI,
   input logic                  Sqrt_enable_SI,
   input logic [1:0]            D_DI,

   output logic [1:0]           D_DO,
   output logic [WIDTH-1:0]     Sum_DO,
   output logic                 Carry_out_DO
    );

   logic                        D_carry_D;
   logic                        Sqrt_cin_D;
   logic                        Cin_D;

   assign D_DO[0]=~D_DI[0];
   assign D_DO[1]=~(D_DI[1] ^ D_DI[0]);
   assign D_carry_D=D_DI[1] | D_DI[0];
   assign Sqrt_cin_D=Sqrt_enable_SI&&D_carry_D;
   assign Cin_D=Div_enable_SI?1'b0:Sqrt_cin_D;
   assign {Carry_out_DO,Sum_DO}=A_DI+B_DI+Cin_D;

endmodule
// Copyright (c) 2018 - 2019 ETH Zurich, University of Bologna
// All rights reserved.
//
// This code is under development and not yet released to the public.
// Until it is released, the code is under the copyright of ETH Zurich and
// the University of Bologna, and may contain confidential and/or unpublished
// work. Any reuse/redistribution is strictly forbidden without written
// permission from ETH Zurich.
//
// Bug fixes and contributions will eventually be released under the
// SolderPad open hardware license in the context of the PULP platform
// (http://www.pulp-platform.org), under the copyright of ETH Zurich and the
// University of Bologna.

/// A trailing zero counter / leading zero counter.
/// Set MODE to 0 for trailing zero counter => cnt_o is the number of trailing zeros (from the LSB)
/// Set MODE to 1 for leading zero counter  => cnt_o is the number of leading zeros  (from the MSB)
/// If the input does not contain a zero, `empty_o` is asserted. Additionally `cnt_o` contains
/// the maximum number of zeros - 1. For example:
///   in_i = 000_0000, empty_o = 1, cnt_o = 6 (mode = 0)
///   in_i = 000_0001, empty_o = 0, cnt_o = 0 (mode = 0)
///   in_i = 000_1000, empty_o = 0, cnt_o = 3 (mode = 0)
/// Furthermore, this unit contains a more efficient implementation for Verilator (simulation only).
/// This speeds up simulation significantly.
module lzc #(
  /// The width of the input vector.
  parameter int unsigned WIDTH = 2,
  /// Mode selection: 0 -> trailing zero, 1 -> leading zero
  parameter bit          MODE  = 1'b0,
  /// Dependent parameter. Do **not** change!
  ///
  /// Width of the output signal with the zero count.
  parameter int unsigned CNT_WIDTH = cf_math_pkg::idx_width(WIDTH)
) (
  /// Input vector to be counted.
  input  logic [WIDTH-1:0]     in_i,
  /// Count of the leading / trailing zeros.
  output logic [CNT_WIDTH-1:0] cnt_o,
  /// Counter is empty: Asserted if all bits in in_i are zero.
  output logic                 empty_o
);

  if (WIDTH == 1) begin : gen_degenerate_lzc

    assign cnt_o[0] = !in_i[0];
    assign empty_o = !in_i[0];

  end else begin : gen_lzc

    localparam int unsigned NumLevels = $clog2(WIDTH);

    // pragma translate_off
    initial begin
      assert(WIDTH > 0) else $fatal(1, "input must be at least one bit wide");
    end
    // pragma translate_on

    logic [WIDTH-1:0][NumLevels-1:0] index_lut;
    logic [2**NumLevels-1:0] sel_nodes;
    logic [2**NumLevels-1:0][NumLevels-1:0] index_nodes;

    logic [WIDTH-1:0] in_tmp;

    // reverse vector if required
    always_comb begin : flip_vector
      for (int unsigned i = 0; i < WIDTH; i++) begin
        in_tmp[i] = (MODE) ? in_i[WIDTH-1-i] : in_i[i];
      end
    end

    for (genvar j = 0; unsigned'(j) < WIDTH; j++) begin : g_index_lut
      assign index_lut[j] = (NumLevels)'(unsigned'(j));
    end

    for (genvar level = 0; unsigned'(level) < NumLevels; level++) begin : g_levels
      if (unsigned'(level) == NumLevels - 1) begin : g_last_level
        for (genvar k = 0; k < 2 ** level; k++) begin : g_level
          // if two successive indices are still in the vector...
          if (unsigned'(k) * 2 < WIDTH - 1) begin : g_reduce
            assign sel_nodes[2 ** level - 1 + k] = in_tmp[k * 2] | in_tmp[k * 2 + 1];
            assign index_nodes[2 ** level - 1 + k] = (in_tmp[k * 2] == 1'b1)
              ? index_lut[k * 2] :
                index_lut[k * 2 + 1];
          end
          // if only the first index is still in the vector...
          if (unsigned'(k) * 2 == WIDTH - 1) begin : g_base
            assign sel_nodes[2 ** level - 1 + k] = in_tmp[k * 2];
            assign index_nodes[2 ** level - 1 + k] = index_lut[k * 2];
          end
          // if index is out of range
          if (unsigned'(k) * 2 > WIDTH - 1) begin : g_out_of_range
            assign sel_nodes[2 ** level - 1 + k] = 1'b0;
            assign index_nodes[2 ** level - 1 + k] = '0;
          end
        end
      end else begin : g_not_last_level
        for (genvar l = 0; l < 2 ** level; l++) begin : g_level
          assign sel_nodes[2 ** level - 1 + l] =
              sel_nodes[2 ** (level + 1) - 1 + l * 2] | sel_nodes[2 ** (level + 1) - 1 + l * 2 + 1];
          assign index_nodes[2 ** level - 1 + l] = (sel_nodes[2 ** (level + 1) - 1 + l * 2] == 1'b1)
            ? index_nodes[2 ** (level + 1) - 1 + l * 2] :
              index_nodes[2 ** (level + 1) - 1 + l * 2 + 1];
        end
      end
    end

    assign cnt_o = NumLevels > unsigned'(0) ? index_nodes[0] : {($clog2(WIDTH)) {1'b0}};
    assign empty_o = NumLevels > unsigned'(0) ? ~sel_nodes[0] : ~(|in_i);

  end : gen_lzc

endmodule : lzc
// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Company:        IIS @ ETHZ - Federal Institute of Technology               //
//                                                                            //
// Engineers:      Lei Li    lile@iis.ee.ethz.ch                              //
//                                                                            //
// Additional contributions by:                                               //
//                                                                            //
//                                                                            //
//                                                                            //
// Create Date:    09/03/2018                                                 //
// Design Name:    FPU                                                        //
// Module Name:    norm_div_sqrt_mvp.sv                                       //
// Project Name:                                                              //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Floating point Normalizer/Rounding unit                    //
//                 Since this module is design as a combinatinal logic, it can//
//                 be added arbinary register stages for different frequency  //
//                 in the wrapper module.                                     //
//                                                                            //
//                                                                            //
//                                                                            //
// Revision Date:  12/04/2018                                                 //
//                 Lei Li                                                     //
//                 To address some requirements by Stefan                     //
//                                                                            //
//                                                                            //
//                                                                            //
//                                                                            //
//                                                                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

//import defs_div_sqrt_mvp::*;

module norm_div_sqrt_mvp
  (//Inputs
   input logic [C_MANT_FP64+4:0]                Mant_in_DI,  // Include the needed 4-bit for rounding and hidden bit
   input logic signed [C_EXP_FP64+1:0]          Exp_in_DI,
   input logic                                  Sign_in_DI,
   input logic                                  Div_enable_SI,
   input logic                                  Sqrt_enable_SI,
   input logic                                  Inf_a_SI,
   input logic                                  Inf_b_SI,
   input logic                                  Zero_a_SI,
   input logic                                  Zero_b_SI,
   input logic                                  NaN_a_SI,
   input logic                                  NaN_b_SI,
   input logic                                  SNaN_SI,
   input logic [C_RM-1:0]                       RM_SI,
   input logic                                  Full_precision_SI,
   input logic                                  FP32_SI,
   input logic                                  FP64_SI,
   input logic                                  FP16_SI,
   input logic                                  FP16ALT_SI,
   //Outputs
   output logic [C_EXP_FP64+C_MANT_FP64:0]      Result_DO,
   output logic [4:0]                           Fflags_SO //{NV,DZ,OF,UF,NX}
   );


   logic                                        Sign_res_D;

   logic                                        NV_OP_S;
   logic                                        Exp_OF_S;
   logic                                        Exp_UF_S;
   logic                                        Div_Zero_S;
   logic                                        In_Exact_S;

   /////////////////////////////////////////////////////////////////////////////
   // Normalization                                                           //
   /////////////////////////////////////////////////////////////////////////////
   logic [C_MANT_FP64:0]                        Mant_res_norm_D;
   logic [C_EXP_FP64-1:0]                       Exp_res_norm_D;

   /////////////////////////////////////////////////////////////////////////////
   // Right shift operations for negtive exponents                            //
   /////////////////////////////////////////////////////////////////////////////

  logic  [C_EXP_FP64+1:0]                       Exp_Max_RS_FP64_D;
  logic  [C_EXP_FP32+1:0]                       Exp_Max_RS_FP32_D;
  logic  [C_EXP_FP16+1:0]                       Exp_Max_RS_FP16_D;
  logic  [C_EXP_FP16ALT+1:0]                    Exp_Max_RS_FP16ALT_D;
  //
  assign Exp_Max_RS_FP64_D=Exp_in_DI[C_EXP_FP64:0]+C_MANT_FP64+1; // to check exponent after (C_MANT_FP64+1)-bit >> when Exp_in_DI is negative
  assign Exp_Max_RS_FP32_D=Exp_in_DI[C_EXP_FP32:0]+C_MANT_FP32+1; // to check exponent after (C_MANT_FP32+1)-bit >> when Exp_in_DI is negative
  assign Exp_Max_RS_FP16_D=Exp_in_DI[C_EXP_FP16:0]+C_MANT_FP16+1; // to check exponent after (C_MANT_FP16+1)-bit >> when Exp_in_DI is negative
  assign Exp_Max_RS_FP16ALT_D=Exp_in_DI[C_EXP_FP16ALT:0]+C_MANT_FP16ALT+1; // to check exponent after (C_MANT_FP16ALT+1)-bit >> when Exp_in_DI is negative
  logic  [C_EXP_FP64+1:0]                       Num_RS_D;
  assign Num_RS_D=~Exp_in_DI+1+1;            // How many right shifts(RS) are needed to generate a denormal number? >> is need only when Exp_in_DI is negative
  logic  [C_MANT_FP64:0]                        Mant_RS_D;
  logic  [C_MANT_FP64+4:0]                      Mant_forsticky_D;
  assign  {Mant_RS_D,Mant_forsticky_D} ={Mant_in_DI,{(C_MANT_FP64+1){1'b0}} } >>(Num_RS_D); //
//
  logic [C_EXP_FP64+1:0]                        Exp_subOne_D;
  assign Exp_subOne_D = Exp_in_DI -1;

   //normalization
   logic [1:0]                                  Mant_lower_D;
   logic                                        Mant_sticky_bit_D;
   logic [C_MANT_FP64+4:0]                      Mant_forround_D;

   always_comb
     begin

       if(NaN_a_SI)  //  if a is NaN, return NaN
         begin
           Div_Zero_S=1'b0;
           Exp_OF_S=1'b0;
           Exp_UF_S=1'b0;
           Mant_res_norm_D={1'b0,C_MANT_NAN_FP64};
           Exp_res_norm_D='1;
           Mant_forround_D='0;
           Sign_res_D=1'b0;
           NV_OP_S = SNaN_SI;
         end

      else if(NaN_b_SI)   //if b is NaN, return NaN
        begin
          Div_Zero_S=1'b0;
          Exp_OF_S=1'b0;
          Exp_UF_S=1'b0;
          Mant_res_norm_D={1'b0,C_MANT_NAN_FP64};
          Exp_res_norm_D='1;
          Mant_forround_D='0;
          Sign_res_D=1'b0;
          NV_OP_S = SNaN_SI;
        end

      else if(Inf_a_SI)
        begin
          if(Div_enable_SI&&Inf_b_SI)                     //Inf/Inf, retrurn NaN
            begin
              Div_Zero_S=1'b0;
              Exp_OF_S=1'b0;
              Exp_UF_S=1'b0;
              Mant_res_norm_D={1'b0,C_MANT_NAN_FP64};
              Exp_res_norm_D='1;
              Mant_forround_D='0;
              Sign_res_D=1'b0;
              NV_OP_S = 1'b1;
            end
          else if (Sqrt_enable_SI && Sign_in_DI) begin // catch sqrt(-inf)
            Div_Zero_S=1'b0;
            Exp_OF_S=1'b0;
            Exp_UF_S=1'b0;
            Mant_res_norm_D={1'b0,C_MANT_NAN_FP64};
            Exp_res_norm_D='1;
            Mant_forround_D='0;
            Sign_res_D=1'b0;
            NV_OP_S = 1'b1;
          end else begin
            Div_Zero_S=1'b0;
            Exp_OF_S=1'b1;
            Exp_UF_S=1'b0;
            Mant_res_norm_D= '0;
            Exp_res_norm_D='1;
            Mant_forround_D='0;
            Sign_res_D=Sign_in_DI;
            NV_OP_S = 1'b0;
          end
        end

      else if(Div_enable_SI&&Inf_b_SI)
        begin
          Div_Zero_S=1'b0;
          Exp_OF_S=1'b1;
          Exp_UF_S=1'b0;
          Mant_res_norm_D= '0;
          Exp_res_norm_D='0;
          Mant_forround_D='0;
          Sign_res_D=Sign_in_DI;
          NV_OP_S = 1'b0;
        end

     else if(Zero_a_SI)
       begin
         if(Div_enable_SI&&Zero_b_SI)
           begin
              Div_Zero_S=1'b1;
              Exp_OF_S=1'b0;
              Exp_UF_S=1'b0;
              Mant_res_norm_D={1'b0,C_MANT_NAN_FP64};
              Exp_res_norm_D='1;
              Mant_forround_D='0;
              Sign_res_D=1'b0;
              NV_OP_S = 1'b1;
           end
         else
           begin
             Div_Zero_S=1'b0;
             Exp_OF_S=1'b0;
             Exp_UF_S=1'b0;
             Mant_res_norm_D='0;
             Exp_res_norm_D='0;
             Mant_forround_D='0;
             Sign_res_D=Sign_in_DI;
             NV_OP_S = 1'b0;
           end
       end

     else  if(Div_enable_SI&&(Zero_b_SI))  //div Zero
       begin
         Div_Zero_S=1'b1;
         Exp_OF_S=1'b0;
         Exp_UF_S=1'b0;
         Mant_res_norm_D='0;
         Exp_res_norm_D='1;
         Mant_forround_D='0;
         Sign_res_D=Sign_in_DI;
         NV_OP_S = 1'b0;
       end

      else if(Sign_in_DI&&Sqrt_enable_SI)   //sqrt(-a)
        begin
          Div_Zero_S=1'b0;
          Exp_OF_S=1'b0;
          Exp_UF_S=1'b0;
          Mant_res_norm_D={1'b0,C_MANT_NAN_FP64};
          Exp_res_norm_D='1;
          Mant_forround_D='0;
          Sign_res_D=1'b0;
          NV_OP_S = 1'b1;
        end

     else if((Exp_in_DI[C_EXP_FP64:0]=='0))
       begin
         if(Mant_in_DI!='0)       //Exp=0, Mant!=0, it is denormal
           begin
             Div_Zero_S=1'b0;
             Exp_OF_S=1'b0;
             Exp_UF_S=1'b1;
             Mant_res_norm_D={1'b0,Mant_in_DI[C_MANT_FP64+4:5]};
             Exp_res_norm_D='0;
             Mant_forround_D={Mant_in_DI[4:0],{(C_MANT_FP64){1'b0}} };
             Sign_res_D=Sign_in_DI;
             NV_OP_S = 1'b0;
           end
         else                 // Zero
           begin
             Div_Zero_S=1'b0;
             Exp_OF_S=1'b0;
             Exp_UF_S=1'b0;
             Mant_res_norm_D='0;
             Exp_res_norm_D='0;
             Mant_forround_D='0;
             Sign_res_D=Sign_in_DI;
             NV_OP_S = 1'b0;
           end
        end

      else if((Exp_in_DI[C_EXP_FP64:0]==C_EXP_ONE_FP64)&&(~Mant_in_DI[C_MANT_FP64+4]))  //denormal
        begin
          Div_Zero_S=1'b0;
          Exp_OF_S=1'b0;
          Exp_UF_S=1'b1;
          Mant_res_norm_D=Mant_in_DI[C_MANT_FP64+4:4];
          Exp_res_norm_D='0;
          Mant_forround_D={Mant_in_DI[3:0],{(C_MANT_FP64+1){1'b0}}};
          Sign_res_D=Sign_in_DI;
          NV_OP_S = 1'b0;
        end

      else if(Exp_in_DI[C_EXP_FP64+1])    //minus              //consider format
        begin
          Div_Zero_S=1'b0;
          Exp_OF_S=1'b0;
          Exp_UF_S=1'b1;
          Mant_res_norm_D={Mant_RS_D[C_MANT_FP64:0]};
          Exp_res_norm_D='0;
          Mant_forround_D={Mant_forsticky_D[C_MANT_FP64+4:0]};   //??
          Sign_res_D=Sign_in_DI;
          NV_OP_S = 1'b0;
        end

      else if( (Exp_in_DI[C_EXP_FP32]&&FP32_SI) | (Exp_in_DI[C_EXP_FP64]&&FP64_SI) | (Exp_in_DI[C_EXP_FP16]&&FP16_SI) | (Exp_in_DI[C_EXP_FP16ALT]&&FP16ALT_SI) )            //OF
        begin
          Div_Zero_S=1'b0;
          Exp_OF_S=1'b1;
          Exp_UF_S=1'b0;
          Mant_res_norm_D='0;
          Exp_res_norm_D='1;
          Mant_forround_D='0;
          Sign_res_D=Sign_in_DI;
          NV_OP_S = 1'b0;
        end

      else if( ((Exp_in_DI[C_EXP_FP32-1:0]=='1)&&FP32_SI) | ((Exp_in_DI[C_EXP_FP64-1:0]=='1)&&FP64_SI) |  ((Exp_in_DI[C_EXP_FP16-1:0]=='1)&&FP16_SI) | ((Exp_in_DI[C_EXP_FP16ALT-1:0]=='1)&&FP16ALT_SI) )//255
        begin
          if(~Mant_in_DI[C_MANT_FP64+4]) // MSB=0
            begin
              Div_Zero_S=1'b0;
              Exp_OF_S=1'b0;
              Exp_UF_S=1'b0;
              Mant_res_norm_D=Mant_in_DI[C_MANT_FP64+3:3];
              Exp_res_norm_D=Exp_subOne_D;
              Mant_forround_D={Mant_in_DI[2:0],{(C_MANT_FP64+2){1'b0}}};
              Sign_res_D=Sign_in_DI;
              NV_OP_S = 1'b0;
            end
          else if(Mant_in_DI!='0)         //NaN
            begin
              Div_Zero_S=1'b0;
              Exp_OF_S=1'b1;
              Exp_UF_S=1'b0;
              Mant_res_norm_D= '0;
              Exp_res_norm_D='1;
              Mant_forround_D='0;
              Sign_res_D=Sign_in_DI;
              NV_OP_S = 1'b0;
            end
          else                         //infinity
            begin
              Div_Zero_S=1'b0;
              Exp_OF_S=1'b1;
              Exp_UF_S=1'b0;
              Mant_res_norm_D= '0;
              Exp_res_norm_D='1;
              Mant_forround_D='0;
              Sign_res_D=Sign_in_DI;
              NV_OP_S = 1'b0;
            end
         end

      else if(Mant_in_DI[C_MANT_FP64+4])  //normal numbers with 1.XXX
        begin
           Div_Zero_S=1'b0;
           Exp_OF_S=1'b0;
           Exp_UF_S=1'b0;
           Mant_res_norm_D= Mant_in_DI[C_MANT_FP64+4:4];
           Exp_res_norm_D=Exp_in_DI[C_EXP_FP64-1:0];
           Mant_forround_D={Mant_in_DI[3:0],{(C_MANT_FP64+1){1'b0}}};
           Sign_res_D=Sign_in_DI;
           NV_OP_S = 1'b0;
        end

      else                                   //normal numbers with 0.1XX
         begin
           Div_Zero_S=1'b0;
           Exp_OF_S=1'b0;
           Exp_UF_S=1'b0;
           Mant_res_norm_D=Mant_in_DI[C_MANT_FP64+3:3];
           Exp_res_norm_D=Exp_subOne_D;
           Mant_forround_D={Mant_in_DI[2:0],{(C_MANT_FP64+2){1'b0}}};
           Sign_res_D=Sign_in_DI;
           NV_OP_S = 1'b0;
         end

     end

   /////////////////////////////////////////////////////////////////////////////
   // Rounding enable only for full precision (Full_precision_SI==1'b1)       //
   /////////////////////////////////////////////////////////////////////////////

   logic [C_MANT_FP64:0]                   Mant_upper_D;
   logic [C_MANT_FP64+1:0]                 Mant_upperRounded_D;
   logic                                   Mant_roundUp_S;
   logic                                   Mant_rounded_S;

  always_comb //determine which bits for Mant_lower_D and Mant_sticky_bit_D
    begin
      if(FP32_SI)
        begin
          Mant_upper_D = {Mant_res_norm_D[C_MANT_FP64:C_MANT_FP64-C_MANT_FP32], {(C_MANT_FP64-C_MANT_FP32){1'b0}} };
          Mant_lower_D = Mant_res_norm_D[C_MANT_FP64-C_MANT_FP32-1:C_MANT_FP64-C_MANT_FP32-2];
          Mant_sticky_bit_D = | Mant_res_norm_D[C_MANT_FP64-C_MANT_FP32-3:0];
        end
      else if(FP64_SI)
        begin
          Mant_upper_D = Mant_res_norm_D[C_MANT_FP64:0];
          Mant_lower_D = Mant_forround_D[C_MANT_FP64+4:C_MANT_FP64+3];
          Mant_sticky_bit_D = | Mant_forround_D[C_MANT_FP64+3:0];
        end
      else if(FP16_SI)
        begin
          Mant_upper_D = {Mant_res_norm_D[C_MANT_FP64:C_MANT_FP64-C_MANT_FP16], {(C_MANT_FP64-C_MANT_FP16){1'b0}} };
          Mant_lower_D = Mant_res_norm_D[C_MANT_FP64-C_MANT_FP16-1:C_MANT_FP64-C_MANT_FP16-2];
          Mant_sticky_bit_D = | Mant_res_norm_D[C_MANT_FP64-C_MANT_FP16-3:30];
        end
      else  //FP16ALT
      begin
          Mant_upper_D = {Mant_res_norm_D[C_MANT_FP64:C_MANT_FP64-C_MANT_FP16ALT], {(C_MANT_FP64-C_MANT_FP16ALT){1'b0}} };
          Mant_lower_D = Mant_res_norm_D[C_MANT_FP64-C_MANT_FP16ALT-1:C_MANT_FP64-C_MANT_FP16ALT-2];
          Mant_sticky_bit_D = | Mant_res_norm_D[C_MANT_FP64-C_MANT_FP16ALT-3:30];
      end
    end

   assign Mant_rounded_S = (|(Mant_lower_D))| Mant_sticky_bit_D;




   always_comb //determine whether to round up or not
     begin
        Mant_roundUp_S = 1'b0;
        case (RM_SI)
          C_RM_NEAREST :
            Mant_roundUp_S = Mant_lower_D[1] && ((Mant_lower_D[0] | Mant_sticky_bit_D )| ( (FP32_SI&&Mant_upper_D[C_MANT_FP64-C_MANT_FP32]) | (FP64_SI&&Mant_upper_D[0]) | (FP16_SI&&Mant_upper_D[C_MANT_FP64-C_MANT_FP16]) | (FP16ALT_SI&&Mant_upper_D[C_MANT_FP64-C_MANT_FP16ALT]) ) );
          C_RM_TRUNC   :
            Mant_roundUp_S = 0;
          C_RM_PLUSINF :
            Mant_roundUp_S = Mant_rounded_S & ~Sign_in_DI;
          C_RM_MINUSINF:
            Mant_roundUp_S = Mant_rounded_S & Sign_in_DI;
          default          :
            Mant_roundUp_S = 0;
        endcase // case (RM_DI)
     end // always_comb begin

  logic                                 Mant_renorm_S;
  logic  [C_MANT_FP64:0]                Mant_roundUp_Vector_S; // for all the formats

  assign Mant_roundUp_Vector_S={7'h0,(FP16ALT_SI&&Mant_roundUp_S),2'h0,(FP16_SI&&Mant_roundUp_S),12'h0,(FP32_SI&&Mant_roundUp_S),28'h0,(FP64_SI&&Mant_roundUp_S)};


  assign Mant_upperRounded_D = Mant_upper_D + Mant_roundUp_Vector_S;
  assign Mant_renorm_S       = Mant_upperRounded_D[C_MANT_FP64+1];

  /////////////////////////////////////////////////////////////////////////////
  // Renormalization for Rounding                                           //
  /////////////////////////////////////////////////////////////////////////////
  logic [C_MANT_FP64-1:0]               Mant_res_round_D;
  logic [C_EXP_FP64-1:0]                Exp_res_round_D;


  assign Mant_res_round_D = (Mant_renorm_S)?Mant_upperRounded_D[C_MANT_FP64:1]:Mant_upperRounded_D[C_MANT_FP64-1:0]; // including the process of the hidden bit
  assign Exp_res_round_D  = Exp_res_norm_D+Mant_renorm_S;

  /////////////////////////////////////////////////////////////////////////////
  //  Output Assignments                                                     //
  /////////////////////////////////////////////////////////////////////////////
  logic [C_MANT_FP64-1:0]               Mant_before_format_ctl_D;
  logic [C_EXP_FP64-1:0]                Exp_before_format_ctl_D;
  assign Mant_before_format_ctl_D = Full_precision_SI ? Mant_res_round_D : Mant_res_norm_D;
  assign Exp_before_format_ctl_D = Full_precision_SI ? Exp_res_round_D : Exp_res_norm_D;

  always_comb    //NaN Boxing
    begin  //
      if(FP32_SI)
          begin
            Result_DO ={32'hffff_ffff,Sign_res_D,Exp_before_format_ctl_D[C_EXP_FP32-1:0],Mant_before_format_ctl_D[C_MANT_FP64-1:C_MANT_FP64-C_MANT_FP32]};
          end
       else if(FP64_SI)
          begin
            Result_DO ={Sign_res_D,Exp_before_format_ctl_D[C_EXP_FP64-1:0],Mant_before_format_ctl_D[C_MANT_FP64-1:0]};
          end
      else if(FP16_SI)
          begin
            Result_DO ={48'hffff_ffff_ffff,Sign_res_D,Exp_before_format_ctl_D[C_EXP_FP16-1:0],Mant_before_format_ctl_D[C_MANT_FP64-1:C_MANT_FP64-C_MANT_FP16]};
          end
      else
          begin
            Result_DO ={48'hffff_ffff_ffff,Sign_res_D,Exp_before_format_ctl_D[C_EXP_FP16ALT-1:0],Mant_before_format_ctl_D[C_MANT_FP64-1:C_MANT_FP64-C_MANT_FP16ALT]};
          end
    end

assign In_Exact_S = (~Full_precision_SI) | Mant_rounded_S;
assign Fflags_SO = {NV_OP_S,Div_Zero_S,Exp_OF_S,Exp_UF_S,In_Exact_S}; //{NV,DZ,OF,UF,NX}

endmodule // norm_div_sqrt_mvp
// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
////////////////////////////////////////////////////////////////////////////////
// Company:        IIS @ ETHZ - Federal Institute of Technology               //
//                                                                            //
// Engineers:      Lei Li      lile@iis.ee.ethz.ch                            //
//                                                                            //
// Additional contributions by:                                               //
//                                                                            //
//                                                                            //
//                                                                            //
// Create Date:    10/04/2018                                                 //
// Design Name:    FPU                                                        //
// Module Name:    nrbd_nrsc_mvp.sv                                           //
// Project Name:   Private FPU                                                //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:   non restroring binary  divisior/ square root                //
//                                                                            //
// Revision Date:  12/04/2018                                                 //
//                 Lei Li                                                     //
//                 To address some requirements by Stefan and add low power   //
//                 control for special cases                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

//import defs_div_sqrt_mvp::*;

module nrbd_nrsc_mvp

  (//Input
   input logic                                 Clk_CI,
   input logic                                 Rst_RBI,
   input logic                                 Div_start_SI,
   input logic                                 Sqrt_start_SI,
   input logic                                 Start_SI,
   input logic                                 Kill_SI,
   input logic                                 Special_case_SBI,
   input logic                                 Special_case_dly_SBI,
   input logic [C_PC-1:0]                      Precision_ctl_SI,
   input logic [1:0]                           Format_sel_SI,
   input logic [C_MANT_FP64:0]                 Mant_a_DI,
   input logic [C_MANT_FP64:0]                 Mant_b_DI,
   input logic [C_EXP_FP64:0]                  Exp_a_DI,
   input logic [C_EXP_FP64:0]                  Exp_b_DI,
  //output
   output logic                                Div_enable_SO,
   output logic                                Sqrt_enable_SO,

   output logic                                Full_precision_SO,
   output logic                                FP32_SO,
   output logic                                FP64_SO,
   output logic                                FP16_SO,
   output logic                                FP16ALT_SO,
   output logic                                Ready_SO,
   output logic                                Done_SO,
   output logic  [C_MANT_FP64+4:0]             Mant_z_DO,
   output logic [C_EXP_FP64+1:0]               Exp_z_DO
    );


    logic                                     Div_start_dly_S,Sqrt_start_dly_S;


control_mvp         control_U0
(  .Clk_CI                                   (Clk_CI                          ),
   .Rst_RBI                                  (Rst_RBI                         ),
   .Div_start_SI                             (Div_start_SI                    ),
   .Sqrt_start_SI                            (Sqrt_start_SI                   ),
   .Start_SI                                 (Start_SI                        ),
   .Kill_SI                                  (Kill_SI                         ),
   .Special_case_SBI                         (Special_case_SBI                ),
   .Special_case_dly_SBI                     (Special_case_dly_SBI            ),
   .Precision_ctl_SI                         (Precision_ctl_SI                ),
   .Format_sel_SI                            (Format_sel_SI                   ),
   .Numerator_DI                             (Mant_a_DI                       ),
   .Exp_num_DI                               (Exp_a_DI                        ),
   .Denominator_DI                           (Mant_b_DI                       ),
   .Exp_den_DI                               (Exp_b_DI                        ),
   .Div_start_dly_SO                         (Div_start_dly_S                 ),
   .Sqrt_start_dly_SO                        (Sqrt_start_dly_S                ),
   .Div_enable_SO                            (Div_enable_SO                   ),
   .Sqrt_enable_SO                           (Sqrt_enable_SO                  ),
   .Full_precision_SO                        (Full_precision_SO               ),
   .FP32_SO                                  (FP32_SO                         ),
   .FP64_SO                                  (FP64_SO                         ),
   .FP16_SO                                  (FP16_SO                         ),
   .FP16ALT_SO                               (FP16ALT_SO                      ),
   .Ready_SO                                 (Ready_SO                        ),
   .Done_SO                                  (Done_SO                         ),
   .Mant_result_prenorm_DO                   (Mant_z_DO                       ),
   .Exp_result_prenorm_DO                    (Exp_z_DO                        )
);



endmodule
// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
////////////////////////////////////////////////////////////////////////////////
// Company:        IIS @ ETHZ - Federal Institute of Technology               //
//                                                                            //
// Engineers:                Lei Li  //lile@iis.ee.ethz.ch                    //
//		                                                                        //
// Additional contributions by:                                               //
//                                                                            //
//                                                                            //
//                                                                            //
// Create Date:    01/03/2018                                                 //
// Design Name:    FPU                                                        //
// Module Name:    preprocess_mvp.sv                                          //
// Project Name:   Private FPU                                                //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:           decode and data preparation                         //
//                                                                            //
// Revision Date:  12/04/2018                                                 //
//                 Lei Li                                                     //
//                 To address some requirements by Stefan and add low power   //
//                 control for special cases                                  //
//                                                                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

//import defs_div_sqrt_mvp::*;

module preprocess_mvp
  (
   input logic                   Clk_CI,
   input logic                   Rst_RBI,
   input logic                   Div_start_SI,
   input logic                   Sqrt_start_SI,
   input logic                   Ready_SI,
   //Input Operands
   input logic [C_OP_FP64-1:0]   Operand_a_DI,
   input logic [C_OP_FP64-1:0]   Operand_b_DI,
   input logic [C_RM-1:0]        RM_SI,    //Rounding Mode
   input logic [C_FS-1:0]        Format_sel_SI,  // Format Selection

   // to control
   output logic                  Start_SO,
   output logic [C_EXP_FP64:0]   Exp_a_DO_norm,
   output logic [C_EXP_FP64:0]   Exp_b_DO_norm,
   output logic [C_MANT_FP64:0]  Mant_a_DO_norm,
   output logic [C_MANT_FP64:0]  Mant_b_DO_norm,

   output logic [C_RM-1:0]       RM_dly_SO,

   output logic                  Sign_z_DO,
   output logic                  Inf_a_SO,
   output logic                  Inf_b_SO,
   output logic                  Zero_a_SO,
   output logic                  Zero_b_SO,
   output logic                  NaN_a_SO,
   output logic                  NaN_b_SO,
   output logic                  SNaN_SO,
   output logic                  Special_case_SBO,
   output logic                  Special_case_dly_SBO
   );

   //Hidden Bits
   logic                         Hb_a_D;
   logic                         Hb_b_D;

   logic [C_EXP_FP64-1:0]        Exp_a_D;
   logic [C_EXP_FP64-1:0]        Exp_b_D;
   logic [C_MANT_FP64-1:0]       Mant_a_NonH_D;
   logic [C_MANT_FP64-1:0]       Mant_b_NonH_D;
   logic [C_MANT_FP64:0]         Mant_a_D;
   logic [C_MANT_FP64:0]         Mant_b_D;

   /////////////////////////////////////////////////////////////////////////////
   // Disassemble operands
   /////////////////////////////////////////////////////////////////////////////
   logic                      Sign_a_D,Sign_b_D;
   logic                      Start_S;

     always_comb
       begin
         case(Format_sel_SI)
           2'b00:
             begin
               Sign_a_D = Operand_a_DI[C_OP_FP32-1];
               Sign_b_D = Operand_b_DI[C_OP_FP32-1];
               Exp_a_D  = {3'h0, Operand_a_DI[C_OP_FP32-2:C_MANT_FP32]};
               Exp_b_D  = {3'h0, Operand_b_DI[C_OP_FP32-2:C_MANT_FP32]};
               Mant_a_NonH_D = {Operand_a_DI[C_MANT_FP32-1:0],29'h0};
               Mant_b_NonH_D = {Operand_b_DI[C_MANT_FP32-1:0],29'h0};
             end
           2'b01:
             begin
               Sign_a_D = Operand_a_DI[C_OP_FP64-1];
               Sign_b_D = Operand_b_DI[C_OP_FP64-1];
               Exp_a_D  = Operand_a_DI[C_OP_FP64-2:C_MANT_FP64];
               Exp_b_D  = Operand_b_DI[C_OP_FP64-2:C_MANT_FP64];
               Mant_a_NonH_D = Operand_a_DI[C_MANT_FP64-1:0];
               Mant_b_NonH_D = Operand_b_DI[C_MANT_FP64-1:0];
             end
           2'b10:
             begin
               Sign_a_D = Operand_a_DI[C_OP_FP16-1];
               Sign_b_D = Operand_b_DI[C_OP_FP16-1];
               Exp_a_D  = {6'h00, Operand_a_DI[C_OP_FP16-2:C_MANT_FP16]};
               Exp_b_D  = {6'h00, Operand_b_DI[C_OP_FP16-2:C_MANT_FP16]};
               Mant_a_NonH_D = {Operand_a_DI[C_MANT_FP16-1:0],42'h0};
               Mant_b_NonH_D = {Operand_b_DI[C_MANT_FP16-1:0],42'h0};
             end
           2'b11:
             begin
               Sign_a_D = Operand_a_DI[C_OP_FP16ALT-1];
               Sign_b_D = Operand_b_DI[C_OP_FP16ALT-1];
               Exp_a_D  = {3'h0, Operand_a_DI[C_OP_FP16ALT-2:C_MANT_FP16ALT]};
               Exp_b_D  = {3'h0, Operand_b_DI[C_OP_FP16ALT-2:C_MANT_FP16ALT]};
               Mant_a_NonH_D = {Operand_a_DI[C_MANT_FP16ALT-1:0],45'h0};
               Mant_b_NonH_D = {Operand_b_DI[C_MANT_FP16ALT-1:0],45'h0};
             end
           endcase
       end


   assign Mant_a_D = {Hb_a_D,Mant_a_NonH_D};
   assign Mant_b_D = {Hb_b_D,Mant_b_NonH_D};

   assign Hb_a_D = | Exp_a_D; // hidden bit
   assign Hb_b_D = | Exp_b_D; // hidden bit

   assign Start_S= Div_start_SI | Sqrt_start_SI;



   /////////////////////////////////////////////////////////////////////////////
   // preliminary checks for infinite/zero/NaN operands                       //
   /////////////////////////////////////////////////////////////////////////////

   logic               Mant_a_prenorm_zero_S;
   logic               Mant_b_prenorm_zero_S;

   logic               Exp_a_prenorm_zero_S;
   logic               Exp_b_prenorm_zero_S;
   assign Exp_a_prenorm_zero_S = ~Hb_a_D;
   assign Exp_b_prenorm_zero_S = ~Hb_b_D;

   logic               Exp_a_prenorm_Inf_NaN_S;
   logic               Exp_b_prenorm_Inf_NaN_S;

   logic               Mant_a_prenorm_QNaN_S;
   logic               Mant_a_prenorm_SNaN_S;
   logic               Mant_b_prenorm_QNaN_S;
   logic               Mant_b_prenorm_SNaN_S;

   assign Mant_a_prenorm_QNaN_S=Mant_a_NonH_D[C_MANT_FP64-1]&&(~(|Mant_a_NonH_D[C_MANT_FP64-2:0]));
   assign Mant_a_prenorm_SNaN_S=(~Mant_a_NonH_D[C_MANT_FP64-1])&&((|Mant_a_NonH_D[C_MANT_FP64-2:0]));
   assign Mant_b_prenorm_QNaN_S=Mant_b_NonH_D[C_MANT_FP64-1]&&(~(|Mant_b_NonH_D[C_MANT_FP64-2:0]));
   assign Mant_b_prenorm_SNaN_S=(~Mant_b_NonH_D[C_MANT_FP64-1])&&((|Mant_b_NonH_D[C_MANT_FP64-2:0]));

     always_comb
       begin
         case(Format_sel_SI)
           2'b00:
             begin
               Mant_a_prenorm_zero_S=(Operand_a_DI[C_MANT_FP32-1:0] == C_MANT_ZERO_FP32);
               Mant_b_prenorm_zero_S=(Operand_b_DI[C_MANT_FP32-1:0] == C_MANT_ZERO_FP32);
               Exp_a_prenorm_Inf_NaN_S=(Operand_a_DI[C_OP_FP32-2:C_MANT_FP32] == C_EXP_INF_FP32);
               Exp_b_prenorm_Inf_NaN_S=(Operand_b_DI[C_OP_FP32-2:C_MANT_FP32] == C_EXP_INF_FP32);
             end
           2'b01:
             begin
               Mant_a_prenorm_zero_S=(Operand_a_DI[C_MANT_FP64-1:0] == C_MANT_ZERO_FP64);
               Mant_b_prenorm_zero_S=(Operand_b_DI[C_MANT_FP64-1:0] == C_MANT_ZERO_FP64);
               Exp_a_prenorm_Inf_NaN_S=(Operand_a_DI[C_OP_FP64-2:C_MANT_FP64] == C_EXP_INF_FP64);
               Exp_b_prenorm_Inf_NaN_S=(Operand_b_DI[C_OP_FP64-2:C_MANT_FP64] == C_EXP_INF_FP64);
             end
           2'b10:
             begin
               Mant_a_prenorm_zero_S=(Operand_a_DI[C_MANT_FP16-1:0] == C_MANT_ZERO_FP16);
               Mant_b_prenorm_zero_S=(Operand_b_DI[C_MANT_FP16-1:0] == C_MANT_ZERO_FP16);
               Exp_a_prenorm_Inf_NaN_S=(Operand_a_DI[C_OP_FP16-2:C_MANT_FP16] == C_EXP_INF_FP16);
               Exp_b_prenorm_Inf_NaN_S=(Operand_b_DI[C_OP_FP16-2:C_MANT_FP16] == C_EXP_INF_FP16);
             end
           2'b11:
             begin
               Mant_a_prenorm_zero_S=(Operand_a_DI[C_MANT_FP16ALT-1:0] == C_MANT_ZERO_FP16ALT);
               Mant_b_prenorm_zero_S=(Operand_b_DI[C_MANT_FP16ALT-1:0] == C_MANT_ZERO_FP16ALT);
               Exp_a_prenorm_Inf_NaN_S=(Operand_a_DI[C_OP_FP16ALT-2:C_MANT_FP16ALT] == C_EXP_INF_FP16ALT);
               Exp_b_prenorm_Inf_NaN_S=(Operand_b_DI[C_OP_FP16ALT-2:C_MANT_FP16ALT] == C_EXP_INF_FP16ALT);
             end
           endcase
       end




   logic               Zero_a_SN,Zero_a_SP;
   logic               Zero_b_SN,Zero_b_SP;
   logic               Inf_a_SN,Inf_a_SP;
   logic               Inf_b_SN,Inf_b_SP;
   logic               NaN_a_SN,NaN_a_SP;
   logic               NaN_b_SN,NaN_b_SP;
   logic               SNaN_SN,SNaN_SP;

   assign Zero_a_SN = (Start_S&&Ready_SI)?(Exp_a_prenorm_zero_S&&Mant_a_prenorm_zero_S):Zero_a_SP;
   assign Zero_b_SN = (Start_S&&Ready_SI)?(Exp_b_prenorm_zero_S&&Mant_b_prenorm_zero_S):Zero_b_SP;
   assign Inf_a_SN = (Start_S&&Ready_SI)?(Exp_a_prenorm_Inf_NaN_S&&Mant_a_prenorm_zero_S):Inf_a_SP;
   assign Inf_b_SN = (Start_S&&Ready_SI)?(Exp_b_prenorm_Inf_NaN_S&&Mant_b_prenorm_zero_S):Inf_b_SP;
   assign NaN_a_SN = (Start_S&&Ready_SI)?(Exp_a_prenorm_Inf_NaN_S&&(~Mant_a_prenorm_zero_S)):NaN_a_SP;
   assign NaN_b_SN = (Start_S&&Ready_SI)?(Exp_b_prenorm_Inf_NaN_S&&(~Mant_b_prenorm_zero_S)):NaN_b_SP;
   assign SNaN_SN = (Start_S&&Ready_SI) ? ((Mant_a_prenorm_SNaN_S&&NaN_a_SN) | (Mant_b_prenorm_SNaN_S&&NaN_b_SN)) : SNaN_SP;

   always_ff @(posedge Clk_CI, negedge Rst_RBI)
     begin
        if(~Rst_RBI)
          begin
            Zero_a_SP <='0;
            Zero_b_SP <='0;
            Inf_a_SP <='0;
            Inf_b_SP <='0;
            NaN_a_SP <='0;
            NaN_b_SP <='0;
            SNaN_SP <= '0;
          end
        else
         begin
           Inf_a_SP <=Inf_a_SN;
           Inf_b_SP <=Inf_b_SN;
           Zero_a_SP <=Zero_a_SN;
           Zero_b_SP <=Zero_b_SN;
           NaN_a_SP <=NaN_a_SN;
           NaN_b_SP <=NaN_b_SN;
           SNaN_SP <= SNaN_SN;
         end
      end

   /////////////////////////////////////////////////////////////////////////////
   // Low power control
   /////////////////////////////////////////////////////////////////////////////

   assign Special_case_SBO=(~{(Div_start_SI)?(Zero_a_SN | Zero_b_SN |  Inf_a_SN | Inf_b_SN | NaN_a_SN | NaN_b_SN): (Zero_a_SN | Inf_a_SN | NaN_a_SN | Sign_a_D) })&&(Start_S&&Ready_SI);


   always_ff @(posedge Clk_CI, negedge Rst_RBI)
     begin
       if(~Rst_RBI)
          begin
            Special_case_dly_SBO <= '0;
          end
       else if((Start_S&&Ready_SI))
         begin
            Special_case_dly_SBO <= Special_case_SBO;
         end
       else if(Special_case_dly_SBO)
         begin
         Special_case_dly_SBO <= 1'b1;
         end
      else
         begin
            Special_case_dly_SBO <= '0;
         end
    end

   /////////////////////////////////////////////////////////////////////////////
   // Delay sign for normalization and round                                  //
   /////////////////////////////////////////////////////////////////////////////

   logic                   Sign_z_DN;
   logic                   Sign_z_DP;

   always_comb
     begin
       if(Div_start_SI&&Ready_SI)
           Sign_z_DN = Sign_a_D ^ Sign_b_D;
       else if(Sqrt_start_SI&&Ready_SI)
           Sign_z_DN = Sign_a_D;
       else
           Sign_z_DN = Sign_z_DP;
    end

   always_ff @(posedge Clk_CI, negedge Rst_RBI)
     begin
       if(~Rst_RBI)
          begin
            Sign_z_DP <= '0;
          end
       else
         begin
            Sign_z_DP <= Sign_z_DN;
         end
    end

   logic [C_RM-1:0]                  RM_DN;
   logic [C_RM-1:0]                  RM_DP;

   always_comb
     begin
       if(Start_S&&Ready_SI)
           RM_DN = RM_SI;
       else
           RM_DN = RM_DP;
    end

   always_ff @(posedge Clk_CI, negedge Rst_RBI)
     begin
       if(~Rst_RBI)
          begin
            RM_DP <= '0;
          end
       else
         begin
            RM_DP <= RM_DN;
         end
    end
   assign RM_dly_SO = RM_DP;

   logic [5:0]                  Mant_leadingOne_a, Mant_leadingOne_b;
   logic                        Mant_zero_S_a,Mant_zero_S_b;

  lzc #(
    .WIDTH ( C_MANT_FP64+1 ),
    .MODE  ( 1             )
  ) LOD_Ua (
    .in_i    ( Mant_a_D          ),
    .cnt_o   ( Mant_leadingOne_a ),
    .empty_o ( Mant_zero_S_a     )
  );

   logic [C_MANT_FP64:0]            Mant_a_norm_DN,Mant_a_norm_DP;

   assign  Mant_a_norm_DN = ((Start_S&&Ready_SI))?(Mant_a_D<<(Mant_leadingOne_a)):Mant_a_norm_DP;

   always_ff @(posedge Clk_CI, negedge Rst_RBI)
     begin
        if(~Rst_RBI)
          begin
            Mant_a_norm_DP <= '0;
          end
        else
          begin
            Mant_a_norm_DP<=Mant_a_norm_DN;
          end
     end

   logic [C_EXP_FP64:0]            Exp_a_norm_DN,Exp_a_norm_DP;
   assign  Exp_a_norm_DN = ((Start_S&&Ready_SI))?(Exp_a_D-Mant_leadingOne_a+(|Mant_leadingOne_a)):Exp_a_norm_DP;  //Covering the process of denormal numbers

   always_ff @(posedge Clk_CI, negedge Rst_RBI)
     begin
        if(~Rst_RBI)
          begin
            Exp_a_norm_DP <= '0;
          end
        else
          begin
            Exp_a_norm_DP<=Exp_a_norm_DN;
          end
     end

  lzc #(
    .WIDTH ( C_MANT_FP64+1 ),
    .MODE  ( 1             )
  ) LOD_Ub (
    .in_i    ( Mant_b_D          ),
    .cnt_o   ( Mant_leadingOne_b ),
    .empty_o ( Mant_zero_S_b     )
  );


   logic [C_MANT_FP64:0]            Mant_b_norm_DN,Mant_b_norm_DP;

   assign  Mant_b_norm_DN = ((Start_S&&Ready_SI))?(Mant_b_D<<(Mant_leadingOne_b)):Mant_b_norm_DP;

   always_ff @(posedge Clk_CI, negedge Rst_RBI)
     begin
        if(~Rst_RBI)
          begin
            Mant_b_norm_DP <= '0;
          end
        else
          begin
            Mant_b_norm_DP<=Mant_b_norm_DN;
          end
     end

   logic [C_EXP_FP64:0]            Exp_b_norm_DN,Exp_b_norm_DP;
   assign  Exp_b_norm_DN = ((Start_S&&Ready_SI))?(Exp_b_D-Mant_leadingOne_b+(|Mant_leadingOne_b)):Exp_b_norm_DP; //Covering the process of denormal numbers

   always_ff @(posedge Clk_CI, negedge Rst_RBI)
     begin
        if(~Rst_RBI)
          begin
            Exp_b_norm_DP <= '0;
          end
        else
          begin
            Exp_b_norm_DP<=Exp_b_norm_DN;
          end
     end

   /////////////////////////////////////////////////////////////////////////////
   // Output assignments                                                      //
   /////////////////////////////////////////////////////////////////////////////

   assign Start_SO=Start_S;
   assign Exp_a_DO_norm=Exp_a_norm_DP;
   assign Exp_b_DO_norm=Exp_b_norm_DP;
   assign Mant_a_DO_norm=Mant_a_norm_DP;
   assign Mant_b_DO_norm=Mant_b_norm_DP;
   assign Sign_z_DO=Sign_z_DP;
   assign Inf_a_SO=Inf_a_SP;
   assign Inf_b_SO=Inf_b_SP;
   assign Zero_a_SO=Zero_a_SP;
   assign Zero_b_SO=Zero_b_SP;
   assign NaN_a_SO=NaN_a_SP;
   assign NaN_b_SO=NaN_b_SP;
   assign SNaN_SO=SNaN_SP;

endmodule
// Copyright 2019 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Michael Schaffner <schaffner@iis.ee.ethz.ch>, ETH Zurich
//         Wolfgang Roenninger <wroennin@iis.ee.ethz.ch>, ETH Zurich
// Date: 02.04.2019
// Description: logarithmic arbitration tree with round robin arbitration scheme.

/// The rr_arb_tree employs non-starving round robin-arbitration - i.e., the priorities
/// rotate each cycle.
///
/// ## Fair vs. unfair Arbitration
///
/// This refers to fair throughput distribution when not all inputs have active requests.
/// This module has an internal state `rr_q` which defines the highest priority input. (When
/// `ExtPrio` is `1'b1` this state is provided from the outside.) The arbitration tree will
/// choose the input with the same index as currently defined by the state if it has an active
/// request. Otherwise a *random* other active input is selected. The parameter `FairArb` is used
/// to distinguish between two methods of calculating the next state.
/// * `1'b0`: The next state is calculated by advancing the current state by one. This leads to the
///           state being calculated without the context of the active request. Leading to an
///           unfair throughput distribution if not all inputs have active requests.
/// * `1'b1`: The next state jumps to the next unserved request with higher index.
///           This is achieved by using two trailing-zero-counters (`lzc`). The upper has the masked
///           `req_i` signal with all indices which will have a higher priority in the next state.
///           The trailing zero count defines the input index with the next highest priority after
///           the current one is served. When the upper is empty the lower `lzc` provides the
///           wrapped index if there are outstanding requests with lower or same priority.
/// The implication of throughput fairness on the module timing are:
/// * The trailing zero counter (`lzc`) has a loglog relation of input to output timing. This means
///   that in this module the input to register path scales with Log(Log(`NumIn`)).
/// * The `rr_arb_tree` data multiplexing scales with Log(`NumIn`). This means that the input to output
///   timing path of this module also scales scales with Log(`NumIn`).
/// This implies that in this module the input to output path is always longer than the input to
/// register path. As the output data usually also terminates in a register the parameter `FairArb`
/// only has implications on the area. When it is `1'b0` a static plus one adder is instantiated.
/// If it is `1'b1` two `lzc`, a masking logic stage and a two input multiplexer are instantiated.
/// However these are small in respect of the data multiplexers needed, as the width of the `req_i`
/// signal is usually less as than `DataWidth`.
module rr_arb_tree #(
  /// Number of inputs to be arbitrated.
  parameter int unsigned NumIn      = 64,
  /// Data width of the payload in bits. Not needed if `DataType` is overwritten.
  parameter int unsigned DataWidth  = 32,
  /// Data type of the payload, can be overwritten with custom type. Only use of `DataWidth`.
  parameter type         DataType   = logic [DataWidth-1:0],
  /// The `ExtPrio` option allows to override the internal round robin counter via the
  /// `rr_i` signal. This can be useful in case multiple arbiters need to have
  /// rotating priorities that are operating in lock-step. If static priority arbitration
  /// is needed, just connect `rr_i` to '0.
  ///
  /// Set to 1'b1 to enable.
  parameter bit          ExtPrio    = 1'b0,
  /// If `AxiVldRdy` is set, the req/gnt signals are compliant with the AXI style vld/rdy
  /// handshake. Namely, upstream vld (req) must not depend on rdy (gnt), as it can be deasserted
  /// again even though vld is asserted. Enabling `AxiVldRdy` leads to a reduction of arbiter
  /// delay and area.
  ///
  /// Set to `1'b1` to treat req/gnt as vld/rdy.
  parameter bit          AxiVldRdy  = 1'b0,
  /// The `LockIn` option prevents the arbiter from changing the arbitration
  /// decision when the arbiter is disabled. I.e., the index of the first request
  /// that wins the arbitration will be locked in case the destination is not
  /// able to grant the request in the same cycle.
  ///
  /// Set to `1'b1` to enable.
  parameter bit          LockIn     = 1'b0,
  /// When set, ensures that throughput gets distributed evenly between all inputs.
  ///
  /// Set to `1'b0` to disable.
  parameter bit          FairArb    = 1'b1,
  /// Dependent parameter, do **not** overwrite.
  /// Width of the arbitration priority signal and the arbitrated index.
  parameter int unsigned IdxWidth   = (NumIn > 32'd1) ? unsigned'($clog2(NumIn)) : 32'd1,
  /// Dependent parameter, do **not** overwrite.
  /// Type for defining the arbitration priority and arbitrated index signal.
  parameter type         idx_t      = logic [IdxWidth-1:0]
) (
  /// Clock, positive edge triggered.
  input  logic                clk_i,
  /// Asynchronous reset, active low.
  input  logic                rst_ni,
  /// Clears the arbiter state. Only used if `ExtPrio` is `1'b0` or `LockIn` is `1'b1`.
  input  logic                flush_i,
  /// External round-robin priority. Only used if `ExtPrio` is `1'b1.`
  input  idx_t                rr_i,
  /// Input requests arbitration.
  input  logic    [NumIn-1:0] req_i,
  /* verilator lint_off UNOPTFLAT */
  /// Input request is granted.
  output logic    [NumIn-1:0] gnt_o,
  /* verilator lint_on UNOPTFLAT */
  /// Input data for arbitration.
  input  DataType [NumIn-1:0] data_i,
  /// Output request is valid.
  output logic                req_o,
  /// Output request is granted.
  input  logic                gnt_i,
  /// Output data.
  output DataType             data_o,
  /// Index from which input the data came from.
  output idx_t                idx_o
);

  /* pragma translate_off
  `ifndef VERILATOR
  `ifndef XSIM
  // Default SVA reset
  default disable iff (!rst_ni || flush_i);
  `endif
  `endif
  // pragma translate_on
  */
  // just pass through in this corner case
  if (NumIn == unsigned'(1)) begin : gen_pass_through
    assign req_o    = req_i[0];
    assign gnt_o[0] = gnt_i;
    assign data_o   = data_i[0];
    assign idx_o    = '0;
  // non-degenerate cases
  end else begin : gen_arbiter
    localparam int unsigned NumLevels = unsigned'($clog2(NumIn));

    /* verilator lint_off UNOPTFLAT */
    idx_t    [2**NumLevels-2:0] index_nodes; // used to propagate the indices
    DataType [2**NumLevels-2:0] data_nodes;  // used to propagate the data
    logic    [2**NumLevels-2:0] gnt_nodes;   // used to propagate the grant to masters
    logic    [2**NumLevels-2:0] req_nodes;   // used to propagate the requests to slave
    /* lint_off */
    idx_t                       rr_q;
    logic [NumIn-1:0]           req_d;

    // the final arbitration decision can be taken from the root of the tree
    assign req_o        = req_nodes[0];
    assign data_o       = data_nodes[0];
    assign idx_o        = index_nodes[0];

    if (ExtPrio) begin : gen_ext_rr
      assign rr_q       = rr_i;
      assign req_d      = req_i;
    end else begin : gen_int_rr
      idx_t rr_d;

      // lock arbiter decision in case we got at least one req and no acknowledge
      if (LockIn) begin : gen_lock
        logic  lock_d, lock_q;
        logic [NumIn-1:0] req_q;

        assign lock_d     = req_o & ~gnt_i;
        assign req_d      = (lock_q) ? req_q : req_i;

        always_ff @(posedge clk_i or negedge rst_ni) begin : p_lock_reg
          if (!rst_ni) begin
            lock_q <= '0;
          end else begin
            if (flush_i) begin
              lock_q <= '0;
            end else begin
              lock_q <= lock_d;
            end
          end
        end

        // pragma translate_off
        `ifndef VERILATOR
          lock: assert property(
            @(posedge clk_i) LockIn |-> req_o && !gnt_i |=> idx_o == $past(idx_o)) else
                $fatal (1, "Lock implies same arbiter decision in next cycle if output is not \
                            ready.");

          logic [NumIn-1:0] req_tmp;
          assign req_tmp = req_q & req_i;
          lock_req: assume property(
            @(posedge clk_i) LockIn |-> lock_d |=> req_tmp == req_q) else
                $fatal (1, "It is disallowed to deassert unserved request signals when LockIn is \
                            enabled.");
        `endif
        // pragma translate_on

        always_ff @(posedge clk_i or negedge rst_ni) begin : p_req_regs
          if (!rst_ni) begin
            req_q  <= '0;
          end else begin
            if (flush_i) begin
              req_q  <= '0;
            end else begin
              req_q  <= req_d;
            end
          end
        end
      end else begin : gen_no_lock
        assign req_d = req_i;
      end

      if (FairArb) begin : gen_fair_arb
        logic [NumIn-1:0] upper_mask,  lower_mask;
        idx_t             upper_idx,   lower_idx,   next_idx;
        logic             upper_empty, lower_empty;

        for (genvar i = 0; i < NumIn; i++) begin : gen_mask
          assign upper_mask[i] = (i >  rr_q) ? req_d[i] : 1'b0;
          assign lower_mask[i] = (i <= rr_q) ? req_d[i] : 1'b0;
        end

        lzc #(
          .WIDTH ( NumIn ),
          .MODE  ( 1'b0  )
        ) i_lzc_upper (
          .in_i    ( upper_mask  ),
          .cnt_o   ( upper_idx   ),
          .empty_o ( upper_empty )
        );

        lzc #(
          .WIDTH ( NumIn ),
          .MODE  ( 1'b0  )
        ) i_lzc_lower (
          .in_i    ( lower_mask  ),
          .cnt_o   ( lower_idx   ),
          .empty_o ( /*unused*/  )
        );

        assign next_idx = upper_empty      ? lower_idx : upper_idx;
        assign rr_d     = (gnt_i && req_o) ? next_idx  : rr_q;

      end else begin : gen_unfair_arb
        assign rr_d = (gnt_i && req_o) ? ((rr_q == idx_t'(NumIn-1)) ? '0 : rr_q + 1'b1) : rr_q;
      end

      // this holds the highest priority
      always_ff @(posedge clk_i or negedge rst_ni) begin : p_rr_regs
        if (!rst_ni) begin
          rr_q   <= '0;
        end else begin
          if (flush_i) begin
            rr_q   <= '0;
          end else begin
            rr_q   <= rr_d;
          end
        end
      end
    end

    assign gnt_nodes[0] = gnt_i;

    // arbiter tree
    for (genvar level = 0; unsigned'(level) < NumLevels; level++) begin : gen_levels
      for (genvar l = 0; l < 2**level; l++) begin : gen_level
        // local select signal
        logic sel;
        // index calcs
        localparam int unsigned Idx0 = 2**level-1+l;// current node
        localparam int unsigned Idx1 = 2**(level+1)-1+l*2;
        //////////////////////////////////////////////////////////////
        // uppermost level where data is fed in from the inputs
        if (unsigned'(level) == NumLevels-1) begin : gen_first_level
          // if two successive indices are still in the vector...
          if (unsigned'(l) * 2 < NumIn-1) begin : gen_reduce
            assign req_nodes[Idx0]   = req_d[l*2] | req_d[l*2+1];

            // arbitration: round robin
            assign sel =  ~req_d[l*2] | req_d[l*2+1] & rr_q[NumLevels-1-level];

            assign index_nodes[Idx0] = idx_t'(sel);
            assign data_nodes[Idx0]  = (sel) ? data_i[l*2+1] : data_i[l*2];
            assign gnt_o[l*2]        = gnt_nodes[Idx0] & (AxiVldRdy | req_d[l*2])   & ~sel;
            assign gnt_o[l*2+1]      = gnt_nodes[Idx0] & (AxiVldRdy | req_d[l*2+1]) & sel;
          end
          // if only the first index is still in the vector...
          if (unsigned'(l) * 2 == NumIn-1) begin : gen_first
            assign req_nodes[Idx0]   = req_d[l*2];
            assign index_nodes[Idx0] = '0;// always zero in this case
            assign data_nodes[Idx0]  = data_i[l*2];
            assign gnt_o[l*2]        = gnt_nodes[Idx0] & (AxiVldRdy | req_d[l*2]);
          end
          // if index is out of range, fill up with zeros (will get pruned)
          if (unsigned'(l) * 2 > NumIn-1) begin : gen_out_of_range
            assign req_nodes[Idx0]   = 1'b0;
            assign index_nodes[Idx0] = idx_t'('0);
            assign data_nodes[Idx0]  = DataType'('0);
          end
        //////////////////////////////////////////////////////////////
        // general case for other levels within the tree
        end else begin : gen_other_levels
          assign req_nodes[Idx0]   = req_nodes[Idx1] | req_nodes[Idx1+1];

          // arbitration: round robin
          assign sel =  ~req_nodes[Idx1] | req_nodes[Idx1+1] & rr_q[NumLevels-1-level];

          assign index_nodes[Idx0] = (sel) ?
            idx_t'({1'b1, index_nodes[Idx1+1][NumLevels-unsigned'(level)-2:0]}) :
            idx_t'({1'b0, index_nodes[Idx1][NumLevels-unsigned'(level)-2:0]});

          assign data_nodes[Idx0]  = (sel) ? data_nodes[Idx1+1] : data_nodes[Idx1];
          assign gnt_nodes[Idx1]   = gnt_nodes[Idx0] & ~sel;
          assign gnt_nodes[Idx1+1] = gnt_nodes[Idx0] & sel;
        end
        //////////////////////////////////////////////////////////////
      end
    end

    // pragma translate_off
    `ifndef VERILATOR
    `ifndef XSIM
    initial begin : p_assert
      assert(NumIn)
        else $fatal(1, "Input must be at least one element wide.");
      assert(!(LockIn && ExtPrio))
        else $fatal(1,"Cannot use LockIn feature together with external ExtPrio.");
    end

    hot_one : assert property(
      @(posedge clk_i) $onehot0(gnt_o))
        else $fatal (1, "Grant signal must be hot1 or zero.");

    gnt0 : assert property(
      @(posedge clk_i) |gnt_o |-> gnt_i)
        else $fatal (1, "Grant out implies grant in.");

    gnt1 : assert property(
      @(posedge clk_i) req_o |-> gnt_i |-> |gnt_o)
        else $fatal (1, "Req out and grant in implies grant out.");

    gnt_idx : assert property(
      @(posedge clk_i) req_o |->  gnt_i |-> gnt_o[idx_o])
        else $fatal (1, "Idx_o / gnt_o do not match.");

    req0 : assert property(
      @(posedge clk_i) |req_i |-> req_o)
        else $fatal (1, "Req in implies req out.");

    req1 : assert property(
      @(posedge clk_i) req_o |-> |req_i)
        else $fatal (1, "Req out implies req in.");
    `endif
    `endif
    // pragma translate_on
  end

endmodule : rr_arb_tree
