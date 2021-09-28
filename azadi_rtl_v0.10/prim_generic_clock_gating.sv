// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Common Library: Clock Gating cell

module prim_generic_clock_gating #(
  parameter bit NoFpgaGate = 1'b0 // this parameter has no function in generic
) (
  input        clk_i,
  input        en_i,
  input        test_en_i,
  output logic clk_o
);

sky130_fd_sc_hd__dlclkp_1 CG( .CLK(clk_i), .GCLK(clk_o), .GATE(en_i | test_en_i));

endmodule
