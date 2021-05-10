set_db init_lib_search_path /merledu1/pdks/sky130A/libs.ref/sky130_fd_sc_hd/lib/
set_db init_hdl_search_path ../src/
read_libs /merledu1/pdks/sky130A/libs.ref/sky130_fd_sc_hd/lib/sky130_fd_sc_hd__tt_100C_1v80.lib

#read_hdl *.v

read_hdl -sv {brq_pkg.sv dm_pkg.sv fpnew_pkg.sv gpio_reg_pkg.sv jtag_pkg.sv prim_pkg.sv rv_plic_reg_pkg.sv rv_timer_reg_pkg.sv tl_main_pkg.sv tl_periph_pkg.sv tlul_pkg.sv uart_reg_pkg.sv prim_assert_dummy_macros.svh azadi_soc_top.sv brq_core.sv brq_core_top.sv brq_core_tracing.sv brq_counter.sv brq_cs_registers.sv brq_csr.sv brq_exu_alu.sv brq_exu_multdiv_fast.sv brq_exu_multdiv_slow.sv brq_exu.sv brq_fpnew_top_wrapper.sv brq_fp_register_file_ff.sv brq_idu_controller.sv brq_idu_decoder.sv brq_idu.sv brq_ifu_compressed_decoder.sv brq_ifu_dummy_instr.sv brq_ifu_fifo.sv brq_ifu_icache.sv brq_ifu_prefetch_buffer.sv brq_ifu.sv brq_lsu.sv brq_pmp.sv brq_register_file_ff.sv brq_wbu.sv control_mvp.sv data_mem_top.sv defs_div_sqrt_mvp.sv DFFRAM.sv div_sqrt_mvp_wrapper.sv div_sqrt_top_mvp.sv dm_csrs.sv dmi_cdc.sv dmi_jtag.sv dmi_jtag_tap.sv dm_mem.sv dm_obi_top.sv dm_sba.sv dm_top.sv fifo_async.sv fifo_sync.sv fpnew_cast_multi.sv fpnew_classifier.sv fpnew_divsqrt_multi.sv fpnew_fma_multi.sv fpnew_fma.sv fpnew_noncomp.sv fpnew_opgroup_block.sv fpnew_opgroup_fmt_slice.sv fpnew_opgroup_multifmt_slice.sv fpnew_rounding.sv fpnew_top.sv gpio_reg_top.sv gpio.sv instr_mem_top.sv iteration_div_sqrt_mvp.sv jtagdpi.sv lzc.sv norm_div_sqrt_mvp.sv nrbd_nrsc_mvp.sv preprocess_mvp.sv prim_arbiter_ppc.sv prim_clock_gating.sv prim_filter_ctr.sv prim_filter.sv prim_generic_clock_gating.sv prim_generic_clock_inv.sv prim_generic_clock_mux2.sv prim_generic_flop_2sync.sv prim_generic_flop.sv prim_intr_hw.sv prim_subreg_arb.sv prim_subreg_ext.sv prim_subreg.sv pwm_top.sv rr_arb_tree.sv rstmgr.sv rv_dm.sv rv_plic_gateway.sv rv_plic_reg_top.sv rv_plic.sv rv_plic_target.sv rv_timer_reg_top.sv rv_timer.sv spi_top.sv timer_core.sv tlul_adapter_reg.sv tlul_err_resp.sv tlul_err.sv tlul_host_adapter.sv tlul_socket_1n.sv tlul_socket_m1.sv tlul_sram_adapter.sv tl_xbar_main.sv uart_core.sv uart_reg_top.sv uart_rx.sv uart.sv uart_tx.sv xbar_periph.sv}

read_hdl -v {spi_defines.v down_clocking_even.v down_clocking_odd.v iccm_controller.v minus_one.v PWM.v spi_clgen.v spi_core.v spi_defines.v spi_shift.v uart_receiver.v}

elaborate
check_design -unresolved
read_sdc ../constraints/constraints_top.sdc

set_db syn_generic_effort medium
set_db syn_map_effort medium
set_db syn_opt_effort medium

syn_generic
syn_map
syn_opt

#generate reports
report_area 
report_power
report_design_rules
report_hierarchy
report_memory
report_messages
report_qor
report_timing
report_summary

write_netlist > Azadi.synthesis.v
write_design -innovusi
write_hdl > Azazi_netlist.v
write_sdc > Azadi_sdc.sdc
write_sdf -timescale ns -nonegchecks -recrem split -edges check_edge  -setuphold split > delays.sdf
