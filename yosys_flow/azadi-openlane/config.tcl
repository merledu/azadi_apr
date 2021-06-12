# User config
set ::env(DESIGN_NAME) azadi_soc_top

# Change if needed
set ::env(VERILOG_FILES) [glob $::env(DESIGN_DIR)/src/azadi_soc_top_conv-dummy.v]

# Synthesis
set ::env(SYNTH_NO_FLAT) 1

set ::env(FP_CORE_UTIL) 35
set ::env(FP_PDN_VOFFSET) 0
set ::env(FP_PDN_VPITCH) 30

# For manual macro placement of sram
#set ::env(VERILOG_FILES) [glob $::env(DESIGN_DIR)/src/azadi_soc_top_conv_memBB.v]
#set ::env(MACRO_PLACEMENT_CFG) $::env(OPENLANE_ROOT)/designs/$::env(DESIGN_NAME)/macro_placement.cfg
set ::env(EXTRA_LEFS) [glob $::env(DESIGN_DIR)/macros/lef/*.lef]
set ::env(EXTRA_GDS_FILES) [glob $::env(DESIGN_DIR)/macros/gds/*.gds]

set ::env(PL_TARGET_DENSITY) 0.35
set ::env(PL_BASIC_PLACEMENT) 1
set ::env(CELL_PAD) 0

# Fill this
set ::env(CLOCK_PERIOD) "40"
set ::env(CLOCK_PORT) "clk_i"
set ::env(SYNTH_READ_BLACKBOX_LIB) 1

set filename $::env(DESIGN_DIR)/$::env(PDK)_$::env(STD_CELL_LIBRARY)_config.tcl
if { [file exists $filename] == 1} {
	source $filename
}

