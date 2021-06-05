# User config
set ::env(DESIGN_NAME) azadi_soc_top

# Change if needed
set ::env(VERILOG_FILES) [glob $::env(DESIGN_DIR)/src/azadi_soc_top_conv.v]
#/home/merl-tools/openlane-v0.15/openlane/designs/azadi-verilog

# Fill this
set ::env(CLOCK_PERIOD) "10"
set ::env(CLOCK_PORT) "clk_i"
set ::env(SYNTH_READ_BLACKBOX_LIB) 1

set filename $::env(DESIGN_DIR)/$::env(PDK)_$::env(STD_CELL_LIBRARY)_config.tcl
if { [file exists $filename] == 1} {
	source $filename
}

