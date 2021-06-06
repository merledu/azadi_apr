# User config
set ::env(DESIGN_NAME) azadi_soc_top

# Change if needed
set ::env(VERILOG_FILES) [glob $::env(DESIGN_DIR)/src/pkg_files/*.sv $::env(DESIGN_DIR)/src/pkg_files/registers.svh $::env(DESIGN_DIR)/src/pkg_files/utils.vh $::env(DESIGN_DIR)/src/*.sv $::env(DESIGN_DIR)/src/*.v]

# Macros
set ::env(EXTRA_LEFS) [glob $::env(DESIGN_DIR)/macros/lef/*.lef]
set ::env(EXTRA_GDS_FILES) [glob $::env(DESIGN_DIR)/macros/gds/*.gds]
#set ::env(MACRO_PLACEMENT_CFG) $::env(OPENLANE_ROOT)/designs/$::env(DESIGN_NAME)/macro_placement.cfg

set ::env(PDN_CFG) $::env(DESIGN_DIR)/pdn.tcl
# Fixed Die Area
set ::env(PL_TARGET_DENSITY) 0.4
set ::env(FP_SIZING) "absolute"
set ::env(DIE_AREA) "0 0 2000 2100"

set ::env(DESIGN_IS_CORE) 1
set ::env(FP_HORIZONTAL_HALO) 25
set ::env(FP_VERTICAL_HALO) 20 


set ::env(FP_PDN_CHECK_NODES) 0
set ::env(CTS_REPORT_TIMING) 
set ::env(GLB_RT_ADJUSTMENT) 0.3
set ::env(FP_PDN_CORE_RING) 0

# Go fast
set ::env(ROUTING_CORES) 6

# It's overly worried about congestion, but it's fine
set ::env(GLB_RT_ALLOW_CONGESTION) 1

# Avoid li1 for routing if possible
set ::env(GLB_RT_MINLAYER) 2

# Don't route on met5
set ::env(GLB_RT_MAXLAYER) 5

# Obstructions
    # li1 over the SRAM areas
	# met5 over the whole design
set ::env(GLB_RT_OBS) "li1 0.00 1286.6 1793.26 1952.7, met5 0.0 0.0 2000.0 2100.0"

#set ::env(FP_PIN_ORDER_CFG) $::env(DESIGN_DIR)/pin_order.cfg
set ::env(SYNTH_STRATEGY) "AREA 0"
set ::env(GENERATE_FINAL_SUMMARY_REPORT) 1
set ::env(SYNTH_READ_BLACKBOX_LIB) 1
#set ::env(BASE_SDC_FILE) $::env(DESIGN_DIR)/costraints/.sdc
#set ::env(ALLOW_RT_CONGESTION) 1                                                                             
# Fill this
set ::env(CLOCK_PERIOD) "40"
set ::env(CLOCK_PORT) "clk_i"

set filename $::env(DESIGN_DIR)/$::env(PDK)_$::env(STD_CELL_LIBRARY)_config.tcl
if { [file exists $filename] == 1} {
	source $filename
}

