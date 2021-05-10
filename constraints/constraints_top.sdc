create_clock -name clk_i -period 80 -waveform {0 5} [get_ports "clk_i"]
set_clock_transition -rise 0.1 [get_clocks "clk_i"]
set_clock_transition -fall 0.1 [get_clocks "clk_i"]
set_clock_uncertainty 0.01 [get_ports "clk_i"]
set_input_delay -max 1.0 [get_ports "reset"] -clock [get_clocks "clk_i"]
set_output_delay -max 1.0 [get_ports "count"] -clock [get_clocks "clk_i"]



