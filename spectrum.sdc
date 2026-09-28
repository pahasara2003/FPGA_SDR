create_clock -name clk -period 20.000 [get_ports {clk}]
create_clock -name ft_clk -period 16.667 [get_ports {ft_clk}]

derive_clock_uncertainty

# Declare 50 MHz FPGA domain and 60 MHz FT232H domain as asynchronous
set_clock_groups -asynchronous \
    -group [get_clocks {clk}] \
    -group [get_clocks {ft_clk}]
