create_clock -name clk    -period 20.000 [get_ports {clk}]
create_clock -name ft_clk -period 16.667 [get_ports {ft_clk}]

derive_clock_uncertainty

# Declare 50 MHz FPGA domain and 60 MHz FT232H domain as asynchronous
set_clock_groups -asynchronous \
    -group [get_clocks {clk}] \
    -group [get_clocks {ft_clk}]

# ---------------------------------------------------------------------------
# FT232H 245 Synchronous FIFO, FPGA -> PC (write) direction
# AN_130 / TN_167:  t11 CLKOUT->TXE# max 7.15 (AN_130) / 9 ns (TN_167)
#                   t12 write DATA setup >= 7.5..8 ns, t14 WR# setup >= 7.5..8 ns
#                   t13 DATA hold = 0,  t15 WR# hold = 0
# Worst case of both documents is used.
# ---------------------------------------------------------------------------
set_input_delay  -clock ft_clk -max 9.0 [get_ports {ft_txe_n}]
set_input_delay  -clock ft_clk -min 0.0 [get_ports {ft_txe_n}]
set_output_delay -clock ft_clk -max 8.0 [get_ports {ft_data[*] ft_wr_n}]
set_output_delay -clock ft_clk -min 0.0 [get_ports {ft_data[*] ft_wr_n}]
