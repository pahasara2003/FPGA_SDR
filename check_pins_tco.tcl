project_open spectrum
create_timing_netlist
read_sdc
update_timing_netlist
for {set i 0} {$i < 8} {incr i} {
    report_datasheet -expand_bus -stdout
}
project_close
