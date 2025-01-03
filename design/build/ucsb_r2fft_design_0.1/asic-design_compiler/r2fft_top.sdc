###################################################################

# Created by write_sdc on Thu Jan  2 14:46:36 2025

###################################################################
set sdc_version 2.1

set_units -time ns -resistance MOhm -capacitance fF -voltage V -current mA
set_operating_conditions typical -library NangateOpenCellLibrary
create_clock [get_ports clk]  -period 4  -waveform {0 2}
set_clock_uncertainty 0.25  [get_clocks clk]
set_input_delay -clock clk  2  [get_ports rst]
set_input_delay -clock clk  2  [get_ports autorun]
set_input_delay -clock clk  2  [get_ports run]
set_input_delay -clock clk  2  [get_ports fin]
set_input_delay -clock clk  2  [get_ports ifft]
set_input_delay -clock clk  2  [get_ports sact_istream]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[15]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[14]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[13]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[12]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[11]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[10]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[9]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[8]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[7]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[6]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[5]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[4]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[3]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[2]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[1]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_real[0]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[15]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[14]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[13]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[12]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[11]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[10]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[9]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[8]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[7]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[6]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[5]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[4]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[3]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[2]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[1]}]
set_input_delay -clock clk  2  [get_ports {sdw_istream_imag[0]}]
set_input_delay -clock clk  2  [get_ports dmaact]
set_input_delay -clock clk  2  [get_ports {dmaa[9]}]
set_input_delay -clock clk  2  [get_ports {dmaa[8]}]
set_input_delay -clock clk  2  [get_ports {dmaa[7]}]
set_input_delay -clock clk  2  [get_ports {dmaa[6]}]
set_input_delay -clock clk  2  [get_ports {dmaa[5]}]
set_input_delay -clock clk  2  [get_ports {dmaa[4]}]
set_input_delay -clock clk  2  [get_ports {dmaa[3]}]
set_input_delay -clock clk  2  [get_ports {dmaa[2]}]
set_input_delay -clock clk  2  [get_ports {dmaa[1]}]
set_input_delay -clock clk  2  [get_ports {dmaa[0]}]
set_output_delay -clock clk  2  [get_ports done]
set_output_delay -clock clk  2  [get_ports {status[2]}]
set_output_delay -clock clk  2  [get_ports {status[1]}]
set_output_delay -clock clk  2  [get_ports {status[0]}]
set_output_delay -clock clk  2  [get_ports {bfpexp[7]}]
set_output_delay -clock clk  2  [get_ports {bfpexp[6]}]
set_output_delay -clock clk  2  [get_ports {bfpexp[5]}]
set_output_delay -clock clk  2  [get_ports {bfpexp[4]}]
set_output_delay -clock clk  2  [get_ports {bfpexp[3]}]
set_output_delay -clock clk  2  [get_ports {bfpexp[2]}]
set_output_delay -clock clk  2  [get_ports {bfpexp[1]}]
set_output_delay -clock clk  2  [get_ports {bfpexp[0]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[15]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[14]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[13]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[12]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[11]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[10]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[9]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[8]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[7]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[6]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[5]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[4]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[3]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[2]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[1]}]
set_output_delay -clock clk  2  [get_ports {dmadr_real[0]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[15]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[14]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[13]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[12]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[11]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[10]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[9]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[8]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[7]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[6]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[5]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[4]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[3]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[2]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[1]}]
set_output_delay -clock clk  2  [get_ports {dmadr_imag[0]}]
