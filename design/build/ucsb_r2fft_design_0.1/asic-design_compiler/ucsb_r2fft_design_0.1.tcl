############################################
#
# Auto-generated project tcl file:
#  * sets up variable
#  * runs customized script
#
############################################
sh date

set TOP_MODULE r2fft_top
set DC_SCRIPT design.tcl

set READ_SOURCES ucsb_r2fft_design_0.1-read-sources

set SCRIPT_DIR /fs/student/nturtayeva/R2FFT/design/dc

set REPORT_DIR "./"
set target_library /fs/student/nturtayeva/R2FFT/design/NangateOpenCellLibrary.db
set link_library   [concat "*"  /fs/student/nturtayeva/R2FFT/design/NangateOpenCellLibrary.db]
############################################
#
# Run custom script
#
############################################
source ${SCRIPT_DIR}/${DC_SCRIPT}

############################################
#
#  all done -- exit
#
############################################
sh date