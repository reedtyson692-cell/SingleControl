#ifndef _Delta_EtherCAT_Master_L231_B1
#define _Delta_EtherCAT_Master_L231_B1

#include "Type_def.h"

#if defined __cplusplus
extern "C" {
#endif

//======PCIe-L231-B1 Series========
// QEP information -> nQepIndex:0~2
U16 _stdcall _ECAT_L231_B1_qep_set_mode(U16 CardNo, U16 nQepIndex, U16 nMode);
//  nMode:[0]CW/CCW  [1]A/B Phase
U16 _stdcall _ECAT_L231_B1_qep_set_inverse(U16 CardNo, U16 nQepIndex, U16 bEnable);
//  nEnable:[0]Normal  [1]Inverse
U16 _stdcall _ECAT_L231_B1_qep_get_reload_counter(U16 CardNo, U16 nQepIndex, I32* nCounter);
U16 _stdcall _ECAT_L231_B1_qep_set_reload_counter(U16 CardNo, U16 nQepIndex, I32 nCounter);
U16 _stdcall _ECAT_L231_B1_qep_get_counter(U16 CardNo, U16 nQepIndex, I32* nCounter);

////////////////////////////////////////////////////////////////////////////////////////////////////
//// QEP information -> GPIO setting
U16 _stdcall _ECAT_L231_B1_gpio_software_control_status(U16 CardNo, U16 nPin, U16* nStatus);
// nStatus:bit# -> Pin#
U16 _stdcall _ECAT_L231_B1_gpio_software_control_setting(U16 CardNo, U16 nPin, U16 nSetting);
// nSetting:bit# -> Pin#

////////////////////////////////////////////////////////////////////////////////////////////////////
//// IPCMP#(0~3) operation
U16 _stdcall _ECAT_L231_B1_ipcmp_initial(U16 CardNo, U16 nIpcmpIndex, U16 nQepIndex, U16 nGpioPin, U16 bInverse);  //nIpcmpIndex=0~3組比較器, nQepIndex=0~2, nGpioPin=0~9, U16 bInverse=準位
U16 _stdcall _ECAT_L231_B1_ipcmp_oneshot_time(U16 CardNo, U16 nIpcmpIndex, U16 nPulseWidth);
//  nPulseWidth -> 0~4095(us)
U16 _stdcall _ECAT_L231_B1_ipcmp_software_trigger(U16 CardNo, U16 nIpcmpIndex);
U16 _stdcall _ECAT_L231_B1_ipcmp_compare_position(U16 CardNo, U16 nIpcmpIndex, I32 nStartPosition, U16 nInterval, U32 nCount, U16 nDir);
//  nDir -> [0]CW  [1]CCW
U16 _stdcall _ECAT_L231_B1_ipcmp_compare_trigger_count(U16 CardNo, U16 nIpcmpIndex, U32* nCount);
U16 _stdcall _ECAT_L231_B1_ipcmp_compare_trigger_count_clear(U16 CardNo, U16 nIpcmpIndex);
////////////////////////////////////////////////////////////////////////////////////////////////////
//// TPCMP#(0~5) operation
U16 _stdcall _ECAT_L231_B1_tpcmp_initial(U16 CardNo, U16 nFifoIndex, U16 nQepIndex, U16 nGpioPin, U16 bInverse, U16 bInterrupt);
//  nSource -> QEP#(0~2)
U16 _stdcall _ECAT_L231_B1_tpcmp_oneshot_time(U16 CardNo, U16 nFifoIndex, U16 nPulseWidth);
//  nPulseWidth -> 0~4095(us)
U16 _stdcall _ECAT_L231_B1_tpcmp_fifo_size(U16 CardNo, U16 nFifoIndex, U16* nSize);
U16 _stdcall _ECAT_L231_B1_tpcmp_fifo_status(U16 CardNo, U16 nFifoIndex, U16* nStatus);
//  nStatus -> Bit:[0]Empty  [1]Almost empty  [2]Full
U16 _stdcall _ECAT_L231_B1_tpcmp_software_trigger(U16 CardNo, U16 nFifoIndex);
U16 _stdcall _ECAT_L231_B1_tpcmp_software_trigger_level(U16 CardNo, U16 nFifoIndex, U16 bActive);
U16 _stdcall _ECAT_L231_B1_tpcmp_compare_position_table_reset(U16 CardNo, U16 nFifoIndex);
U16 _stdcall _ECAT_L231_B1_tpcmp_compare_position_table(U16 CardNo, U16 nFifoIndex, I32 *nPositionTable, U32 nTableSize);
U16 _stdcall _ECAT_L231_B1_tpcmp_compare_position_table_level(U16 CardNo, U16 nFifoIndex, I32 *nPositionTable, U16 *nLevelTable, U32 nTableSize);
U16 _stdcall _ECAT_L231_B1_tpcmp_compare_trigger_count(U16 CardNo, U16 nFifoIndex, U32* nCount);
U16 _stdcall _ECAT_L231_B1_tpcmp_compare_trigger_count_clear(U16 CardNo, U16 nFifoIndex);
U16 _stdcall _ECAT_L231_B1_tpcmp_compare_data_lock(U16 CardNo, U16 nFifoIndex, U16* nLock);
////////////////////////////////////////////////////////////////////////////////////////////////////
//// External IO#(0~7) and latch FIFO operation
U16 _stdcall _ECAT_L231_B1_ext_io_get_input_value(U16 CardNo, U16* nState);
U16 _stdcall _ECAT_L231_B1_ext_io_get_output_value(U16 CardNo, U16* nState);
U16 _stdcall _ECAT_L231_B1_ext_io_set_output_value(U16 CardNo, U16 nState);
U16 _stdcall _ECAT_L231_B1_ext_io_latch_initial(U16 CardNo, U16 nExternalIoNo, U16 nSource, U16 nFilter);
// nSource -> [0]TPCMP#0  [1]TPCMP#1  [2]TPCMP#2  [3]TPCMP#3  [4]TPCMP#4  [5]TPCMP#5  [6]Software
// nFilter -> [0]Disable  [1]0.1ms  [2]0.5ms  [3]1ms
U16 _stdcall _ECAT_L231_B1_ext_io_latch_function(U16 CardNo, U16 nExternalIoNo, U16 bQep0, U16 bQep1, U16 bQep2, U16 bSw, U16 bH2L, U16 bL2H);
U16 _stdcall _ECAT_L231_B1_ext_io_latch_enable(U16 CardNo, U16 nExternalIoNo, U16 bEnable);
U16 _stdcall _ECAT_L231_B1_latch_fifo_reset(U16 CardNo, U16 bEnable);
U16 _stdcall _ECAT_L231_B1_latch_fifo_status(U16 CardNo, U16* nStatus);
//  nStatus -> Bit:[0]Empty  [1]Full
U16 _stdcall _ECAT_L231_B1_latch_fifo_size(U16 CardNo, U16* nSize);
U16 _stdcall _ECAT_L231_B1_latch_fifo_get_qep_counter(U16 CardNo, I32* nCounter, U16* nIoType, U16* nQepSource);

#if defined __cplusplus
}
#endif

#endif