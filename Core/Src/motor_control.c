/**
  ****************************************************************************
  * @file    motor_control.c
  * @brief   Compatibility dispatcher for the patched motor-control unit.
  *
  * The real implementation is stored in motor_control_base.inc and is included
  * by motor_control_patched.c.  Some existing STM32CubeIDE workspaces may still
  * compile this path as a standalone source file.  In that case this translation
  * unit intentionally exports no symbols, preventing duplicate definitions when
  * motor_control_patched.o is linked at the same time.
  ****************************************************************************
  */

#if defined(__INCLUDE_LEVEL__) && (__INCLUDE_LEVEL__ > 0)
#include "motor_control_base.inc"
#else
typedef char motor_control_standalone_tombstone_t;
#endif
