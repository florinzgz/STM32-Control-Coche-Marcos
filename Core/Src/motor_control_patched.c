/*
 * Legacy compatibility tombstone.
 *
 * The production motor-control implementation lives exclusively in
 * Core/Src/motor_control.c.  Older local workspaces may still reference this
 * filename from generated STM32CubeIDE Debug/Release makefiles.  Keeping this
 * translation unit intentionally empty prevents those stale references from
 * introducing a second copy of Motor_*, Traction_*, Steering_* and Ackermann_*
 * symbols at link time.
 *
 * Do not add motor-control logic here.  Apply all fixes to motor_control.c.
 */
