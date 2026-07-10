/**
  ****************************************************************************
  * @file    test_obstacle_fault_state.c
  * @brief   Host regression tests for obstacle sensor-fault state handling.
  *
  *          Focuses on the branch in Obstacle_Update() that preserves the
  *          last safe restriction when sensor data becomes invalid.
  ****************************************************************************
  */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define OBSTACLE_FAULT_SCALE 0.3f

typedef enum {
    OBS_STATE_NO_SENSOR = 0,
    OBS_STATE_NORMAL,
    OBS_STATE_CONFIRMING,
    OBS_STATE_ACTIVE,
    OBS_STATE_CLEARING,
    OBS_STATE_SENSOR_FAULT
} ObstacleState_t;

typedef struct {
    ObstacleState_t state;
    float scale;
    uint8_t forward_blocked;
} ObstacleFaultState_t;

static void apply_sensor_fault(ObstacleFaultState_t *s)
{
    if (s->state == OBS_STATE_ACTIVE ||
        s->state == OBS_STATE_CONFIRMING ||
        s->forward_blocked != 0U) {
        if (s->scale > OBSTACLE_FAULT_SCALE) {
            s->scale = OBSTACLE_FAULT_SCALE;
        }
    } else {
        s->scale = OBSTACLE_FAULT_SCALE;
        s->forward_blocked = 0U;
    }
    s->state = OBS_STATE_SENSOR_FAULT;
}

static int tests_run = 0;
static int tests_failed = 0;

#define CHECK(cond, msg) do {                                     \
        tests_run++;                                              \
        if (!(cond)) { tests_failed++;                            \
            printf("  FAIL: %s\n", msg); }                        \
        else { printf("  ok:   %s\n", msg); }                     \
    } while (0)

int main(void)
{
    printf("=== test_obstacle_fault_state ===\n");

    ObstacleFaultState_t s = {
        .state = OBS_STATE_ACTIVE,
        .scale = 0.0f,
        .forward_blocked = 1U
    };

    apply_sensor_fault(&s);
    CHECK(s.state == OBS_STATE_SENSOR_FAULT,
          "first fault cycle enters SENSOR_FAULT state");
    CHECK(s.scale == 0.0f,
          "emergency-zone scale stays latched at zero on first fault cycle");
    CHECK(s.forward_blocked == 1U,
          "first fault cycle preserves forward-block latch");

    apply_sensor_fault(&s);
    CHECK(s.state == OBS_STATE_SENSOR_FAULT,
          "repeated fault cycle stays in SENSOR_FAULT state");
    CHECK(s.scale == 0.0f,
          "repeated fault cycle keeps zero-scale emergency latch");
    CHECK(s.forward_blocked == 1U,
          "repeated fault cycle keeps forward blocked");

    s.state = OBS_STATE_ACTIVE;
    s.scale = 0.6f;
    s.forward_blocked = 0U;
    apply_sensor_fault(&s);
    CHECK(s.scale == OBSTACLE_FAULT_SCALE,
          "non-emergency active obstacle is capped at fault scale");
    CHECK(s.forward_blocked == 0U,
          "non-emergency active obstacle does not invent forward block");

    apply_sensor_fault(&s);
    CHECK(s.scale == OBSTACLE_FAULT_SCALE,
          "repeated non-emergency fault stays capped at fault scale");
    CHECK(s.forward_blocked == 0U,
          "repeated non-emergency fault stays unblocked");

    s.state = OBS_STATE_NORMAL;
    s.scale = 1.0f;
    s.forward_blocked = 0U;
    apply_sensor_fault(&s);
    CHECK(s.scale == OBSTACLE_FAULT_SCALE,
          "non-tracked fault falls back to conservative fault scale");
    CHECK(s.forward_blocked == 0U,
          "non-tracked fault stays unblocked");

    printf("=== %d run, %d failed ===\n", tests_run, tests_failed);
    return tests_failed ? 1 : 0;
}
