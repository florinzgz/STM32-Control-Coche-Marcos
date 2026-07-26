#ifndef CURRENT_PLAUSIBILITY_H
#define CURRENT_PLAUSIBILITY_H

#include <stdbool.h>
#include <math.h>

/* Pure, host-testable current plausibility policy.
 *
 * Motor INA226 channels are bidirectional: regeneration, inductive
 * flyback and back-EMF can legitimately produce negative current.
 * Their plausibility envelope is therefore based on magnitude.
 *
 * The battery channel keeps the historical -1 A offset tolerance;
 * a more-negative battery reading remains implausible because this
 * topology has no battery-charging regeneration path.
 *
 * CH5 polarity/wiring diagnosis remains owned by the dedicated
 * Ina226_ClassifyChannel()/steering supervisor path. */
static inline bool CurrentPlausibility_IsFault(bool battery_channel,
        float amps,
        float ceiling_a)
{
    if (isnan(amps) || isinf(amps) ||
        isnan(ceiling_a) || isinf(ceiling_a) || ceiling_a <= 0.0f) {
        return true;
    }

    if (battery_channel) {
        return (amps < -1.0f) || (amps > ceiling_a);
    }

    return fabsf(amps) > ceiling_a;
}

#endif /* CURRENT_PLAUSIBILITY_H */
