// =============================================================================
// ESP32-S3 HMI — Standby Screen (stub)
// =============================================================================

#ifndef STANDBY_SCREEN_H
#define STANDBY_SCREEN_H

#include "screen.h"

class StandbyScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;
};

#endif // STANDBY_SCREEN_H
