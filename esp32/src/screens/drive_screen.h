// =============================================================================
// ESP32-S3 HMI — Drive Screen (stub)
// =============================================================================

#ifndef DRIVE_SCREEN_H
#define DRIVE_SCREEN_H

#include "screen.h"

class DriveScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;
};

#endif // DRIVE_SCREEN_H
