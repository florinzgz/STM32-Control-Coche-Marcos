// =============================================================================
// ESP32-S3 HMI — Boot Screen (stub)
// =============================================================================

#ifndef BOOT_SCREEN_H
#define BOOT_SCREEN_H

#include "screen.h"

class BootScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;
};

#endif // BOOT_SCREEN_H
