// =============================================================================
// ESP32-S3 HMI — Safe Screen (stub)
// =============================================================================

#ifndef SAFE_SCREEN_H
#define SAFE_SCREEN_H

#include "screen.h"

class SafeScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;
};

#endif // SAFE_SCREEN_H
