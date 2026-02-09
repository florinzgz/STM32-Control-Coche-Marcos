// =============================================================================
// ESP32-S3 HMI — Error Screen (stub)
// =============================================================================

#ifndef ERROR_SCREEN_H
#define ERROR_SCREEN_H

#include "screen.h"

class ErrorScreen : public Screen {
public:
    void onEnter() override;
    void onExit()  override;
    void update(const vehicle::VehicleData& data) override;
    void draw()    override;
};

#endif // ERROR_SCREEN_H
