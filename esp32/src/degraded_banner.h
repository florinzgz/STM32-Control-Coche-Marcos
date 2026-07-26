#ifndef DEGRADED_BANNER_H
#define DEGRADED_BANNER_H

#include <cstdint>

namespace degraded_banner {

inline const char* text(uint8_t level)
{
    switch (level) {
        case 1: return "DEGRADED L1 70%";
        case 2: return "DEGRADED L2 50%";
        case 3: return "DEGRADED L3 40%";
        default: return "DEGRADED";
    }
}

} // namespace degraded_banner

#endif /* DEGRADED_BANNER_H */
