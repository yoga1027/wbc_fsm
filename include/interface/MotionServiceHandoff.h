#pragma once

#include <ostream>
#include <stdexcept>
#include <string>

// Callbacks permit exercising every RPC outcome without DDS or a robot.
// Only a successful CheckMode with an empty name confirms control is released.
template <typename Check, typename Release, typename Wait>
void releaseMotionService(bool realRobot, Check check, Release release, Wait wait,
                          std::ostream &log)
{
    if (!realRobot) {
        log << "[Startup] Simulation: skipping motion service release." << std::endl;
        return;
    }
    constexpr int maxReleases = 5;
    for (int attempt = 0; attempt <= maxReleases; ++attempt) {
        std::string form, name;
        const auto checked = check(form, name);
        if (checked != 0)
            throw std::runtime_error("CheckMode failed (ret=" + std::to_string(checked) +
                                     "); refusing low-level control");
        if (name.empty()) {
            log << "[Startup] Motion service release confirmed." << std::endl;
            return;
        }
        if (attempt == maxReleases) break;
        log << "[Startup] Releasing motion service: " << name
            << " (attempt " << attempt + 1 << "/" << maxReleases << ")" << std::endl;
        const auto released = release();
        if (released != 0)
            throw std::runtime_error("ReleaseMode failed (ret=" + std::to_string(released) +
                                     "); refusing low-level control");
        wait();
    }
    throw std::runtime_error("Motion service is still active after retries; refusing low-level control");
}
