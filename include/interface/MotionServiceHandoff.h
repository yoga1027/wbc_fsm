#pragma once

#include <algorithm>
#include <chrono>
#include <ostream>
#include <stdexcept>
#include <string>

struct MotionHandoffTiming
{
    std::chrono::milliseconds timeout{60000};
    std::chrono::milliseconds rpcTimeout{5000};
    std::chrono::milliseconds pollInterval{1000};
    std::chrono::milliseconds quietPeriod{2000};
};

// Adapted from BeiChe's release-before-feedback startup. Callbacks let tests
// exercise the production ordering without DDS, a robot or wall-clock sleeps.
// enable() is the only path to creating the motor command publisher.
template <typename Check, typename Release, typename StateReady, typename Enable,
          typename Wait, typename Now, typename Cancelled>
void prepareMotionControl(bool realRobot, Check check, Release release,
                          StateReady stateReady, Enable enable, Wait wait, Now now,
                          Cancelled cancelled, std::ostream &log,
                          const MotionHandoffTiming &timing = {})
{
    using namespace std::chrono;
    const auto started = now();
    const auto deadline = started + (realRobot ? timing.timeout : milliseconds(5000));
    auto guard = [&] {
        if (cancelled()) throw std::runtime_error("Startup cancelled; low-level publication disabled");
        if (now() >= deadline)
            throw std::runtime_error("Startup timed out without confirmed handoff and fresh feedback; low-level publication disabled");
    };
    auto budget = [&] {
        guard();
        const auto remaining = duration_cast<milliseconds>(deadline - now());
        if (remaining.count() <= 0) throw std::runtime_error("Startup deadline reached");
        return std::min(timing.rpcTimeout, remaining);
    };
    auto pause = [&](milliseconds duration) {
        guard();
        wait(std::min(duration, duration_cast<milliseconds>(deadline - now())));
    };
    if (!realRobot) {
        log << "[Startup] SIMULATION: motion service RPCs disabled. Use --real --interface <NIC> for G1 hardware." << std::endl;
        while (true) {
            guard();
            if (stateReady()) { guard(); enable(); return; }
            pause(milliseconds(50));
        }
    }

    log << "[Startup] handoff-v2: releasing stock motion service before waiting for lowstate; timeout="
        << timing.timeout.count() << " ms." << std::endl;
    int emptyChecks = 0;
    auto emptySince = started;
    auto query = [&](std::string &name, const char *stage) {
        std::string form;
        int ret = -1;
        const auto rpcBudget = budget();
        try {
            ret = check(form, name, rpcBudget);
            log << "[Startup] " << stage << " CheckMode ret=" << ret
                << " form='" << form << "' name='" << name << "'" << std::endl;
        } catch (const std::exception &e) {
            log << "[Startup] " << stage << " CheckMode exception: " << e.what() << std::endl;
        }
        guard(); // A late reply must never authorize publication.
        return ret;
    };

    while (true) {
        std::string name;
        const int ret = query(name, "handoff");
        if (ret != 0) {
            emptyChecks = 0; // Empty output on RPC failure is not released mode.
        } else if (!name.empty()) {
            emptyChecks = 0;
            const auto rpcBudget = budget();
            try {
                const int released = release(rpcBudget);
                log << "[Startup] ReleaseMode ret=" << released
                    << "; awaiting subsequent CheckMode confirmation." << std::endl;
            } catch (const std::exception &e) {
                log << "[Startup] ReleaseMode exception: " << e.what() << std::endl;
            }
            guard();
        } else {
            if (emptyChecks == 0) emptySince = now();
            emptyChecks = std::min(emptyChecks + 1, 3);
            if (emptyChecks == 3 && now() - emptySince >= timing.quietPeriod) {
                if (stateReady()) {
                    std::string finalName;
                    const int finalRet = query(finalName, "pre-publish");
                    // RPC may have blocked while feedback stopped. Check both
                    // service ownership and feedback again before publication.
                    if (finalRet == 0 && finalName.empty() && stateReady()) {
                        guard();
                        log << "[Startup] Motion service release confirmed: stable empty mode and fresh feedback." << std::endl;
                        enable();
                        return;
                    }
                    emptyChecks = 0;
                } else {
                    log << "[Startup] Service released; waiting for fresh valid rt/lowstate (no motor commands sent)." << std::endl;
                }
            }
        }
        pause(timing.pollInterval);
    }
}
