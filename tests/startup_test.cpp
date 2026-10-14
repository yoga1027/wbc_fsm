#include "interface/MotionServiceHandoff.h"
#include "interface/StartupOptions.h"
#include <iostream>
#include <functional>
#include <sstream>
#include <vector>

void require(bool ok, const char *message)
{
    if (!ok) throw std::runtime_error(message);
}

template <typename F> void mustFail(F run, const char *message)
{
    bool failed = false;
    try { run(); } catch (const std::runtime_error &) { failed = true; }
    require(failed, message);
}

StartupOptions parse(std::vector<std::string> args)
{
    std::vector<char *> argv;
    for (auto &arg : args) argv.push_back(arg.data());
    return StartupOptions::parse(static_cast<int>(argv.size()), argv.data());
}

using namespace std::chrono;
struct Reply { int ret = 0; std::string name; };
struct Handoff
{
    steady_clock::time_point clock{};
    std::vector<Reply> replies;
    Reply fallback{};
    std::vector<int> releaseResults;
    int checks = 0, releases = 0, enabled = 0;
    bool feedback = true, releaseEnablesFeedback = false;
    bool invalidateAtFinal = false;
    milliseconds rpcCost{0};
    milliseconds feedbackAfter{0};
    milliseconds cancelAfter{999999};
    milliseconds enabledAt{0};
    std::vector<milliseconds> budgets;
    std::ostringstream log;

    milliseconds elapsed() const { return duration_cast<milliseconds>(clock.time_since_epoch()); }
    void run(bool real = true)
    {
        prepareMotionControl(real,
            [&](std::string &form, std::string &name, milliseconds budget) {
                budgets.push_back(budget);
                require(budget > milliseconds(0) && budget <= milliseconds(5000), "RPC timeout out of bounds");
                const auto reply = checks < static_cast<int>(replies.size()) ? replies[checks] : fallback;
                ++checks;
                clock += rpcCost;
                if (invalidateAtFinal && checks == 4) feedback = false;
                if (reply.ret == -999) throw std::runtime_error("injected RPC exception");
                form = "0";
                name = reply.name;
                return reply.ret;
            },
            [&](milliseconds budget) {
                budgets.push_back(budget);
                const auto index = releases++;
                if (releaseEnablesFeedback) feedback = true;
                const int ret = index < static_cast<int>(releaseResults.size()) ? releaseResults[index] : 0;
                if (ret == -999) throw std::runtime_error("injected release exception");
                return ret;
            },
            [&] { return feedback && elapsed() >= feedbackAfter; },
            [&] { ++enabled; enabledAt = elapsed(); },
            [&](milliseconds duration) { clock += duration; },
            [&] { return clock; },
            [&] { return elapsed() >= cancelAfter; }, log);
    }
    void fails()
    {
        mustFail([&] { run(); }, "Unconfirmed startup was accepted");
        require(enabled == 0, "Failure created the motor command publisher");
    }
};

int main()
{
    try {
        const auto sim = parse({"wbc_fsm"});
        require(!sim.realRobot && sim.domainId() == 1 && sim.networkInterface == "lo",
                "Default startup must be local simulation");
        const auto real = parse({"wbc_fsm", "--real", "--interface", "enp3s0"});
        require(!sim.locoAutoRecovery && !sim.locoRecovery2, "Original task enabled a recovery task");
        const auto combined = parse({"wbc_fsm", "--loco-auto-recovery"});
        require(combined.locoAutoRecovery && !combined.locoRecovery2 && !combined.realRobot,
                "Combined task must be opt-in and default to simulation");
        const auto recovery2 = parse({"wbc_fsm", "--sim", "--loco-recovery2"});
        require(recovery2.locoRecovery2 && !recovery2.locoAutoRecovery && !recovery2.realRobot &&
                recovery2.domainId() == 1,
                "Recovery2 must be independent and default to simulation");
        const auto realRecovery2 = parse({"wbc_fsm", "--loco-recovery2", "--real", "--interface", "enp3s0"});
        require(realRecovery2.locoRecovery2 && realRecovery2.realRobot && realRecovery2.domainId() == 0,
                "Recovery2 changed hardware startup selection");
        require(real.realRobot && real.domainId() == 0 && real.networkInterface == "enp3s0",
                "Real mode/domain/interface disagree");
        const auto simNet = parse({"wbc_fsm", "--sim", "--interface", "enp3s0"});
        require(!simNet.realRobot && simNet.domainId() == 1, "Interface must not implicitly enable hardware");
        for (const auto &args : std::vector<std::vector<std::string>>{
                 {"wbc_fsm", "--real"}, {"wbc_fsm", "--real", "--interface", "lo"},
                 {"wbc_fsm", "--real", "--sim"}, {"wbc_fsm", "--interface"},
                 {"wbc_fsm", "--interface", "--real"}, {"wbc_fsm", "--reel"},
                 {"wbc_fsm", "--loco-auto-recovery", "--loco-auto-recovery"},
                 {"wbc_fsm", "--loco-recovery2", "--loco-recovery2"},
                 {"wbc_fsm", "--loco-auto-recovery", "--loco-recovery2"},
                 {"wbc_fsm", "--loco-recovery2", "--loco-auto-recovery"},
                 {"wbc_fsm", "--loco-recovery2", "--real"}}) {
            bool rejected = false;
            try { parse(args); } catch (const std::invalid_argument &) { rejected = true; }
            require(rejected, "Invalid startup arguments accepted");
        }

        Handoff simHandoff;
        simHandoff.feedbackAfter = milliseconds(100);
        simHandoff.run(false);
        require(simHandoff.checks == 0 && simHandoff.releases == 0 && simHandoff.enabled == 1,
                "Simulation must wait for feedback without any RPCs");
        Handoff simNoState;
        simNoState.feedback = false;
        mustFail([&] { simNoState.run(false); }, "Simulation without feedback started");
        require(simNoState.enabled == 0 && simNoState.elapsed() == seconds(5), "Simulation wait is unbounded");

        Handoff idle;
        idle.run();
        require(idle.checks == 4 && idle.releases == 0 && idle.enabledAt == seconds(2),
                "Need three stable empty queries plus a pre-publication check");

        Handoff delayed;
        delayed.replies.assign(10, {0, "normal"});
        delayed.run();
        require(delayed.releases == 10 && delayed.enabledAt == seconds(12), "Slow service shutdown not retried");

        Handoff transient;
        transient.replies = {{3102, ""}, {-999, ""}, {0, "normal"}, {0, "normal"}, {0, "normal"}};
        transient.releaseResults = {7002, -999, 0};
        transient.run();
        require(transient.enabled == 1 && transient.releases == 3, "Transient query/busy/release errors not retried");

        Handoff noStateBeforeRelease;
        noStateBeforeRelease.feedback = false;
        noStateBeforeRelease.releaseEnablesFeedback = true;
        noStateBeforeRelease.replies = {{0, "normal"}};
        noStateBeforeRelease.run();
        require(noStateBeforeRelease.enabled == 1 && noStateBeforeRelease.releases == 1,
                "Feedback was incorrectly required before ReleaseMode");

        Handoff flap;
        flap.replies = {{0, ""}, {0, ""}, {0, ""}, {0, "normal"}, {0, "normal"}};
        flap.run();
        require(flap.enabledAt == seconds(6) && flap.releases == 1,
                "Service reappearing at pre-publication check was accepted");

        Handoff interruptedConfirmation;
        interruptedConfirmation.replies = {{0, ""}, {3102, ""}};
        interruptedConfirmation.run();
        require(interruptedConfirmation.enabledAt == seconds(4), "Failed query did not reset stable confirmation");

        Handoff queryFailure;
        queryFailure.fallback = {3102, ""};
        queryFailure.fails();
        require(queryFailure.releases == 0 && queryFailure.elapsed() == seconds(60),
                "Failed queries with empty names must time out without release/publication");
        Handoff active;
        active.fallback = {0, "normal"};
        active.fails();
        require(active.releases > 0 && active.elapsed() == seconds(60), "Release success bypassed ownership confirmation");
        Handoff missingState;
        missingState.feedback = false;
        missingState.fails();
        Handoff staleAtFinal;
        staleAtFinal.invalidateAtFinal = true;
        staleAtFinal.fails();

        Handoff cancelled;
        cancelled.cancelAfter = milliseconds(0);
        cancelled.fails();
        require(cancelled.checks == 0, "Cancellation still contacted motion service");
        Handoff cancelDuringWait;
        cancelDuringWait.cancelAfter = seconds(1);
        cancelDuringWait.fails();
        Handoff lateReply;
        lateReply.rpcCost = seconds(61);
        lateReply.fails();
        Handoff bounded;
        bounded.rpcCost = milliseconds(4200);
        bounded.fallback = {3102, ""};
        bounded.fails();
        require(bounded.budgets.back() < seconds(5), "RPC timeout did not shrink to the remaining deadline");
        std::cout << "PASS: BeiChe-style handoff ordering, retries, stable ownership, feedback, deadline and cancellation (no DDS)\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
