#include "interface/MotionServiceHandoff.h"
#include "interface/StartupOptions.h"
#include <iostream>
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

int main()
{
    try {
        const auto sim = parse({"wbc_fsm"});
        require(!sim.realRobot && sim.domainId() == 1 && sim.networkInterface == "lo",
                "Default startup must be local simulation");
        const auto real = parse({"wbc_fsm", "--real", "--interface", "enp3s0"});
        require(real.realRobot && real.domainId() == 0 && real.networkInterface == "enp3s0",
                "Real mode/domain/interface disagree");
        const auto simNet = parse({"wbc_fsm", "--sim", "--interface", "enp3s0"});
        require(!simNet.realRobot && simNet.domainId() == 1, "Interface must not implicitly enable hardware");
        for (const auto &args : std::vector<std::vector<std::string>>{
                 {"wbc_fsm", "--real"}, {"wbc_fsm", "--real", "--interface", "lo"},
                 {"wbc_fsm", "--real", "--sim"}, {"wbc_fsm", "--interface"},
                 {"wbc_fsm", "--interface", "--real"}, {"wbc_fsm", "--reel"}}) {
            bool rejected = false;
            try { parse(args); } catch (const std::invalid_argument &) { rejected = true; }
            require(rejected, "Invalid startup arguments accepted");
        }

        std::ostringstream log;
        int checks = 0, releases = 0, waits = 0;
        auto idle = [&](std::string &, std::string &name) { ++checks; name.clear(); return 0; };
        auto release = [&] { ++releases; return 0; };
        auto wait = [&] { ++waits; };
        releaseMotionService(false, idle, release, wait, log);
        require(checks == 0 && releases == 0 && waits == 0, "Simulation contacted motion service");
        releaseMotionService(true, idle, release, wait, log);
        require(checks == 1 && releases == 0, "Already released mode was not accepted");

        checks = releases = waits = 0;
        releaseMotionService(true, [&](std::string &, std::string &name) {
            name = ++checks == 1 ? "normal" : "";
            return 0;
        }, release, wait, log);
        require(checks == 2 && releases == 1 && waits == 1,
                "Release must be followed by a confirming query");

        checks = releases = waits = 0;
        mustFail([&] {
            releaseMotionService(true, [&](std::string &, std::string &) {
                ++checks; return 3102; // name remains empty on RPC failure
            }, release, wait, log);
        }, "Empty name on failed CheckMode was mistaken for successful release");
        require(releases == 0, "Query failure issued ReleaseMode");

        auto active = [&](std::string &, std::string &name) { ++checks; name = "normal"; return 0; };
        mustFail([&] {
            releaseMotionService(true, active, [] { return 42; }, wait, log);
        }, "ReleaseMode error accepted");

        checks = releases = waits = 0;
        mustFail([&] { releaseMotionService(true, active, release, wait, log); },
                 "Successful ReleaseMode without empty service name accepted");
        require(checks == 6 && releases == 5 && waits == 5, "Retry limit/final confirmation wrong");

        checks = releases = waits = 0;
        mustFail([&] {
            releaseMotionService(true, [&](std::string &, std::string &name) {
                name = ++checks == 1 ? "normal" : "";
                return checks == 1 ? 0 : -1;
            }, release, wait, log);
        }, "Failed confirmation after release accepted");

        checks = releases = waits = 0;
        releaseMotionService(true, [&](std::string &, std::string &name) {
            name = ++checks <= 5 ? "normal" : "";
            return 0;
        }, release, wait, log);
        require(checks == 6 && releases == 5, "Last release was not checked");
        std::cout << "PASS: startup modes, RPC failures, release confirmation and bounded retries (no DDS)\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
