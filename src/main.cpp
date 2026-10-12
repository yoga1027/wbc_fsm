#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <iomanip>
#include <memory>
#include <vector>
#include <cstring>
#include <openssl/sha.h>
#include <openssl/rsa.h>
#include <openssl/pem.h>
#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/buffer.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "interface/IOSDK.h"

bool running = true;  
volatile std::sig_atomic_t stopRequested = 0;


void ShutDown(int sig) 
{
    stopRequested = 1;
}

void setProcessScheduler()  // 实时调度设置
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv) {
    try {
        const auto options = StartupOptions::parse(argc, argv);
        if (options.help) {
            std::cout << "Usage:\n"
                      << "  " << argv[0] << " [--sim] [--interface lo]\n"
                      << "  " << argv[0] << " --real --interface <robot-network-interface>\n"
                      << "Default: simulation, DDS domain 1. Real robot: domain 0 with verified motion service release.\n"
                      << "  --loco-auto-recovery: RL Loco+MJAmpRecovery; START then R2+B to enter.\n"
                      << "Startup implementation: handoff-v2 (stable confirmation, bounded RPC retry).\n"
                      << "Recovery: PASSIVE -> R2+X -> release buttons -> R2+Y. START is not required.\n";
            return 0;
        }
        signal(SIGINT, ShutDown);
        signal(SIGTERM, ShutDown);
        setProcessScheduler();
        std::cout << std::fixed << std::setprecision(3);
        char executable[4096]{};
        const auto length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
        std::cout << "[Startup] handoff-v2, built " << __DATE__ << ' ' << __TIME__
                  << ", executable=" << (length > 0 ? executable : argv[0]) << std::endl;

        // Load all models/configs before touching the robot's motion service.
        CtrlComponents ctrlComp(nullptr);
        ctrlComp.locoAutoRecovery = options.locoAutoRecovery;
        ctrlComp.ctrlPlatform = options.realRobot ? CtrlPlatform::REALROBOT : CtrlPlatform::MUJOCO;
        ctrlComp.dt = 0.02;
        ctrlComp.running = &running;
        ControlFrame ctrlFrame(&ctrlComp);
        const LowlevelCmd damping = *ctrlComp.lowCmd; // initialized by PASSIVE::enter
        auto sdk = std::make_unique<IOSDK>(options, [] { return stopRequested != 0; });
        ctrlComp.ioInter = sdk.release(); // owned by CtrlComponents; virtual IO destructor

        int result = 0;
        try {
            while (running && !stopRequested && !ctrlComp.exitFlag) ctrlFrame.run();
        } catch (const std::exception &e) {
            std::cerr << "[Control] " << e.what() << std::endl;
            result = 1;
        }
        // Includes Ctrl+C and SELECT from non-recovery states. Never resume the
        // stock motion service automatically when exiting a fallen-state test.
        *ctrlComp.lowCmd = damping;
        try {
            ctrlComp.sendRecv();
        } catch (const std::exception &e) {
            std::cerr << "[Shutdown] Damping publication failed: " << e.what() << std::endl;
            result = 1;
        }
        return result;
    } catch (const std::exception &e) {
        std::cerr << "[Startup] " << e.what() << std::endl;
        return stopRequested ? 130 : 1;
    }
}
