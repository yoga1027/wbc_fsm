#pragma once

#include <stdexcept>
#include <string>

struct StartupOptions
{
    bool realRobot = false;
    bool help = false;
    std::string networkInterface = "lo";
    unsigned domainId() const { return realRobot ? 0 : 1; }

    static StartupOptions parse(int argc, char **argv)
    {
        StartupOptions result;
        bool modeSpecified = false;
        bool interfaceSpecified = false;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                result.help = true;
            } else if (arg == "--sim" || arg == "--real") {
                if (modeSpecified) throw std::invalid_argument("Specify only one of --sim / --real");
                modeSpecified = true;
                result.realRobot = arg == "--real";
            } else if (arg == "--interface") {
                if (interfaceSpecified || i + 1 == argc)
                    throw std::invalid_argument("--interface requires one network interface name");
                interfaceSpecified = true;
                result.networkInterface = argv[++i];
                if (result.networkInterface.empty() || result.networkInterface[0] == '-')
                    throw std::invalid_argument("Invalid network interface name");
            } else {
                throw std::invalid_argument("Unknown argument: " + arg);
            }
        }
        if (!result.help && result.realRobot &&
            (!interfaceSpecified || result.networkInterface == "lo"))
            throw std::invalid_argument("Real robot requires --real --interface <robot-network-interface>, not lo");
        return result;
    }
};
