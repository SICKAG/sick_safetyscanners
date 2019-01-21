#ifndef SICK_SAFETYSCANNERS_CONSOLE_CONSOLEWRAPPER_H
#define SICK_SAFETYSCANNERS_CONSOLE_CONSOLEWRAPPER_H

#ifdef NO_ROS_CONSOLE

#include <cstdio>

// Turn ROS-Console commands into no-ops
#define ROS_INFO(...) std::printf(__VA_ARGS__)
#define ROS_WARN(...) std::printf(__VA_ARGS__)
#define ROS_ERROR(...) std::fprintf(stderr, __VA_ARGS__)

#else

#include <ros/console.h>

#endif // NO_ROS_CONSOLE

#endif // SICK_SAFETYSCANNERS_CONSOLE_CONSOLEWRAPPER_H