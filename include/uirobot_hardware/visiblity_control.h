#ifndef UIROBOT_HARDWARE__VISIBLITY_CONTROL_H_
#define UIROBOT_HARDWARE__VISIBLITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define UIROBOT_HARDWARE_EXPORT __attribute__((dllexport))
#define UIROBOT_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define UIROBOT_HARDWARE_EXPORT __declspec(dllexport)
#define UIROBOT_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef UIROBOT_HARDWARE_BUILDING_DLL
#define UIROBOT_HARDWARE_PUBLIC UIROBOT_HARDWARE_EXPORT
#else
#define UIROBOT_HARDWARE_PUBLIC UIROBOT_HARDWARE_IMPORT
#endif
#define UIROBOT_HARDWARE_PUBLIC_TYPE UIROBOT_HARDWARE_PUBLIC
#define UIROBOT_HARDWARE_LOCAL
#else
#define UIROBOT_HARDWARE_EXPORT __attribute__((visibility("default")))
#define UIROBOT_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define UIROBOT_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define UIROBOT_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define UIROBOT_HARDWARE_PUBLIC
#define UIROBOT_HARDWARE_LOCAL
#endif
#define UIROBOT_HARDWARE_PUBLIC_TYPE
#endif

#endif  // UIROBOT_HARDWARE__VISIBLITY_CONTROL_H_
