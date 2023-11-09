#ifndef OMNI_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_
#define OMNI_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define OMNI_DRIVE_CONTROLLER_EXPORT __attribute__((dllexport))
#define OMNI_DRIVE_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define OMNI_DRIVE_CONTROLLER_EXPORT __declspec(dllexport)
#define OMNI_DRIVE_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef OMNI_DRIVE_CONTROLLER_BUILDING_DLL
#define OMNI_DRIVE_CONTROLLER_PUBLIC OMNI_DRIVE_CONTROLLER_EXPORT
#else
#define OMNI_DRIVE_CONTROLLER_PUBLIC OMNI_DRIVE_CONTROLLER_IMPORT
#endif
#define OMNI_DRIVE_CONTROLLER_PUBLIC_TYPE OMNI_DRIVE_CONTROLLER_PUBLIC
#define OMNI_DRIVE_CONTROLLER_LOCAL
#else
#define OMNI_DRIVE_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define OMNI_DRIVE_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define OMNI_DRIVE_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define OMNI_DRIVE_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define OMNI_DRIVE_CONTROLLER_PUBLIC
#define OMNI_DRIVE_CONTROLLER_LOCAL
#endif
#define OMNI_DRIVE_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // OMNI_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_