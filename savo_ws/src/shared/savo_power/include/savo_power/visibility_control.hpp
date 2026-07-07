#ifndef SAVO_POWER__VISIBILITY_CONTROL_HPP_
#define SAVO_POWER__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SAVO_POWER_EXPORT __attribute__ ((dllexport))
    #define SAVO_POWER_IMPORT __attribute__ ((dllimport))
  #else
    #define SAVO_POWER_EXPORT __declspec(dllexport)
    #define SAVO_POWER_IMPORT __declspec(dllimport)
  #endif

  #ifdef SAVO_POWER_BUILDING_LIBRARY
    #define SAVO_POWER_PUBLIC SAVO_POWER_EXPORT
  #else
    #define SAVO_POWER_PUBLIC SAVO_POWER_IMPORT
  #endif

  #define SAVO_POWER_PUBLIC_TYPE SAVO_POWER_PUBLIC
  #define SAVO_POWER_LOCAL
#else
  #define SAVO_POWER_EXPORT __attribute__ ((visibility("default")))
  #define SAVO_POWER_IMPORT

  #if __GNUC__ >= 4
    #define SAVO_POWER_PUBLIC __attribute__ ((visibility("default")))
    #define SAVO_POWER_LOCAL __attribute__ ((visibility("hidden")))
  #else
    #define SAVO_POWER_PUBLIC
    #define SAVO_POWER_LOCAL
  #endif

  #define SAVO_POWER_PUBLIC_TYPE
#endif

#endif  // SAVO_POWER__VISIBILITY_CONTROL_HPP_
