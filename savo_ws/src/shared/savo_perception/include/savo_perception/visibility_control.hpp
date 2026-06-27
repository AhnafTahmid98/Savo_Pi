#ifndef SAVO_PERCEPTION__VISIBILITY_CONTROL_HPP_
#define SAVO_PERCEPTION__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SAVO_PERCEPTION_EXPORT __attribute__((dllexport))
    #define SAVO_PERCEPTION_IMPORT __attribute__((dllimport))
  #else
    #define SAVO_PERCEPTION_EXPORT __declspec(dllexport)
    #define SAVO_PERCEPTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef SAVO_PERCEPTION_BUILDING_DLL
    #define SAVO_PERCEPTION_PUBLIC SAVO_PERCEPTION_EXPORT
  #else
    #define SAVO_PERCEPTION_PUBLIC SAVO_PERCEPTION_IMPORT
  #endif
  #define SAVO_PERCEPTION_PUBLIC_TYPE SAVO_PERCEPTION_PUBLIC
  #define SAVO_PERCEPTION_LOCAL
#else
  #define SAVO_PERCEPTION_EXPORT __attribute__((visibility("default")))
  #define SAVO_PERCEPTION_IMPORT
  #if __GNUC__ >= 4
    #define SAVO_PERCEPTION_PUBLIC __attribute__((visibility("default")))
    #define SAVO_PERCEPTION_LOCAL __attribute__((visibility("hidden")))
  #else
    #define SAVO_PERCEPTION_PUBLIC
    #define SAVO_PERCEPTION_LOCAL
  #endif
  #define SAVO_PERCEPTION_PUBLIC_TYPE
#endif

#endif  // SAVO_PERCEPTION__VISIBILITY_CONTROL_HPP_