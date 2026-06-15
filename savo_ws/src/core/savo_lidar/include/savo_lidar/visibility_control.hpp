#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SAVO_LIDAR_EXPORT __attribute__((dllexport))
    #define SAVO_LIDAR_IMPORT __attribute__((dllimport))
  #else
    #define SAVO_LIDAR_EXPORT __declspec(dllexport)
    #define SAVO_LIDAR_IMPORT __declspec(dllimport)
  #endif

  #ifdef SAVO_LIDAR_BUILDING_LIBRARY
    #define SAVO_LIDAR_PUBLIC SAVO_LIDAR_EXPORT
  #else
    #define SAVO_LIDAR_PUBLIC SAVO_LIDAR_IMPORT
  #endif

  #define SAVO_LIDAR_PUBLIC_TYPE SAVO_LIDAR_PUBLIC
  #define SAVO_LIDAR_LOCAL
#else
  #define SAVO_LIDAR_EXPORT __attribute__((visibility("default")))
  #define SAVO_LIDAR_IMPORT

  #if __GNUC__ >= 4
    #define SAVO_LIDAR_PUBLIC __attribute__((visibility("default")))
    #define SAVO_LIDAR_LOCAL __attribute__((visibility("hidden")))
  #else
    #define SAVO_LIDAR_PUBLIC
    #define SAVO_LIDAR_LOCAL
  #endif

  #define SAVO_LIDAR_PUBLIC_TYPE
#endif