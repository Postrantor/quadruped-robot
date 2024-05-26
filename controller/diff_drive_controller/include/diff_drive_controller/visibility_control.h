/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef DIFF_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_
#define DIFF_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DIFF_DRIVE_CONTROLLER_EXPORT __attribute__((dllexport))
#define DIFF_DRIVE_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define DIFF_DRIVE_CONTROLLER_EXPORT __declspec(dllexport)
#define DIFF_DRIVE_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef DIFF_DRIVE_CONTROLLER_BUILDING_DLL
#define DIFF_DRIVE_CONTROLLER_PUBLIC DIFF_DRIVE_CONTROLLER_EXPORT
#else
#define DIFF_DRIVE_CONTROLLER_PUBLIC DIFF_DRIVE_CONTROLLER_IMPORT
#endif
#define DIFF_DRIVE_CONTROLLER_PUBLIC_TYPE DIFF_DRIVE_CONTROLLER_PUBLIC
#define DIFF_DRIVE_CONTROLLER_LOCAL
#else
#define DIFF_DRIVE_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define DIFF_DRIVE_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define DIFF_DRIVE_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define DIFF_DRIVE_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define DIFF_DRIVE_CONTROLLER_PUBLIC
#define DIFF_DRIVE_CONTROLLER_LOCAL
#endif
#define DIFF_DRIVE_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // DIFF_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_
