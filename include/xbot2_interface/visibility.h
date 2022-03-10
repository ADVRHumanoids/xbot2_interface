#ifndef XBOT2_VISIBILITY_H
#define XBOT2_VISIBILITY_H

// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
#define XBOT2_HELPER_DLL_IMPORT __declspec(dllimport)
#define XBOT2_HELPER_DLL_EXPORT __declspec(dllexport)
#define XBOT2_HELPER_DLL_LOCAL
#else
#if __GNUC__ >= 4
#define XBOT2_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
#define XBOT2_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
#define XBOT2_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
#else
#define XBOT2_HELPER_DLL_IMPORT
#define XBOT2_HELPER_DLL_EXPORT
#define XBOT2_HELPER_DLL_LOCAL
#endif
#endif

// Now we use the generic helper definitions above to define XBOT2_API and XBOT2_LOCAL.
// XBOT2_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// XBOT2_LOCAL is used for non-api symbols.

#ifdef XBOT2_DLL // defined if XBOT2 is compiled as a DLL
#ifdef XBOT2_DLL_EXPORTS // defined if we are building the XBOT2 DLL (instead of using it)
#define XBOT2_API XBOT2_HELPER_DLL_EXPORT
#else
#define XBOT2_API XBOT2_HELPER_DLL_IMPORT
#endif // XBOT2_DLL_EXPORTS
#define XBOT2_LOCAL XBOT2_HELPER_DLL_LOCAL
#else // XBOT2_DLL is not defined: this means XBOT2 is a static lib.
#define XBOT2_API
#define XBOT2_LOCAL
#endif // XBOT2_DLL

#ifndef XBOT2_API
#define XBOT2_API
#endif

#ifndef XBOT2_LOCAL
#define XBOT2_LOCAL
#endif

#endif // VISIBILITY_H
