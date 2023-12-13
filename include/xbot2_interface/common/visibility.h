#ifndef XBOT2IFC_VISIBILITY_H
#define XBOT2IFC_VISIBILITY_H

// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
#define XBOT2IFC_HELPER_DLL_IMPORT __declspec(dllimport)
#define XBOT2IFC_HELPER_DLL_EXPORT __declspec(dllexport)
#define XBOT2IFC_HELPER_DLL_LOCAL
#else
#if __GNUC__ >= 4
#define XBOT2IFC_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
#define XBOT2IFC_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
#define XBOT2IFC_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
#else
#define XBOT2IFC_HELPER_DLL_IMPORT
#define XBOT2IFC_HELPER_DLL_EXPORT
#define XBOT2IFC_HELPER_DLL_LOCAL
#endif
#endif

// Now we use the generic helper definitions above to define XBOT2IFC_API and XBOT2IFC_LOCAL.
// XBOT2IFC_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// XBOT2IFC_LOCAL is used for non-api symbols.

#ifdef XBOT2IFC_DLL // defined if XBOT2IFC is compiled as a DLL
#ifdef XBOT2IFC_DLL_EXPORTS // defined if we are building the XBOT2IFC DLL (instead of using it)
#define XBOT2IFC_API XBOT2IFC_HELPER_DLL_EXPORT
#else
#define XBOT2IFC_API XBOT2IFC_HELPER_DLL_IMPORT
#endif // XBOT2IFC_DLL_EXPORTS
#define XBOT2IFC_LOCAL XBOT2IFC_HELPER_DLL_LOCAL
#else // XBOT2IFC_DLL is not defined: this means XBOT2IFC is a static lib.
#define XBOT2IFC_API
#define XBOT2IFC_LOCAL
#endif // XBOT2IFC_DLL

#ifndef XBOT2IFC_API
#define XBOT2IFC_API
#endif

#ifndef XBOT2IFC_LOCAL
#define XBOT2IFC_LOCAL
#endif

#endif // VISIBILITY_H
