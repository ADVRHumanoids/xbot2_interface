#ifndef PLUGIN_H
#define PLUGIN_H

#include "visibility.h"

#define XBOT2IFC_VERSION_MAJOR 3
#define XBOT2IFC_VERSION_MINOR 0
#define XBOT2IFC_VERSION_PATCH 0

#define XBOT2_MODEL_PLUGIN_FACTORY(Name) xbot2_create_model_plugin_##Name
#define XBOT2_MODEL_PLUGIN_GETABI(Name) xbot2_get_model_plugin_abi_version_##Name

#define XBOT2_REGISTER_MODEL_PLUGIN(Class, Type) \
    extern "C" XBOT2_HELPER_DLL_EXPORT \
    XBOT2_API ::XBot::XBotInterface2 * XBOT2_MODEL_PLUGIN_FACTORY(Type)( \
                            const ::XBot::XBotInterface2::ConfigOptions& opt \
                            ) \
    { \
       return new Class(opt); \
    } \
    \
    extern "C" XBOT2_HELPER_DLL_EXPORT \
    void XBOT2_MODEL_PLUGIN_GETABI(Type)(int* major, int* minor, int* patch) \
    { \
        *major = XBOT2IFC_VERSION_MAJOR; \
        *minor = XBOT2IFC_VERSION_MINOR; \
        *patch = XBOT2IFC_VERSION_PATCH; \
    }

#endif // PLUGIN_H
