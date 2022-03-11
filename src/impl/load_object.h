#ifndef LOAD_OBJECT_H
#define LOAD_OBJECT_H

#include <cstdio>
#include <memory>
#include <string>
#include <dlfcn.h>
#include <link.h>

template <typename Fun>
struct is_function_pointer
    : std::integral_constant<bool, std::is_pointer<Fun>::value
                                       && std::is_function<
                                              typename std::remove_pointer<Fun>::type
                                              >::value>
{
};

template <typename T>
static inline std::string GetSymbolPath(T fptr)
{
    static_assert(is_function_pointer<T>::value,
                  "GetSymbolPath only accepts function pointers");

    Dl_info dl_info;
    dladdr(reinterpret_cast<void *>(fptr),
           &dl_info);

    return std::string(dl_info.dli_fname);
}

inline std::string GetLibPath(std::string lib_name)
{
    /* Try to open the provided library */
    std::shared_ptr<void> lib_handle(dlopen(lib_name.c_str(), RTLD_NOW),
                                     [](void * ptr)
                                     {
                                         dlclose(ptr);
                                     });

    /* Not able to open so, report error */
    if(!lib_handle)
    {
        throw std::runtime_error("lib not found");
    }
    else
    {
        struct link_map* map;

        dlinfo(lib_handle.get(), RTLD_DI_LINKMAP, &map);

        if(!map)
        {
//            XBOT2_DEBUG("unable to extract lib path for '{}' \n",
//                        lib_name);
            return "";
        }

        std::shared_ptr<char> real(realpath(map->l_name, nullptr),
                                   [](char * ptr)
                                   {
                                        std::free(ptr);
                                   });

        if(!real)
        {
//            XBOT2_DEBUG("unable to extract real path for '{}' \n",
//                        map->l_name);
            return "";
        }

        std::string ret = real.get();

        return ret;

    }
}


template <typename RetType, typename... Args>
RetType CallFunction(std::string lib_name,
                     std::string function_name,
                     Args&&... args)
{

    void * lib_handle = nullptr;

    /* Try to open the provided library (if any) */
    if(!lib_name.empty())
    {
        lib_handle = dlopen(lib_name.c_str(), RTLD_NOW);
    }
    else
    {
        // otherwise, search the symbol among the already loaded
        // libs
        lib_handle = RTLD_DEFAULT;
    }

    /* Not able to open so, report error */
    if(!lib_name.empty() && !lib_handle)
    {
        throw std::runtime_error("lib '" + lib_name + "' not found");
    }

    /* Typedef for the factory type */
    typedef RetType (*FactoryType)(Args&&... args);

    /* Try to obtain the address of the factory */
    dlerror();  // clear errors
    FactoryType function = reinterpret_cast<FactoryType>(dlsym(lib_handle,
                                                               function_name.c_str())
                                                         );

    const char * error = dlerror();
    if(error != nullptr)
    {
        throw std::runtime_error("symbol '" + function_name + "' not found in " + lib_name);
    }

    return function(std::forward<Args>(args)...);

}

#endif // LOAD_OBJECT_H
