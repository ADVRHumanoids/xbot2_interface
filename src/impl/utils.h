#ifndef UTILS_H
#define UTILS_H

#include <xbot2_interface/common/types.h>
#include <string>

namespace XBot {

template <typename T1, typename T2>
inline void check_and_set(const T1& from, T2& to, std::string_view name)
{
    if(from.size() == to.size())
    {
        to = from;
        return;
    }

    throw std::out_of_range("size mismatch: ");
}

}

#endif // UTILS_H
