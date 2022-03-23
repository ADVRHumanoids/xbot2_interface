#ifndef UTILS_H
#define UTILS_H

#include <xbot2_interface/common/types.h>
#include <string>

namespace XBot {

template <typename T1, typename T2>
inline void check_and_set(const T1& from, T2& to, string_const_ref name)
{
    if(from.size() == to.size())
    {
        to = from;
        return;
    }

    throw std::out_of_range("size mismatch: " + name);
}

template <typename T1, typename T2>
inline void check_mat_size(const T1& from, T2& to, string_const_ref name)
{
    if(from.rows() == to.rows() &&
            from.cols() == to.cols())
    {
        return;
    }

    throw std::out_of_range("mat size mismatch: " + name);
}

template <typename T1>
inline void check_mat_size(const T1& mat, int rows, int cols, string_const_ref name)
{
    if(mat.rows() == rows &&
            mat.cols() == cols)
    {
        return;
    }

    throw std::out_of_range("mat size mismatch: " + name);
}

}

#endif // UTILS_H
