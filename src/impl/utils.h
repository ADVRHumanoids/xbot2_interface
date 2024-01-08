#ifndef IMPL_UTILS_H
#define IMPL_UTILS_H

#include <xbot2_interface/common/types.h>
#include <string>

#include <fmt/core.h>

namespace XBot {

template <typename T1, typename T2>
inline void check_and_set(const T1& from, T2& to, const char *name)
{
    if(from.size() == to.size())
    {
        to = from;
        return;
    }

    throw std::out_of_range(
        fmt::format("size mismatch in {}: {} (actual) != {} (expected)",
                    name, from.size(), to.size())
        );
}

template <typename T1, typename T2>
inline void check_mat_size(const T1& from, T2& to, const char *name)
{
    if(from.rows() == to.rows() &&
            from.cols() == to.cols())
    {
        return;
    }

    throw std::out_of_range(
        fmt::format("size mismatch in {}: {} x {} (actual) != {} x {} (expected)",
                    name,
                    from.rows(), from.cols(),
                    to.rows(), to.cols())
        );
}

template <typename T1>
inline void check_mat_size(const T1& mat, int rows, int cols, const char *name)
{
    if(mat.rows() == rows &&
            mat.cols() == cols)
    {
        return;
    }

    throw std::out_of_range(
        fmt::format("size mismatch in {}: {} x {} (actual) != {} x {} (expected)",
                    name,
                    mat.rows(), mat.cols(),
                    rows, cols)
        );
}

}

#endif // UTILS_H
