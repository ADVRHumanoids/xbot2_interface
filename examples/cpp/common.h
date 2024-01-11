#ifndef COMMON_H
#define COMMON_H

#include <xbot2_interface/xbotinterface2.h>
#include <fmt/format.h>
#include <cmrc/cmrc.hpp>

CMRC_DECLARE(example_resources);

auto resources = cmrc::example_resources::get_filesystem();

#endif // COMMON_H
