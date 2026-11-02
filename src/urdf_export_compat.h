#pragma once
#include <memory>
#include <tinyxml2.h>
#include <urdf_model/model.h>

namespace XBot { namespace detail {
std::unique_ptr<tinyxml2::XMLDocument> exportUrdf(const urdf::ModelInterface& model);
}}
