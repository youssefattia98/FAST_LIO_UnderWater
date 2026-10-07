#pragma once

#include <filesystem>

#include "loop_closure/loop_closure_manager.hpp"

namespace uwfl2::loop_closure
{

LoopClosureConfig load_config_profile(const std::filesystem::path &path);

}  // namespace uwfl2::loop_closure
