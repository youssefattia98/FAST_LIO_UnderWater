#pragma once

#include <cerrno>
#include <filesystem>
#include <fcntl.h>
#include <system_error>
#include <unistd.h>

namespace uwfl2
{

// Reserve the filename exclusively so concurrent writers cannot overwrite a map.
inline std::filesystem::path reserve_map_path(const std::filesystem::path &requested)
{
    const auto base = requested.empty() ? std::filesystem::path("test.pcd") : requested;
    if (!base.parent_path().empty())
        std::filesystem::create_directories(base.parent_path());
    for (std::size_t index = 0; ; ++index)
    {
        const auto candidate = index == 0 ? base : base.parent_path() /
            (base.stem().string() + "(" + std::to_string(index) + ")" + base.extension().string());
        const int fd = ::open(candidate.c_str(), O_CREAT | O_EXCL | O_WRONLY, 0666);
        if (fd >= 0)
        {
            ::close(fd);
            return candidate;
        }
        if (errno != EEXIST)
            throw std::system_error(errno, std::generic_category(), candidate.string());
    }
}

}  // namespace uwfl2
