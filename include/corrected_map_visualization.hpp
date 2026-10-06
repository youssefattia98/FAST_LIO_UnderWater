#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_set>

namespace uwfl2
{

class CorrectedMapDisplayVoxels
{
public:
    explicit CorrectedMapDisplayVoxels(double voxel_size_m = 0.03)
        : inverse_voxel_size_(
              1.0 / ((std::isfinite(voxel_size_m) && voxel_size_m > 0.0)
                         ? voxel_size_m
                         : 0.03))
    {
    }

    bool insert(double x, double y, double z)
    {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
            return false;
        }
        return occupied_.insert(
            {index(x), index(y), index(z)}).second;
    }

    void clear() { occupied_.clear(); }
    std::size_t size() const { return occupied_.size(); }

private:
    struct Key
    {
        std::int64_t x;
        std::int64_t y;
        std::int64_t z;

        bool operator==(const Key &other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct KeyHash
    {
        std::size_t operator()(const Key &key) const
        {
            std::size_t seed = std::hash<std::int64_t>{}(key.x);
            seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U +
                    (seed << 6U) + (seed >> 2U);
            seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U +
                    (seed << 6U) + (seed >> 2U);
            return seed;
        }
    };

    std::int64_t index(double coordinate) const
    {
        return static_cast<std::int64_t>(
            std::floor(coordinate * inverse_voxel_size_));
    }

    double inverse_voxel_size_;
    std::unordered_set<Key, KeyHash> occupied_;
};

}  // namespace uwfl2
