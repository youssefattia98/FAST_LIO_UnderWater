#include "loop_closure/shadow_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

#include <pcl/kdtree/kdtree_flann.h>

namespace uwfl2::loop_closure
{
namespace
{

using Clock = std::chrono::steady_clock;

struct VoxelKey
{
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const VoxelKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator<(const VoxelKey &other) const
    {
        return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
    }
};

struct VoxelKeyHash
{
    std::size_t operator()(const VoxelKey &key) const noexcept
    {
        std::size_t result = std::hash<std::int64_t>{}(key.x);
        result ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9 +
                  (result << 6U) + (result >> 2U);
        result ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9 +
                  (result << 6U) + (result >> 2U);
        return result;
    }
};

struct VoxelCandidate
{
    ShadowPoint point;
    double center_distance_squared = std::numeric_limits<double>::infinity();
};

using VoxelMap =
    std::unordered_map<VoxelKey, VoxelCandidate, VoxelKeyHash>;

double milliseconds(Clock::time_point begin, Clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

std::vector<std::size_t> select_keyframes(
    const ShadowMapRequest &request,
    const ShadowMapConfig &config)
{
    const auto count = request.graph.optimized_poses.size();
    if (count == 0 || request.graph.keyframes.size() != count)
    {
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr positions(
        new pcl::PointCloud<pcl::PointXYZ>());
    positions->reserve(count);
    for (const Pose3d &pose : request.graph.optimized_poses)
    {
        positions->emplace_back(static_cast<float>(pose.translation.x()),
                                static_cast<float>(pose.translation.y()),
                                static_cast<float>(pose.translation.z()));
    }

    std::vector<int> indices;
    std::vector<float> distances;
    if (config.radius_m > 0.0)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> index;
        index.setInputCloud(positions);
        const auto &center = request.graph.optimized_poses.back().translation;
        pcl::PointXYZ query(static_cast<float>(center.x()),
                            static_cast<float>(center.y()),
                            static_cast<float>(center.z()));
        index.radiusSearch(query, config.radius_m, indices, distances);
    }
    else
    {
        indices.resize(count);
        distances.resize(count);
        for (std::size_t index = 0; index < count; ++index)
        {
            indices[index] = static_cast<int>(index);
            distances[index] = 0.0F;
        }
    }

    std::vector<std::pair<float, std::size_t>> ranked;
    ranked.reserve(indices.size());
    for (std::size_t index = 0; index < indices.size(); ++index)
    {
        ranked.emplace_back(distances[index],
                            static_cast<std::size_t>(indices[index]));
    }
    std::sort(ranked.begin(), ranked.end(), [](const auto &lhs, const auto &rhs) {
        return std::tie(lhs.first, lhs.second) < std::tie(rhs.first, rhs.second);
    });
    if (ranked.size() > config.maximum_keyframes)
    {
        ranked.resize(config.maximum_keyframes);
    }
    std::vector<std::size_t> selected;
    selected.reserve(ranked.size());
    for (const auto &entry : ranked)
    {
        selected.push_back(entry.second);
    }
    std::sort(selected.begin(), selected.end());
    return selected;
}

bool lexicographically_less(const ShadowPoint &lhs, const ShadowPoint &rhs)
{
    return std::tie(lhs.x, lhs.y, lhs.z, lhs.intensity) <
           std::tie(rhs.x, rhs.y, rhs.z, rhs.intensity);
}

const std::vector<PointXYZI> &map_source(const Keyframe &keyframe)
{
    if (keyframe.map_points_world && !keyframe.map_points_world->empty())
    {
        return *keyframe.map_points_world;
    }
    if (!keyframe.sonar_points)
    {
        throw std::runtime_error("Keyframe has no immutable map cloud");
    }
    return *keyframe.sonar_points;
}

ShadowPoint corrected_point(const ShadowMapRequest &request,
                            std::size_t index,
                            const PointXYZI &source)
{
    const Keyframe &keyframe = request.graph.keyframes[index];
    Eigen::Vector3d world;
    if (keyframe.map_points_world && !keyframe.map_points_world->empty())
    {
        const Pose3d correction = compose(
            request.graph.optimized_poses[index],
            inverse(request.graph.raw_poses[index]));
        world = correction.rotation *
                    Eigen::Vector3d(source.x, source.y, source.z) +
                correction.translation;
    }
    else
    {
        const Pose3d T_local_sonar = compose(
            request.graph.optimized_poses[index], keyframe.T_vehicle_sonar);
        world = T_local_sonar.rotation *
                    Eigen::Vector3d(source.x, source.y, source.z) +
                T_local_sonar.translation;
    }

    ShadowPoint point;
    point.x = static_cast<float>(world.x());
    point.y = static_cast<float>(world.y());
    point.z = static_cast<float>(world.z());
    point.intensity = source.intensity;
    point.normal_x = 0.0F;
    point.normal_y = 0.0F;
    point.normal_z = 0.0F;
    point.curvature = 0.0F;
    return point;
}

std::size_t count_input_points(const ShadowMapRequest &request,
                               const std::vector<std::size_t> &indices,
                               std::size_t maximum_input_points)
{
    std::size_t total_points = 0;
    for (const std::size_t index : indices)
    {
        const auto &points = map_source(request.graph.keyframes[index]);
        if (total_points > maximum_input_points ||
            points.size() > maximum_input_points - total_points)
        {
            throw std::runtime_error("Shadow-map input point budget exceeded");
        }
        total_points += points.size();
    }
    return total_points;
}

void accumulate_voxel(VoxelMap &voxels, const ShadowPoint &point,
                      double voxel_size_m)
{
    const double inverse_leaf = 1.0 / voxel_size_m;
    const Eigen::Vector3d world(point.x, point.y, point.z);
    const VoxelKey key{
        static_cast<std::int64_t>(std::floor(world.x() * inverse_leaf)),
        static_cast<std::int64_t>(std::floor(world.y() * inverse_leaf)),
        static_cast<std::int64_t>(std::floor(world.z() * inverse_leaf))};
    const Eigen::Vector3d center(
        (static_cast<double>(key.x) + 0.5) * voxel_size_m,
        (static_cast<double>(key.y) + 0.5) * voxel_size_m,
        (static_cast<double>(key.z) + 0.5) * voxel_size_m);
    const double center_distance_squared = (world - center).squaredNorm();
    auto [iterator, inserted] = voxels.try_emplace(
        key, VoxelCandidate{point, center_distance_squared});
    if (!inserted &&
        (center_distance_squared <
             iterator->second.center_distance_squared - 1e-15 ||
         (std::abs(center_distance_squared -
                   iterator->second.center_distance_squared) <= 1e-15 &&
          lexicographically_less(point, iterator->second.point))))
    {
        iterator->second = {point, center_distance_squared};
    }
}

ShadowPointVector ordered_voxel_points(const VoxelMap &voxels)
{
    std::vector<std::pair<VoxelKey, ShadowPoint>> ordered;
    ordered.reserve(voxels.size());
    for (const auto &entry : voxels)
    {
        ordered.emplace_back(entry.first, entry.second.point);
    }
    std::sort(ordered.begin(), ordered.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.first < rhs.first;
    });
    ShadowPointVector filtered;
    filtered.reserve(ordered.size());
    for (const auto &entry : ordered)
    {
        filtered.push_back(entry.second);
    }
    return filtered;
}

bool selects_full_history(const std::vector<std::size_t> &selected,
                          std::size_t keyframe_count)
{
    if (selected.size() != keyframe_count)
    {
        return false;
    }
    for (std::size_t index = 0; index < selected.size(); ++index)
    {
        if (selected[index] != index)
        {
            return false;
        }
    }
    return true;
}

}  // namespace

ShadowMapBuilder::ShadowMapBuilder(ShadowMapConfig config) : config_(config)
{
    if (!std::isfinite(config_.radius_m) || config_.radius_m < 0.0 ||
        !std::isfinite(config_.voxel_size_m) || config_.voxel_size_m <= 0.0 ||
        config_.maximum_keyframes == 0 || config_.maximum_input_points == 0)
    {
        throw std::invalid_argument("Invalid shadow-map configuration");
    }
}

ShadowPointVector ShadowMapBuilder::reconstruct_and_downsample(
    const ShadowMapRequest &request,
    const ShadowMapConfig &config,
    std::size_t *selected_keyframe_count,
    std::size_t *input_point_count,
    double *reconstruction_time_ms,
    double *downsample_time_ms)
{
    const auto reconstruction_started = Clock::now();
    const std::vector<std::size_t> selected = select_keyframes(request, config);
    if (selected_keyframe_count)
    {
        *selected_keyframe_count = selected.size();
    }

    const std::size_t total_points = count_input_points(
        request, selected, config.maximum_input_points);
    if (input_point_count)
    {
        *input_point_count = total_points;
    }

    VoxelMap voxels;
    voxels.reserve(total_points);
    for (const std::size_t index : selected)
    {
        for (const PointXYZI &source : map_source(request.graph.keyframes[index]))
        {
            const ShadowPoint point = corrected_point(request, index, source);
            if (std::isfinite(point.x) && std::isfinite(point.y) &&
                std::isfinite(point.z))
            {
                accumulate_voxel(voxels, point, config.voxel_size_m);
            }
        }
    }
    const auto reconstruction_finished = Clock::now();
    ShadowPointVector filtered = ordered_voxel_points(voxels);
    const auto downsample_finished = Clock::now();
    if (reconstruction_time_ms)
    {
        *reconstruction_time_ms =
            milliseconds(reconstruction_started, reconstruction_finished);
    }
    if (downsample_time_ms)
    {
        *downsample_time_ms =
            milliseconds(reconstruction_finished, downsample_finished);
    }
    return filtered;
}

ShadowMapResult ShadowMapBuilder::build(const ShadowMapRequest &request) const
{
    const auto build_started = Clock::now();
    ShadowMapResult result;
    result.graph_version = request.graph.version;
    result.graph_loop_factor_count = request.graph.loop_factor_count;
    result.source_tree_generation = request.source_tree_generation;
    result.shadow_tree_generation = request.graph.version;
    try
    {
        const auto reconstruction_started = Clock::now();
        const std::vector<std::size_t> selected =
            select_keyframes(request, config_);
        result.selected_keyframes = selected.size();
        if (selected.empty())
        {
            result.reason = "corrected_map_empty";
            result.total_time_ms = milliseconds(build_started, Clock::now());
            return result;
        }
        result.input_points = count_input_points(
            request, selected, config_.maximum_input_points);
        std::vector<std::size_t> full_indices(request.graph.keyframes.size());
        for (std::size_t index = 0; index < full_indices.size(); ++index)
        {
            full_indices[index] = index;
        }
        const std::size_t full_input_points = count_input_points(
            request, full_indices, config_.maximum_input_points);
        const bool active_is_full =
            selects_full_history(selected, full_indices.size());
        std::vector<bool> active_keyframe(full_indices.size(), false);
        for (const std::size_t index : selected)
        {
            active_keyframe[index] = true;
        }

        VoxelMap history_voxels;
        history_voxels.reserve(full_input_points);
        VoxelMap active_voxels;
        if (!active_is_full)
        {
            active_voxels.reserve(result.input_points);
        }
        for (const std::size_t index : full_indices)
        {
            for (const PointXYZI &source :
                 map_source(request.graph.keyframes[index]))
            {
                const ShadowPoint point = corrected_point(request, index, source);
                if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                    !std::isfinite(point.z))
                {
                    continue;
                }
                accumulate_voxel(history_voxels, point, config_.voxel_size_m);
                if (!active_is_full && active_keyframe[index])
                {
                    accumulate_voxel(active_voxels, point,
                                     config_.voxel_size_m);
                }
            }
        }
        const auto reconstruction_finished = Clock::now();
        ShadowPointVector corrected_history =
            ordered_voxel_points(history_voxels);
        ShadowPointVector filtered = active_is_full
                                         ? corrected_history
                                         : ordered_voxel_points(active_voxels);
        const auto downsample_finished = Clock::now();
        result.reconstruction_time_ms = milliseconds(
            reconstruction_started, reconstruction_finished);
        result.downsample_time_ms = milliseconds(
            reconstruction_finished, downsample_finished);
        result.corrected_history_points =
            std::make_shared<const ShadowPointVector>(
                std::move(corrected_history));
        if (filtered.empty())
        {
            result.reason = "corrected_map_empty";
            result.total_time_ms = milliseconds(build_started, Clock::now());
            return result;
        }
        const auto tree_started = Clock::now();
        auto tree = std::make_shared<ShadowTree>();
        tree->set_downsample_param(static_cast<float>(config_.voxel_size_m));
        tree->Build(filtered);
        result.tree_build_time_ms = milliseconds(tree_started, Clock::now());
        result.filtered_points = static_cast<std::size_t>(tree->validnum());
        result.estimated_tree_bytes =
            result.filtered_points * sizeof(typename ShadowTree::KD_TREE_NODE);
        result.tree = std::move(tree);
        result.valid = result.tree->Root_Node != nullptr &&
                       result.filtered_points > 0;
        result.reason = result.valid ? "ready" : "shadow_tree_empty";
    }
    catch (const std::exception &exception)
    {
        result.reason = exception.what();
    }
    result.total_time_ms = milliseconds(build_started, Clock::now());
    return result;
}

bool shadow_result_matches_graph_version(const ShadowMapResult &result,
                                         std::uint64_t graph_version)
{
    return result.valid && result.graph_version == graph_version;
}

bool shadow_result_matches_graph(const ShadowMapResult &result,
                                 const PoseGraphSnapshot &graph)
{
    return result.valid && result.graph_version <= graph.version &&
           result.graph_loop_factor_count == graph.loop_factor_count;
}

}  // namespace uwfl2::loop_closure
