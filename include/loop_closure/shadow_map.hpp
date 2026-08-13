#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_types.h>

#include "ikd-Tree/ikd_Tree.h"
#include "loop_closure/pose_graph.hpp"

namespace uwfl2::loop_closure
{

using ShadowPoint = pcl::PointXYZINormal;
using ShadowPointVector =
    std::vector<ShadowPoint, Eigen::aligned_allocator<ShadowPoint>>;
using ShadowTree = KD_TREE<ShadowPoint>;

struct ShadowMapConfig
{
    double radius_m = 80.0;
    double voxel_size_m = 0.3;
    std::size_t maximum_keyframes = 1000;
    std::size_t maximum_input_points = 3000000;
};

struct ShadowMapRequest
{
    PoseGraphSnapshot graph;
    std::uint64_t source_tree_generation = 0;
};

struct ShadowMapResult
{
    std::uint64_t graph_version = 0;
    std::size_t graph_loop_factor_count = 0;
    std::uint64_t source_tree_generation = 0;
    std::uint64_t shadow_tree_generation = 0;
    std::size_t selected_keyframes = 0;
    std::size_t input_points = 0;
    std::size_t filtered_points = 0;
    std::size_t estimated_tree_bytes = 0;
    double reconstruction_time_ms = 0.0;
    double downsample_time_ms = 0.0;
    double tree_build_time_ms = 0.0;
    bool valid = false;
    std::string reason;
    std::shared_ptr<ShadowTree> tree;
    std::shared_ptr<const ShadowPointVector> corrected_history_points;
};

class ShadowMapBuilder
{
public:
    explicit ShadowMapBuilder(ShadowMapConfig config = {});

    ShadowMapResult build(const ShadowMapRequest &request) const;

    static ShadowPointVector reconstruct_and_downsample(
        const ShadowMapRequest &request,
        const ShadowMapConfig &config,
        std::size_t *selected_keyframes = nullptr,
        std::size_t *input_points = nullptr,
        double *reconstruction_time_ms = nullptr,
        double *downsample_time_ms = nullptr);

private:
    ShadowMapConfig config_;
};

bool shadow_result_matches_graph_version(const ShadowMapResult &result,
                                         std::uint64_t graph_version);
bool shadow_result_matches_graph(const ShadowMapResult &result,
                                 const PoseGraphSnapshot &graph);

}  // namespace uwfl2::loop_closure
