#include "loop_closure/std_detector.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <pcl/kdtree/kdtree_flann.h>

namespace uwfl2::loop_closure
{
namespace
{

using Clock = std::chrono::steady_clock;

struct VoxelKey
{
    std::int64_t x, y, z;
    bool operator<(const VoxelKey &other) const
    {
        return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
    }
};

struct Keypoint
{
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    std::uint64_t binary = 0;
};

struct Triangle
{
    std::array<Eigen::Vector3d, 3> vertices;
    std::array<Eigen::Vector3d, 3> normals;
    std::array<std::uint64_t, 3> binary{};
    Eigen::Vector3d sides = Eigen::Vector3d::Zero();
};

struct TriangleKey
{
    int a = 0, b = 0, c = 0;
    bool operator==(const TriangleKey &other) const
    {
        return a == other.a && b == other.b && c == other.c;
    }
    bool operator<(const TriangleKey &other) const
    {
        return std::tie(a, b, c) < std::tie(other.a, other.b, other.c);
    }
};

struct TriangleKeyHash
{
    std::size_t operator()(const TriangleKey &key) const
    {
        std::size_t seed = std::hash<int>{}(key.a);
        seed ^= std::hash<int>{}(key.b) + 0x9e3779b9U + (seed << 6U) +
                (seed >> 2U);
        seed ^= std::hash<int>{}(key.c) + 0x9e3779b9U + (seed << 6U) +
                (seed >> 2U);
        return seed;
    }
};

struct TriangleMatch
{
    const Triangle *current = nullptr;
    const Triangle *historical = nullptr;
};

struct Candidate
{
    std::size_t entry_index = 0;
    double pose_distance_m = 0.0;
    std::vector<TriangleMatch> matches;
};

double elapsed_ms(Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

std::uint64_t binary_signature(const Eigen::Vector3d &center,
                               const std::vector<Eigen::Vector3d> &points,
                               double voxel_size)
{
    std::uint64_t signature = 0;
    const double scale = std::max(0.1, voxel_size);
    for (const Eigen::Vector3d &point : points)
    {
        const double radius = (point - center).norm();
        if (radius <= 1e-9 || radius > 4.0 * scale)
        {
            continue;
        }
        const int bin = std::clamp(
            static_cast<int>(std::floor(radius / (4.0 * scale) * 64.0)), 0, 63);
        signature |= (std::uint64_t{1} << bin);
    }
    return signature;
}

double binary_similarity(std::uint64_t lhs, std::uint64_t rhs)
{
    const int united = __builtin_popcountll(lhs | rhs);
    return united == 0 ? 0.0
                       : static_cast<double>(__builtin_popcountll(lhs & rhs)) /
                             static_cast<double>(united);
}

std::vector<Keypoint> extract_keypoints(const Keyframe &frame,
                                        const StdConfig &config)
{
    struct Accumulator
    {
        std::vector<Eigen::Vector3d> points;
    };
    std::map<VoxelKey, Accumulator> voxels;
    if (!frame.sonar_points)
    {
        return {};
    }
    const double inverse = 1.0 / config.voxel_size_m;
    std::vector<Eigen::Vector3d> all_points;
    all_points.reserve(frame.sonar_points->size());
    for (const PointXYZI &point : *frame.sonar_points)
    {
        const Eigen::Vector3d value(point.x, point.y, point.z);
        all_points.push_back(value);
        voxels[{static_cast<std::int64_t>(std::floor(value.x() * inverse)),
                static_cast<std::int64_t>(std::floor(value.y() * inverse)),
                static_cast<std::int64_t>(std::floor(value.z() * inverse))}]
            .points.push_back(value);
    }
    struct Ranked
    {
        Keypoint keypoint;
        double score;
    };
    std::vector<Ranked> ranked;
    for (const auto &entry : voxels)
    {
        const auto &points = entry.second.points;
        if (points.size() < config.minimum_voxel_points)
        {
            continue;
        }
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (const auto &point : points)
        {
            center += point;
        }
        center /= static_cast<double>(points.size());
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const auto &point : points)
        {
            const Eigen::Vector3d delta = point - center;
            covariance.noalias() += delta * delta.transpose();
        }
        covariance /= static_cast<double>(points.size());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success)
        {
            continue;
        }
        const auto values = solver.eigenvalues();
        const double ratio = values(0) / std::max(1e-12, values.sum());
        if (ratio > config.plane_eigenvalue_ratio || values(1) < 1e-8)
        {
            continue;
        }
        Keypoint keypoint;
        keypoint.point = center;
        keypoint.normal = solver.eigenvectors().col(0).normalized();
        keypoint.binary = binary_signature(center, all_points, config.voxel_size_m);
        ranked.push_back({keypoint,
                          static_cast<double>(points.size()) *
                              (1.0 - ratio)});
    }
    std::sort(ranked.begin(), ranked.end(), [](const auto &lhs, const auto &rhs) {
        if (lhs.score != rhs.score)
        {
            return lhs.score > rhs.score;
        }
        return std::tie(lhs.keypoint.point.x(), lhs.keypoint.point.y(),
                        lhs.keypoint.point.z()) <
               std::tie(rhs.keypoint.point.x(), rhs.keypoint.point.y(),
                        rhs.keypoint.point.z());
    });
    if (ranked.size() > config.maximum_keypoints)
    {
        ranked.resize(config.maximum_keypoints);
    }
    std::vector<Keypoint> result;
    result.reserve(ranked.size());
    for (const auto &item : ranked)
    {
        result.push_back(item.keypoint);
    }
    return result;
}

std::vector<Triangle> make_triangles(const std::vector<Keypoint> &keypoints,
                                     const StdConfig &config)
{
    std::vector<Triangle> triangles;
    if (keypoints.size() < 3)
    {
        return triangles;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &keypoint : keypoints)
    {
        cloud->emplace_back(static_cast<float>(keypoint.point.x()),
                            static_cast<float>(keypoint.point.y()),
                            static_cast<float>(keypoint.point.z()));
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    std::set<std::array<std::size_t, 3>> used;
    for (std::size_t index = 0; index < keypoints.size() &&
                                triangles.size() < config.maximum_triangles;
         ++index)
    {
        std::vector<int> neighbors(config.triangle_neighbor_count + 1);
        std::vector<float> distances(config.triangle_neighbor_count + 1);
        const int found = tree.nearestKSearch(
            cloud->points[index], static_cast<int>(neighbors.size()), neighbors,
            distances);
        for (int first = 1; first < found - 1 &&
                            triangles.size() < config.maximum_triangles;
             ++first)
        {
            for (int second = first + 1; second < found &&
                                         triangles.size() < config.maximum_triangles;
                 ++second)
            {
                std::array<std::size_t, 3> indices{
                    index, static_cast<std::size_t>(neighbors[first]),
                    static_cast<std::size_t>(neighbors[second])};
                std::sort(indices.begin(), indices.end());
                if (!used.insert(indices).second)
                {
                    continue;
                }
                std::array<double, 3> opposite{
                    (keypoints[indices[1]].point - keypoints[indices[2]].point).norm(),
                    (keypoints[indices[0]].point - keypoints[indices[2]].point).norm(),
                    (keypoints[indices[0]].point - keypoints[indices[1]].point).norm()};
                if (*std::min_element(opposite.begin(), opposite.end()) <
                        config.triangle_minimum_side_m ||
                    *std::max_element(opposite.begin(), opposite.end()) >
                        config.triangle_maximum_side_m)
                {
                    continue;
                }
                std::array<int, 3> order{0, 1, 2};
                std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
                    return opposite[lhs] < opposite[rhs];
                });
                const std::array<double, 3> sorted{
                    opposite[order[0]], opposite[order[1]], opposite[order[2]]};
                if (sorted[2] >= sorted[0] + sorted[1] - 0.05 ||
                    std::abs(sorted[1] - sorted[0]) < 0.02 ||
                    std::abs(sorted[2] - sorted[1]) < 0.02)
                {
                    continue;
                }
                Triangle triangle;
                triangle.sides = {sorted[0], sorted[1], sorted[2]};
                for (int vertex = 0; vertex < 3; ++vertex)
                {
                    const auto &keypoint = keypoints[indices[order[vertex]]];
                    triangle.vertices[vertex] = keypoint.point;
                    triangle.normals[vertex] = keypoint.normal;
                    triangle.binary[vertex] = keypoint.binary;
                }
                triangles.push_back(std::move(triangle));
            }
        }
    }
    return triangles;
}

TriangleKey triangle_key(const Triangle &triangle, double resolution)
{
    return {static_cast<int>(std::llround(triangle.sides.x() / resolution)),
            static_cast<int>(std::llround(triangle.sides.y() / resolution)),
            static_cast<int>(std::llround(triangle.sides.z() / resolution))};
}

Pose3d rigid_transform(const std::vector<Eigen::Vector3d> &source,
                       const std::vector<Eigen::Vector3d> &target)
{
    Eigen::Vector3d source_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_center = Eigen::Vector3d::Zero();
    for (std::size_t index = 0; index < source.size(); ++index)
    {
        source_center += source[index];
        target_center += target[index];
    }
    source_center /= static_cast<double>(source.size());
    target_center /= static_cast<double>(target.size());
    Eigen::Matrix3d correlation = Eigen::Matrix3d::Zero();
    for (std::size_t index = 0; index < source.size(); ++index)
    {
        correlation.noalias() += (source[index] - source_center) *
                                 (target[index] - target_center).transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        correlation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
    if (rotation.determinant() < 0.0)
    {
        Eigen::Matrix3d corrected_v = svd.matrixV();
        corrected_v.col(2) *= -1.0;
        rotation = corrected_v * svd.matrixU().transpose();
    }
    return {Eigen::Quaterniond(rotation).normalized(),
            target_center - rotation * source_center};
}

std::size_t triangle_inliers(const Pose3d &transform,
                             const std::vector<TriangleMatch> &matches,
                             double threshold,
                             std::vector<std::size_t> *indices = nullptr)
{
    std::size_t count = 0;
    if (indices)
    {
        indices->clear();
    }
    for (std::size_t match_index = 0; match_index < matches.size(); ++match_index)
    {
        bool inlier = true;
        for (int vertex = 0; vertex < 3; ++vertex)
        {
            const Eigen::Vector3d transformed =
                transform.rotation * matches[match_index].current->vertices[vertex] +
                transform.translation;
            if ((transformed -
                 matches[match_index].historical->vertices[vertex])
                    .norm() > threshold)
            {
                inlier = false;
                break;
            }
        }
        if (inlier)
        {
            ++count;
            if (indices)
            {
                indices->push_back(match_index);
            }
        }
    }
    return count;
}

double overlap(const Keyframe &current, const Keyframe &historical,
               const Pose3d &T_historical_sonar_current_sonar,
               double threshold)
{
    if (!current.sonar_points || !historical.sonar_points ||
        current.sonar_points->empty() || historical.sonar_points->empty())
    {
        return 0.0;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &point : *historical.sonar_points)
    {
        target->emplace_back(point.x, point.y, point.z);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(target);
    std::size_t matched = 0;
    std::vector<int> index(1);
    std::vector<float> distance(1);
    const double squared_threshold = threshold * threshold;
    for (const auto &point : *current.sonar_points)
    {
        const Eigen::Vector3d transformed =
            T_historical_sonar_current_sonar.rotation *
                Eigen::Vector3d(point.x, point.y, point.z) +
            T_historical_sonar_current_sonar.translation;
        pcl::PointXYZ query(static_cast<float>(transformed.x()),
                            static_cast<float>(transformed.y()),
                            static_cast<float>(transformed.z()));
        if (tree.nearestKSearch(query, 1, index, distance) == 1 &&
            distance[0] <= squared_threshold)
        {
            ++matched;
        }
    }
    return static_cast<double>(matched) /
           static_cast<double>(current.sonar_points->size());
}

Pose3d refine_alignment(const Keyframe &current, const Keyframe &historical,
                        Pose3d estimate, const StdConfig &config)
{
    if (!current.sonar_points || !historical.sonar_points ||
        current.sonar_points->empty() || historical.sonar_points->empty())
    {
        return estimate;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());
    target->reserve(historical.sonar_points->size());
    for (const auto &point : *historical.sonar_points)
    {
        target->emplace_back(point.x, point.y, point.z);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(target);
    const double threshold = std::max(
        0.5, 2.0 * config.geometric_overlap_distance_m);
    const double squared_threshold = threshold * threshold;
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_distance(1);
    for (std::size_t iteration = 0; iteration < config.refinement_iterations;
         ++iteration)
    {
        std::vector<Eigen::Vector3d> source;
        std::vector<Eigen::Vector3d> destination;
        source.reserve(current.sonar_points->size());
        destination.reserve(current.sonar_points->size());
        for (const auto &point : *current.sonar_points)
        {
            const Eigen::Vector3d transformed =
                estimate.rotation * Eigen::Vector3d(point.x, point.y, point.z) +
                estimate.translation;
            const pcl::PointXYZ query(static_cast<float>(transformed.x()),
                                      static_cast<float>(transformed.y()),
                                      static_cast<float>(transformed.z()));
            if (tree.nearestKSearch(query, 1, nearest_index, nearest_distance) == 1 &&
                nearest_distance[0] <= squared_threshold)
            {
                source.push_back(transformed);
                const auto &matched = target->points[nearest_index[0]];
                destination.emplace_back(matched.x, matched.y, matched.z);
            }
        }
        if (source.size() < 20)
        {
            break;
        }
        const Pose3d delta = rigid_transform(source, destination);
        estimate = compose(delta, estimate);
        if (delta.translation.norm() < 1e-3 &&
            delta.rotation.angularDistance(Eigen::Quaterniond::Identity()) < 1e-3)
        {
            break;
        }
    }
    return estimate;
}

}  // namespace

struct StableTriangleDetector::Impl
{
    struct Entry
    {
        Keyframe frame;
        std::vector<Keypoint> keypoints;
        std::vector<Triangle> triangles;
    };

    struct Reference
    {
        std::size_t entry = 0;
        std::size_t triangle = 0;
    };

    struct Pending
    {
        std::uint64_t target_id = 0;
        std::uint64_t source_id = 0;
        Pose3d correction;
        std::size_t confirmations = 0;
    };

    struct AcceptedLoop
    {
        std::uint64_t source_id = 0;
        std::uint64_t target_id = 0;
        double source_timestamp = 0.0;
    };

    explicit Impl(StdConfig input) : config(std::move(input)) {}

    void add(Entry entry)
    {
        const std::size_t entry_index = entries.size();
        entries.push_back(std::move(entry));
        const Entry &stored = entries.back();
        for (std::size_t index = 0; index < stored.triangles.size(); ++index)
        {
            inverted_index[triangle_key(
                               stored.triangles[index],
                               config.triangle_side_resolution_m)]
                .push_back({entry_index, index});
        }
    }

    StdConfig config;
    std::vector<Entry> entries;
    std::unordered_map<TriangleKey, std::vector<Reference>, TriangleKeyHash>
        inverted_index;
    std::optional<Pending> pending;
    std::optional<AcceptedLoop> last_accepted;
};

StableTriangleDetector::StableTriangleDetector(StdConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
    if (impl_->config.voxel_size_m <= 0.0 ||
        impl_->config.triangle_side_resolution_m <= 0.0 ||
        impl_->config.geometric_overlap_distance_m <= 0.0 ||
        !std::isfinite(impl_->config.minimum_loop_duration_s) ||
        impl_->config.minimum_loop_duration_s < 0.0 ||
        !std::isfinite(impl_->config.accepted_loop_cooldown_s) ||
        impl_->config.accepted_loop_cooldown_s < 0.0)
    {
        throw std::invalid_argument("STD metric resolutions must be positive");
    }
}

StableTriangleDetector::~StableTriangleDetector() = default;

std::size_t StableTriangleDetector::database_size() const
{
    return impl_->entries.size();
}

void StableTriangleDetector::notify_loop_accepted(
    std::uint64_t source_id, std::uint64_t target_id, double source_timestamp)
{
    impl_->last_accepted =
        Impl::AcceptedLoop{source_id, target_id, source_timestamp};
}

StdDetectionResult StableTriangleDetector::process(const Keyframe &keyframe)
{
    StdDetectionResult result;
    result.source_id = keyframe.id;
    const auto descriptor_started = Clock::now();
    Impl::Entry current;
    current.frame = keyframe;
    current.keypoints = extract_keypoints(keyframe, impl_->config);
    current.triangles = make_triangles(current.keypoints, impl_->config);
    result.keypoints = current.keypoints.size();
    result.triangles = current.triangles.size();
    result.descriptor_time_ms = elapsed_ms(descriptor_started);
    const auto search_started = Clock::now();
    std::vector<Candidate> candidates;
    std::unordered_map<std::size_t, std::vector<TriangleMatch>> matches_by_entry;
    for (const Triangle &triangle : current.triangles)
    {
        const TriangleKey key = triangle_key(
            triangle, impl_->config.triangle_side_resolution_m);
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    const auto found = impl_->inverted_index.find(
                        {key.a + dx, key.b + dy, key.c + dz});
                    if (found == impl_->inverted_index.end())
                    {
                        continue;
                    }
                    for (const Impl::Reference &reference : found->second)
                    {
                        const Impl::Entry &historical =
                            impl_->entries[reference.entry];
                        if (keyframe.id <= historical.frame.id ||
                            keyframe.id - historical.frame.id <
                                impl_->config.minimum_keyframe_separation ||
                            keyframe.timestamp - historical.frame.timestamp <
                                impl_->config.minimum_loop_duration_s)
                        {
                            continue;
                        }
                        if (impl_->last_accepted &&
                            keyframe.timestamp -
                                    impl_->last_accepted->source_timestamp <=
                                impl_->config.accepted_loop_cooldown_s &&
                            std::llabs(
                                static_cast<long long>(historical.frame.id) -
                                static_cast<long long>(
                                    impl_->last_accepted->target_id)) <=
                                static_cast<long long>(
                                    impl_->config.confirmation_target_id_tolerance))
                        {
                            continue;
                        }
                        if ((keyframe.T_local_vehicle.translation -
                             historical.frame.T_local_vehicle.translation)
                                .norm() >
                            impl_->config.maximum_pose_distance_m)
                        {
                            continue;
                        }
                        const Triangle &other =
                            historical.triangles[reference.triangle];
                        if ((triangle.sides - other.sides).norm() >
                            impl_->config.triangle_relative_error *
                                std::max(1e-6, triangle.sides.norm()))
                        {
                            continue;
                        }
                        double similarity = 0.0;
                        for (int vertex = 0; vertex < 3; ++vertex)
                        {
                            similarity += binary_similarity(
                                triangle.binary[vertex], other.binary[vertex]);
                        }
                        if (similarity / 3.0 >=
                            impl_->config.binary_similarity_minimum)
                        {
                            matches_by_entry[reference.entry].push_back(
                                {&triangle, &other});
                        }
                    }
                }
            }
        }
    }
    for (auto &[entry_index, matches] : matches_by_entry)
    {
        if (matches.size() >= impl_->config.minimum_triangle_matches)
        {
            candidates.push_back(
                {entry_index,
                 (keyframe.T_local_vehicle.translation -
                  impl_->entries[entry_index].frame.T_local_vehicle.translation)
                     .norm(),
                 std::move(matches)});
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto &lhs, const auto &rhs) {
        if (std::abs(lhs.pose_distance_m - rhs.pose_distance_m) > 1e-9)
        {
            return lhs.pose_distance_m < rhs.pose_distance_m;
        }
        return lhs.matches.size() > rhs.matches.size();
    });
    if (candidates.size() > impl_->config.maximum_candidates)
    {
        candidates.resize(impl_->config.maximum_candidates);
    }
    result.search_time_ms = elapsed_ms(search_started);

    const auto verification_started = Clock::now();
    std::size_t best_inliers = 0;
    double best_overlap = 0.0;
    std::size_t best_entry = 0;
    Pose3d best_transform;
    std::size_t best_matches = 0;
    for (const Candidate &candidate : candidates)
    {
        const Impl::Entry &historical = impl_->entries[candidate.entry_index];
        Pose3d ransac;
        std::size_t inliers = 0;
        for (std::size_t hypothesis = 0;
             hypothesis < candidate.matches.size() && hypothesis < 128;
             ++hypothesis)
        {
            std::vector<Eigen::Vector3d> source(3), target(3);
            for (int vertex = 0; vertex < 3; ++vertex)
            {
                source[vertex] = candidate.matches[hypothesis].current->vertices[vertex];
                target[vertex] = candidate.matches[hypothesis].historical->vertices[vertex];
            }
            const Pose3d transform = rigid_transform(source, target);
            const std::size_t vote = triangle_inliers(
                transform, candidate.matches,
                impl_->config.ransac_vertex_threshold_m);
            if (vote > inliers)
            {
                inliers = vote;
                ransac = transform;
            }
        }
        if (inliers < impl_->config.minimum_ransac_inliers)
        {
            continue;
        }
        std::vector<std::size_t> inlier_indices;
        triangle_inliers(ransac, candidate.matches,
                         impl_->config.ransac_vertex_threshold_m, &inlier_indices);
        std::vector<Eigen::Vector3d> source;
        std::vector<Eigen::Vector3d> target;
        for (const std::size_t index : inlier_indices)
        {
            for (int vertex = 0; vertex < 3; ++vertex)
            {
                source.push_back(candidate.matches[index].current->vertices[vertex]);
                target.push_back(candidate.matches[index].historical->vertices[vertex]);
            }
        }
        const Pose3d rough = rigid_transform(source, target);
        const Pose3d predicted_vehicle = between(
            historical.frame.T_local_vehicle, keyframe.T_local_vehicle);
        const Pose3d predicted_sonar = compose(
            inverse(historical.frame.T_vehicle_sonar),
            compose(predicted_vehicle, keyframe.T_vehicle_sonar));
        const Pose3d rough_measurement = compose(
            historical.frame.T_vehicle_sonar,
            compose(rough, inverse(keyframe.T_vehicle_sonar)));
        const Pose3d prior_error = between(rough_measurement, predicted_vehicle);
        if (prior_error.translation.norm() >
                impl_->config.maximum_prior_translation_error_m ||
            rotation_distance_rad(rough_measurement, predicted_vehicle) >
                impl_->config.maximum_prior_rotation_rad)
        {
            continue;
        }
        const Pose3d descriptor_refined = refine_alignment(
            keyframe, historical.frame, rough, impl_->config);
        const Pose3d prior_refined = refine_alignment(
            keyframe, historical.frame, predicted_sonar, impl_->config);
        const double descriptor_overlap = overlap(
            keyframe, historical.frame, descriptor_refined,
            impl_->config.geometric_overlap_distance_m);
        const double prior_overlap = overlap(
            keyframe, historical.frame, prior_refined,
            impl_->config.geometric_overlap_distance_m);
        const bool use_prior = prior_overlap > descriptor_overlap;
        const Pose3d verified_transform =
            use_prior ? prior_refined : descriptor_refined;
        const double verified_overlap =
            use_prior ? prior_overlap : descriptor_overlap;
        const Pose3d verified_measurement = compose(
            historical.frame.T_vehicle_sonar,
            compose(verified_transform, inverse(keyframe.T_vehicle_sonar)));
        const Pose3d verified_prior_error =
            between(verified_measurement, predicted_vehicle);
        if (verified_prior_error.translation.norm() >
                impl_->config.maximum_prior_translation_error_m ||
            rotation_distance_rad(verified_measurement, predicted_vehicle) >
                impl_->config.maximum_prior_rotation_rad)
        {
            continue;
        }
        if (verified_overlap > best_overlap ||
            (verified_overlap == best_overlap && inliers > best_inliers))
        {
            best_overlap = verified_overlap;
            best_inliers = inliers;
            best_transform = verified_transform;
            best_entry = candidate.entry_index;
            best_matches = candidate.matches.size();
        }
    }
    result.verification_time_ms = elapsed_ms(verification_started);
    result.descriptor_matches = best_matches;
    result.ransac_inliers = best_inliers;
    result.overlap = best_overlap;

    if (best_inliers < impl_->config.minimum_ransac_inliers ||
        best_overlap < impl_->config.geometric_overlap_minimum)
    {
        result.reason = candidates.empty() ? "descriptor_rejected"
                                           : "geometry_rejected";
        impl_->add(std::move(current));
        return result;
    }

    const Impl::Entry &historical = impl_->entries[best_entry];
    result.proposed = true;
    result.target_id = historical.frame.id;
    const Pose3d measurement = compose(
        historical.frame.T_vehicle_sonar,
        compose(best_transform, inverse(keyframe.T_vehicle_sonar)));
    const Pose3d estimated_current = compose(
        historical.frame.T_local_vehicle, measurement);
    const Pose3d correction = compose(
        estimated_current, inverse(keyframe.T_local_vehicle));

    bool consistent = false;
    if (impl_->pending &&
        std::llabs(static_cast<long long>(impl_->pending->target_id) -
                   static_cast<long long>(historical.frame.id)) <=
            static_cast<long long>(impl_->config.confirmation_target_id_tolerance) &&
        (impl_->pending->correction.translation - correction.translation).norm() <=
            impl_->config.confirmation_translation_m &&
        impl_->pending->correction.rotation.angularDistance(correction.rotation) <=
            impl_->config.confirmation_rotation_rad)
    {
        impl_->pending->source_id = keyframe.id;
        impl_->pending->target_id = historical.frame.id;
        impl_->pending->correction = correction;
        ++impl_->pending->confirmations;
        consistent = true;
    }
    else
    {
        impl_->pending =
            Impl::Pending{historical.frame.id, keyframe.id, correction, 1};
    }

    const std::uint64_t keyframe_separation =
        keyframe.id - historical.frame.id;
    const bool high_confidence_single_detection =
        impl_->config.single_detection_overlap_minimum > 0.0 &&
        best_overlap >= impl_->config.single_detection_overlap_minimum &&
        best_inliers >= 2 * impl_->config.minimum_ransac_inliers &&
        best_matches >= 10 * impl_->config.minimum_triangle_matches &&
        keyframe_separation >=
            5 * impl_->config.minimum_keyframe_separation;

    if (impl_->config.required_consistent_detections <= 1 ||
        high_confidence_single_detection ||
        (consistent && impl_->pending->confirmations >=
                           impl_->config.required_consistent_detections))
    {
        LoopConstraint loop;
        loop.from_id = historical.frame.id;
        loop.to_id = keyframe.id;
        loop.T_from_to = measurement;
        loop.covariance.setZero();
        const double rotation_sigma = 2.0 * 3.14159265358979323846 / 180.0;
        const double translation_sigma = std::max(
            0.05, impl_->config.geometric_overlap_distance_m /
                      std::sqrt(static_cast<double>(best_inliers)));
        loop.covariance.block<3, 3>(0, 0) =
            Eigen::Matrix3d::Identity() * rotation_sigma * rotation_sigma;
        loop.covariance.block<3, 3>(3, 3) =
            Eigen::Matrix3d::Identity() * translation_sigma * translation_sigma;
        result.confirmed = true;
        result.loop = loop;
        result.reason = high_confidence_single_detection
                            ? "confirmed_high_confidence"
                            : "confirmed";
        impl_->pending.reset();
    }
    else
    {
        result.reason = "awaiting_confirmation";
    }
    impl_->add(std::move(current));
    return result;
}

}  // namespace uwfl2::loop_closure
