#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <limits>
#include <stdexcept>
#include <utility>
#include "sonar_registration_math.hpp"

namespace uwfl2
{
// Independently implements GenZ-LIO Algorithm 1. Bounds/gains must be supplied
// from the frozen experiment profile, not borrowed from donor LiDAR presets.
struct SonarVoxelControlConfig
{
    double initial_leaf, minimum_leaf, maximum_leaf;
    std::size_t range_window, minimum_points, maximum_points;
    double scale_threshold, exponent, proportional_fraction, derivative_fraction;
    double minimum_kp, maximum_kp, minimum_kd, maximum_kd;
};

class SonarVoxelController
{
public:
    explicit SonarVoxelController(SonarVoxelControlConfig config) : config_(config), leaf_(config.initial_leaf)
    {
        const std::array<double, 12> values = {config.initial_leaf, config.minimum_leaf, config.maximum_leaf,
            config.scale_threshold, config.exponent, config.proportional_fraction, config.derivative_fraction,
            config.minimum_kp, config.maximum_kp, config.minimum_kd, config.maximum_kd, leaf_};
        for (double value : values)
            if (!std::isfinite(value)) throw std::invalid_argument("Non-finite voxel controller configuration");
        if (config.minimum_leaf <= 0 || config.initial_leaf < config.minimum_leaf ||
            config.initial_leaf > config.maximum_leaf || config.range_window == 0 ||
            config.minimum_points == 0 || config.minimum_points > config.maximum_points ||
            config.scale_threshold <= 0 || config.exponent <= 0 ||
            config.proportional_fraction <= 0 || config.proportional_fraction > 1 ||
            config.derivative_fraction <= 0 || config.derivative_fraction > 1 ||
            config.minimum_kp < 0 || config.minimum_kp > config.maximum_kp ||
            config.minimum_kd < 0 || config.minimum_kd > config.maximum_kd)
            throw std::invalid_argument("Invalid voxel controller configuration");
    }

    double Leaf() const { return leaf_; }
    double DesiredPoints() const { return desired_; }

    double Advance(std::size_t temporary_points, std::size_t usable_input_points,
                   double median_range, double scan_interval)
    {
        // No scene-scale observation exists on empty input. Do not update the
        // window/error or integrate a stale error while sonar is unavailable.
        if (usable_input_points == 0) return leaf_;
        if (temporary_points == 0 || temporary_points > usable_input_points ||
            !std::isfinite(median_range) || median_range <= 0 ||
            !std::isfinite(scan_interval) || scan_interval <= 0)
            throw std::invalid_argument("Invalid voxel controller observation");
        medians_.push_back(median_range);
        if (medians_.size() > config_.range_window) medians_.pop_front();
        double scale = 0;
        for (double range : medians_) scale += range / medians_.size();
        const double phi = std::min(scale / config_.scale_threshold, 1.0);
        const double rho = 1 - std::pow(1 - phi, config_.exponent);
        desired_ = config_.minimum_points + (config_.maximum_points - config_.minimum_points) * rho;
        // Underwater adaptation: an unattainable target must not chase returns
        // which do not exist. Retain available sparse points without synthesis.
        desired_ = std::min(desired_, static_cast<double>(usable_input_points));
        const double error = desired_ - temporary_points;
        const double derivative = have_error_ ? (error - previous_error_) / scan_interval : 0;
        const double psi_p = std::min(std::abs(error) / (config_.proportional_fraction * desired_), 1.0);
        const double psi_d = std::min(std::abs(derivative) * scan_interval /
                                     (config_.derivative_fraction * desired_), 1.0);
        const double kp = config_.minimum_kp + (config_.maximum_kp - config_.minimum_kp) * std::sqrt(phi * psi_p);
        const double kd = config_.minimum_kd + (config_.maximum_kd - config_.minimum_kd) * std::sqrt(phi * psi_d);
        leaf_ = std::clamp(leaf_ - kp * error - kd * derivative, config_.minimum_leaf, config_.maximum_leaf);
        previous_error_ = error;
        have_error_ = true;
        return leaf_;
    }

private:
    SonarVoxelControlConfig config_;
    double leaf_, desired_ = 0, previous_error_ = 0;
    bool have_error_ = false;
    std::deque<double> medians_;
};

inline bool ValidSonarCovariance(const Eigen::Matrix3d &covariance)
{
    if (!covariance.allFinite() || (covariance - covariance.transpose()).norm() > 1e-10)
        return false;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    return eigen.info() == Eigen::Success && eigen.eigenvalues().minCoeff() >= -1e-12;
}

inline bool ValidSonarRotation(const Eigen::Matrix3d &rotation)
{
    return rotation.allFinite() && std::abs(rotation.determinant() - 1) < 1e-10 &&
           (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 1e-10;
}

using SonarCell = std::array<std::int64_t, 3>;

inline std::optional<SonarCell> SonarVoxelCell(const Eigen::Vector3d &position, double leaf)
{
    if (!position.allFinite() || !std::isfinite(leaf) || leaf <= 0) return std::nullopt;
    SonarCell cell;
    for (int axis = 0; axis < 3; ++axis)
    {
        const double value = std::floor(position[axis] / leaf);
        if (!std::isfinite(value) || std::abs(value) > 1e12) return std::nullopt;
        cell[axis] = static_cast<std::int64_t>(value);
    }
    return cell;
}

inline SonarCell SonarParentCell(SonarCell fine_cell)
{
    for (auto &coordinate : fine_cell)
        coordinate = coordinate >= 0 ? coordinate / 2 : (coordinate - 1) / 2;
    return fine_cell;
}

struct SonarVoxelizedPoints
{
    std::vector<SonarReferencePoint> points;
    std::vector<SonarCell> cells;
    bool complete = false;
};

// Centroid covariance for independent per-return noise (the injected synthetic
// profile). Common-mode calibration/pose error is not averaged down here.
inline SonarVoxelizedPoints VoxelizeSonarPoints(const std::vector<SonarReferencePoint> &input,
                                               double leaf, std::size_t maximum_input_points)
{
    SonarVoxelizedPoints result;
    if (!std::isfinite(leaf) || leaf <= 0 || input.size() > maximum_input_points) return result;
    struct Accumulator
    {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        Eigen::Matrix3d covariance_sum = Eigen::Matrix3d::Zero();
        std::size_t count = 0;
    };
    std::map<std::array<std::int64_t, 3>, Accumulator> cells;
    for (const auto &point : input)
    {
        if (!point.position.allFinite() || !ValidSonarCovariance(point.covariance)) return result;
        const auto key = SonarVoxelCell(point.position, leaf);
        if (!key) return result;
        auto &cell = cells[*key];
        cell.sum += point.position;
        cell.covariance_sum += point.covariance;
        ++cell.count;
    }
    for (const auto &entry : cells)
    {
        const auto &cell = entry.second;
        const double count = cell.count;
        result.points.push_back({cell.sum / count, cell.covariance_sum / (count * count)});
        result.cells.push_back(entry.first);
    }
    result.complete = true;
    return result;
}

struct SonarAdaptiveScan
{
    SonarVoxelizedPoints update, integration;
    double leaf = 0;
    bool complete = false;
};

inline SonarAdaptiveScan PrepareAdaptiveSonarScan(SonarVoxelController &controller,
    const std::vector<SonarReferencePoint> &input, double scan_interval, std::size_t maximum_input_points)
{
    SonarAdaptiveScan result;
    const auto temporary = VoxelizeSonarPoints(input, controller.Leaf(), maximum_input_points);
    if (!temporary.complete) return result;
    if (input.empty())
    {
        result.leaf = controller.Leaf();
        result.update.complete = result.integration.complete = result.complete = true;
        return result;
    }
    std::vector<double> ranges;
    for (const auto &point : temporary.points) ranges.push_back(point.position.norm());
    std::sort(ranges.begin(), ranges.end());
    const std::size_t middle = ranges.size() / 2;
    const double median = ranges.size() % 2 ? ranges[middle] : (ranges[middle - 1] + ranges[middle]) / 2;
    // Build with a candidate controller so any failure leaves its state intact.
    auto candidate = controller;
    result.leaf = candidate.Advance(temporary.points.size(), input.size(), median, scan_interval);
    result.integration = VoxelizeSonarPoints(input, result.leaf / 2, maximum_input_points);
    // Voxelizing fine centroids again without original multiplicity would give
    // the wrong coarse mean/covariance. Direct aggregation has the same nested
    // voxel membership and preserves each original return's weight.
    result.update = VoxelizeSonarPoints(input, result.leaf, maximum_input_points);
    result.complete = result.integration.complete && result.update.complete;
    if (result.complete) controller = candidate;
    return result;
}

// Noise belongs to the original sonar ray. Deskew/correction rotations carry it
// into local coordinates; translation affects position, not measurement noise.
inline SonarReferencePoint TransformSonarReference(const SonarReferencePoint &point,
    const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation)
{
    if (!point.position.allFinite() || !ValidSonarCovariance(point.covariance) ||
        !ValidSonarRotation(rotation) || !translation.allFinite())
        throw std::invalid_argument("Invalid sonar reference transform");
    return {rotation * point.position + translation, rotation * point.covariance * rotation.transpose()};
}

struct SonarSearchResult
{
    std::vector<SonarReferencePoint> neighbors;
    std::vector<SonarReferencePlane> plane_candidates;
    std::size_t accessed_voxels = 0, evaluated_points = 0;
    bool complete = false;
};

// Bounded read-only snapshot of authoritative accepted map points. This is not
// a map integration pipeline: replacement (including pruning/LC) is explicit
// and atomic. Generation mismatch or exhausted search never yields a row.
class SonarRegistrationView
{
public:
    SonarRegistrationView(double leaf, std::size_t maximum_points, std::size_t maximum_evaluations)
        : leaf_(leaf), maximum_points_(maximum_points), maximum_evaluations_(maximum_evaluations)
    {
        if (!std::isfinite(leaf) || leaf <= 0 || maximum_points == 0 || maximum_evaluations == 0)
            throw std::invalid_argument("Invalid sonar registration view bounds");
    }

    bool Replace(std::uint64_t generation, const std::vector<SonarReferencePoint> &points)
    {
        if (points.size() > maximum_points_ || (initialized_ && generation <= generation_)) return false;
        std::map<Key, std::vector<SonarReferencePoint>> replacement;
        for (const auto &point : points)
        {
            const auto key = Cell(point.position);
            if (!key || !ValidSonarCovariance(point.covariance)) return false;
            replacement[*key].push_back(point);
        }
        std::map<Key, PlanePatch> replacement_planes;
        for (const auto &entry : replacement)
        {
            if (entry.second.size() > maximum_evaluations_) continue;
            const auto plane = FitSonarReferencePlane(entry.second);
            if (!plane) continue;
            PlanePatch patch;
            patch.plane = *plane;
            double noise_sum = 0.0;
            for (const auto &point : entry.second)
            {
                const Eigen::Vector3d offset = point.position - plane->center;
                const double distance = plane->normal.dot(offset);
                patch.normal_variance += distance * distance / entry.second.size();
                noise_sum += plane->normal.dot(point.covariance * plane->normal);
                patch.radius_squared = std::max(patch.radius_squared,
                    (offset - distance * plane->normal).squaredNorm());
            }
            patch.centered_noise_variance = (1.0 - 1.0 / entry.second.size()) *
                noise_sum / entry.second.size();
            replacement_planes.emplace(entry.first, std::move(patch));
        }
        voxels_.swap(replacement);
        planes_.swap(replacement_planes);
        size_ = points.size();
        generation_ = generation;
        initialized_ = true;
        return true;
    }

    std::size_t Size() const { return size_; }
    std::uint64_t Generation() const { return generation_; }
    double Leaf() const { return leaf_; }

    SonarSearchResult Search(const Eigen::Vector3d &query, std::uint64_t generation,
                            double radius, std::size_t maximum_neighbors,
                            double maximum_plane_distance = 0.1,
                            double residual_sigma = 3.0) const
    {
        SonarSearchResult result;
        const auto center = Cell(query);
        if (!initialized_ || generation != generation_ || !center || !std::isfinite(radius) ||
            radius <= 0 || radius > leaf_ || maximum_neighbors == 0 || maximum_neighbors > maximum_evaluations_ ||
            !std::isfinite(maximum_plane_distance) || maximum_plane_distance <= 0 ||
            !std::isfinite(residual_sigma) || residual_sigma <= 0)
            return result;
        struct Candidate
        {
            double distance;
            std::size_t order;
            const SonarReferencePoint *point;
        };
        std::vector<Candidate> candidates;
        std::map<Key, SonarReferencePlane> supported_planes;
        const double radius2 = radius * radius;
        for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
        for (int z = -1; z <= 1; ++z)
        {
            const Key key = {(*center)[0] + x, (*center)[1] + y, (*center)[2] + z};
            Eigen::Vector3d nearest;
            for (int axis = 0; axis < 3; ++axis)
                nearest[axis] = std::clamp(query[axis], key[axis] * leaf_, (key[axis] + 1) * leaf_);
            if ((nearest - query).squaredNorm() > radius2) continue;
            const auto voxel = voxels_.find(key);
            if (voxel == voxels_.end()) continue;
            ++result.accessed_voxels;
            const auto cached = planes_.find(key);
            bool usable_plane = cached != planes_.end();
            if (usable_plane)
            {
                const auto &patch = cached->second;
                const Eigen::Vector3d offset = query - patch.plane.center;
                const double distance = patch.plane.normal.dot(offset);
                usable_plane = patch.normal_variance <=
                    std::pow(maximum_plane_distance / residual_sigma, 2) + patch.centered_noise_variance &&
                    (offset - distance * patch.plane.normal).squaredNorm() <= patch.radius_squared;
            }
            for (const auto &point : voxel->second)
            {
                if (result.evaluated_points == maximum_evaluations_) return result;
                ++result.evaluated_points;
                if (usable_plane)
                {
                    const auto &plane = cached->second.plane;
                    usable_plane = std::abs(plane.normal.dot(point.position - plane.center)) <=
                        maximum_plane_distance + residual_sigma * std::sqrt(std::max(0.0,
                            plane.normal.dot(point.covariance * plane.normal)));
                }
                const double distance2 = (point.position - query).squaredNorm();
                if (distance2 <= radius2) candidates.push_back({distance2, candidates.size(), &point});
            }
            if (usable_plane) supported_planes.emplace(key, cached->second.plane);
        }
        // Select only the requested neighbors, preserving the exact original
        // stable distance order without sorting/copying every covariance.
        const std::size_t count = std::min(maximum_neighbors, candidates.size());
        std::partial_sort(candidates.begin(), candidates.begin() + count, candidates.end(),
            [](const auto &a, const auto &b) {
                return a.distance < b.distance || (a.distance == b.distance && a.order < b.order);
            });
        for (std::size_t i = 0; i < count; ++i)
            result.neighbors.push_back(*candidates[i].point);
        // Associate geometry first. Comparing likelihoods of different plane
        // normals rewards low variance, not necessarily the nearest surface.
        if (count != 0)
        {
            const auto support = Cell(candidates.front().point->position);
            const auto plane = supported_planes.find(*support);
            if (plane != supported_planes.end()) result.plane_candidates.push_back(plane->second);
        }
        result.complete = true;
        return result;
    }

private:
    using Key = std::array<std::int64_t, 3>;
    struct PlanePatch
    {
        SonarReferencePlane plane;
        double normal_variance = 0.0, centered_noise_variance = 0.0, radius_squared = 0.0;
    };
    std::optional<Key> Cell(const Eigen::Vector3d &position) const
    {
        if (!position.allFinite()) return std::nullopt;
        Key key;
        for (int axis = 0; axis < 3; ++axis)
        {
            const double value = std::floor(position[axis] / leaf_);
            // Guard key arithmetic and conversions, including neighbor +/- 1.
            constexpr double bound = 1e12;
            if (!std::isfinite(value) || std::abs(value) > bound) return std::nullopt;
            key[axis] = static_cast<std::int64_t>(value);
        }
        return key;
    }
    double leaf_;
    std::size_t maximum_points_, maximum_evaluations_, size_ = 0;
    std::uint64_t generation_ = 0;
    bool initialized_ = false;
    std::map<Key, std::vector<SonarReferencePoint>> voxels_;
    // Cached sufficient geometry, not another integrated map. Replacement and
    // pruning use exactly the same accepted ikd-tree snapshot as voxels_.
    std::map<Key, PlanePatch> planes_;
};

// Metadata only: the ikd-tree remains the authoritative accepted map. Exact
// float coordinates identify its retained source representatives, not neighbors
// guessed from world rays. Snapshot drops metadata for pruned/replaced nodes.
class SonarMapUncertainty
{
public:
    explicit SonarMapUncertainty(std::size_t maximum_points) : maximum_points_(maximum_points) {}
    bool Record(const std::vector<SonarReferencePoint> &sources)
    {
        if (sources.size() > maximum_points_ || covariance_.size() > maximum_points_ - sources.size()) return false;
        for (const auto &source : sources)
            if (!source.position.allFinite() || !ValidSonarCovariance(source.covariance)) return false;
        for (const auto &source : sources)
        {
            auto [entry, inserted] = covariance_.emplace(Key(source.position), source.covariance);
            if (!inserted)
            {
                // Float-identical coordinates can come from different views.
                // Cover either retained source without averaging repeated data
                // or pretending its provenance is independently observable.
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(source.covariance - entry->second);
                entry->second += eigen.eigenvectors() * eigen.eigenvalues().cwiseMax(0).asDiagonal() * eigen.eigenvectors().transpose();
                entry->second = (0.5 * (entry->second + entry->second.transpose())).eval();
            }
        }
        return true;
    }
    std::optional<std::vector<SonarReferencePoint>> Select(const std::vector<Eigen::Vector3d> &active_points)
    {
        if (active_points.size() > maximum_points_) return std::nullopt;
        std::vector<SonarReferencePoint> references;
        std::map<std::array<float, 3>, Eigen::Matrix3d> retained;
        for (const auto &point : active_points)
        {
            if (!point.allFinite()) return std::nullopt;
            const auto key = Key(point);
            const auto found = covariance_.find(key);
            if (found == covariance_.end()) return std::nullopt;
            references.push_back({point, found->second});
            retained.emplace(key, found->second);
        }
        covariance_.swap(retained);
        return references;
    }
    void Clear() { covariance_.clear(); }
    std::size_t Size() const { return covariance_.size(); }
private:
    static std::array<float, 3> Key(const Eigen::Vector3d &position)
    {
        return {static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z())};
    }
    std::size_t maximum_points_;
    std::map<std::array<float, 3>, Eigen::Matrix3d> covariance_;
};

struct SonarCorrespondenceConfig
{
    double search_radius, maximum_plane_distance, residual_sigma;
    std::size_t maximum_neighbors;
    double point_metric_scale;
    Eigen::Matrix3d registration_covariance = Eigen::Matrix3d::Zero();
};

struct SonarCorrespondence
{
    std::optional<SonarResidualRow> row;
    bool is_plane = false;
    bool unresolved_plane_normal = false;
    std::optional<SonarReferencePlane> cached_plane;
    SonarSearchResult search;
};

// Prediction covariance is used only for association validation, not added to
// residual R (the IKF already accounts for its prior). Geometry gates are local
// experiment settings, not asserted universal underwater calibration values.
inline SonarCorrespondence MatchSonarPoint(const SonarRegistrationView &view,
    std::uint64_t generation, const SonarCorrespondenceConfig &config,
    const Eigen::Vector3d &point_body, const Eigen::Matrix3d &rotation,
    const Eigen::Vector3d &translation, const Eigen::Matrix3d &query_covariance_local,
    const Eigen::Matrix3d &prediction_covariance_local)
{
    SonarCorrespondence result;
    if (!ValidSonarRotation(rotation) || !translation.allFinite() || !point_body.allFinite() ||
        !ValidSonarCovariance(query_covariance_local) || !ValidSonarCovariance(prediction_covariance_local) ||
        !std::isfinite(config.maximum_plane_distance) ||
        config.maximum_plane_distance <= 0 || !std::isfinite(config.residual_sigma) || config.residual_sigma <= 0 ||
        !std::isfinite(config.point_metric_scale) || config.point_metric_scale <= 0 ||
        !ValidSonarCovariance(config.registration_covariance)) return result;
    const Eigen::Vector3d query = rotation * point_body + translation;
    result.search = view.Search(query, generation, config.search_radius, config.maximum_neighbors,
                               config.maximum_plane_distance, config.residual_sigma);
    // A single isolated return is not evidence of a stable correspondence.
    if (!result.search.complete || result.search.neighbors.size() < 3) return result;
    if (!result.search.plane_candidates.empty())
    {
        double best_score = std::numeric_limits<double>::infinity();
        for (const auto &plane : result.search.plane_candidates)
        {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> normal(plane.covariance.topLeftCorner<3, 3>());
            if (normal.info() != Eigen::Success || config.residual_sigma *
                std::sqrt(std::max(0.0, normal.eigenvalues().maxCoeff())) >= std::acos(-1.0) / 2)
            {
                result.unresolved_plane_normal = true;
                continue;
            }
            auto row = SonarPlaneRow(point_body, rotation, translation, query_covariance_local, plane);
            if (!row) continue;
            row->variance += plane.normal.dot(config.registration_covariance * plane.normal);
            const double innovation_variance = row->variance +
                plane.normal.dot(prediction_covariance_local * plane.normal);
            if (row->residual * row->residual > config.residual_sigma * config.residual_sigma *
                innovation_variance) continue;
            const double score = row->residual * row->residual / innovation_variance +
                std::log(innovation_variance);
            if (score < best_score)
            {
                best_score = score;
                result.row = row;
                result.is_plane = true;
                result.cached_plane = plane;
            }
        }
        // Invalid plane associations cannot bypass validation as point rows.
        return result;
    }
    const auto plane = FitSonarReferencePlane(result.search.neighbors);
    if (plane)
    {
        Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
        double projected_noise_sum = 0;
        bool within_geometric_and_noise_bounds = true;
        for (const auto &reference : result.search.neighbors)
        {
            const Eigen::Vector3d offset = reference.position - plane->center;
            scatter += offset * offset.transpose();
            const double noise_variance = std::max(0.0,
                plane->normal.dot(reference.covariance * plane->normal));
            projected_noise_sum += noise_variance;
            within_geometric_and_noise_bounds &= std::abs(plane->normal.dot(offset)) <=
                config.maximum_plane_distance + config.residual_sigma * std::sqrt(noise_variance);
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(scatter);
        // Sonar adaptation: observed thickness includes calibrated return noise.
        // Centering independent returns leaves (1-1/N) times their mean projected
        // noise variance; do not interpret this contribution as surface curvature.
        const double maximum_variance = std::pow(config.maximum_plane_distance / config.residual_sigma, 2);
        const double count = result.search.neighbors.size();
        const double centered_noise_variance = (1.0 - 1.0 / count) * projected_noise_sum / count;
        if (eigen.info() == Eigen::Success && eigen.eigenvalues().sum() > 0 &&
            eigen.eigenvalues()[0] / count <= maximum_variance + centered_noise_variance &&
            within_geometric_and_noise_bounds)
        {
            // At the patch center the normal-noise term in scalar R vanishes.
            // That does not make an unidentified direction a valid Jacobian.
            // Require its sigma confidence interval to stay within a hemisphere.
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> normal_covariance(plane->covariance.topLeftCorner<3, 3>());
            const double half_pi = std::acos(-1.0) / 2;
            if (normal_covariance.info() != Eigen::Success ||
                config.residual_sigma * std::sqrt(std::max(0.0, normal_covariance.eigenvalues().maxCoeff())) >= half_pi)
            {
                result.unresolved_plane_normal = true;
                return result;
            }
            auto row = SonarPlaneRow(point_body, rotation, translation, query_covariance_local, *plane);
            if (row) row->variance += plane->normal.dot(config.registration_covariance * plane->normal);
            if (row && row->residual * row->residual <= config.residual_sigma * config.residual_sigma *
                (row->variance + plane->normal.dot(prediction_covariance_local * plane->normal)))
            {
                result.row = row;
                result.is_plane = true;
            }
            // A rejected plane match must not sneak through as a point match.
            return result;
        }
    }
    const auto &reference = result.search.neighbors.front();
    const Eigen::Vector3d offset = query - reference.position;
    // Association is a separate geometric validity check. The scalar
    // discretization term weights an accepted distance residual; it must not
    // turn an implausible physical correspondence into a valid match.
    const Eigen::Matrix3d validation_covariance = query_covariance_local + reference.covariance +
        config.registration_covariance + prediction_covariance_local;
    const Eigen::LDLT<Eigen::Matrix3d> ldlt(validation_covariance);
    if (ldlt.info() != Eigen::Success || ldlt.vectorD().minCoeff() <= 0 ||
        offset.dot(ldlt.solve(offset)) > config.residual_sigma * config.residual_sigma) return result;
    // A point residual assumes feature identity. Require the selected reference
    // to be distinguishable from alternatives under physical association
    // uncertainty; scalar discretization noise cannot establish that identity.
    // -2 log likelihood includes log determinant for anisotropic covariances.
    const double selected_likelihood = offset.dot(ldlt.solve(offset)) + ldlt.vectorD().array().log().sum();
    for (std::size_t i = 1; i < result.search.neighbors.size(); ++i)
    {
        const auto &alternative = result.search.neighbors[i];
        const Eigen::Matrix3d covariance = query_covariance_local + alternative.covariance +
            config.registration_covariance + prediction_covariance_local;
        const Eigen::LDLT<Eigen::Matrix3d> alternative_ldlt(covariance);
        if (alternative_ldlt.info() != Eigen::Success || alternative_ldlt.vectorD().minCoeff() <= 0) return result;
        const Eigen::Vector3d alternative_offset = query - alternative.position;
        const double likelihood = alternative_offset.dot(alternative_ldlt.solve(alternative_offset)) +
            alternative_ldlt.vectorD().array().log().sum();
        if (likelihood - selected_likelihood < config.residual_sigma * config.residual_sigma) return result;
    }
    result.row = SonarPointRow(point_body, rotation, translation, query_covariance_local, reference,
                              view.Leaf(), result.search.accessed_voxels, result.search.evaluated_points,
                              config.point_metric_scale);
    if (result.row)
    {
        const Eigen::Vector3d direction = result.row->jacobian.head<3>().transpose();
        result.row->variance += direction.dot(config.registration_covariance * direction);
        if (result.row->residual * result.row->residual > config.residual_sigma * config.residual_sigma *
            (result.row->variance + direction.dot(prediction_covariance_local * direction))) result.row.reset();
    }
    return result;
}
}  // namespace uwfl2
