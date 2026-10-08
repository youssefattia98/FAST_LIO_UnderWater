#include <algorithm>
#include <memory>
#include <tuple>

#include <gtest/gtest.h>
#include <pcl/point_types.h>

#include "ikd-Tree/ikd_Tree.h"

namespace
{
using Point = pcl::PointXYZINormal;
using Tree = KD_TREE<Point>;
using Points = Tree::PointVector;

Point point(float x, float y, float z)
{
    Point result{};
    result.x = x;
    result.y = y;
    result.z = z;
    return result;
}

void expect_point(const Point &actual, const Point &expected)
{
    EXPECT_FLOAT_EQ(actual.x, expected.x);
    EXPECT_FLOAT_EQ(actual.y, expected.y);
    EXPECT_FLOAT_EQ(actual.z, expected.z);
}

TEST(KdTreeRepeatability, EquidistantNeighborsAreIndependentOfPointOrder)
{
    Points points;
    for (float x : {-1.0F, 1.0F})
        for (float y : {-1.0F, 1.0F})
            for (float z : {-1.0F, 1.0F})
                points.push_back(point(x, y, z));
    const Points expected{point(-1, -1, -1), point(-1, -1, 1), point(-1, 1, -1)};
    for (int shift = 0; shift < 8; ++shift)
    {
        std::rotate(points.begin(), points.begin() + 1, points.end());
        auto tree = std::make_unique<Tree>();
        tree->Build(points);
        Points nearest;
        std::vector<float> distances;
        tree->Nearest_Search(point(0, 0, 0), 3, nearest, distances);
        ASSERT_EQ(nearest.size(), expected.size());
        for (std::size_t i = 0; i < expected.size(); ++i)
        {
            expect_point(nearest[i], expected[i]);
            EXPECT_FLOAT_EQ(distances[i], 3.0F);
        }
    }
}

TEST(KdTreeRepeatability, DistanceTakesPrecedenceOverCoordinateTieBreaker)
{
    const Tree::PointType_CMP closer(point(1, 0, 0), 1e-12F);
    const Tree::PointType_CMP farther(point(-1, 0, 0), 2e-12F);
    EXPECT_TRUE(closer < farther);
    EXPECT_FALSE(farther < closer);
}

TEST(KdTreeRepeatability, VoxelRepresentativeIsIndependentOfTreeOrder)
{
    Points points;
    for (float y : {0.75F, 1.25F})
        for (float z : {0.75F, 1.25F})
            points.push_back(point(1, y, z));
    for (int shift = 0; shift < 4; ++shift)
    {
        std::rotate(points.begin(), points.begin() + 1, points.end());
        auto tree = std::make_unique<Tree>();
        tree->set_downsample_param(2.0);
        tree->Build(points);
        Points added{point(1, 1, 1.9F)};
        tree->Add_Points(added, true);
        Points nearest;
        std::vector<float> distances;
        tree->Nearest_Search(point(1, 1, 1), 5, nearest, distances);
        ASSERT_EQ(nearest.size(), 1U);
        expect_point(nearest.front(), point(1, 0.75F, 0.75F));
    }
}

TEST(KdTreeRepeatability, NearestNeighborsMatchBruteForceIncludingRadiusBoundary)
{
    Points points;
    for (int x = -2; x <= 2; ++x)
        for (int y = -2; y <= 2; ++y)
            for (int z = -2; z <= 2; ++z)
                points.push_back(point(x, y, z));
    auto tree = std::make_unique<Tree>();
    tree->Build(points);
    for (const auto &query : {point(0, 0, 0), point(0.5F, -0.5F, 0.5F), point(2, 2, 2)})
    {
        const auto distance = [&](const Point &p) {
            const float x = p.x - query.x, y = p.y - query.y, z = p.z - query.z;
            return x * x + y * y + z * z;
        };
        Points expected;
        for (const auto &p : points)
            if (distance(p) <= 1.0F) expected.push_back(p);
        std::sort(expected.begin(), expected.end(), [&](const Point &a, const Point &b) {
            return std::make_tuple(distance(a), a.x, a.y, a.z) <
                   std::make_tuple(distance(b), b.x, b.y, b.z);
        });
        expected.resize(std::min<std::size_t>(5, expected.size()));
        Points nearest;
        std::vector<float> distances;
        tree->Nearest_Search(query, 5, nearest, distances, 1.0F);
        ASSERT_EQ(nearest.size(), expected.size());
        for (std::size_t i = 0; i < expected.size(); ++i)
        {
            expect_point(nearest[i], expected[i]);
            EXPECT_FLOAT_EQ(distances[i], distance(expected[i]));
        }
    }
}
}  // namespace
