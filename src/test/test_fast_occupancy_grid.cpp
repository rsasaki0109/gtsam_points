#include <gtest/gtest.h>

#include <vector>
#include <Eigen/Core>

#include <gtsam_points/ann/fast_occupancy_grid.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

TEST(FastOccupancyGrid, HandlesOutOfRangeCoordinates) {
  const std::vector<Eigen::Vector4d> points = {
    Eigen::Vector4d(-2'000'000.0, 0.0, 0.0, 1.0),
    Eigen::Vector4d(0.0, 2'000'000.0, 0.0, 1.0),
    Eigen::Vector4d(0.0, 0.0, -2'000'000.0, 1.0),
    Eigen::Vector4d(1e100, 0.0, 0.0, 1.0),
  };
  const auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  const auto& cloud_base = static_cast<const gtsam_points::PointCloud&>(*cloud);

  gtsam_points::FastOccupancyGrid grid(1.0);
  EXPECT_NO_THROW(grid.insert(cloud_base));
  EXPECT_EQ(grid.num_occupied_cells(), 0);
  EXPECT_NO_THROW(EXPECT_EQ(grid.calc_overlap(cloud_base), 0));
  EXPECT_NO_THROW(EXPECT_EQ(grid.calc_overlap_rate(cloud_base), 0.0));
}

TEST(FastOccupancyGrid, HandlesNegativeInRangeCoordinates) {
  const std::vector<Eigen::Vector4d> points = {
    Eigen::Vector4d(-1.2, -2.3, -3.4, 1.0),
    Eigen::Vector4d(4.5, 5.6, 6.7, 1.0),
  };
  const auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  const auto& cloud_base = static_cast<const gtsam_points::PointCloud&>(*cloud);

  gtsam_points::FastOccupancyGrid grid(1.0);
  EXPECT_NO_THROW(grid.insert(cloud_base));
  EXPECT_EQ(grid.num_occupied_cells(), 2);
  EXPECT_EQ(grid.calc_overlap(cloud_base), 2);
  EXPECT_DOUBLE_EQ(grid.calc_overlap_rate(cloud_base), 1.0);
}
