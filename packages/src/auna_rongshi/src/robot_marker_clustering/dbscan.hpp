#pragma once
#include <vector>
#include <Eigen/Dense>

class DBSCAN {

public:
  DBSCAN(double eps, int minPts)
      : eps_(eps), minPts_(minPts) {}

  // Performs DBSCAN clustering on the input 2D points
  // @param points: Vector of 2D points (Eigen::Vector2d)
  // @return A vector of cluster labels (same size as input points), where -1 indicates noise
  std::vector<int> fit(const std::vector<Eigen::Vector2d>& points);

private:
  double eps_; // Radius of the neighborhood
  int minPts_; // Minimum number of points required to form a dense region (core point)

  // Finds the indices of all points in the dataset that are within eps_ distance of the point at index 'idx'
  // @param points: The full dataset of points
  // @param idx: Index of the point from which to find neighbors
  // @return A vector of indices representing the neighborhood
  std::vector<int> regionQuery(const std::vector<Eigen::Vector2d>& points, int idx);

  // Expands the cluster starting from a core point
  // @param points: The dataset of points
  // @param idx: Index of the starting point (a core point)
  // @param labels: Current cluster labels for each point
  // @param clusterId: ID of the cluster being formed
  void expandCluster(const std::vector<Eigen::Vector2d>& points, int idx,
                     std::vector<int>& labels, int clusterId);
};
