#include "dbscan.hpp"

// Main function to perform DBSCAN clustering
std::vector<int> DBSCAN::fit(const std::vector<Eigen::Vector2d>& points) {
  const int n = points.size();
  std::vector<int> labels(n, -1); // -1: unclassified, -2: noise
  int clusterId = 0; // Cluster ID counter

  // Iterate through each point in the dataset
  for (int i = 0; i < n; ++i) {
    if (labels[i] != -1) continue; // Skip already classified points

    // Get neighbors of point i within eps radius
    auto neighbors = regionQuery(points, i);

    // If the point is not a core point (not enough neighbors), mark it as noise
    if (neighbors.size() < minPts_) {
      labels[i] = -2; // noise
    } else {
      // Otherwise, expand a new cluster from this point
      expandCluster(points, i, labels, clusterId++);
    }
  }

  return labels; // Return the cluster labels
}

// Finds all points in the dataset that are within eps_ distance of point at index 'idx'
std::vector<int> DBSCAN::regionQuery(const std::vector<Eigen::Vector2d>& points, int idx) {
  std::vector<int> neighbors;

  // Check distance from point[idx] to every other point
  for (int i = 0; i < points.size(); ++i) {
    if ((points[i] - points[idx]).norm() <= eps_) {
      neighbors.push_back(i); // If within eps_, add to neighbors
    }
  }
  return neighbors;
}

// Expands a new cluster from the given seed point (idx)
void DBSCAN::expandCluster(const std::vector<Eigen::Vector2d>& points, int idx,
                           std::vector<int>& labels, int clusterId) {

  // Get neighbors of the seed point
  auto neighbors = regionQuery(points, idx);
  // Assign the seed point to the current cluster
  labels[idx] = clusterId;

  // Iterate over each neighbor
  for (size_t i = 0; i < neighbors.size(); ++i) {
    int pt = neighbors[i];
    // If the neighbor was previously labeled as noise, assign it to the current cluster
    if (labels[pt] == -2) labels[pt] = clusterId; // noise -> cluster
    // If the point is already assigned to a cluster, skip it
    if (labels[pt] != -1) continue;

    // Assign the point to the current cluster
    labels[pt] = clusterId;
    // Get this point's neighbors
    auto pt_neighbors = regionQuery(points, pt);
    // If it's a core point, add its neighbors to the cluster's neighbor list
    if (pt_neighbors.size() >= minPts_) {
      neighbors.insert(neighbors.end(), pt_neighbors.begin(), pt_neighbors.end());
    }
  }
}
