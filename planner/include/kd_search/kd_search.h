#ifndef KD_SEARCH_H
#define KD_SEARCH_H

#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include "util.h"
#include "nanoflann.hpp"

using namespace nanoflann;
typedef KDTreeSingleIndexDynamicAdaptor<L2_Simple_Adaptor<float, KDTreePt::PointCloud<float>>,KDTreePt::PointCloud<float>,3> DynamicTree;

class nanoKDTree{
private:
    DynamicTree* kdtree_;
    int dim_, max_leaf_;
    std::shared_ptr<KDTreePt::PointCloud<float>> cloud_;
public:
    nanoKDTree(int dim, int max_leaf, std::shared_ptr<KDTreePt::PointCloud<float>> cloud);
    ~nanoKDTree(){};
    void AddPoint(size_t start_ind, size_t end_ind);
    void RemovePoint(size_t removeInd);
    void RadiusSearch(float query_pt[3], float radius, vector<size_t>& indices, vector<float>& dists);
    void NearestSearch(float query_pt[3], size_t num_results, vector<size_t>& indices, vector<float>& dists);
};

#endif  // KD_SEARCH_H