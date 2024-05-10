#include "kd_search/kd_search.h"
#include "kd_search/util.h"
#include "kd_search/nanoflann.hpp"

using namespace std;

nanoKDTree::nanoKDTree(int dim, int max_leaf, std::shared_ptr<KDTreePt::PointCloud<float>> cloud):
cloud_(cloud), dim_(dim), max_leaf_(max_leaf)
{
    // Intial KDTree
    DynamicTree* Dykdtree_ = new DynamicTree(dim_, *cloud_, {10});
    kdtree_ = Dykdtree_;
}

void nanoKDTree::AddPoint(size_t start_ind, size_t end_ind){
    kdtree_->addPoints(start_ind, end_ind);
}

void nanoKDTree::RemovePoint(size_t removeInd){
    kdtree_->removePoint(removeInd);
}

void nanoKDTree::RadiusSearch(float query_pt[3], float radius, vector<size_t>& indices, vector<float>& dists){
    radius *= radius; // Search based on distance_square
    vector<nanoflann::ResultItem<size_t, float>> indices_dists;
    nanoflann::RadiusResultSet<float, size_t> resultSet(radius, indices_dists);
    kdtree_->findNeighbors(resultSet, query_pt);
    // // Sort by dists(small -> large)
    for(size_t i=0; i<indices_dists.size(); i++){
        indices.push_back(indices_dists[i].first);
        dists.push_back(indices_dists[i].second);
    }
}

void nanoKDTree::NearestSearch(float query_pt[3], size_t num_results, vector<size_t>& indices, vector<float>& dists){
    // knn search
    size_t ret_index[num_results];
    float out_dist_sqr[num_results];
    nanoflann::KNNResultSet<float> resultSet(num_results);
    resultSet.init(ret_index, out_dist_sqr);
    kdtree_->findNeighbors(resultSet, query_pt);
    // output
    for (size_t i = 0; i < resultSet.size(); ++i)
    {
        indices.push_back(ret_index[i]);
        dists.push_back(out_dist_sqr[i]);
    }
}