#ifndef UTIL_H
#define UTIL_H

#include <vector>
using namespace std;

namespace KDTreePt{
    
template <typename num_t>
struct Point{
    num_t x, y, z;

    Point(num_t x, num_t y, num_t z) : x(x), y(y), z(z) {}
    bool operator == (const Point & pt){
        float dis_sq = (this->x-pt.x)*(this->x-pt.x)+(this->y-pt.y)*(this->y-pt.y)+(this->z-pt.z)*(this->z-pt.z);
        if (dis_sq < 1.0e-4)
        {
            return true;
        }
        return false;
    }
};

// Define basic tree data type
template <typename num_t>
class PointCloud{
public:
std::vector<Point<num_t>> points;

size_t kdtree_get_point_index(const Point<num_t>& pt)
{
    return std::find(points.begin(), points.end(), pt)-points.begin();
}

inline size_t kdtree_get_point_count() const
{
    return points.size();
}

inline num_t kdtree_get_pt(const size_t idx, const size_t dim) const
{
    if (dim == 0)
    return points[idx].x;
    else if (dim == 1)
    return points[idx].y;
    else if (dim == 2)
    return points[idx].z;
    return 0.0;
}

template <class BBOX>
bool kdtree_get_bbox(BBOX &) const
{
    return false;
}

};

}


#endif  // UTIL_H