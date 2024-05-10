#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/GridMap.h"
#include "grid_map_core/grid_map_core.hpp"
#include "sea_planner/utility.h"


float util::distance(float x1, float x2, float y1, float y2){
    return sqrt(SQ(x1-x2)+SQ(y1-y2));
}

float util::getElevation(float x, float y, grid_map::GridMap elevation_map, string layer){
    Position p{x, y};
    if(!elevation_map.isInside(p)){
        return NAN;
    }
    return elevation_map.atPosition(layer, p);
}

float util::gradient(float distance, float z_diff){
    return atan2(fabs(z_diff), distance);
}

float util::costFunc(float sub_distance, float distance_heuristic , float sub_gradient, float total_distance, float total_gradient){
    return Eta * (0.5*((sub_distance+distance_heuristic)/total_distance) + 0.5*(sub_gradient/total_gradient));
}

float util::getInfoGain(float node_x, float node_y, grid_map::GridMap elevation_map, string layer){
    float infomation_gain = 0;
    float resolution = elevation_map.getResolution();
    Position center(node_x, node_y);
    double radius = informationRadius;
    float circleArea = M_PI * SQ(radius);
    int totalCellNum = round(circleArea / SQ(resolution));  // all cells contained by circle
    float availableCellNum = 0.0; // Iteratable cell numbers, float type is to get float num result
    float unknownCellNum = 0.0;
    for (grid_map::CircleIterator iterator(elevation_map, center, radius);
      !iterator.isPastEnd(); ++iterator) {
        availableCellNum++;
        if(isnan(elevation_map.at(layer, *iterator))){
            unknownCellNum ++;
        }
    }
    unknownCellNum += (totalCellNum-availableCellNum);
    infomation_gain = unknownCellNum/totalCellNum>1? 1 : unknownCellNum/totalCellNum;
    return infomation_gain;
}

int util::getOrientation(const int arr[8]){
    int num = 0;
    for(int i=0; i<8; i++){
        if(arr[i] == 1){
            num++;
        }
    }
    return num;
}

float util::getPenalty(const float min_distance_to_obs){
    float scale = 100.0;
    float x = min_distance_to_obs * (2*scale / inflationRadius) - scale;
    return (maxPenalty-1.0) * (1.0 / (1+regressionFactor*exp(min_distance_to_obs))) +1.0;
}