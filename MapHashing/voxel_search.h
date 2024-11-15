#ifndef VOXEL_SEARCH_H
#define VOXEL_SEARCH_H

#include "voxel_hashing.h"
#include <vector>

namespace voxelStruct {

    template <typename PointT>
    class VoxelSearch : public VoxelHashing<PointT> {
    public:
        using VoxelHashing<PointT>::VoxelHashing;

        
        std::vector<PointT> findNeighborPoints(const PointT& point, int neighbor_distance = 1);
        PointT findNearestPoint(const PointT& point);
    };

} 

#include "voxel_search.hpp"

#endif 
