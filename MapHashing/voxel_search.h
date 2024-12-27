#ifndef VOXEL_SEARCH_H
#define VOXEL_SEARCH_H

#include "voxel_hashing.h"
#include <vector>
#include <tuple>
#include <queue>
#include <cmath>
#include <algorithm>
#include <type_traits>

namespace voxelStruct {

    template <typename PointT, typename BaseClass = VoxelHashing<PointT>>
    class VoxelSearch : public BaseClass {
    public:
        VoxelSearch(float voxel_size, float mini_voxel_size, int mini_grid_size);

        std::vector<PointT> findKNearestNeighbors(const PointT& query_point, int k, float max_distance) const;
        std::vector<PointT> findAllPointsWithinRadius(const PointT& query_point, float max_distance) const;

    private:
        std::tuple<int, int, int> adjustIndices(int x, int y, int z, std::tuple<int, int, int>& voxel_index) const;
    };

} 

#include "impl/voxel_search.hpp"

#endif 
