#ifndef ARRAY_SEARCH_H
#define ARRAY_SEARCH_H

#include "voxel_hashing_array.h"
#include <vector>
#include <tuple>
#include <queue>
#include <cmath>
#include <algorithm>

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    class ArraySearch : public VoxelHashingArray<PointT, mini_grid_size> {
    public:
        ArraySearch(float voxel_size, float mini_voxel_size);

        std::vector<PointT> findKNearestNeighbors(const PointT& query_point, int k, float max_distance) const;
        std::vector<PointT> findAllPointsWithinRadius(const PointT& query_point, float max_distance) const;

    private:
        std::tuple<int, int, int> adjustIndices(
            int x, int y, int z, std::tuple<int, int, int>& voxel_index) const;
    };

}

#include "array_search.hpp"

#endif // ARRAY_SEARCH_H
