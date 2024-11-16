#ifndef VOXEL_MID_POINT_ARRAY_HPP
#define VOXEL_MID_POINT_ARRAY_HPP

#include "../voxel_mid_point_array.h"

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    VoxelMidPointArray<PointT, mini_grid_size>::VoxelMidPointArray(float voxel_size, float mini_voxel_size)
        : VoxelHashingArray<PointT, mini_grid_size>(voxel_size, mini_voxel_size) {}

    template <typename PointT, std::size_t mini_grid_size>
    void VoxelMidPointArray<PointT, mini_grid_size>::addPoint(const PointT& point) {
        auto voxel_index = this->getVoxelIndex(point);

        if (mid_point_map_.find(voxel_index) == mid_point_map_.end()) {
            mid_point_map_[voxel_index] = std::make_shared<std::array<std::array<std::array<PointT, mini_grid_size>, mini_grid_size>, mini_grid_size>>();
        }

        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);
        auto& mid_point = (*mid_point_map_[voxel_index])[std::get<0>(mini_voxel_index)][std::get<1>(mini_voxel_index)][std::get<2>(mini_voxel_index)];

        if (mid_point.x == 0 && mid_point.y == 0 && mid_point.z == 0) {
            mid_point = point;
        }
        else {
            mid_point.x = (mid_point.x + point.x) / 2.0f;
            mid_point.y = (mid_point.y + point.y) / 2.0f;
            mid_point.z = (mid_point.z + point.z) / 2.0f;
        }
    }

    template <typename PointT, std::size_t mini_grid_size>
    PointT VoxelMidPointArray<PointT, mini_grid_size>::getMidPointInMiniVoxel(
        const std::tuple<int, int, int>& voxel_index,
        const std::tuple<int, int, int>& mini_voxel_index) const {

        auto voxel_it = mid_point_map_.find(voxel_index);
        if (voxel_it != mid_point_map_.end()) {
            return (*voxel_it->second)[std::get<0>(mini_voxel_index)][std::get<1>(mini_voxel_index)][std::get<2>(mini_voxel_index)];
        }
        return PointT();
    }

}

#endif 
