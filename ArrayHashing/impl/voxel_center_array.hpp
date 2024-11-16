#ifndef VOXEL_CENTER_ARRAY_HPP
#define VOXEL_CENTER_ARRAY_HPP

#include "../voxel_center_array.h"

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    VoxelCenterArray<PointT, mini_grid_size>::VoxelCenterArray(float voxel_size, float mini_voxel_size)
        : VoxelHashingArray<PointT, mini_grid_size>(voxel_size, mini_voxel_size) {}

    template <typename PointT, std::size_t mini_grid_size>
    void VoxelCenterArray<PointT, mini_grid_size>::addPoint(const PointT& point) {
        auto voxel_index = this->getVoxelIndex(point);

        if (center_map_.find(voxel_index) == center_map_.end()) {
            center_map_[voxel_index] = std::make_shared<std::array<std::array<std::array<PointT, mini_grid_size>, mini_grid_size>, mini_grid_size>>();
        }

        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);

        if ((*center_map_[voxel_index])[std::get<0>(mini_voxel_index)][std::get<1>(mini_voxel_index)][std::get<2>(mini_voxel_index)].x == 0 &&
            (*center_map_[voxel_index])[std::get<0>(mini_voxel_index)][std::get<1>(mini_voxel_index)][std::get<2>(mini_voxel_index)].y == 0 &&
            (*center_map_[voxel_index])[std::get<0>(mini_voxel_index)][std::get<1>(mini_voxel_index)][std::get<2>(mini_voxel_index)].z == 0) {
            PointT center;
            center.x = (std::get<0>(mini_voxel_index) + 0.5f) * this->mini_voxel_size_;
            center.y = (std::get<1>(mini_voxel_index) + 0.5f) * this->mini_voxel_size_;
            center.z = (std::get<2>(mini_voxel_index) + 0.5f) * this->mini_voxel_size_;
            (*center_map_[voxel_index])[std::get<0>(mini_voxel_index)][std::get<1>(mini_voxel_index)][std::get<2>(mini_voxel_index)] = center;
        }
    }

    template <typename PointT, std::size_t mini_grid_size>
    PointT VoxelCenterArray<PointT, mini_grid_size>::getCenterInMiniVoxel(
        const std::tuple<int, int, int>& voxel_index,
        const std::tuple<int, int, int>& mini_voxel_index) const {

        auto voxel_it = center_map_.find(voxel_index);
        if (voxel_it != center_map_.end()) {
            return (*voxel_it->second)[std::get<0>(mini_voxel_index)][std::get<1>(mini_voxel_index)][std::get<2>(mini_voxel_index)];
        }
        return PointT();
    }

}

#endif 
