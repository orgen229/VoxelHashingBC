#ifndef VOXEL_INDEX_ONLY_HPP
#define VOXEL_INDEX_ONLY_HPP

#include "../voxel_index_only.h"

namespace voxelStruct {

    template <typename PointT>
    VoxelIndexOnly<PointT>::VoxelIndexOnly(float voxel_size, float mini_voxel_size, int mini_grid_size)
        : VoxelHashing<PointT>(voxel_size, mini_voxel_size, mini_grid_size) {}

    template <typename PointT>
    void VoxelIndexOnly<PointT>::addPoint(const PointT& point) {
        auto voxel_index = this->getVoxelIndex(point);

        
        if (index_map_.find(voxel_index) == index_map_.end()) {
            index_map_[voxel_index] = {};
        }

        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);
        index_map_[voxel_index][mini_voxel_index].emplace_back(point.x, point.y, point.z);
    }

    template <typename PointT>
    std::vector<std::tuple<int, int, int>> VoxelIndexOnly<PointT>::getIndicesInMiniVoxel(
        const std::tuple<int, int, int>& voxel_index,
        const std::tuple<int, int, int>& mini_voxel_index) const {

        auto voxel_it = index_map_.find(voxel_index);
        if (voxel_it != index_map_.end()) {
            auto mini_it = voxel_it->second.find(mini_voxel_index);
            if (mini_it != voxel_it->second.end()) {
                return mini_it->second;
            }
        }
        return {};
    }

}

#endif 
