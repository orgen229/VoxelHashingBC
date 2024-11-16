#ifndef VOXEL_CENTER_ONLY_HPP
#define VOXEL_CENTER_ONLY_HPP

#include "../voxel_center_only.h"

namespace voxelStruct {

    template <typename PointT>
    VoxelCenterOnly<PointT>::VoxelCenterOnly(float voxel_size, float mini_voxel_size, int mini_grid_size)
        : VoxelHashing<PointT>(voxel_size, mini_voxel_size, mini_grid_size) {}

    template <typename PointT>
    void VoxelCenterOnly<PointT>::addPoint(const PointT& point) {
        auto voxel_index = this->getVoxelIndex(point);

        
        if (center_map_.find(voxel_index) == center_map_.end()) {
            center_map_[voxel_index] = {};
        }

        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);

        if (center_map_[voxel_index].find(mini_voxel_index) == center_map_[voxel_index].end()) {
            
            PointT center;
            center.x = (std::get<0>(mini_voxel_index) + 0.5f) * this->mini_voxel_size_;
            center.y = (std::get<1>(mini_voxel_index) + 0.5f) * this->mini_voxel_size_;
            center.z = (std::get<2>(mini_voxel_index) + 0.5f) * this->mini_voxel_size_;
            center_map_[voxel_index][mini_voxel_index] = center;
        }
    }

    template <typename PointT>
    PointT VoxelCenterOnly<PointT>::getCenterInMiniVoxel(
        const std::tuple<int, int, int>& voxel_index,
        const std::tuple<int, int, int>& mini_voxel_index) const {

        auto voxel_it = center_map_.find(voxel_index);
        if (voxel_it != center_map_.end()) {
            auto mini_it = voxel_it->second.find(mini_voxel_index);
            if (mini_it != voxel_it->second.end()) {
                return mini_it->second;
            }
        }
        return PointT();
    }

}

#endif 
