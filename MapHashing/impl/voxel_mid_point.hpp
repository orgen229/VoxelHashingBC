#ifndef VOXEL_MID_POINT_HPP
#define VOXEL_MID_POINT_HPP

#include "../voxel_mid_point.h"

namespace voxelStruct {

    template <typename PointT>
    VoxelMidPoint<PointT>::VoxelMidPoint(float voxel_size, float mini_voxel_size, int mini_grid_size)
        : VoxelHashing<PointT>(voxel_size, mini_voxel_size, mini_grid_size) {}

    template <typename PointT>
    void VoxelMidPoint<PointT>::addPoint(const PointT& point) {
        auto voxel_index = this->getVoxelIndex(point);

        
        if (mid_point_map_.find(voxel_index) == mid_point_map_.end()) {
            mid_point_map_[voxel_index] = {};
        }

        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);

        if (mid_point_map_[voxel_index].find(mini_voxel_index) == mid_point_map_[voxel_index].end()) {
            mid_point_map_[voxel_index][mini_voxel_index] = point;
        }
        else {
          
            auto& mid_point = mid_point_map_[voxel_index][mini_voxel_index];
            mid_point.x = (mid_point.x + point.x) / 2.0f;
            mid_point.y = (mid_point.y + point.y) / 2.0f;
            mid_point.z = (mid_point.z + point.z) / 2.0f;
        }
    }

    template <typename PointT>
    PointT VoxelMidPoint<PointT>::getMidPointInMiniVoxel(
        const std::tuple<int, int, int>& voxel_index,
        const std::tuple<int, int, int>& mini_voxel_index) const {

        auto voxel_it = mid_point_map_.find(voxel_index);
        if (voxel_it != mid_point_map_.end()) {
            auto mini_it = voxel_it->second.find(mini_voxel_index);
            if (mini_it != voxel_it->second.end()) {
                return mini_it->second;
            }
        }
        return PointT();
    }

}

#endif
