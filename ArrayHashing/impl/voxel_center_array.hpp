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

       
        if (this->voxel_map_.find(voxel_index) == this->voxel_map_.end()) {
            this->voxel_map_[voxel_index] = this->createEmptyMiniVoxels();
        }

        
        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);

        int mini_x = std::get<0>(mini_voxel_index);
        int mini_y = std::get<1>(mini_voxel_index);
        int mini_z = std::get<2>(mini_voxel_index);

        
        auto& mini_voxel_point = (*this->voxel_map_[voxel_index])[mini_x][mini_y][mini_z];

        
        if (mini_voxel_point.empty()) {
            PointT center;
            center.x = voxel_index_to_world(mini_x, this->mini_voxel_size_);
            center.y = voxel_index_to_world(mini_y, this->mini_voxel_size_);
            center.z = voxel_index_to_world(mini_z, this->mini_voxel_size_);
            mini_voxel_point.push_back(center);
        }
    }

    template <typename PointT, std::size_t mini_grid_size>
    void VoxelCenterArray<PointT, mini_grid_size>::addPoints(const pcl::PointCloud<PointT>& cloud) {
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            this->addPoint(cloud.points[i]);
        }
    }




    template <typename PointT, std::size_t mini_grid_size>
    float VoxelCenterArray<PointT, mini_grid_size>::voxel_index_to_world(int index, float voxel_size) const {
        return (index + 0.5f) * voxel_size;
    }

}

#endif
