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

       
        if (this->voxel_map_.find(voxel_index) == this->voxel_map_.end()) {
            this->voxel_map_[voxel_index] = std::unordered_map<std::tuple<int, int, int>, std::vector<PointT>, VoxelHash>();
        }

      
        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);

       
        auto& mini_voxel_map = this->voxel_map_[voxel_index];
        if (mini_voxel_map.find(mini_voxel_index) == mini_voxel_map.end()) {
           
            mini_voxel_map[mini_voxel_index] = std::vector<PointT>();

           
            PointT center;
            center.x = voxel_index_to_world(std::get<0>(mini_voxel_index), this->mini_voxel_size_);
            center.y = voxel_index_to_world(std::get<1>(mini_voxel_index), this->mini_voxel_size_);
            center.z = voxel_index_to_world(std::get<2>(mini_voxel_index), this->mini_voxel_size_);

           
            mini_voxel_map[mini_voxel_index].push_back(center);
        }
    }

    template <typename PointT>
    void VoxelCenterOnly<PointT>::addPoints(const pcl::PointCloud<PointT>& cloud) {
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            this->addPoint(cloud.points[i]);  
        }
    }

    template <typename PointT>
    float VoxelCenterOnly<PointT>::voxel_index_to_world(int index, float voxel_size) const {
        return (index + 0.5f) * voxel_size;
    }

}

#endif 
