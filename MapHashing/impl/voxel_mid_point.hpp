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

       
        if (this->voxel_map_.find(voxel_index) == this->voxel_map_.end()) {
            this->voxel_map_[voxel_index] = std::unordered_map<std::tuple<int, int, int>, std::vector<PointT>, VoxelHash>();
        }

       
        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);

        
        auto& mini_voxel_map = this->voxel_map_[voxel_index];
        if (mini_voxel_map.find(mini_voxel_index) == mini_voxel_map.end()) {
          
            mini_voxel_map[mini_voxel_index] = std::vector<PointT>();
        }

      
        auto& point_vector = mini_voxel_map[mini_voxel_index];

        
        point_vector.push_back(point);

       
        PointT mid_point = {};
        for (const auto& p : point_vector) {
            mid_point.x += p.x;
            mid_point.y += p.y;
            mid_point.z += p.z;
        }

        size_t count = point_vector.size();
        mid_point.x /= count;
        mid_point.y /= count;
        mid_point.z /= count;

        
        point_vector.clear();
        point_vector.push_back(mid_point);
    }

    template <typename PointT>
    void VoxelMidPoint<PointT>::addPoints(const pcl::PointCloud<PointT>& cloud) {
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            this->addPoint(cloud.points[i]);
        }
    }

}

#endif
