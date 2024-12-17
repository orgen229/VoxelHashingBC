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

        
        if (this->voxel_map_.find(voxel_index) == this->voxel_map_.end()) {
            this->voxel_map_[voxel_index] = this->createEmptyMiniVoxels();
        }

       
        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);

        
        auto& point_vector = (*this->voxel_map_[voxel_index])[std::get<0>(mini_voxel_index)][std::get<1>(mini_voxel_index)][std::get<2>(mini_voxel_index)];

        
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



}

#endif 
