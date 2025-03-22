#ifndef VOXEL_INDEX_ONLY_HPP
#define VOXEL_INDEX_ONLY_HPP

#include "../voxel_index_only.h"

namespace voxelStruct {

    
    template <typename PointT>
    std::size_t VoxelIndexOnly<PointT>::global_index_ = 0;

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

       
        index_map_[voxel_index][mini_voxel_index].push_back(global_index_++);
    }

    template <typename PointT>
    void VoxelIndexOnly<PointT>::addPoints(const pcl::PointCloud<PointT>& cloud) {
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            this->addPoint(cloud.points[i]);
        }
    }


    template <typename PointT>
    std::vector<std::size_t> VoxelIndexOnly<PointT>::getIndicesInMiniVoxel(
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