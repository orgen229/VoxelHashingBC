#ifndef VOXEL_INDEX_ARRAY_HPP
#define VOXEL_INDEX_ARRAY_HPP

#include "../voxel_index_array.h"

namespace voxelStruct {


    template <typename PointT, std::size_t mini_grid_size>
    std::size_t VoxelIndexArray<PointT, mini_grid_size>::global_index_ = 0;

    template <typename PointT, std::size_t mini_grid_size>
    VoxelIndexArray<PointT, mini_grid_size>::VoxelIndexArray(float voxel_size, float mini_voxel_size)
        : VoxelHashingArray<PointT, mini_grid_size>(voxel_size, mini_voxel_size) {}

    template <typename PointT, std::size_t mini_grid_size>
    void VoxelIndexArray<PointT, mini_grid_size>::addPoint(const PointT& point) {
        auto voxel_index = this->getVoxelIndex(point);

        
        if (index_map_.find(voxel_index) == index_map_.end()) {
            index_map_[voxel_index] = std::make_shared<std::array<std::array<std::array<std::vector<std::size_t>, mini_grid_size>, mini_grid_size>, mini_grid_size>>();
        }

        
        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);

        int mini_x = std::get<0>(mini_voxel_index);
        int mini_y = std::get<1>(mini_voxel_index);
        int mini_z = std::get<2>(mini_voxel_index);

        
        (*index_map_[voxel_index])[mini_x][mini_y][mini_z].push_back(global_index_++); 
    }

    template <typename PointT, std::size_t mini_grid_size>
    void VoxelIndexArray<PointT, mini_grid_size>::addPoints(const pcl::PointCloud<PointT>& cloud) {
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            this->addPoint(cloud.points[i]);
        }
    }

    template <typename PointT, std::size_t mini_grid_size>
    std::vector<std::size_t> VoxelIndexArray<PointT, mini_grid_size>::getIndicesInMiniVoxel(
        const std::tuple<int, int, int>& voxel_index,
        const std::tuple<int, int, int>& mini_voxel_index) const {

        auto voxel_it = index_map_.find(voxel_index);
        if (voxel_it != index_map_.end()) {
            return (*voxel_it->second)[std::get<0>(mini_voxel_index)][std::get<1>(mini_voxel_index)][std::get<2>(mini_voxel_index)];
        }
        return {};
    }

}

#endif 