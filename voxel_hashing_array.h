#ifndef VOXEL_HASHING_ARRAY_H
#define VOXEL_HASHING_ARRAY_H

#include <iostream>
#include <unordered_map>
#include <vector>
#include <tuple>
#include <cmath>
#include <array>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace voxelStruct {

    struct VoxelHash {
        std::size_t operator()(const std::tuple<int, int, int>& key) const {
            return std::get<0>(key) + 31 * std::get<1>(key) + 17 * std::get<2>(key);
        }
    };

    template <typename PointT, std::size_t mini_grid_size>
    class VoxelHashing {
    public:
        VoxelHashing(float voxel_size, float mini_voxel_size);

        void addPoint(const PointT& point);
        bool IsPointInVoxel(const PointT& point);
        std::vector<PointT> selectAllPointsFromVoxel(const std::tuple<int, int, int>& voxel_index);

    private:
        float voxel_size_, mini_voxel_size_;

        std::unordered_map<
            std::tuple<int, int, int>,
            std::array<std::array<std::array<std::vector<PointT>, mini_grid_size>, mini_grid_size>, mini_grid_size>,
            VoxelHash
        > voxel_map_;

        auto createEmptyMiniVoxels();
        std::tuple<int, int, int> getVoxelIndex(const PointT& point) const;
        std::tuple<int, int, int> getMiniVoxelIndex(const PointT& point, const std::tuple<int, int, int>& voxel_index) const;
    };
}



#endif 
