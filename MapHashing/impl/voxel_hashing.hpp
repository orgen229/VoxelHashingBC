#ifndef VOXEL_HASHING_HPP
#define VOXEL_HASHING_HPP

#include "../voxel_hashing.h"

namespace voxelStruct {

    template <typename PointT>
    VoxelHashing<PointT>::VoxelHashing(float voxel_size, float mini_voxel_size, int mini_grid_size)
        : voxel_size_(voxel_size),
        mini_voxel_size_(mini_voxel_size),
        mini_grid_size_(mini_grid_size) {}

    template <typename PointT>
    void VoxelHashing<PointT>::addPoint(const PointT& point) {
        auto voxel_index = getVoxelIndex(point);
        if (voxel_map_.find(voxel_index) == voxel_map_.end()) {
            voxel_map_[voxel_index] = std::unordered_map<std::tuple<int, int, int>, std::vector<PointT>, VoxelHash>();
        }
        auto mini_voxel_index = getMiniVoxelIndex(point, voxel_index);
        voxel_map_[voxel_index][mini_voxel_index].push_back(point);
    }

    template <typename PointT>
    bool VoxelHashing<PointT>::IsPointInVoxel(const PointT& point) {
        auto voxel_index = getVoxelIndex(point);
        auto voxel_it = voxel_map_.find(voxel_index);
        if (voxel_it == voxel_map_.end()) {
            return false;
        }

        auto mini_voxel_index = getMiniVoxelIndex(point, voxel_index);
        const auto& mini_voxel_map = voxel_it->second;
        auto mini_voxel_it = mini_voxel_map.find(mini_voxel_index);

        if (mini_voxel_it == mini_voxel_map.end()) {
            return false;
        }

        const auto& points = mini_voxel_it->second;
        for (const auto& p : points) {
            if (p.x == point.x && p.y == point.y && p.z == point.z) {
                return true;
            }
        }

        return false;
    }

    template <typename PointT>
    std::vector<PointT> VoxelHashing<PointT>::selectAllPointsFromVoxel(const std::tuple<int, int, int>& voxel_index) {
        auto voxel_it = voxel_map_.find(voxel_index);
        std::vector<PointT> vector;

        if (voxel_it == voxel_map_.end()) {
            return vector;
        }

        const auto& mini_voxel_map = voxel_it->second;

        for (const auto& mini_voxel_pair : mini_voxel_map) {
            const auto& points = mini_voxel_pair.second;
            vector.insert(vector.end(), points.begin(), points.end());
        }

        return vector;
    }


    template <typename PointT>
    std::vector<PointT> VoxelHashing<PointT>::selectAllPointsFromMiniVoxel(
        const std::tuple<int, int, int>& voxel_index,
        const std::tuple<int, int, int>& mini_voxel_index) {

        std::vector<PointT> result;

        auto voxel_it = voxel_map_.find(voxel_index);
        if (voxel_it == voxel_map_.end()) {
            return result;
        }

        const auto& mini_voxel_map = voxel_it->second;
        auto mini_voxel_it = mini_voxel_map.find(mini_voxel_index);

        if (mini_voxel_it == mini_voxel_map.end()) {
            return result;
        }


        result = mini_voxel_it->second;
        return result;
    }

    template <typename PointT>
    std::tuple<int, int, int> VoxelHashing<PointT>::getVoxelIndex(const PointT& point) const {
        return std::make_tuple(
            static_cast<int>(std::floor(point.x / voxel_size_)),
            static_cast<int>(std::floor(point.y / voxel_size_)),
            static_cast<int>(std::floor(point.z / voxel_size_))
        );
    }

    template <typename PointT>
    std::tuple<int, int, int> VoxelHashing<PointT>::getMiniVoxelIndex(const PointT& point, const std::tuple<int, int, int>& voxel_index) const {
        float base_x = std::get<0>(voxel_index) * voxel_size_;
        float base_y = std::get<1>(voxel_index) * voxel_size_;
        float base_z = std::get<2>(voxel_index) * voxel_size_;

        int mini_x = static_cast<int>(std::floor((point.x - base_x) / mini_voxel_size_));
        int mini_y = static_cast<int>(std::floor((point.y - base_y) / mini_voxel_size_));
        int mini_z = static_cast<int>(std::floor((point.z - base_z) / mini_voxel_size_));

        return std::make_tuple(mini_x, mini_y, mini_z);
    }

}

#endif