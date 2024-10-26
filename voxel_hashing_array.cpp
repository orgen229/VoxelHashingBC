#include "voxel_hashing_array.h"

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    VoxelHashing<PointT, mini_grid_size>::VoxelHashing(float voxel_size, float mini_voxel_size)
        : voxel_size_(voxel_size), mini_voxel_size_(mini_voxel_size) {}

    template <typename PointT, std::size_t mini_grid_size>
    void VoxelHashing<PointT, mini_grid_size>::addPoint(const PointT& point) {
        auto voxel_index = getVoxelIndex(point);
        if (voxel_map_.find(voxel_index) == voxel_map_.end()) {
            voxel_map_[voxel_index] = createEmptyMiniVoxels();
        }
        auto mini_voxel_index = getMiniVoxelIndex(point, voxel_index);
        voxel_map_[voxel_index][std::get<0>(mini_voxel_index)]
            [std::get<1>(mini_voxel_index)]
            [std::get<2>(mini_voxel_index)].push_back(point);
    }

    template <typename PointT, std::size_t mini_grid_size>
    bool VoxelHashing<PointT, mini_grid_size>::IsPointInVoxel(const PointT& point) {
        auto voxel_index = getVoxelIndex(point);
        auto voxel_it = voxel_map_.find(voxel_index);
        if (voxel_it == voxel_map_.end()) {
            return false;
        }

        auto mini_voxel_index = getMiniVoxelIndex(point, voxel_index);
        const auto& mini_voxel_array = voxel_it->second;
        const auto& points = mini_voxel_array[std::get<0>(mini_voxel_index)]
            [std::get<1>(mini_voxel_index)]
            [std::get<2>(mini_voxel_index)];

        for (const auto& p : points) {
            if (p.x == point.x && p.y == point.y && p.z == point.z) {
                return true;
            }
        }

        return false;
    }

    template <typename PointT, std::size_t mini_grid_size>
    std::vector<PointT> VoxelHashing<PointT, mini_grid_size>::selectAllPointsFromVoxel(const std::tuple<int, int, int>& voxel_index) {
        auto voxel_it = voxel_map_.find(voxel_index);
        std::vector<PointT> vector;

        if (voxel_it == voxel_map_.end()) {
            return vector;
        }

        const auto& mini_voxel_array = voxel_it->second;

        for (const auto& mini_voxel_x : mini_voxel_array) {
            for (const auto& mini_voxel_y : mini_voxel_x) {
                for (const auto& mini_voxel_z : mini_voxel_y) {
                    vector.insert(vector.end(), mini_voxel_z.begin(), mini_voxel_z.end());
                }
            }
        }

        return vector;
    }

    template <typename PointT, std::size_t mini_grid_size>
    auto VoxelHashing<PointT, mini_grid_size>::createEmptyMiniVoxels() {
        return std::array<std::array<std::array<std::vector<PointT>, mini_grid_size>, mini_grid_size>, mini_grid_size>{};
    }

    template <typename PointT, std::size_t mini_grid_size>
    std::tuple<int, int, int> VoxelHashing<PointT, mini_grid_size>::getVoxelIndex(const PointT& point) const {
        return std::make_tuple(
            static_cast<int>(std::floor(point.x / voxel_size_)),
            static_cast<int>(std::floor(point.y / voxel_size_)),
            static_cast<int>(std::floor(point.z / voxel_size_))
        );
    }

    template <typename PointT, std::size_t mini_grid_size>
    std::tuple<int, int, int> VoxelHashing<PointT, mini_grid_size>::getMiniVoxelIndex(const PointT& point, const std::tuple<int, int, int>& voxel_index) const {
        float base_x = std::get<0>(voxel_index) * voxel_size_;
        float base_y = std::get<1>(voxel_index) * voxel_size_;
        float base_z = std::get<2>(voxel_index) * voxel_size_;

        int mini_x = static_cast<int>(std::floor((point.x - base_x) / mini_voxel_size_));
        int mini_y = static_cast<int>(std::floor((point.y - base_y) / mini_voxel_size_));
        int mini_z = static_cast<int>(std::floor((point.z - base_z) / mini_voxel_size_));

        return std::make_tuple(mini_x, mini_y, mini_z);
    }

    
    template class VoxelHashing<pcl::PointXYZ, 10>;
    template class VoxelHashing<pcl::PointXYZRGB, 10>;
}
