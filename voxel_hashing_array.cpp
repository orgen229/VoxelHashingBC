#include "voxel_hashing_array.h"

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    VoxelHashingArray<PointT, mini_grid_size>::VoxelHashingArray(float voxel_size, float mini_voxel_size)
        : VoxelHashing<PointT>(voxel_size, mini_voxel_size, mini_grid_size) {}

    template <typename PointT, std::size_t mini_grid_size>
    void VoxelHashingArray<PointT, mini_grid_size>::addPoint(const PointT& point) {
        auto voxel_index = this->getVoxelIndex(point);
        if (voxel_map_.find(voxel_index) == voxel_map_.end()) {
            voxel_map_[voxel_index] = createEmptyMiniVoxels();
        }
        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);
        voxel_map_[voxel_index][std::get<0>(mini_voxel_index)]
            [std::get<1>(mini_voxel_index)]
            [std::get<2>(mini_voxel_index)].push_back(point);
    }

    template <typename PointT, std::size_t mini_grid_size>
    bool VoxelHashingArray<PointT, mini_grid_size>::IsPointInVoxel(const PointT& point) {
        auto voxel_index = this->getVoxelIndex(point);
        auto voxel_it = voxel_map_.find(voxel_index);
        if (voxel_it == voxel_map_.end()) {
            return false;
        }

        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);
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
    std::vector<PointT> VoxelHashingArray<PointT, mini_grid_size>::selectAllPointsFromVoxel(const std::tuple<int, int, int>& voxel_index) {
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
    auto VoxelHashingArray<PointT, mini_grid_size>::createEmptyMiniVoxels() {
        return std::array<std::array<std::array<std::vector<PointT>, mini_grid_size>, mini_grid_size>, mini_grid_size>{};
    }

    template <typename PointT, std::size_t mini_grid_size>
    std::tuple<int, int, int> VoxelHashingArray<PointT, mini_grid_size>::getVoxelIndex(const PointT& point) const {
        return VoxelHashing<PointT>::getVoxelIndex(point);
    }

    template <typename PointT, std::size_t mini_grid_size>
    std::tuple<int, int, int> VoxelHashingArray<PointT, mini_grid_size>::getMiniVoxelIndex(const PointT& point, const std::tuple<int, int, int>& voxel_index) const {
        return VoxelHashing<PointT>::getMiniVoxelIndex(point, voxel_index);
    }

    template class VoxelHashingArray<pcl::PointXYZ, 10>;
    template class VoxelHashingArray<pcl::PointXYZRGB, 10>;
}
