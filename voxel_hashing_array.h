#ifndef VOXEL_HASHING_ARRAY_H
#define VOXEL_HASHING_ARRAY_H

#include "voxel_hashing.h"
#include <array>

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    class VoxelHashingArray : public VoxelHashing<PointT> {
    public:
        VoxelHashingArray(float voxel_size, float mini_voxel_size);

        void addPoint(const PointT& point) override;
        bool IsPointInVoxel(const PointT& point) override;
        std::vector<PointT> selectAllPointsFromVoxel(const std::tuple<int, int, int>& voxel_index) override;

        std::unordered_map<
            std::tuple<int, int, int>,
            std::array<std::array<std::array<std::vector<PointT>, mini_grid_size>, mini_grid_size>, mini_grid_size>,
            VoxelHash
        > voxel_map_;

    private:
        using VoxelHashing<PointT>::voxel_size_;
        using VoxelHashing<PointT>::mini_voxel_size_;

        

        auto createEmptyMiniVoxels();
        std::tuple<int, int, int> getVoxelIndex(const PointT& point) const override;
        std::tuple<int, int, int> getMiniVoxelIndex(const PointT& point, const std::tuple<int, int, int>& voxel_index) const override;
    };
}

#endif
