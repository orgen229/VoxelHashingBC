#ifndef VOXEL_INDEX_ARRAY_H
#define VOXEL_INDEX_ARRAY_H

#include "voxel_hashing_array.h"
#include <memory>
#include <array>

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    class VoxelIndexArray : public VoxelHashingArray<PointT, mini_grid_size> {
    public:
        VoxelIndexArray(float voxel_size, float mini_voxel_size);

        void addPoint(const PointT& point) override;
        std::vector<std::size_t> getIndicesInMiniVoxel(
            const std::tuple<int, int, int>& voxel_index,
            const std::tuple<int, int, int>& mini_voxel_index) const;

    private:
        static std::size_t global_index_;
        std::unordered_map<
            std::tuple<int, int, int>,
            std::shared_ptr<std::array<std::array<std::array<std::vector<std::size_t>, mini_grid_size>, mini_grid_size>, mini_grid_size>>,
            VoxelHash
        > index_map_;
    };

}

#include "impl/voxel_index_array.hpp"

#endif 