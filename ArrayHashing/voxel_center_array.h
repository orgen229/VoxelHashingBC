#ifndef VOXEL_CENTER_ARRAY_H
#define VOXEL_CENTER_ARRAY_H

#include "voxel_hashing_array.h"
#include <memory>
#include <array>

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    class VoxelCenterArray : public VoxelHashingArray<PointT, mini_grid_size> {
    public:
        VoxelCenterArray(float voxel_size, float mini_voxel_size);

        void addPoint(const PointT& point) override;
        PointT getCenterInMiniVoxel(
            const std::tuple<int, int, int>& voxel_index,
            const std::tuple<int, int, int>& mini_voxel_index) const;

    private:
        std::unordered_map<
            std::tuple<int, int, int>,
            std::shared_ptr<std::array<std::array<std::array<PointT, mini_grid_size>, mini_grid_size>, mini_grid_size>>,
            VoxelHash
        > center_map_;
    };

}

#include "impl/voxel_center_array.hpp"

#endif 
