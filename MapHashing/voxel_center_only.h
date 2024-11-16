#ifndef VOXEL_CENTER_ONLY_H
#define VOXEL_CENTER_ONLY_H

#include "voxel_hashing.h"

namespace voxelStruct {

    template <typename PointT>
    class VoxelCenterOnly : public VoxelHashing<PointT> {
    public:
        VoxelCenterOnly(float voxel_size, float mini_voxel_size, int mini_grid_size);

        void addPoint(const PointT& point) override;
        PointT getCenterInMiniVoxel(
            const std::tuple<int, int, int>& voxel_index,
            const std::tuple<int, int, int>& mini_voxel_index) const;

    private:
        std::unordered_map<
            std::tuple<int, int, int>,
            std::unordered_map<std::tuple<int, int, int>, PointT, VoxelHash>,
            VoxelHash
        > center_map_;
    };

}

#include "impl/voxel_center_only.hpp"

#endif 
