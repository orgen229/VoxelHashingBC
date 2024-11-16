#ifndef VOXEL_MID_POINT_H
#define VOXEL_MID_POINT_H

#include "voxel_hashing.h"

namespace voxelStruct {

    template <typename PointT>
    class VoxelMidPoint : public VoxelHashing<PointT> {
    public:
        VoxelMidPoint(float voxel_size, float mini_voxel_size, int mini_grid_size);

        void addPoint(const PointT& point) override;
        PointT getMidPointInMiniVoxel(
            const std::tuple<int, int, int>& voxel_index,
            const std::tuple<int, int, int>& mini_voxel_index) const;

    private:
        std::unordered_map<
            std::tuple<int, int, int>,
            std::unordered_map<std::tuple<int, int, int>, PointT, VoxelHash>,
            VoxelHash
        > mid_point_map_;
    };

}

#include "impl/voxel_mid_point.hpp"

#endif 
