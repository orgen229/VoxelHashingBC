#ifndef VOXEL_MID_POINT_ARRAY_H
#define VOXEL_MID_POINT_ARRAY_H

#include "voxel_hashing_array.h"
#include <memory>
#include <array>

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    class VoxelMidPointArray : public VoxelHashingArray<PointT, mini_grid_size> {
    public:
        VoxelMidPointArray(float voxel_size, float mini_voxel_size);

        void addPoint(const PointT& point) override;
        void addPoints(const pcl::PointCloud<PointT>& cloud) override;
     
    };

}

#include "impl/voxel_mid_point_array.hpp"

#endif 
