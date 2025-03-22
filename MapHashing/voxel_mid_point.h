#ifndef VOXEL_MID_POINT_H
#define VOXEL_MID_POINT_H

#include "voxel_hashing.h"

namespace voxelStruct {

    template <typename PointT>
    class VoxelMidPoint : public VoxelHashing<PointT> {
    public:
        VoxelMidPoint(float voxel_size, float mini_voxel_size, int mini_grid_size);

        void addPoint(const PointT& point) override;
        void addPoints(const pcl::PointCloud<PointT>& cloud) override;

    private:
        
    };

}

#include "impl/voxel_mid_point.hpp"

#endif 
