#ifndef VOXEL_CENTER_ONLY_H
#define VOXEL_CENTER_ONLY_H

#include "voxel_hashing.h"

namespace voxelStruct {

    template <typename PointT>
    class VoxelCenterOnly : public VoxelHashing<PointT> {
    public:
        VoxelCenterOnly(float voxel_size, float mini_voxel_size, int mini_grid_size);

        void addPoint(const PointT& point) override;

    private:
        float voxel_index_to_world(int index, float voxel_size) const;
    };

}

#include "impl/voxel_center_only.hpp"

#endif
