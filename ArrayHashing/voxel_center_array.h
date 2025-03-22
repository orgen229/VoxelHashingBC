#ifndef VOXEL_CENTER_ARRAY_H
#define VOXEL_CENTER_ARRAY_H

#include "voxel_hashing_array.h"
#include <memory>
#include <array>
#include <unordered_map>
#include <cmath>

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    class VoxelCenterArray : public VoxelHashingArray<PointT, mini_grid_size> {
    public:
        VoxelCenterArray(float voxel_size, float mini_voxel_size);

      
        void addPoint(const PointT& point) override;
        void addPoints(const pcl::PointCloud<PointT>& cloud) override;

     

    private:
        float voxel_index_to_world(int index, float voxel_size) const;

        
    };

}

#include "impl/voxel_center_array.hpp"

#endif
