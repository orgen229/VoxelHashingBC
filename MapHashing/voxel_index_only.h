#ifndef VOXEL_INDEX_ONLY_H
#define VOXEL_INDEX_ONLY_H

#include "voxel_hashing.h"

namespace voxelStruct {

    template <typename PointT>
    class VoxelIndexOnly : public VoxelHashing<PointT> {
    public:
        VoxelIndexOnly(float voxel_size, float mini_voxel_size, int mini_grid_size);

        void addPoint(const PointT& point) override;
        std::vector<std::size_t> getIndicesInMiniVoxel(
            const std::tuple<int, int, int>& voxel_index,
            const std::tuple<int, int, int>& mini_voxel_index) const;

    private:
        static std::size_t global_index_; 
        std::unordered_map<
            std::tuple<int, int, int>,
            std::unordered_map<std::tuple<int, int, int>, std::vector<std::size_t>, VoxelHash>,
            VoxelHash
        > index_map_;
    };

}

#include "impl/voxel_index_only.hpp"

#endif 
