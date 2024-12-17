#ifndef TSDF_VOXEL_HASHING_ARRAY_H
#define TSDF_VOXEL_HASHING_ARRAY_H

#include "voxel_hashing_array.h"
#include <memory>
#include <unordered_map>
#include <tuple>
#include <array>
#include <vector>

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    class TSDFVoxelHashingArray : public VoxelHashingArray<PointT, mini_grid_size> {
    public:
        TSDFVoxelHashingArray(float voxel_size, float mini_voxel_size, float truncation_distance);

        
        void integratePoint(const PointT& point, float weight = 1.0f);

        
        float getTSDFValueAt(const PointT& query_point) const;

       
        PointT getClosestMeasuredPoint(const PointT& query_point) const;

    protected:
        float truncation_distance_;

       
        std::unordered_map<
            std::tuple<int, int, int>,
            std::shared_ptr<std::array<std::array<std::array<float, mini_grid_size>, mini_grid_size>, mini_grid_size>>,
            VoxelHash
        > voxel_tsdf_map_;

        std::unordered_map<
            std::tuple<int, int, int>,
            std::shared_ptr<std::array<std::array<std::array<float, mini_grid_size>, mini_grid_size>, mini_grid_size>>,
            VoxelHash
        > voxel_weight_map_;

        std::shared_ptr<std::array<std::array<std::array<float, mini_grid_size>, mini_grid_size>, mini_grid_size>> createEmptyTSDFGrid() const;
        std::shared_ptr<std::array<std::array<std::array<float, mini_grid_size>, mini_grid_size>, mini_grid_size>> createEmptyWeightGrid() const;
    };

}

#include "impl/tsdf_voxel_hashing_array.hpp"

#endif 
