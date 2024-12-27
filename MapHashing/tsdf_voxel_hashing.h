#ifndef TSDF_VOXEL_HASHING_H
#define TSDF_VOXEL_HASHING_H

#include "voxel_search.h" 
#include <unordered_map>
#include <tuple>

namespace voxelStruct {

    template <typename PointT>
    class TSDFVoxelHashing : public VoxelSearch<PointT> {
    public:
        
        TSDFVoxelHashing(float voxel_size, float mini_voxel_size, float truncation_distance);

       
        void calculateTSDF(const PointT& camera_origin);

        
        float getTSDFValueAt(const PointT& query_point) const;

    protected:
        float truncation_distance_;

       
        std::unordered_map<
            std::tuple<int, int, int>,
            std::unordered_map<std::tuple<int, int, int>, float, VoxelHash>, 
            VoxelHash
        > voxel_tsdf_map_;

        
        void calculateTSDFForVoxel(const std::tuple<int, int, int>& voxel_index, const PointT& camera_origin);
    };

}

#include "impl/tsdf_voxel_hashing.hpp"

#endif 
