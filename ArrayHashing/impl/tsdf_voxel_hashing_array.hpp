#ifndef TSDF_VOXEL_HASHING_ARRAY_HPP
#define TSDF_VOXEL_HASHING_ARRAY_HPP

#include "../tsdf_voxel_hashing_array.h"
#include <cmath>
#include <limits>

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    TSDFVoxelHashingArray<PointT, mini_grid_size>::TSDFVoxelHashingArray(float voxel_size, float mini_voxel_size, float truncation_distance)
        : VoxelHashingArray<PointT, mini_grid_size>(voxel_size, mini_voxel_size),
        truncation_distance_(truncation_distance)
    {}

    template <typename PointT, std::size_t mini_grid_size>
    void TSDFVoxelHashingArray<PointT, mini_grid_size>::integratePoint(const PointT& point, float weight) {
        
        auto voxel_index = this->getVoxelIndex(point);
        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);

        int mx = std::get<0>(mini_voxel_index);
        int my = std::get<1>(mini_voxel_index);
        int mz = std::get<2>(mini_voxel_index);

        
        if (voxel_tsdf_map_.find(voxel_index) == voxel_tsdf_map_.end()) {
            voxel_tsdf_map_[voxel_index] = createEmptyTSDFGrid();
            voxel_weight_map_[voxel_index] = createEmptyWeightGrid();
        }

        
        float depth = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        
        float voxel_world_x = (std::get<0>(voxel_index) + (mx + 0.5f) / mini_grid_size) * this->voxel_size_;
        float voxel_world_y = (std::get<1>(voxel_index) + (my + 0.5f) / mini_grid_size) * this->voxel_size_;
        float voxel_world_z = (std::get<2>(voxel_index) + (mz + 0.5f) / mini_grid_size) * this->voxel_size_;

        float voxel_distance = std::sqrt(voxel_world_x * voxel_world_x +
            voxel_world_y * voxel_world_y +
            voxel_world_z * voxel_world_z);

        float sdf = depth - voxel_distance;

       
        if (sdf > truncation_distance_) sdf = truncation_distance_;
        if (sdf < -truncation_distance_) sdf = -truncation_distance_;

        float tsdf_val = sdf / truncation_distance_;

        float old_tsdf = (*voxel_tsdf_map_[voxel_index])[mx][my][mz];
        float old_weight = (*voxel_weight_map_[voxel_index])[mx][my][mz];

        float new_weight = old_weight + weight;
        float new_tsdf = (old_tsdf * old_weight + tsdf_val * weight) / new_weight;

        (*voxel_tsdf_map_[voxel_index])[mx][my][mz] = new_tsdf;
        (*voxel_weight_map_[voxel_index])[mx][my][mz] = new_weight;

        
        this->addPoint(point);
    }

    template <typename PointT, std::size_t mini_grid_size>
    float TSDFVoxelHashingArray<PointT, mini_grid_size>::getTSDFValueAt(const PointT& query_point) const {
        auto voxel_index = this->getVoxelIndex(query_point);
        auto mini_voxel_index = this->getMiniVoxelIndex(query_point, voxel_index);

        int mx = std::get<0>(mini_voxel_index);
        int my = std::get<1>(mini_voxel_index);
        int mz = std::get<2>(mini_voxel_index);

        auto tsdf_it = voxel_tsdf_map_.find(voxel_index);
        if (tsdf_it == voxel_tsdf_map_.end()) {
            return 1.0f; 
        }

        return (*tsdf_it->second)[mx][my][mz];
    }

    template <typename PointT, std::size_t mini_grid_size>
    PointT TSDFVoxelHashingArray<PointT, mini_grid_size>::getClosestMeasuredPoint(const PointT& query_point) const {
        auto voxel_index = this->getVoxelIndex(query_point);
        auto mini_voxel_index = this->getMiniVoxelIndex(query_point, voxel_index);

        PointT closest_point;
        float min_dist = std::numeric_limits<float>::max();

        const auto voxel_it = this->voxel_map_.find(voxel_index);
        if (voxel_it != this->voxel_map_.end()) {
            int mx = std::get<0>(mini_voxel_index);
            int my = std::get<1>(mini_voxel_index);
            int mz = std::get<2>(mini_voxel_index);

            const auto& points = (*voxel_it->second)[mx][my][mz];
            for (const auto& p : points) {
                float dx = p.x - query_point.x;
                float dy = p.y - query_point.y;
                float dz = p.z - query_point.z;
                float dist = dx * dx + dy * dy + dz * dz;
                if (dist < min_dist&& dist!=0) {
                    min_dist = dist;
                    closest_point = p;
                }
            }
        }

        return closest_point;
    }

    template <typename PointT, std::size_t mini_grid_size>
    std::shared_ptr<std::array<std::array<std::array<float, mini_grid_size>, mini_grid_size>, mini_grid_size>>
        TSDFVoxelHashingArray<PointT, mini_grid_size>::createEmptyTSDFGrid() const {
        auto tsdf_grid = std::make_shared<std::array<std::array<std::array<float, mini_grid_size>, mini_grid_size>, mini_grid_size>>();
        for (auto& gx : *tsdf_grid) {
            for (auto& gy : gx) {
                for (auto& gz : gy) {
                    gz = 1.0f;
                }
            }
        }
        return tsdf_grid;
    }

    template <typename PointT, std::size_t mini_grid_size>
    std::shared_ptr<std::array<std::array<std::array<float, mini_grid_size>, mini_grid_size>, mini_grid_size>>
        TSDFVoxelHashingArray<PointT, mini_grid_size>::createEmptyWeightGrid() const {
        auto weight_grid = std::make_shared<std::array<std::array<std::array<float, mini_grid_size>, mini_grid_size>, mini_grid_size>>();
        for (auto& gx : *weight_grid) {
            for (auto& gy : gx) {
                for (auto& gz : gy) {
                    gz = 0.0f;
                }
            }
        }
        return weight_grid;
    }

}

#endif 