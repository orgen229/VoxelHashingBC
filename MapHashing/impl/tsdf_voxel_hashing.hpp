#ifndef TSDF_VOXEL_HASHING_HPP
#define TSDF_VOXEL_HASHING_HPP

#include "../tsdf_voxel_hashing.h"
#include <cmath>
#include <limits>
#include <omp.h>

namespace voxelStruct {

    template <typename PointT>
    TSDFVoxelHashing<PointT>::TSDFVoxelHashing(float voxel_size, float mini_voxel_size, float truncation_distance)
        : VoxelSearch<PointT>(voxel_size, mini_voxel_size, static_cast<int>(std::ceil(voxel_size / mini_voxel_size))),
        truncation_distance_(truncation_distance) {}

    template <typename PointT>
    void TSDFVoxelHashing<PointT>::calculateTSDF(const PointT& camera_origin) {
#pragma omp parallel for
        for (auto voxel_it = this->voxel_map_.begin(); voxel_it != this->voxel_map_.end(); ++voxel_it) {
            const auto& voxel_index = voxel_it->first;
            calculateTSDFForVoxel(voxel_index, camera_origin);
        }
    }

    template <typename PointT>
    void TSDFVoxelHashing<PointT>::calculateTSDFForVoxel(const std::tuple<int, int, int>& voxel_index, const PointT& camera_origin) {
        auto voxel_it = this->voxel_map_.find(voxel_index);
        if (voxel_it == this->voxel_map_.end()) {
            return;
        }

        auto& mini_voxel_map = voxel_it->second;

        for (int mx = 0; mx < this->mini_grid_size_; ++mx) {
            for (int my = 0; my < this->mini_grid_size_; ++my) {
                for (int mz = 0; mz < this->mini_grid_size_; ++mz) {
                    std::tuple<int, int, int> mini_voxel_index = { mx, my, mz };

                    PointT voxel_center;
                    voxel_center.x = (std::get<0>(voxel_index) + (mx + 0.5f) / this->mini_grid_size_) * this->voxel_size_;
                    voxel_center.y = (std::get<1>(voxel_index) + (my + 0.5f) / this->mini_grid_size_) * this->voxel_size_;
                    voxel_center.z = (std::get<2>(voxel_index) + (mz + 0.5f) / this->mini_grid_size_) * this->voxel_size_;

                    auto neighbors = this->findKNearestNeighbors(voxel_center, 1, truncation_distance_);
                    float tsdf_value = truncation_distance_;

                    if (!neighbors.empty()) {
                        const auto& nearest_point = neighbors.front();

                        float dx = nearest_point.x - voxel_center.x;
                        float dy = nearest_point.y - voxel_center.y;
                        float dz = nearest_point.z - voxel_center.z;
                        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

                        tsdf_value = (dist > truncation_distance_) ? truncation_distance_ : dist;

                        float vx = voxel_center.x - camera_origin.x;
                        float vy = voxel_center.y - camera_origin.y;
                        float vz = voxel_center.z - camera_origin.z;

                        float px = nearest_point.x - camera_origin.x;
                        float py = nearest_point.y - camera_origin.y;
                        float pz = nearest_point.z - camera_origin.z;

                        float dot_product = vx * px + vy * py + vz * pz;
                        if (dot_product < 0) {
                            tsdf_value = -tsdf_value;
                        }
                    }

#pragma omp critical
                    {
                        voxel_tsdf_map_[voxel_index][mini_voxel_index] = tsdf_value;
                    }
                }
            }
        }
    }

    template <typename PointT>
    float TSDFVoxelHashing<PointT>::getTSDFValueAt(const PointT& query_point) const {
        auto voxel_index = this->getVoxelIndex(query_point);
        auto mini_voxel_index = this->getMiniVoxelIndex(query_point, voxel_index);

        auto voxel_it = voxel_tsdf_map_.find(voxel_index);
        if (voxel_it == voxel_tsdf_map_.end()) {
            return truncation_distance_;
        }

        const auto& mini_voxel_map = voxel_it->second;
        auto mini_voxel_it = mini_voxel_map.find(mini_voxel_index);
        if (mini_voxel_it == mini_voxel_map.end()) {
            return truncation_distance_;
        }

        return mini_voxel_it->second; // Возвращаем TSDF значение
    }

} // namespace voxelStruct

#endif // TSDF_VOXEL_HASHING_HPP
