#ifndef TSDF_VOXEL_HASHING_ARRAY_HPP
#define TSDF_VOXEL_HASHING_ARRAY_HPP

#include "../tsdf_voxel_hashing_array.h"
#include <cmath>
#include <limits>

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    TSDFVoxelHashingArray<PointT, mini_grid_size>::TSDFVoxelHashingArray(float voxel_size, float mini_voxel_size, float truncation_distance)
        : ArraySearch<PointT, mini_grid_size>(voxel_size, mini_voxel_size),
        truncation_distance_(truncation_distance)
    {
    }

    template <typename PointT, std::size_t mini_grid_size>
    void TSDFVoxelHashingArray<PointT, mini_grid_size>::calculateTSDF(const PointT& camera_origin) {
       
#pragma omp parallel for
        for (auto voxel_it = this->voxel_map_.begin(); voxel_it != this->voxel_map_.end(); ++voxel_it) {
            const auto& voxel_index = voxel_it->first;

           
#pragma omp critical
            if (voxel_tsdf_map_.find(voxel_index) == voxel_tsdf_map_.end()) {
                voxel_tsdf_map_[voxel_index] = createEmptyTSDFGrid();
            }

            auto& mini_voxels = *voxel_it->second;
            auto& tsdf_grid = *voxel_tsdf_map_[voxel_index];

            
            std::array<std::array<std::array<PointT, mini_grid_size>, mini_grid_size>, mini_grid_size> mini_voxel_centers;
            for (int mx = 0; mx < (int)mini_grid_size; ++mx) {
                for (int my = 0; my < (int)mini_grid_size; ++my) {
                    for (int mz = 0; mz < (int)mini_grid_size; ++mz) {
                        float voxel_world_x = (std::get<0>(voxel_index) + (mx + 0.5f) / mini_grid_size) * this->voxel_size_;
                        float voxel_world_y = (std::get<1>(voxel_index) + (my + 0.5f) / mini_grid_size) * this->voxel_size_;
                        float voxel_world_z = (std::get<2>(voxel_index) + (mz + 0.5f) / mini_grid_size) * this->voxel_size_;

                        PointT voxel_center;
                        voxel_center.x = voxel_world_x;
                        voxel_center.y = voxel_world_y;
                        voxel_center.z = voxel_world_z;

                        mini_voxel_centers[mx][my][mz] = voxel_center;
                    }
                }
            }

            
            for (int mx = 0; mx < (int)mini_grid_size; ++mx) {
                for (int my = 0; my < (int)mini_grid_size; ++my) {
                    for (int mz = 0; mz < (int)mini_grid_size; ++mz) {
                        const PointT& voxel_center = mini_voxel_centers[mx][my][mz];

                       
                        auto neighbors = this->findKNearestNeighbors(voxel_center, 1, truncation_distance_);
                        float tsdf_value;

                        if (neighbors.empty()) {
                            
                            tsdf_value = truncation_distance_;
                        }
                        else {
                           
                            const auto& nearest_point = neighbors.front();

                         
                            float vx = voxel_center.x - camera_origin.x;
                            float vy = voxel_center.y - camera_origin.y;
                            float vz = voxel_center.z - camera_origin.z;

                         
                            float px = nearest_point.x - camera_origin.x;
                            float py = nearest_point.y - camera_origin.y;
                            float pz = nearest_point.z - camera_origin.z;

                           
                            float dx = nearest_point.x - voxel_center.x;
                            float dy = nearest_point.y - voxel_center.y;
                            float dz = nearest_point.z - voxel_center.z;
                            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

                          
                            tsdf_value = (dist > truncation_distance_) ? truncation_distance_ : dist;

                            
                            float dot_product = vx * px + vy * py + vz * pz;
                            if (dot_product < 0) {
                                tsdf_value = -tsdf_value; 
                            }
                        }

                        
                        tsdf_grid[mx][my][mz] = tsdf_value;
                    }
                }
            }
        }
    }

    template <typename PointT, std::size_t mini_grid_size>
    void TSDFVoxelHashingArray<PointT, mini_grid_size>::calculateTSDFForVoxel(const std::tuple<int, int, int>& voxel_index, const PointT& camera_origin) {
       
        auto voxel_it = this->voxel_map_.find(voxel_index);
        if (voxel_it == this->voxel_map_.end()) {
            std::cerr << "Voxel not found for index: (" << std::get<0>(voxel_index) << ", " << std::get<1>(voxel_index) << ", " << std::get<2>(voxel_index) << ")" << std::endl;
            return;
        }

       
        if (voxel_tsdf_map_.find(voxel_index) == voxel_tsdf_map_.end()) {
            voxel_tsdf_map_[voxel_index] = createEmptyTSDFGrid();
        }

        auto& mini_voxels = *voxel_it->second;
        auto& tsdf_grid = *voxel_tsdf_map_[voxel_index];

        
        std::array<std::array<std::array<PointT, mini_grid_size>, mini_grid_size>, mini_grid_size> mini_voxel_centers;
        for (int mx = 0; mx < (int)mini_grid_size; ++mx) {
            for (int my = 0; my < (int)mini_grid_size; ++my) {
                for (int mz = 0; mz < (int)mini_grid_size; ++mz) {
                    float voxel_world_x = (std::get<0>(voxel_index) + (mx + 0.5f) / mini_grid_size) * this->voxel_size_;
                    float voxel_world_y = (std::get<1>(voxel_index) + (my + 0.5f) / mini_grid_size) * this->voxel_size_;
                    float voxel_world_z = (std::get<2>(voxel_index) + (mz + 0.5f) / mini_grid_size) * this->voxel_size_;

                    PointT voxel_center;
                    voxel_center.x = voxel_world_x;
                    voxel_center.y = voxel_world_y;
                    voxel_center.z = voxel_world_z;

                    mini_voxel_centers[mx][my][mz] = voxel_center;
                }
            }
        }

        
        for (int mx = 0; mx < (int)mini_grid_size; ++mx) {
            for (int my = 0; my < (int)mini_grid_size; ++my) {
                for (int mz = 0; mz < (int)mini_grid_size; ++mz) {
                    const PointT& voxel_center = mini_voxel_centers[mx][my][mz];

                   
                    auto neighbors = this->findKNearestNeighbors(voxel_center, 1, truncation_distance_);
                    float tsdf_value;

                    if (neighbors.empty()) {
                        
                        tsdf_value = truncation_distance_;
                    }
                    else {
                       
                        const auto& nearest_point = neighbors.front();

                    
                        float vx = voxel_center.x - camera_origin.x;
                        float vy = voxel_center.y - camera_origin.y;
                        float vz = voxel_center.z - camera_origin.z;

                       
                        float px = nearest_point.x - camera_origin.x;
                        float py = nearest_point.y - camera_origin.y;
                        float pz = nearest_point.z - camera_origin.z;

                     
                        float dx = nearest_point.x - voxel_center.x;
                        float dy = nearest_point.y - voxel_center.y;
                        float dz = nearest_point.z - voxel_center.z;
                        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

                      
                        tsdf_value = (dist > truncation_distance_) ? truncation_distance_ : dist;

                     
                        float dot_product = vx * px + vy * py + vz * pz;
                        if (dot_product < 0) {
                            tsdf_value = -tsdf_value;
                        }
                    }

                    
                    tsdf_grid[mx][my][mz] = tsdf_value;
                }
            }
        }
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
            return truncation_distance_;
        }

        return (*tsdf_it->second)[mx][my][mz];
    }





    template <typename PointT, std::size_t mini_grid_size>
    std::shared_ptr<std::array<std::array<std::array<float, mini_grid_size>, mini_grid_size>, mini_grid_size>>
        TSDFVoxelHashingArray<PointT, mini_grid_size>::createEmptyTSDFGrid() const {
        auto grid = std::make_shared<std::array<std::array<std::array<float, mini_grid_size>, mini_grid_size>, mini_grid_size>>();
        for (auto& gx : *grid) {
            for (auto& gy : gx) {
                for (auto& gz : gy) {
                    gz = truncation_distance_;
                }
            }
        }
        return grid;
    }

}

#endif 
