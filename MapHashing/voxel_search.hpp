#ifndef VOXEL_SEARCH_HPP
#define VOXEL_SEARCH_HPP

#include "voxel_search.h"

namespace voxelStruct {

    template <typename PointT>
    std::vector<PointT> VoxelSearch<PointT>::findNeighborPoints(const PointT& point, int neighbor_distance) {
        std::vector<PointT> neighbors;

       
        auto voxel_index = this->getVoxelIndex(point);

        
        for (int dx = -neighbor_distance; dx <= neighbor_distance; ++dx) {
            for (int dy = -neighbor_distance; dy <= neighbor_distance; ++dy) {
                for (int dz = -neighbor_distance; dz <= neighbor_distance; ++dz) {
                    std::tuple<int, int, int> neighbor_index = {
                        std::get<0>(voxel_index) + dx,
                        std::get<1>(voxel_index) + dy,
                        std::get<2>(voxel_index) + dz
                    };

                    
                    auto points = this->selectAllPointsFromVoxel(neighbor_index);
                    neighbors.insert(neighbors.end(), points.begin(), points.end());
                }
            }
        }

        return neighbors;
    }

    template <typename PointT>
    PointT VoxelSearch<PointT>::findNearestPoint(const PointT& point) {

        PointT closest;
        double min_distance = std::numeric_limits<double>::infinity();


        auto voxel_index = this->getVoxelIndex(point);
        auto mini_voxel_index = this->getMiniVoxelIndex(point, voxel_index);

        auto euclidean_distance = [](const PointT& p1, const PointT& p2) {
            return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
            };

        int search_radius = 1;
        bool point_found = false;

      
        while (!point_found) {
            for (int dx = -search_radius; dx <= search_radius; ++dx) {
                for (int dy = -search_radius; dy <= search_radius; ++dy) {
                    for (int dz = -search_radius; dz <= search_radius; ++dz) {

                        
                       

                        
                        auto neighbor_mini_voxel_index = std::make_tuple(
                            std::get<0>(mini_voxel_index) + dx,
                            std::get<1>(mini_voxel_index) + dy,
                            std::get<2>(mini_voxel_index) + dz
                        );

                        
                        auto mini_voxel_points = this->selectAllPointsFromMiniVoxel(voxel_index, neighbor_mini_voxel_index);

                        
                        for (const auto& candidate_point : mini_voxel_points) {
                            double dist = euclidean_distance(point, candidate_point);
                            if (dist < min_distance) {
                                min_distance = dist;
                                closest = candidate_point;
                                point_found = true; 
                            }
                        }
                    }
                }
            }

            
            ++search_radius;
        }

        return closest;
    }


} 

#endif 
