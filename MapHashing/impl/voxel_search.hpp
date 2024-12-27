#ifndef VOXEL_SEARCH_HPP
#define VOXEL_SEARCH_HPP

#include "../voxel_search.h"
#include <cmath>
#include <limits>
#include <omp.h>

namespace voxelStruct {

    template <typename PointT, typename BaseClass>
    VoxelSearch<PointT, BaseClass>::VoxelSearch(float voxel_size, float mini_voxel_size, int mini_grid_size)
        : BaseClass(voxel_size, mini_voxel_size, mini_grid_size) {}

    template <typename PointT, typename BaseClass>
    std::tuple<int, int, int> VoxelSearch<PointT, BaseClass>::adjustIndices(
        int x, int y, int z, std::tuple<int, int, int>& voxel_index) const {

        if (x < 0) { --std::get<0>(voxel_index); x += this->mini_grid_size_; }
        else if (x >= this->mini_grid_size_) { ++std::get<0>(voxel_index); x -= this->mini_grid_size_; }

        if (y < 0) { --std::get<1>(voxel_index); y += this->mini_grid_size_; }
        else if (y >= this->mini_grid_size_) { ++std::get<1>(voxel_index); y -= this->mini_grid_size_; }

        if (z < 0) { --std::get<2>(voxel_index); z += this->mini_grid_size_; }
        else if (z >= this->mini_grid_size_) { ++std::get<2>(voxel_index); z -= this->mini_grid_size_; }

        return { x, y, z };
    }

    template <typename PointT, typename BaseClass>
    std::vector<PointT> VoxelSearch<PointT, BaseClass>::findKNearestNeighbors(
        const PointT& query_point, int k, float max_distance) const {

        std::cout << " Noez" << std::endl;

        using Neighbor = std::pair<float, PointT>;
        auto cmp = [](const Neighbor& left, const Neighbor& right) {
            return left.first > right.first;
            };
        std::priority_queue<Neighbor, std::vector<Neighbor>, decltype(cmp)> neighbors{ cmp };

        auto voxel_index = this->getVoxelIndex(query_point);
        auto mini_voxel_index = this->getMiniVoxelIndex(query_point, voxel_index);
        int layer_count = static_cast<int>(std::ceil(max_distance / this->mini_voxel_size_));
        float max_distance_squared = max_distance * max_distance;

#pragma omp parallel for
        for (int dx = -layer_count; dx <= layer_count; ++dx) {
            for (int dy = -layer_count; dy <= layer_count; ++dy) {
                for (int dz = -layer_count; dz <= layer_count; ++dz) {
                    auto neighbor_voxel_index = voxel_index;
                    auto [nx, ny, nz] = adjustIndices(
                        std::get<0>(mini_voxel_index) + dx,
                        std::get<1>(mini_voxel_index) + dy,
                        std::get<2>(mini_voxel_index) + dz,
                        neighbor_voxel_index
                    );

                    auto voxel_it = this->voxel_map_.find(neighbor_voxel_index);
                    if (voxel_it == this->voxel_map_.end()) continue;

                    const auto& mini_voxel_map = voxel_it->second;
                    auto mini_voxel_it = mini_voxel_map.find({ nx, ny, nz });
                    if (mini_voxel_it == mini_voxel_map.end()) continue;

                    const auto& points = mini_voxel_it->second;
                    for (const auto& point : points) {
                        float distance_squared =
                            std::pow(query_point.x - point.x, 2) +
                            std::pow(query_point.y - point.y, 2) +
                            std::pow(query_point.z - point.z, 2);

                        if (distance_squared > 0 && distance_squared <= max_distance_squared) {
                            float distance = std::sqrt(distance_squared);
#pragma omp critical
                            {
                                if ((int)neighbors.size() < k) {
                                    neighbors.emplace(distance, point);
                                }
                                else if (distance < neighbors.top().first) {
                                    neighbors.pop();
                                    neighbors.emplace(distance, point);
                                }
                            }
                        }
                    }
                }
            }
        }

        std::vector<PointT> result;
        result.reserve(neighbors.size());
        while (!neighbors.empty()) {
            result.push_back(neighbors.top().second);
            neighbors.pop();
        }
        std::cout << " sosi" << std::endl;
        return result;
    }

    template <typename PointT, typename BaseClass>
    std::vector<PointT> VoxelSearch<PointT, BaseClass>::findAllPointsWithinRadius(
        const PointT& query_point, float max_distance) const {

        std::vector<PointT> result;
        auto voxel_index = this->getVoxelIndex(query_point);
        auto mini_voxel_index = this->getMiniVoxelIndex(query_point, voxel_index);
        int layer_count = static_cast<int>(std::ceil(max_distance / this->mini_voxel_size_));
        float max_distance_squared = max_distance * max_distance;

        for (int dx = -layer_count; dx <= layer_count; ++dx) {
            for (int dy = -layer_count; dy <= layer_count; ++dy) {
                for (int dz = -layer_count; dz <= layer_count; ++dz) {
                    auto neighbor_voxel_index = voxel_index;
                    auto [nx, ny, nz] = adjustIndices(
                        std::get<0>(mini_voxel_index) + dx,
                        std::get<1>(mini_voxel_index) + dy,
                        std::get<2>(mini_voxel_index) + dz,
                        neighbor_voxel_index
                    );

                    auto voxel_it = this->voxel_map_.find(neighbor_voxel_index);
                    if (voxel_it == this->voxel_map_.end()) continue;

                    const auto& mini_voxel_map = voxel_it->second;
                    auto mini_voxel_it = mini_voxel_map.find({ nx, ny, nz });
                    if (mini_voxel_it == mini_voxel_map.end()) continue;

                    const auto& points = mini_voxel_it->second;
                    for (const auto& point : points) {
                        float distance_squared =
                            std::pow(query_point.x - point.x, 2) +
                            std::pow(query_point.y - point.y, 2) +
                            std::pow(query_point.z - point.z, 2);

                        if (distance_squared > 0 && distance_squared <= max_distance_squared) {
                            result.push_back(point);
                        }
                    }
                }
            }
        }

        return result;
    }

} // namespace voxelStruct

#endif // VOXEL_SEARCH_HPP
