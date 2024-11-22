#ifndef ARRAY_SEARCH_HPP
#define ARRAY_SEARCH_HPP

#include "../array_search.h"

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size>
    ArraySearch<PointT, mini_grid_size>::ArraySearch(float voxel_size, float mini_voxel_size)
        : VoxelHashingArray<PointT, mini_grid_size>(voxel_size, mini_voxel_size) {}

    template <typename PointT, std::size_t mini_grid_size>
    std::tuple<int, int, int> ArraySearch<PointT, mini_grid_size>::adjustIndices(
        int x, int y, int z, std::tuple<int, int, int>& voxel_index) const {

        if (x < 0) {
            --std::get<0>(voxel_index);
            x += mini_grid_size;
        }
        else if (x >= mini_grid_size) {
            ++std::get<0>(voxel_index);
            x -= mini_grid_size;
        }

        if (y < 0) {
            --std::get<1>(voxel_index);
            y += mini_grid_size;
        }
        else if (y >= mini_grid_size) {
            ++std::get<1>(voxel_index);
            y -= mini_grid_size;
        }

        if (z < 0) {
            --std::get<2>(voxel_index);
            z += mini_grid_size;
        }
        else if (z >= mini_grid_size) {
            ++std::get<2>(voxel_index);
            z -= mini_grid_size;
        }

        return { x, y, z };
    }

    template <typename PointT, std::size_t mini_grid_size>
    std::vector<PointT> ArraySearch<PointT, mini_grid_size>::findKNearestNeighbors(
        const PointT& query_point, int k, float max_distance) const {

        using Neighbor = std::pair<float, PointT>;

        auto cmp = [](const Neighbor& left, const Neighbor& right) {
            return left.first > right.first;
            };

        std::priority_queue<Neighbor, std::vector<Neighbor>, decltype(cmp)> neighbors{ cmp };
        auto voxel_index = this->getVoxelIndex(query_point);
        auto mini_voxel_index = this->getMiniVoxelIndex(query_point, voxel_index);
        int layer_count = static_cast<int>(std::ceil(max_distance / this->mini_voxel_size_));

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

                    const auto& points = (*voxel_it->second)[nx][ny][nz];
                    for (const auto& point : points) {
                        float distance = std::sqrt(
                            std::pow(query_point.x - point.x, 2) +
                            std::pow(query_point.y - point.y, 2) +
                            std::pow(query_point.z - point.z, 2)
                        );

                        if (distance > 0 && distance <= max_distance) {
                            if (neighbors.size() < k) {
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

        std::vector<PointT> result;
        result.reserve(neighbors.size());
        while (!neighbors.empty()) {
            result.push_back(neighbors.top().second);
            neighbors.pop();
        }
        return result;
    }

    template <typename PointT, std::size_t mini_grid_size>
    std::vector<PointT> ArraySearch<PointT, mini_grid_size>::findAllPointsWithinRadius(
        const PointT& query_point, float max_distance) const {

        std::vector<PointT> result;
        auto voxel_index = this->getVoxelIndex(query_point);
        auto mini_voxel_index = this->getMiniVoxelIndex(query_point, voxel_index);
        int layer_count = static_cast<int>(std::ceil(max_distance / this->mini_voxel_size_));

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

                    const auto& points = (*voxel_it->second)[nx][ny][nz];
                    for (const auto& point : points) {
                        float distance = std::sqrt(
                            std::pow(query_point.x - point.x, 2) +
                            std::pow(query_point.y - point.y, 2) +
                            std::pow(query_point.z - point.z, 2)
                        );

                        if (distance > 0 && distance <= max_distance) {
                            result.push_back(point);
                        }
                    }
                }
            }
        }

        return result;
    }

} 

#endif 
