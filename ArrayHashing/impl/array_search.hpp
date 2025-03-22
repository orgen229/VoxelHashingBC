#ifndef ARRAY_SEARCH_HPP
#define ARRAY_SEARCH_HPP

#include "../array_search.h"
#include <cmath>
#include <limits>
#include <queue>

namespace voxelStruct {

    template <typename PointT, std::size_t mini_grid_size, typename BaseClass>
    ArraySearch<PointT, mini_grid_size, BaseClass>::ArraySearch(float voxel_size, float mini_voxel_size)
        : BaseClass(voxel_size, mini_voxel_size) {}

    template <typename PointT, std::size_t mini_grid_size, typename BaseClass>
    std::tuple<int, int, int> ArraySearch<PointT, mini_grid_size, BaseClass>::adjustIndices(
        int x, int y, int z, std::tuple<int, int, int>& voxel_index) const
    {
        if (x < 0) {
            --std::get<0>(voxel_index);
            x += mini_grid_size;
        }
        else if (x >= (int)mini_grid_size) {
            ++std::get<0>(voxel_index);
            x -= mini_grid_size;
        }

        if (y < 0) {
            --std::get<1>(voxel_index);
            y += mini_grid_size;
        }
        else if (y >= (int)mini_grid_size) {
            ++std::get<1>(voxel_index);
            y -= mini_grid_size;
        }

        if (z < 0) {
            --std::get<2>(voxel_index);
            z += mini_grid_size;
        }
        else if (z >= (int)mini_grid_size) {
            ++std::get<2>(voxel_index);
            z -= mini_grid_size;
        }

        return { x, y, z };
    }

    template <typename PointT, std::size_t mini_grid_size, typename BaseClass>
    std::vector<PointT> ArraySearch<PointT, mini_grid_size, BaseClass>::findKNearestNeighbors(
        const PointT& query_point, int k, float max_distance) const
    {

        using Neighbor = std::pair<float, PointT>;
        auto cmp = [](const Neighbor& left, const Neighbor& right) {
            return left.first < right.first;

            };


        std::priority_queue<Neighbor, std::vector<Neighbor>, decltype(cmp)> neighbors(cmp);


        auto voxel_index = this->getVoxelIndex(query_point);

        auto mini_voxel_index = this->getMiniVoxelIndex(query_point, voxel_index);


        int layer_count = static_cast<int>(std::ceil(max_distance / this->mini_voxel_size_));
        float max_distance_squared = max_distance * max_distance;


        auto checkMiniVoxel = [&](int nx, int ny, int nz, std::tuple<int, int, int> neighbor_voxel_idx)
            {

                auto voxel_it = this->voxel_map_.find(neighbor_voxel_idx);
                if (voxel_it == this->voxel_map_.end()) {
                    return;
                }

                if (nx < 0 || nx >= (int)mini_grid_size ||
                    ny < 0 || ny >= (int)mini_grid_size ||
                    nz < 0 || nz >= (int)mini_grid_size)
                {
                    return;
                }

                const auto& mini_voxel_points = (*voxel_it->second)[nx][ny][nz];
                if (mini_voxel_points.empty()) {
                    return;
                }


                for (const auto& point : mini_voxel_points) {
                    float dx = query_point.x - point.x;
                    float dy = query_point.y - point.y;
                    float dz = query_point.z - point.z;
                    float distance_squared = dx * dx + dy * dy + dz * dz;

                    if (distance_squared > 0 && distance_squared <= max_distance_squared) {
                        float dist = std::sqrt(distance_squared);


                        if ((int)neighbors.size() < k) {

                            neighbors.emplace(dist, point);
                        }
                        else {

                            if (dist < neighbors.top().first) {
                                neighbors.pop();
                                neighbors.emplace(dist, point);
                            }
                        }
                    }
                }
            };

        for (int layer = 0; layer <= layer_count; ++layer)
        {

            for (int dx = -layer; dx <= layer; ++dx) {
                for (int dy = -layer; dy <= layer; ++dy) {
                    for (int dz = -layer; dz <= layer; ++dz) {


                        if (std::max({ std::abs(dx), std::abs(dy), std::abs(dz) }) != layer) {
                            continue;
                        }


                        auto neighbor_voxel_idx = voxel_index;
                        auto [nx, ny, nz] = adjustIndices(
                            std::get<0>(mini_voxel_index) + dx,
                            std::get<1>(mini_voxel_index) + dy,
                            std::get<2>(mini_voxel_index) + dz,
                            neighbor_voxel_idx
                        );


                        checkMiniVoxel(nx, ny, nz, neighbor_voxel_idx);
                    }
                }
            }


            if ((int)neighbors.size() >= k) {
                break;
            }
        }

        std::vector<PointT> result;
        result.reserve(neighbors.size());

        std::vector<Neighbor> temp;
        temp.reserve(neighbors.size());
        while (!neighbors.empty()) {
            temp.push_back(neighbors.top());
            neighbors.pop();
        }

        std::reverse(temp.begin(), temp.end());


        for (auto& t : temp) {
            result.push_back(t.second);
        }

        return result;
    }

    template <typename PointT, std::size_t mini_grid_size, typename BaseClass>
    std::vector<PointT> ArraySearch<PointT, mini_grid_size, BaseClass>::findAllPointsWithinRadius(
        const PointT& query_point, float max_distance) const
    {
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
                    if (voxel_it == this->voxel_map_.end()) {
                        continue;
                    }

                    if (nx < 0 || nx >= (int)mini_grid_size ||
                        ny < 0 || ny >= (int)mini_grid_size ||
                        nz < 0 || nz >= (int)mini_grid_size)
                    {
                        continue;
                    }

                    const auto& mini_voxel_points = (*voxel_it->second)[nx][ny][nz];
                    if (mini_voxel_points.empty()) {
                        continue;
                    }

                    for (const auto& point : mini_voxel_points) {
                        float dx = query_point.x - point.x;
                        float dy = query_point.y - point.y;
                        float dz = query_point.z - point.z;
                        float distance_squared = dx * dx + dy * dy + dz * dz;

                        if (distance_squared > 0 && distance_squared <= max_distance_squared) {
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