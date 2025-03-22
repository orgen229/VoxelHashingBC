#ifndef VOXEL_SEARCH_HPP
#define VOXEL_SEARCH_HPP

#include "../voxel_search.h"
#include <cmath>
#include <limits>
#include <queue>
#include <algorithm>  // для std::reverse

namespace voxelStruct {

    template <typename PointT, typename BaseClass>
    VoxelSearch<PointT, BaseClass>::VoxelSearch(float voxel_size, float mini_voxel_size, int mini_grid_size)
        : BaseClass(voxel_size, mini_voxel_size, mini_grid_size) {}

    template <typename PointT, typename BaseClass>
    std::tuple<int, int, int> VoxelSearch<PointT, BaseClass>::adjustIndices(
        int x, int y, int z, std::tuple<int, int, int>& voxel_index) const
    {
        if (x < 0) {
            --std::get<0>(voxel_index);
            x += this->mini_grid_size_;
        }
        else if (x >= this->mini_grid_size_) {
            ++std::get<0>(voxel_index);
            x -= this->mini_grid_size_;
        }

        if (y < 0) {
            --std::get<1>(voxel_index);
            y += this->mini_grid_size_;
        }
        else if (y >= this->mini_grid_size_) {
            ++std::get<1>(voxel_index);
            y -= this->mini_grid_size_;
        }

        if (z < 0) {
            --std::get<2>(voxel_index);
            z += this->mini_grid_size_;
        }
        else if (z >= this->mini_grid_size_) {
            ++std::get<2>(voxel_index);
            z -= this->mini_grid_size_;
        }

        return { x, y, z };
    }

    template <typename PointT, typename BaseClass>
    std::vector<PointT> VoxelSearch<PointT, BaseClass>::findKNearestNeighbors(
        const PointT& query_point, int k, float max_distance) const
    {
      //  std::cout << " Noez" << std::endl;

        // Создаём приоритетную очередь: top() = самая дальняя из хранимых,
        // чтобы при добавлении новой (более близкой) точки можно было вытеснять самую далёкую.
        using Neighbor = std::pair<float, PointT>;
        auto cmp = [](const Neighbor& left, const Neighbor& right) {
            // При таком cmp сверху очереди (top) окажется элемент с самым БОЛЬШИМ distance
            // (то есть реализуется max-heap по first).
            return left.first > right.first;
            };
        std::priority_queue<Neighbor, std::vector<Neighbor>, decltype(cmp)> neighbors{ cmp };

        // Ищем индекс крупного вокселя и мини-вокселя
        auto voxel_index = this->getVoxelIndex(query_point);
        auto mini_voxel_index = this->getMiniVoxelIndex(query_point, voxel_index);

        int layer_count = static_cast<int>(std::ceil(max_distance / this->mini_voxel_size_));
        float max_distance_squared = max_distance * max_distance;

        // Локальная функция для добавления точек из конкретной мини-ячейки (nx, ny, nz)
        auto tryAddPointsFromMiniVoxel = [&](int nx, int ny, int nz, std::tuple<int, int, int> vox_idx)
            {
                // Проверяем, есть ли вообще такой "крупный" воксель
                auto voxel_it = this->voxel_map_.find(vox_idx);
                if (voxel_it == this->voxel_map_.end()) return;

                // Внутри крупного вокселя ищем нужную мини-ячейку
                const auto& mini_voxel_map = voxel_it->second;
                auto mini_voxel_it = mini_voxel_map.find({ nx, ny, nz });
                if (mini_voxel_it == mini_voxel_map.end()) return;

                // Перебираем все точки и при необходимости добавляем в очередь
                const auto& points = mini_voxel_it->second;
                for (const auto& point : points) {
                    float dx = query_point.x - point.x;
                    float dy = query_point.y - point.y;
                    float dz = query_point.z - point.z;
                    float distance_squared = dx * dx + dy * dy + dz * dz;

                    if (distance_squared > 0 && distance_squared <= max_distance_squared) {
                        float distance = std::sqrt(distance_squared);

                        // Если ещё не набрали k точек, добавляем
                        if ((int)neighbors.size() < k) {
                            neighbors.emplace(distance, point);
                        }
                        // Если уже k точек, проверяем, не ближе ли эта точка,
                        // чем самая дальняя из уже имеющихся
                        else if (distance < neighbors.top().first) {
                            neighbors.pop(); // вытесняем самую дальнюю
                            neighbors.emplace(distance, point);
                        }
                    }
                }
            };

        // Послойно обходим окрестности в пределах layer_count (по мини-индексам)
        for (int layer = 0; layer <= layer_count; ++layer) {
            // Перебираем dx, dy, dz от -layer до layer
            // и берём только "оболочку", где max(|dx|, |dy|, |dz|) == layer
            for (int dx = -layer; dx <= layer; ++dx) {
                for (int dy = -layer; dy <= layer; ++dy) {
                    for (int dz = -layer; dz <= layer; ++dz) {
                        if (std::max({ std::abs(dx), std::abs(dy), std::abs(dz) }) != layer) {
                            continue;
                        }

                        // Корректируем индексы мини-вокселя и при необходимости - крупного
                        auto neighbor_voxel_index = voxel_index;
                        auto [nx, ny, nz] = adjustIndices(
                            std::get<0>(mini_voxel_index) + dx,
                            std::get<1>(mini_voxel_index) + dy,
                            std::get<2>(mini_voxel_index) + dz,
                            neighbor_voxel_index
                        );

                        // Пытаемся добавить точки из этой мини-ячейки
                        tryAddPointsFromMiniVoxel(nx, ny, nz, neighbor_voxel_index);
                    }
                }
            }

            // После обработки "слоя" проверяем, достаточно ли уже точек
            if ((int)neighbors.size() >= k) {
                break;
            }
        }

        // Переносим данные из очереди в вектор
        // В очереди top() = самая дальняя из k, поэтому распакуем,
        // а затем при желании развернём порядок.
        std::vector<PointT> result;
        result.reserve(neighbors.size());
        while (!neighbors.empty()) {
            result.push_back(neighbors.top().second);
            neighbors.pop();
        }
        // Если нужно упорядочить от ближайшей к дальней, то разворачиваем
        std::reverse(result.begin(), result.end());

       // std::cout << " sosi" << std::endl;
        return result;
    }

    template <typename PointT, typename BaseClass>
    std::vector<PointT> VoxelSearch<PointT, BaseClass>::findAllPointsWithinRadius(
        const PointT& query_point, float max_distance) const
    {
        // Тут можно оставить как есть (нет необходимости прерывать поиск досрочно)
        // Либо аналогично обойти послойно - но в данном случае это не даёт выигрыша
        // по логике досрочного выхода, поэтому оставим как есть.
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
                            (query_point.x - point.x) * (query_point.x - point.x) +
                            (query_point.y - point.y) * (query_point.y - point.y) +
                            (query_point.z - point.z) * (query_point.z - point.z);
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
