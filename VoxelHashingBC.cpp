
/*#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cassert>
#include <omp.h> 
#include "ArrayHashing/voxel_hashing_array.h"
#include "ArrayHashing/array_search.h"  // Подключаем ArraySearch

int main() {
    std::string file_path = "C:\\test.ply";

    // Загрузка облака точек
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile(file_path, *cloud) == -1) {
        std::cerr << "Error loading file " << file_path << std::endl;
        return -1;
    }

    std::cout << "Loaded " << cloud->points.size() << " points from file." << std::endl;

    // Проверка наличия точек в облаке
    if (cloud->points.empty()) {
        std::cerr << "The point cloud is empty." << std::endl;
        return -1;
    }

    // Параметры вокселей
    float voxel_size = 0.5f;
    float mini_voxel_size = voxel_size / 10;
    constexpr std::size_t mini_grid_size = 10;

    // Инициализация массива вокселей
    voxelStruct::ArraySearch<pcl::PointXYZ, mini_grid_size> voxelHashingArray(voxel_size, mini_voxel_size);

    // Обработка точек облака
    std::size_t num_points = cloud->points.size();
    int progress_step = 100000;

#pragma omp parallel for
    for (std::size_t i = 0; i < num_points; ++i) {
        voxelHashingArray.addPoint(cloud->points[i]);
        if (i % progress_step == 0) {
#pragma omp critical
            std::cout << "Progress: " << i << " points processed." << std::endl;
        }
    }

    std::cout << "All points have been added to the VoxelHashingArray structure." << std::endl;


    // Используем первую точку из облака точек как query_point
    pcl::PointXYZ query_point = cloud->points[0];
    int k = 5;  // Количество ближайших соседей
    float max_distance = 0.5f;  // Максимальное расстояние для поиска соседей

    // Находим ближайшие соседи к query_point
    std::vector<pcl::PointXYZ> neighbors = voxelHashingArray.findAllPointsWithinRadius(query_point, max_distance);

    // Выводим найденные соседние точки
    std::cout << "Found " << neighbors.size() << " neighbors for the query point ("
        << query_point.x << ", " << query_point.y << ", " << query_point.z << "):" << std::endl;
    for (const auto& neighbor : neighbors) {
        std::cout << "(" << neighbor.x << ", " << neighbor.y << ", " << neighbor.z << ")" << std::endl;
    }

    return 0;
}

*/
/*
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "MapHashing/voxel_hashing.h"
#include "MapHashing/voxel_search.h"  

int main() {
    // Указываем путь к файлу с облаком точек
    std::string file_path = "C:\\test.ply";

    // Загружаем облако точек
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile(file_path, *cloud) == -1) {
        std::cerr << "Error loading file " << file_path << std::endl;
        return -1;
    }

    // Выводим количество загруженных точек
    std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

    // Задаем параметры для VoxelHashing
    float voxel_size = 0.5f;
    float mini_voxel_size = 0.05f;
    int mini_grid_size = 10;

    // Создаем объект VoxelSearch (наследник VoxelHashing)
    voxelStruct::VoxelSearch<pcl::PointXYZ> voxelSearch(voxel_size, mini_voxel_size, mini_grid_size);

    // Добавляем все точки из облака в структуру VoxelSearch
    for (const auto& point : cloud->points) {
        voxelSearch.addPoint(point);
    }

    std::cout << "All points have been added to the VoxelSearch structure." << std::endl;

    // Пример поиска k ближайших соседей для первой точки в облаке
    if (!cloud->points.empty()) {
        pcl::PointXYZ query_point = cloud->points[0];
        int k = 5;
        float max_distance = 0.5f;

        // Ищем k ближайших соседей
        auto neighbors_k_nearest = voxelSearch.findKNearestNeighbors(query_point, k, max_distance);
        std::cout << "Found " << neighbors_k_nearest.size() << " nearest neighbors for the first point." << std::endl;

        // Выводим найденные точки
        for (const auto& neighbor : neighbors_k_nearest) {
            std::cout << "Nearest neighbor point: (" << neighbor.x << ", " << neighbor.y << ", " << neighbor.z << ")" << std::endl;
        }

        // Пример поиска всех соседей в радиусе
        auto neighbors_within_radius = voxelSearch.findAllPointsWithinRadius(query_point, max_distance);
        std::cout << "Found " << neighbors_within_radius.size() << " neighbors within radius " << max_distance << " for the first point." << std::endl;

        // Выводим найденные точки
        for (const auto& neighbor : neighbors_within_radius) {
            std::cout << "Neighbor within radius: (" << neighbor.x << ", " << neighbor.y << ", " << neighbor.z << ")" << std::endl;
        }
    }

    return 0;
}
*/
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "ArrayHashing/voxel_index_array.h"
#include "ArrayHashing/voxel_center_array.h"
#include "ArrayHashing/voxel_mid_point_array.h"

int main() {
    // Создаем точки для теста
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->push_back(pcl::PointXYZ(0.1, 0.1, 0.1));
    cloud->push_back(pcl::PointXYZ(0.2, 0.2, 0.2));
    cloud->push_back(pcl::PointXYZ(0.8, 0.8, 0.8));
    cloud->push_back(pcl::PointXYZ(0.5, 0.5, 0.5));
    cloud->push_back(pcl::PointXYZ(1.1, 1.1, 1.1));

    // Задаем параметры для теста
    float voxel_size = 1.0f;
    float mini_voxel_size = 0.5f;
    constexpr std::size_t mini_grid_size = 2;

    // --- Тестируем VoxelIndexArray ---
    voxelStruct::VoxelIndexArray<pcl::PointXYZ, mini_grid_size> voxelIndexArray(voxel_size, mini_voxel_size);

    for (const auto& point : cloud->points) {
        voxelIndexArray.addPoint(point);
    }

    auto voxel_index = std::make_tuple(0, 0, 0);
    auto mini_voxel_index = std::make_tuple(0, 0, 0);

    std::cout << "VoxelIndexArray - :" << std::endl;
    for (const auto& index : voxelIndexArray.getIndicesInMiniVoxel(voxel_index, mini_voxel_index)) {
        std::cout << "Point index: (" << std::get<0>(index) << ", " << std::get<1>(index) << ", " << std::get<2>(index) << ")" << std::endl;
    }

    // --- Тестируем VoxelCenterArray ---
    voxelStruct::VoxelCenterArray<pcl::PointXYZ, mini_grid_size> voxelCenterArray(voxel_size, mini_voxel_size);

    for (const auto& point : cloud->points) {
        voxelCenterArray.addPoint(point);
    }

    std::cout << "\nVoxelCenterArray" << std::endl;
    auto center = voxelCenterArray.getCenterInMiniVoxel(voxel_index, mini_voxel_index);
    std::cout << "Center: (" << center.x << ", " << center.y << ", " << center.z << ")" << std::endl;

    // --- Тестируем VoxelMidPointArray ---
    voxelStruct::VoxelMidPointArray<pcl::PointXYZ, mini_grid_size> voxelMidPointArray(voxel_size, mini_voxel_size);

    for (const auto& point : cloud->points) {
        voxelMidPointArray.addPoint(point);
    }

    std::cout << "\nVoxelMidPointArray " << std::endl;
    auto mid_point = voxelMidPointArray.getMidPointInMiniVoxel(voxel_index, mini_voxel_index);
    std::cout << "Midpoint: (" << mid_point.x << ", " << mid_point.y << ", " << mid_point.z << ")" << std::endl;

    return 0;
}
