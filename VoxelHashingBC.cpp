/*
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "voxel_hashing.h"

int main() {
    
    std::string file_path = "C:\\test.ply";

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile(file_path, *cloud) == -1) {
        std::cerr << "Error loading file " << file_path << std::endl;
        return -1;
    }

    std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

    
    float voxel_size = 0.1f;
    float mini_voxel_size = 0.05f;
    int mini_grid_size = 10;
    voxelStruct::VoxelHashing<pcl::PointXYZ> voxelHashing(voxel_size, mini_voxel_size, mini_grid_size);

    
    for (const auto& point : cloud->points) {
        voxelHashing.addPoint(point);
    }

    std::cout << "All points have been added to the VoxelHashing structure." << std::endl;

   
    for (const auto& voxel_pair : voxelHashing.voxel_map_) {
        const auto& voxel_index = voxel_pair.first;
        const auto& mini_voxel_map = voxel_pair.second;

        int point_count = 0;
        for (const auto& mini_voxel_pair : mini_voxel_map) {
            point_count += mini_voxel_pair.second.size();
        }

        std::cout << "Voxel (" << std::get<0>(voxel_index) << ", " << std::get<1>(voxel_index) << ", " << std::get<2>(voxel_index) << ") has " << point_count << " points." << std::endl;
    }

    return 0;
}
*/
#include <iostream>
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

/*
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "voxel_hashing.h"
#include "voxel_search.h"  // Подключаем новый файл для поиска соседей

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
    float voxel_size = 0.1f;
    float mini_voxel_size = 0.05f;
    int mini_grid_size = 10;

    // Создаем объект VoxelSearch (наследник VoxelHashing)
    voxelStruct::VoxelSearch<pcl::PointXYZ> voxelSearch(voxel_size, mini_voxel_size, mini_grid_size);

    // Добавляем все точки из облака в структуру VoxelSearch
    for (const auto& point : cloud->points) {
        voxelSearch.addPoint(point);
    }

    std::cout << "All points have been added to the VoxelSearch structure." << std::endl;

    // Пример поиска соседей для первой точки в облаке
    if (!cloud->points.empty()) {
        pcl::PointXYZ query_point = cloud->points[0];
        auto neighbors = voxelSearch.findNeighborPoints(query_point, 1);  // Ищем соседей на расстоянии 1

        std::cout << "Found " << neighbors.size() << " neighbors for the first point." << std::endl;

        // Выводим соседние точки
        for (const auto& neighbor : neighbors) {
            std::cout << "Neighbor point: (" << neighbor.x << ", " << neighbor.y << ", " << neighbor.z << ")" << std::endl;
        }
    }

    return 0;
}
*/