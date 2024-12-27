/*
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cassert>
#include <omp.h>
#include "ArrayHashing/voxel_hashing_array.h"
#include "ArrayHashing/array_search.h"  


using namespace voxelStruct;

int main() {
    // Point type
    using PointT = pcl::PointXYZ;

    // Grid parameters
    constexpr std::size_t mini_grid_size = 4; // Mini-grid size
    float voxel_size = 1.0f;                  // Voxel size
    float mini_voxel_size = 0.25f;            // Mini-voxel size

    // Initialize the voxel hashing class
    VoxelHashingArray<PointT, mini_grid_size> voxel_hashing(voxel_size, mini_voxel_size);

    // Path to the PLY file
    std::string file_path = "C:\\test.ply";

    // Load point cloud from file
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if (pcl::io::loadPLYFile(file_path, *cloud) == -1) {
        std::cerr << "Error: Could not load file: " << file_path << std::endl;
        return -1;
    }

    std::cout << "Successfully loaded " << cloud->size() << " points from file." << std::endl;

    // Add points to the voxel structure
    std::size_t point_count = 0; // Счётчик добавленных точек
    for (const auto& point : cloud->points) {
        voxel_hashing.addPoint(point);
        point_count++;

        // Вывод каждые 100,000 точек
        if (point_count % 100000 == 0) {
            std::cout << point_count << " points have been added to the voxel structure." << std::endl;
        }
    }

    std::cout << "All " << point_count << " points have been added to the voxel structure." << std::endl;

    // Получаем индекс вокселя для первой точки
    if (!cloud->points.empty()) {
        const PointT& first_point = cloud->points[0];
        auto voxel_index = voxel_hashing.getVoxelIndex(first_point);

        // Извлекаем все точки из этого вокселя
        auto points_in_voxel = voxel_hashing.selectAllPointsFromVoxel(voxel_index);

        std::cout << "The voxel containing the first point ("
            << first_point.x << ", " << first_point.y << ", " << first_point.z
            << ") has " << points_in_voxel.size() << " points." << std::endl;

        // Вывод всех точек из вокселя
        for (const auto& point : points_in_voxel) {
            std::cout << "Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
        }
    }
    else {
        std::cout << "The point cloud is empty!" << std::endl;
    }

    return 0;
}*/
/*
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include "ArrayHashing/tsdf_voxel_hashing_array.h"
#include "ArrayHashing/array_search.h"


#include "MapHashing/tsdf_voxel_hashing.h"
#include "MapHashing/voxel_search.h"

int main() {
    // Создаем объект TSDF
    voxelStruct::TSDFVoxelHashing<pcl::PointXYZ> tsdf(0.05f, 0.05f / 4.0f, 0.03f);

    // Загружаем точки из файла
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("C:\\test2.ply", *cloud) == -1) {
        std::cerr << "Could not load file." << std::endl;
        return -1;
    }

    // Добавляем точки в TSDF
    for (const auto& p : cloud->points) {
        tsdf.addPoint(p); // Используем метод базового класса
    }

    // Вычисляем TSDF относительно камеры
    pcl::PointXYZ camera_origin(0.0f, 0.0f, 0.0f);
    tsdf.calculateTSDF(camera_origin);

    // Запрашиваем значение TSDF в произвольной точке
    pcl::PointXYZ query_point(0.1f, 0.0f, 0.0f);
    float tsdf_value = tsdf.getTSDFValueAt(query_point);
    std::cout << "TSDF value at (0.1, 0.0, 0.0): " << tsdf_value << std::endl;

    return 0;
}
*/
// Test Program for Comparing VoxelHashingArray, Octree, and KDTree

// Test Program for Comparing VoxelHashing, VoxelHashingArray, Octree, and KDTree

#include <iostream>
#include <chrono>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "MapHashing/voxel_hashing.h"
#include "ArrayHashing/voxel_hashing_array.h"

using PointT = pcl::PointXYZ;

void test_with_voxel_hashing(const pcl::PointCloud<PointT>::Ptr& cloud) {
    voxelStruct::VoxelHashing<PointT> voxel_hashing(1.0f, 0.25f, 4);

    auto start = std::chrono::high_resolution_clock::now();
    for (const auto& point : cloud->points) {
        voxel_hashing.addPoint(point);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "VoxelHashing Insertion Time: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
}

void test_with_voxel_hashing_array(const pcl::PointCloud<PointT>::Ptr& cloud) {
    constexpr std::size_t mini_grid_size = 4;
    float voxel_size = 1.0f;
    float mini_voxel_size = 0.25f;

    voxelStruct::VoxelHashingArray<PointT, mini_grid_size> voxel_hashing(voxel_size, mini_voxel_size);

    auto start = std::chrono::high_resolution_clock::now();
    for (const auto& point : cloud->points) {
        voxel_hashing.addPoint(point);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "VoxelHashingArray Insertion Time: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
}

void test_with_octree(const pcl::PointCloud<PointT>::Ptr& cloud) {
    float resolution = 1.0f;
    pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);

    auto start = std::chrono::high_resolution_clock::now();
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Octree Insertion Time: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
}

void test_with_kdtree(const pcl::PointCloud<PointT>::Ptr& cloud) {
    pcl::KdTreeFLANN<PointT> kdtree;

    auto start = std::chrono::high_resolution_clock::now();
    kdtree.setInputCloud(cloud);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "KDTree Insertion Time: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
}

int main() {
    std::string base_path = "C:\\tests\\test";
    std::string extension = ".ply";

    for (int i = 1; i <= 5; ++i) {
        std::string file_path = base_path + std::to_string(i) + extension;
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

        if (pcl::io::loadPLYFile(file_path, *cloud) == -1) {
            std::cerr << "Error: Could not load file " << file_path << std::endl;
            continue;
        }

        std::cout << "\nTesting file: " << file_path << std::endl;

        test_with_voxel_hashing(cloud);
        
        test_with_octree(cloud);
        test_with_kdtree(cloud);
        test_with_voxel_hashing_array(cloud);
    }

    return 0;
}
