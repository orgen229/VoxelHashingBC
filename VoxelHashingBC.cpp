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


#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include "ArrayHashing/tsdf_voxel_hashing_array.h" 

int main(int argc, char** argv) {
    // Load the point cloud using PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPLYFile<pcl::PointXYZ>("C:\\test2.ply", *cloud) == -1) {
        std::cerr << "Could not load file C:\\test2.ply" << std::endl;
        return -1;
    }

    std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

    if (cloud->points.empty()) {
        std::cerr << "The cloud is empty." << std::endl;
        return -1;
    }

    // TSDF and grid parameters
    const float voxel_size = 0.05f;
    const float truncation_distance = 0.1f;
    const std::size_t mini_grid_size = 4;

    // Create TSDFVoxelHashingArray with pcl::PointXYZ
    voxelStruct::TSDFVoxelHashingArray<pcl::PointXYZ, mini_grid_size> tsdf(
        voxel_size, voxel_size / (float)mini_grid_size, truncation_distance
    );

    // Integrate the point cloud into TSDF
    for (const auto& p : cloud->points) {
        tsdf.integratePoint(p, 1.0f); // weight = 1.0
    }

    // Use the first point from the cloud as the query point
    pcl::PointXYZ query_point = cloud->points.front();

    // Query TSDF value at the query_point
    float tsdf_value = tsdf.getTSDFValueAt(query_point);
    std::cout << "TSDF value at (" << query_point.x << "," << query_point.y << "," << query_point.z << "): " << tsdf_value << std::endl;

    // Find the closest measured point to query_point
    pcl::PointXYZ closest_point = tsdf.getClosestMeasuredPoint(query_point);
    std::cout << "Closest measured point to (" << query_point.x << "," << query_point.y << "," << query_point.z << "): ("
        << closest_point.x << ", "
        << closest_point.y << ", "
        << closest_point.z << ")" << std::endl;

    return 0;
}