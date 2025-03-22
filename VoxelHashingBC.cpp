#include <iostream>
#include <chrono>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "MapHashing/voxel_search.h"
#include "ArrayHashing/array_search.h"

using PointT = pcl::PointXYZ;

void test_with_voxel_search(const pcl::PointCloud<PointT>::Ptr& cloud) {
    voxelStruct::VoxelSearch<PointT> voxel_search(0.5f, 0.125f, 4);

    auto start = std::chrono::high_resolution_clock::now();
    for (const auto& point : cloud->points) {
        voxel_search.addPoint(point);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "VoxelSearch Insertion Time: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

    // Measure nearest neighbor search time
    PointT query = cloud->points.front();
    start = std::chrono::high_resolution_clock::now();
    auto neighbors = voxel_search.findKNearestNeighbors(query, 1, 10.0f);
    end = std::chrono::high_resolution_clock::now();
    std::cout << "VoxelSearch Nearest Neighbor Search Time: "
        << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds" << std::endl;
}

void test_with_array_search(const pcl::PointCloud<PointT>::Ptr& cloud) {
    voxelStruct::ArraySearch<PointT, 4> array_search(0.5f, 0.125f);

    auto start = std::chrono::high_resolution_clock::now();
    for (const auto& point : cloud->points) {
        array_search.addPoint(point);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "ArraySearch Insertion Time: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

    // Measure nearest neighbor search time
    PointT query = cloud->points.front();
    start = std::chrono::high_resolution_clock::now();
    auto neighbors = array_search.findKNearestNeighbors(query, 1, 10.0f);
    end = std::chrono::high_resolution_clock::now();
    std::cout << "ArraySearch Nearest Neighbor Search Time: "
        << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds" << std::endl;
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

    // Measure nearest neighbor search time
    PointT query = cloud->points.front();
    std::vector<int> indices;
    std::vector<float> distances;
    start = std::chrono::high_resolution_clock::now();
    octree.nearestKSearch(query, 1, indices, distances);
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Octree Search Time: "
        << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds" << std::endl;
}

void test_with_kdtree(const pcl::PointCloud<PointT>::Ptr& cloud) {
    pcl::KdTreeFLANN<PointT> kdtree;

    auto start = std::chrono::high_resolution_clock::now();
    kdtree.setInputCloud(cloud);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "KDTree Insertion Time: "
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;

    // Measure nearest neighbor search time
    PointT query = cloud->points.front();
    std::vector<int> indices;
    std::vector<float> distances;
    start = std::chrono::high_resolution_clock::now();
    kdtree.nearestKSearch(query, 1, indices, distances);
    end = std::chrono::high_resolution_clock::now();
    std::cout << "KDTree Search Time: "
        << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds" << std::endl;
}

void handle_ply_error(const std::string& file_path) {
    std::cerr << "[Warning] Skipping unsupported properties in file: " << file_path << std::endl;
}

int main() {
    std::string base_path = "C:\\tests\\test";
    std::string extension = ".ply";

    for (int i = 1; i < 5; ++i) {
        std::string file_path = base_path + std::to_string(i) + extension;
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

        try {
            if (pcl::io::loadPLYFile(file_path, *cloud) == -1) {
                throw std::runtime_error("Error: Could not load file.");
            }

            std::cout << "\nTesting file: " << file_path << std::endl;
            std::cout << "Number of points: " << cloud->size() << std::endl;  // Вывод количества точек

            test_with_voxel_search(cloud);
            test_with_array_search(cloud);
            test_with_octree(cloud);
            test_with_kdtree(cloud);
        }
        catch (const std::exception& e) {
            handle_ply_error(file_path);
        }
    }

    return 0;
}
