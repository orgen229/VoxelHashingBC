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

/*
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "voxel_hashing_array.h"

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
    constexpr std::size_t mini_grid_size = 10;
    voxelStruct::VoxelHashingArray<pcl::PointXYZ, mini_grid_size> voxelHashingArray(voxel_size, mini_voxel_size);

    
    for (std::size_t i = 0; i < std::min(static_cast<std::size_t>(100), cloud->points.size()); ++i) {
        voxelHashingArray.addPoint(cloud->points[i]);
    }

    std::cout << "First 100 points have been added to the VoxelHashingArray structure." << std::endl;

    
    int total_point_count = 0; 
    for (const auto& voxel_pair : voxelHashingArray.voxel_map_) {
        const auto& mini_voxel_array = voxel_pair.second;

        for (const auto& mini_voxel_x : mini_voxel_array) {
            for (const auto& mini_voxel_y : mini_voxel_x) {
                for (const auto& mini_voxel_z : mini_voxel_y) {
                    total_point_count += mini_voxel_z.size();
                }
            }
        }
    }

    std::cout << "Total number of points in all voxels: " << total_point_count << std::endl;

    return 0;
}


*/