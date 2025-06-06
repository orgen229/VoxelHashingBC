#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>
#include <string>


#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZ;

int main() {

    std::string file_path = "D:\\tests\\test11.ply";

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if (pcl::io::loadPLYFile(file_path, *cloud) == -1) {
        std::cerr << "Не удалось загрузить файл: " << file_path << std::endl;
        return -1;
    }


    std::size_t total_points = cloud->size();
    std::cout << "Total points: " << total_points << std::endl;


    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (const auto& p : cloud->points) {
        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
        if (p.z < min_z) min_z = p.z;
        if (p.z > max_z) max_z = p.z;
    }

    float volume = (max_x - min_x) * (max_y - min_y) * (max_z - min_z);
    float density = volume > 0.0f ? static_cast<float>(total_points) / volume : 0.0f;
    std::cout << "Average point density: " << density << " points per unit volume" << std::endl;


    const PointT& first_pt = cloud->points.front();
    int vx = static_cast<int>(std::floor(first_pt.x));
    int vy = static_cast<int>(std::floor(first_pt.y));
    int vz = static_cast<int>(std::floor(first_pt.z));

    std::size_t voxel_count = 0;
    for (const auto& p : cloud->points) {
        if (p.x >= vx && p.x < vx + 1 &&
            p.y >= vy && p.y < vy + 1 &&
            p.z >= vz && p.z < vz + 1) {
            ++voxel_count;
        }
    }

    std::cout << "Points in 1x1x1 voxel containing the first point: " << voxel_count << std::endl;

    return 0;
}
