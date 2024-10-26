#include "voxel_hashing_array.h"

int main() {
    std::cout << "Testing with pcl::PointXYZ:" << std::endl;
    voxelStruct::VoxelHashing<pcl::PointXYZ, 10> voxel_hashing_xyz(0.1f, 0.01f);

    pcl::PointXYZ point1(0.05f, 0.05f, 0.05f);
    pcl::PointXYZ point2(0.07f, 0.05f, 0.05f);
    pcl::PointXYZ point3(0.08f, 0.06f, 0.05f);
    voxel_hashing_xyz.addPoint(point1);
    voxel_hashing_xyz.addPoint(point2);
    voxel_hashing_xyz.addPoint(point3);

    std::cout << "Is point1 in voxel? "
        << (voxel_hashing_xyz.IsPointInVoxel(point1) ? "Yes" : "No") << std::endl;
    std::cout << "Is point2 in voxel? "
        << (voxel_hashing_xyz.IsPointInVoxel(point2) ? "Yes" : "No") << std::endl;
    std::cout << "Is point3 in voxel? "
        << (voxel_hashing_xyz.IsPointInVoxel(point3) ? "Yes" : "No") << std::endl;

    std::tuple<int, int, int> voxel_index_xyz = std::make_tuple(0, 0, 0);
    auto points_xyz = voxel_hashing_xyz.selectAllPointsFromVoxel(voxel_index_xyz);

    std::cout << "Extracted points (PointXYZ):" << std::endl;
    for (const auto& p : points_xyz) {
        std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
    }

    std::cout << "\nTesting with pcl::PointXYZRGB:" << std::endl;
    voxelStruct::VoxelHashing<pcl::PointXYZRGB, 10> voxel_hashing_xyzrgb(0.1f, 0.01f);

    pcl::PointXYZRGB point_rgb1;
    point_rgb1.x = 0.05f; point_rgb1.y = 0.05f; point_rgb1.z = 0.05f;
    point_rgb1.r = 255; point_rgb1.g = 0; point_rgb1.b = 0;

    pcl::PointXYZRGB point_rgb2;
    point_rgb2.x = 0.06f; point_rgb2.y = 0.05f; point_rgb2.z = 0.05f;
    point_rgb2.r = 0; point_rgb2.g = 255; point_rgb2.b = 0;

    voxel_hashing_xyzrgb.addPoint(point_rgb1);
    voxel_hashing_xyzrgb.addPoint(point_rgb2);

    std::cout << "Is point_rgb1 in voxel? "
        << (voxel_hashing_xyzrgb.IsPointInVoxel(point_rgb1) ? "Yes" : "No") << std::endl;
    std::cout << "Is point_rgb2 in voxel? "
        << (voxel_hashing_xyzrgb.IsPointInVoxel(point_rgb2) ? "Yes" : "No") << std::endl;

    std::tuple<int, int, int> voxel_index_xyzrgb = std::make_tuple(0, 0, 0);
    auto points_xyzrgb = voxel_hashing_xyzrgb.selectAllPointsFromVoxel(voxel_index_xyzrgb);

    std::cout << "Extracted points (PointXYZRGB):" << std::endl;
    for (const auto& p : points_xyzrgb) {
        std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ") - Color: ("
            << static_cast<int>(p.r) << ", "
            << static_cast<int>(p.g) << ", "
            << static_cast<int>(p.b) << ")" << std::endl;
    }

    return 0;
}
