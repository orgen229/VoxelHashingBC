#include <iostream>
#include <fstream>
#include <chrono>
#include <limits>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

// PCL
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>

// Ваши библиотеки
#include "MapHashing/voxel_search.h"
#include "ArrayHashing/array_search.h"

// Для WinAPI (замер памяти)
#include <windows.h>
#include <psapi.h>

using PointT = pcl::PointXYZ;

// Структура для хранения результатов тестового запуска
struct TestResult {
    std::string testName;
    std::string plyFile;
    float pointDensity;
    float voxel_size;         // для VoxelSearch и ArraySearch
    int mini_voxels_count;    // для VoxelSearch и ArraySearch
    float mini_voxel_size;    // для VoxelSearch и ArraySearch
    float resolution;         // для Octree
    long insertionTime_ms;
    long searchTime_us;
    SIZE_T memoryUsed_bytes;  // разница в WorkingSet до и после теста
};

// Функция для получения текущего использования памяти (Working Set) в байтах
SIZE_T getCurrentMemoryUsage() {
    PROCESS_MEMORY_COUNTERS_EX pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&pmc), sizeof(pmc))) {
        return pmc.WorkingSetSize;
    }
    return 0;
}

// Тест для VoxelSearch (runtime-параметр mini_voxels_count)
TestResult test_voxel_search(const pcl::PointCloud<PointT>::Ptr& cloud, float voxel_size, int mini_voxels_count) {
    TestResult result;
    result.testName = "VoxelSearch";
    result.voxel_size = voxel_size;
    result.mini_voxels_count = mini_voxels_count;
    result.mini_voxel_size = voxel_size / mini_voxels_count;

    SIZE_T memBefore = getCurrentMemoryUsage();

    voxelStruct::VoxelSearch<PointT> voxel_search(voxel_size, result.mini_voxel_size, mini_voxels_count);

    auto start = std::chrono::high_resolution_clock::now();
    for (const auto& point : cloud->points) {
        voxel_search.addPoint(point);
    }
    auto end = std::chrono::high_resolution_clock::now();
    result.insertionTime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // Замер времени поиска ближайшего соседа
    PointT query = cloud->points.front();
    start = std::chrono::high_resolution_clock::now();
    auto neighbors = voxel_search.findKNearestNeighbors(query, 1, 10.0f);
    end = std::chrono::high_resolution_clock::now();
    result.searchTime_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    SIZE_T memAfter = getCurrentMemoryUsage();
    result.memoryUsed_bytes = (memAfter > memBefore ? memAfter - memBefore : 0);

    return result;
}

// Шаблонная функция для тестирования ArraySearch с заданным числом минивокселей
template <int miniVoxels>
TestResult test_array_search_impl(const pcl::PointCloud<PointT>::Ptr& cloud, float voxel_size) {
    TestResult result;
    result.testName = "ArraySearch";
    result.voxel_size = voxel_size;
    result.mini_voxels_count = miniVoxels;
    result.mini_voxel_size = voxel_size / miniVoxels;

    SIZE_T memBefore = getCurrentMemoryUsage();

    voxelStruct::ArraySearch<PointT, miniVoxels> array_search(voxel_size, result.mini_voxel_size);

    auto start = std::chrono::high_resolution_clock::now();
    for (const auto& point : cloud->points) {
        array_search.addPoint(point);
    }
    auto end = std::chrono::high_resolution_clock::now();
    result.insertionTime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // Замер времени поиска ближайшего соседа
    PointT query = cloud->points.front();
    start = std::chrono::high_resolution_clock::now();
    auto neighbors = array_search.findKNearestNeighbors(query, 1, 10.0f);
    end = std::chrono::high_resolution_clock::now();
    result.searchTime_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    SIZE_T memAfter = getCurrentMemoryUsage();
    result.memoryUsed_bytes = (memAfter > memBefore ? memAfter - memBefore : 0);

    return result;
}

// Обёртка для test_array_search, выбирающая нужное значение шаблонного параметра
TestResult test_array_search(const pcl::PointCloud<PointT>::Ptr& cloud, float voxel_size, int mini_voxels_count) {
    switch (mini_voxels_count) {
    case 1:
        return test_array_search_impl<1>(cloud, voxel_size);
    case 4:
        return test_array_search_impl<4>(cloud, voxel_size);
    case 16:
        return test_array_search_impl<16>(cloud, voxel_size);
    case 64:
        return test_array_search_impl<64>(cloud, voxel_size);
    case 256:
        return test_array_search_impl<256>(cloud, voxel_size);
    default:
        throw std::runtime_error("Unsupported mini_voxels_count value.");
    }
}

// Тест для Octree (параметр resolution изменяется отдельно)
TestResult test_octree(const pcl::PointCloud<PointT>::Ptr& cloud, float resolution) {
    TestResult result;
    result.testName = "Octree";
    result.resolution = resolution;

    SIZE_T memBefore = getCurrentMemoryUsage();

    pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
    auto start = std::chrono::high_resolution_clock::now();
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    auto end = std::chrono::high_resolution_clock::now();
    result.insertionTime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // Замер поиска ближайшего соседа
    PointT query = cloud->points.front();
    std::vector<int> indices;
    std::vector<float> distances;
    start = std::chrono::high_resolution_clock::now();
    octree.nearestKSearch(query, 1, indices, distances);
    end = std::chrono::high_resolution_clock::now();
    result.searchTime_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    SIZE_T memAfter = getCurrentMemoryUsage();
    result.memoryUsed_bytes = (memAfter > memBefore ? memAfter - memBefore : 0);

    return result;
}

// Тест для KDTree (без параметров)
TestResult test_kdtree(const pcl::PointCloud<PointT>::Ptr& cloud) {
    TestResult result;
    result.testName = "KDTree";

    SIZE_T memBefore = getCurrentMemoryUsage();

    pcl::KdTreeFLANN<PointT> kdtree;
    auto start = std::chrono::high_resolution_clock::now();
    kdtree.setInputCloud(cloud);
    auto end = std::chrono::high_resolution_clock::now();
    result.insertionTime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // Замер поиска ближайшего соседа
    PointT query = cloud->points.front();
    std::vector<int> indices;
    std::vector<float> distances;
    start = std::chrono::high_resolution_clock::now();
    kdtree.nearestKSearch(query, 1, indices, distances);
    end = std::chrono::high_resolution_clock::now();
    result.searchTime_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    SIZE_T memAfter = getCurrentMemoryUsage();
    result.memoryUsed_bytes = (memAfter > memBefore ? memAfter - memBefore : 0);

    return result;
}

void handle_ply_error(const std::string& file_path) {
    std::cerr << "[Warning] Skipping unsupported properties in file: " << file_path << std::endl;
}

int main() {
    std::string base_path = "D:\\tests\\test";
    std::string extension = ".ply";
    std::ofstream ofs("D:\\tests\\result.txt");
    if (!ofs.is_open()) {
        std::cerr << "Cannot open result file!" << std::endl;
        return -1;
    }

    // Записываем заголовок CSV-файла
    ofs << "File;PointDensity;Test;voxel_size;mini_voxels_count;mini_voxel_size;resolution;"
        "InsertionTime_ms;SearchTime_us;MemoryUsed_bytes" << std::endl;

    // Обработка .ply файлов (например, 5 файлов)
    for (int fileIndex = 7; fileIndex <= 7; ++fileIndex) {
        std::string file_path = base_path + std::to_string(fileIndex) + extension;
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        try {
            if (pcl::io::loadPLYFile(file_path, *cloud) == -1) {
                throw std::runtime_error("Error: Could not load file.");
            }
            std::cout << "\nTesting file: " << file_path << std::endl;
            std::cout << "Number of points: " << cloud->size() << std::endl;

            // Вычисление минимальных и максимальных значений по осям
            float min_x = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::lowest();
            float min_y = std::numeric_limits<float>::max();
            float max_y = std::numeric_limits<float>::lowest();
            float min_z = std::numeric_limits<float>::max();
            float max_z = std::numeric_limits<float>::lowest();
            for (const auto& point : cloud->points) {
                if (point.x < min_x) min_x = point.x;
                if (point.x > max_x) max_x = point.x;
                if (point.y < min_y) min_y = point.y;
                if (point.y > max_y) max_y = point.y;
                if (point.z < min_z) min_z = point.z;
                if (point.z > max_z) max_z = point.z;
            }
            std::cout << "Min X: " << min_x << ", Max X: " << max_x << std::endl;
            std::cout << "Min Y: " << min_y << ", Max Y: " << max_y << std::endl;
            std::cout << "Min Z: " << min_z << ", Max Z: " << max_z << std::endl;

            float volume = (max_x - min_x) * (max_y - min_y) * (max_z - min_z);
            float density = (volume > 0) ? static_cast<float>(cloud->size()) / volume : 0;
            std::cout << "Approximate point density: " << density << " points per unit volume" << std::endl;

            // Определяем набор значений mini_voxels_count
            std::vector<int> mini_voxels_values = { 1, 4, 16, 64, 256 };

            // Для каждого mini_voxels_count диапазон voxel_size увеличивается в 5 раз
            for (size_t i = 0; i < mini_voxels_values.size(); i++) {
                int mini_voxels_count = mini_voxels_values[i];
                // scale вычисляется как 5^i
                float scale = std::pow(5.0f, static_cast<float>(i));
                // Диапазон voxel_size = [0.5*scale, 2.0*scale] с шагом 0.1*scale
                for (float voxel_size = 0.5f * scale * 4; voxel_size <= 2.0f * scale + 1e-5f; voxel_size += 0.1f * scale * 4) {
                    // Тест VoxelSearch
                    TestResult res_voxel = test_voxel_search(cloud, voxel_size, mini_voxels_count);
                    res_voxel.plyFile = file_path;
                    res_voxel.pointDensity = density;
                    ofs << res_voxel.plyFile << ";" << res_voxel.pointDensity << ";" << res_voxel.testName << ";"
                        << res_voxel.voxel_size << ";" << res_voxel.mini_voxels_count << ";" << res_voxel.mini_voxel_size << ";-;"
                        << res_voxel.insertionTime_ms << ";" << res_voxel.searchTime_us << ";" << res_voxel.memoryUsed_bytes << std::endl;

                    // Тест ArraySearch
                    TestResult res_array = test_array_search(cloud, voxel_size, mini_voxels_count);
                    res_array.plyFile = file_path;
                    res_array.pointDensity = density;
                    ofs << res_array.plyFile << ";" << res_array.pointDensity << ";" << res_array.testName << ";"
                        << res_array.voxel_size << ";" << res_array.mini_voxels_count << ";" << res_array.mini_voxel_size << ";-;"
                        << res_array.insertionTime_ms << ";" << res_array.searchTime_us << ";" << res_array.memoryUsed_bytes << std::endl;
                }
            }

            // При желании можно запускать Octree и KDTree тесты отдельно (без изменения параметров)
            TestResult res_octree = test_octree(cloud, 1.0f);
            res_octree.plyFile = file_path;
            res_octree.pointDensity = density;
            ofs << res_octree.plyFile << ";" << res_octree.pointDensity << ";" << res_octree.testName << ";-;-;-;"
                << res_octree.resolution << ";" << res_octree.insertionTime_ms << ";" << res_octree.searchTime_us << ";" << res_octree.memoryUsed_bytes << std::endl;

            TestResult res_kdtree = test_kdtree(cloud);
            res_kdtree.plyFile = file_path;
            res_kdtree.pointDensity = density;
            ofs << res_kdtree.plyFile << ";" << res_kdtree.pointDensity << ";" << res_kdtree.testName << ";-;-;-;-;"
                << res_kdtree.insertionTime_ms << ";" << res_kdtree.searchTime_us << ";" << res_kdtree.memoryUsed_bytes << std::endl;
        }
        catch (const std::exception& e) {
            handle_ply_error(file_path);
        }
    }

    ofs.close();
    std::cout << "Results saved to D:\\tests\\result.txt" << std::endl;
    return 0;
}
