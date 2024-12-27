#ifndef TSDF_VOXEL_HASHING_H
#define TSDF_VOXEL_HASHING_H

#include "voxel_search.h" // ����������� VoxelSearch
#include <unordered_map>
#include <tuple>

namespace voxelStruct {

    template <typename PointT>
    class TSDFVoxelHashing : public VoxelSearch<PointT> {
    public:
        /**
         * @param voxel_size ������ �������
         * @param mini_voxel_size ������ ����-�������
         * @param truncation_distance ����� ��������� ��� TSDF
         */
        TSDFVoxelHashing(float voxel_size, float mini_voxel_size, float truncation_distance);

        /**
         * ���������� TSDF ��� ���� ����-�������� ����� ����,
         * ��� ��� ����� ���� ��������� ����� addPoint()
         * @param camera_origin ���������� ������
         */
        void calculateTSDF(const PointT& camera_origin);

        /**
         * �������� �������� TSDF � �������� �����
         * @param query_point �����, ��� ������� ����� �������� �������� TSDF
         * @return �������� TSDF ��� truncation_distance_, ���� ������ ���
         */
        float getTSDFValueAt(const PointT& query_point) const;

    protected:
        float truncation_distance_;

        // �������� ��������� ������ ��� �������� TSDF ��������
        std::unordered_map<
            std::tuple<int, int, int>, // ������ ��������� �������
            std::unordered_map<std::tuple<int, int, int>, float, VoxelHash>, // TSDF ��� ������� ����-�������
            VoxelHash
        > voxel_tsdf_map_;

        // ����� ��� ������� TSDF �������� ��� ������ �������
        void calculateTSDFForVoxel(const std::tuple<int, int, int>& voxel_index, const PointT& camera_origin);
    };

}

#include "impl/tsdf_voxel_hashing.hpp"

#endif // TSDF_VOXEL_HASHING_H
