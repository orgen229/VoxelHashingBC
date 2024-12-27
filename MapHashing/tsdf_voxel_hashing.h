#ifndef TSDF_VOXEL_HASHING_H
#define TSDF_VOXEL_HASHING_H

#include "voxel_search.h" // Подключение VoxelSearch
#include <unordered_map>
#include <tuple>

namespace voxelStruct {

    template <typename PointT>
    class TSDFVoxelHashing : public VoxelSearch<PointT> {
    public:
        /**
         * @param voxel_size Размер вокселя
         * @param mini_voxel_size Размер мини-вокселя
         * @param truncation_distance Порог отсечения для TSDF
         */
        TSDFVoxelHashing(float voxel_size, float mini_voxel_size, float truncation_distance);

        /**
         * Рассчитать TSDF для всех мини-вокселей после того,
         * как все точки были добавлены через addPoint()
         * @param camera_origin Координаты камеры
         */
        void calculateTSDF(const PointT& camera_origin);

        /**
         * Получить значение TSDF в заданной точке
         * @param query_point Точка, для которой нужно получить значение TSDF
         * @return Значение TSDF или truncation_distance_, если данных нет
         */
        float getTSDFValueAt(const PointT& query_point) const;

    protected:
        float truncation_distance_;

        // Основная структура данных для хранения TSDF значений
        std::unordered_map<
            std::tuple<int, int, int>, // Индекс основного вокселя
            std::unordered_map<std::tuple<int, int, int>, float, VoxelHash>, // TSDF для каждого мини-вокселя
            VoxelHash
        > voxel_tsdf_map_;

        // Метод для расчёта TSDF значений для одного вокселя
        void calculateTSDFForVoxel(const std::tuple<int, int, int>& voxel_index, const PointT& camera_origin);
    };

}

#include "impl/tsdf_voxel_hashing.hpp"

#endif // TSDF_VOXEL_HASHING_H
