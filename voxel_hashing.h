#include <unordered_map>
#include <vector>
#include <tuple>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace voxelStruct {

    struct VoxelHash {
        std::size_t operator()(const std::tuple<int, int, int>& key) const {
            return std::get<0>(key) + 31 * std::get<1>(key) + 17 * std::get<2>(key);
        }
    };

    template <typename PointT>
    class VoxelHashing {
    public:
        std::unordered_map<
            std::tuple<int, int, int>,
            std::unordered_map<std::tuple<int, int, int>, std::vector<PointT>, VoxelHash>,
            VoxelHash
        > voxel_map_;

        VoxelHashing(float voxel_size, float mini_voxel_size, int mini_grid_size);

        virtual void addPoint(const PointT& point);
        virtual bool IsPointInVoxel(const PointT& point);
        virtual std::vector<PointT> selectAllPointsFromVoxel(const std::tuple<int, int, int>& voxel_index);

    protected:
        float voxel_size_, mini_voxel_size_;
        int mini_grid_size_;

        

        virtual std::tuple<int, int, int> getVoxelIndex(const PointT& point) const;
        virtual std::tuple<int, int, int> getMiniVoxelIndex(const PointT& point, const std::tuple<int, int, int>& voxel_index) const;
    };
}