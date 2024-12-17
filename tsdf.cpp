#include "tsdf.h"


#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

// ��������� ��� ����� � 3D ������������
struct Point3D {
    float x, y, z;
    Point3D(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};

// ������� ��� ���������� ���������� ����� ����� �������
float distance(const Point3D& p1, const Point3D& p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) +
        (p1.y - p2.y) * (p1.y - p2.y) +
        (p1.z - p2.z) * (p1.z - p2.z));
}

// TSDF Grid
class TSDFGrid {
private:
    std::vector<Point3D> measurements; // ����� �� ������ (��������, ������)
    float truncation_distance;         // ��������� ����������

public:
    TSDFGrid(const std::vector<Point3D>& points, float trunc_dist)
        : measurements(points), truncation_distance(trunc_dist) {}

    // 1. ������� ��� ���������� TSDF �������� ��� ����� � �����
    float computeTSDFValue(const Point3D& query_point) {
        float min_distance = std::numeric_limits<float>::max();

        // ���� ����������� ���������� �� ����� �� ���������
        for (const auto& point : measurements) {
            float dist = distance(query_point, point);
            if (dist < min_distance) {
                min_distance = dist;
            }
        }

        // ������������ �������� ���������� �� truncation_distance
        float tsdf_value = std::min(min_distance, truncation_distance);

        // ������������� ����: ������������� ������ �������, ������������� �������
        return (isInsideObject(query_point)) ? -tsdf_value : tsdf_value;
    }

    // 2. ��������, ��������� �� ����� "������" ������� (������� ������)
    bool isInsideObject(const Point3D& query_point) {
        for (const auto& point : measurements) {
            if (distance(query_point, point) < truncation_distance / 2) {
                return true; // �������, ��� ����� ������
            }
        }
        return false;
    }

    // 3. ����� ��������� ����� �� ��������� � �������� �����
    Point3D findClosestPoint(const Point3D& query_point) {
        Point3D closest_point;
        float min_distance = std::numeric_limits<float>::max();

        for (const auto& point : measurements) {
            float dist = distance(query_point, point);
            if (dist < min_distance) {
                min_distance = dist;
                closest_point = point;
            }
        }
        return closest_point;
    }
};

int main() {
    // ������: ����� �� ������ ������
    std::vector<Point3D> cloud_points = {
        {1.0, 2.0, 3.0}, {2.0, 3.0, 4.0}, {3.0, 3.0, 2.0}, {4.0, 2.0, 1.0}
    };

    // ������ TSDF Grid � ��������� ����������� 2.0
    TSDFGrid tsdf(cloud_points, 2.0);

    // ����� ��� �������
    Point3D query(2.5, 3.0, 3.0);

    // ��������� TSDF �������� ��� �����
    float tsdf_value = tsdf.computeTSDFValue(query);
    std::cout << "TSDF value at query point: " << tsdf_value << std::endl;

    // ������� ��������� ����� ���������
    Point3D closest_point = tsdf.findClosestPoint(query);
    std::cout << "Closest point: (" << closest_point.x << ", "
        << closest_point.y << ", " << closest_point.z << ")" << std::endl;

    return 0;
}
