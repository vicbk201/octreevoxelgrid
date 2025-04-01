#ifndef GROUND_REMOVAL_PROCESSOR_H
#define GROUND_REMOVAL_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

struct GroundRemovalResult {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double runtime_ms;
    // 新增：儲存第一個成功分割得到的地面平面係數
    Eigen::Vector4f ground_coefficients;
};

class GroundRemovalProcessor {
public:
    static GroundRemovalResult removeGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
};

#endif // GROUND_REMOVAL_PROCESSOR_H
