#ifndef STATISTICAL_OUTLIER_REMOVAL_PROCESSOR_H
#define STATISTICAL_OUTLIER_REMOVAL_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class StatisticalOutlierRemovalProcessor {
public:
    // 使用統計離群點濾波 (Statistical Outlier Removal, SOR)
    // meanK: 每個點在濾波時使用的鄰近點數量 (預設 50)
    // stdDevMulThresh: 標準差乘數閥值 (預設 1.0)
    static pcl::PointCloud<pcl::PointXYZI>::Ptr removeOutliers(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        int meanK = 50,
        double stdDevMulThresh = 1.0);
};

#endif // STATISTICAL_OUTLIER_REMOVAL_PROCESSOR_H
