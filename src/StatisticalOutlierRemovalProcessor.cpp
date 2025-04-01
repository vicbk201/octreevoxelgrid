#include "StatisticalOutlierRemovalProcessor.h"
#include <pcl/filters/statistical_outlier_removal.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr StatisticalOutlierRemovalProcessor::removeOutliers(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    int meanK,
    double stdDevMulThresh)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stdDevMulThresh);
    sor.filter(*filteredCloud);
    return filteredCloud;
}
