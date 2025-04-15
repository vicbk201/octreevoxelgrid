#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>

namespace BackgroundRemovalProcessor {

    // 依據 PCL 官方 octree change detector 實作
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeBackgroundByOctree(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& current_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& background_cloud,
        float resolution);

}
