#include "VoxelGridProcessor.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/time.h>
#include <iostream>
#include <iomanip>

VoxelGridResult VoxelGridProcessor::processVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution)
{
    pcl::console::TicToc tt;
    tt.tic();

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(resolution, resolution, resolution);
    vg.filter(*filteredCloud);

    double elapsed = tt.toc();
    
    VoxelGridResult result;
    result.cloud = filteredCloud;
    result.runtime_ms = elapsed;
    return result;
}