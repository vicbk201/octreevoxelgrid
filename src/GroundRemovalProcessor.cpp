#include "GroundRemovalProcessor.h"
#include <pcl/console/time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <iostream>
#include <iomanip>

typedef pcl::PointXYZI PointTypeIO;

GroundRemovalResult GroundRemovalProcessor::removeGround(
    const pcl::PointCloud<PointTypeIO>::Ptr &cloud)
{
    pcl::console::TicToc tt;
    tt.tic();

    pcl::PointCloud<PointTypeIO>::Ptr cloud_out(new pcl::PointCloud<PointTypeIO>(*cloud));

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<PointTypeIO> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.3); // 根據點雲密度與感測器噪聲調整

    int max_iterations = 5;
    const int min_inliers_threshold = 100;
    int iteration = 0;
    // 設置一個角度閾值（以弧度計算，這裡設定約 15 度）
    const float angle_threshold = 15.0f * M_PI / 180.0f;
    // 新增變數用來保存地面係數
    Eigen::Vector4f ground_coeff(0, 0, 0, 0);
    bool groundFound = false;
    
    while (iteration < max_iterations)
    {
        seg.setInputCloud(cloud_out);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.empty() || inliers->indices.size() < min_inliers_threshold)
        {
            break;
        }
    
        // 取得當前平面的法向量並正規化
        Eigen::Vector3f current_normal(coefficients->values[0],
                                       coefficients->values[1],
                                       coefficients->values[2]);
        current_normal.normalize();

        if (!groundFound)
        {
            // 第一次分割時，檢查平面是否大致水平
            // 水平平面的法向量應該與全局 z 軸接近平行 (dot product 接近 1)
            if (std::fabs(current_normal.dot(Eigen::Vector3f::UnitZ())) < 0.9f)
            {
                // 若平面太傾斜，就認為不是地面，退出迴圈
                break;
            }
            // 記錄下來作為地面平面的基準
            ground_coeff << coefficients->values[0], coefficients->values[1],
                            coefficients->values[2], coefficients->values[3];
            groundFound = true;
        }
        else
        {
            // 已找到地面，後續的平面必須與第一個地面平面保持一致
            Eigen::Vector3f ground_normal = ground_coeff.head<3>().normalized();
            float angle = std::acos(current_normal.dot(ground_normal));
            if (angle > angle_threshold)
            {
                // 如果角度差異太大，則認為該平面已經不是原本的地面，退出迴圈
                break;
            }
        }
    
        // 移除內點（地面點）
        pcl::ExtractIndices<PointTypeIO> extract;
        extract.setInputCloud(cloud_out);
        extract.setIndices(inliers);
        extract.setNegative(true); // 移除內點
        pcl::PointCloud<PointTypeIO>::Ptr cloud_no_ground(new pcl::PointCloud<PointTypeIO>());
        extract.filter(*cloud_no_ground);
        cloud_out = cloud_no_ground;
        iteration++;
    }

    double elapsed = tt.toc();

    GroundRemovalResult result;
    result.cloud = cloud_out;
    result.runtime_ms = elapsed;
    result.ground_coefficients = ground_coeff;
    return result;
}
