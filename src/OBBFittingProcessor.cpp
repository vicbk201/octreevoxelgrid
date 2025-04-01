#include "OBBFittingProcessor.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <iostream>

// 計算物體的有向包圍盒 (OBB)，僅保留繞 Z 軸的旋轉（只取 yaw）
OrientedBoundingBox OBBFittingProcessor::computeOBB(const pcl::PointCloud<PointType>::Ptr &cloud_cluster)
{
    OrientedBoundingBox obb;
    if (cloud_cluster->empty())
    {
        std::cerr << "錯誤：輸入點雲為空！" << std::endl;
        return obb;
    }
    
    // 1. 取得 AABB 的最小/最大值，並計算 AABB 的中心
    PointType min_pt, max_pt;
    pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
    Eigen::Vector3f center = (min_pt.getVector3fMap() + max_pt.getVector3fMap()) / 2.0f;
    
    // 使用 AABB 中心作為平移基準（改變原本用 compute3DCentroid 的做法）
    Eigen::Vector4f centroid;
    centroid << center, 1.0f;
    
    // 2. 計算協方差矩陣（必須使用平移基準，此處使用 AABB 中心）
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud_cluster, centroid, covariance);
    
    // 3. 進行 PCA 計算特徵向量與特徵值
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    
    // 確保正交化：重新計算第三個軸
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
    
    // 按照特徵值從大到小排列（預設 eigen_solver 返回的特徵向量是從小到大排列的）
    Eigen::Matrix3f eigenVectorsOrdered;
    eigenVectorsOrdered.col(0) = eigenVectorsPCA.col(2);
    eigenVectorsOrdered.col(1) = eigenVectorsPCA.col(1);
    eigenVectorsOrdered.col(2) = eigenVectorsPCA.col(0);
    eigenVectorsPCA = eigenVectorsOrdered;
    
    // 4. 取得 Euler 角 (yaw, pitch, roll)，僅取 yaw
    Eigen::Vector3f eulerAngles = eigenVectorsPCA.eulerAngles(2, 1, 0);
    float yaw = eulerAngles[0];
    Eigen::AngleAxisf keep_Z_Rot(yaw, Eigen::Vector3f::UnitZ());
    
    // 5. 建立變換矩陣：先平移到 AABB 中心，再旋轉 (只取 yaw)
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(center);
    transform.rotate(keep_Z_Rot);
    
    // 6. 將原始點雲轉到局部坐標系中，計算局部 AABB 的尺寸
    pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*cloud_cluster, *transformedCloud, transform.inverse());
    
    PointType min_pt_T, max_pt_T;
    pcl::getMinMax3D(*transformedCloud, min_pt_T, max_pt_T);
    float length = max_pt_T.x - min_pt_T.x;  // X 軸範圍（長度）
    float width  = max_pt_T.y - min_pt_T.y;    // Y 軸範圍（寬度）
    float height = max_pt_T.z - min_pt_T.z;    // Z 軸範圍（高度）
    
    // 7. 計算局部 AABB 的中心
    Eigen::Vector3f local_center = (min_pt_T.getVector3fMap() + max_pt_T.getVector3fMap()) / 2.0f;
    
    // 8. 將局部中心轉回全域坐標
    Eigen::Vector3f global_center = transform * local_center;
    
    // 9. 儲存結果到 OBB 結構中
    obb.center = global_center;
    obb.dimensions = Eigen::Vector3f(length, width, height);
    obb.orientation = Eigen::Quaternionf(keep_Z_Rot); // 僅用繞 Z 軸旋轉表示
    
    return obb;
}
