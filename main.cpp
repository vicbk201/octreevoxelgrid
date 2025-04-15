#include <iostream>
#include <pcl/io/pcd_io.h>  // 用於儲存 PCD 檔案
#include "ArgumentParser.h"
#include "PCDLoader.h"
#include "OctreeProcessor.h"
#include "VoxelGridProcessor.h"
#include "PointCloudViewer.h"
#include "BackgroundRemovalProcessor.h"
#include <iomanip>  // 為了使用 std::setprecision


int main(int argc, char** argv)
{
    std::string cloudFile;
    float resolution;
    std::string method;
    std::string outputCloudFile;  // 新增此變數接收輸出檔案的完整路徑與檔名

    // 解析命令列參數，若解析失敗則退出
    if (!ArgumentParser::parseArguments(argc, argv, cloudFile, resolution, method, outputCloudFile)) {
        std::cerr << "正確使用方式: " << argv[0]
                  << " --cloudfile <PCD檔案路徑> --method <octree|voxelgrid> [--resolution <解析度>] [--o|--output <完整輸出PCD檔案路徑>]"
                  << std::endl;
        return -1;
    }

    std::cout << "PCD File: " << cloudFile << std::endl;
    std::cout << "Method: " << method << std::endl;
    std::cout << "Resolution: " << resolution << std::endl;
    /*
    auto cloud = PCDLoader::loadPCD(cloudFile);
    if (!cloud || cloud->empty()) {
        std::cerr << "Error: 無法載入點雲或點雲為空。" << std::endl;
        return -1;
    }
    std::cout << "原始點雲數量: " << cloud->size() << std::endl;
    */ 

    //載入資料
    auto raw_input = PCDLoader::loadPCD(cloudFile);
    auto raw_background = PCDLoader::loadPCD("/home/semilux/Documents/fortsense_test_pcd/background/4084-563116000.pcd");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    float background_resolution = 0.15f;
    
    if (!raw_input || raw_input->empty())
    {
        std::cerr << "Error: 輸入點雲為空。" << std::endl;
        return -1;
    }
    if (!raw_background || raw_background->empty())
    {
        std::cerr << "警告：背景點雲為空，跳過背景去除。" << std::endl;
        cloud = raw_input;
    }
    else
    {
        cloud = BackgroundRemovalProcessor::removeBackgroundByOctree(raw_input, raw_background, background_resolution);
    }

    // 選擇下採樣方法
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud;
    if (method == "voxelgrid")
    {
        auto voxelResult = VoxelGridProcessor::processVoxelGrid(cloud, resolution);
        downsampledCloud = voxelResult.cloud;
        std::cout << "降採樣後點雲數量: " << downsampledCloud->size()
                  << " (算法耗時: " << std::fixed << std::setprecision(2) << voxelResult.runtime_ms << " ms)" << std::endl;
    }
    else if (method == "octree")
    {
        auto octreeResult = OctreeProcessor::processOctree(cloud, resolution);
        downsampledCloud = octreeResult.cloud;
        std::cout << "Octree下採樣後點雲數量: " << downsampledCloud->size()
                  << " (算法耗時: " << std::fixed << std::setprecision(2) << octreeResult.runtime_ms << " ms)" << std::endl;
    }
    else
    {
        std::cerr << "Error: 未知的 downsample method: " << method << std::endl;
        return -1;
    }
    
    // 若有指定輸出檔案完整路徑，則儲存處理後的點雲
    if (!outputCloudFile.empty()) {
        std::cout << "Saving processed cloud to: " << outputCloudFile << std::endl;
        if (pcl::io::savePCDFile(outputCloudFile, *downsampledCloud) == -1) {
            std::cerr << "Error: 儲存點雲失敗！" << std::endl;
            return -1;
        }  
    } 

    try {
        PointCloudViewer::displayProcessedCloud(downsampledCloud, resolution);
    } catch(const std::exception &e) {
        std::cerr << "顯示點雲時發生錯誤: " << e.what() << std::endl;
    }

    return 0;
}
