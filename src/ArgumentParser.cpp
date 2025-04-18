#include "ArgumentParser.h"
#include <iostream>
#include <stdexcept>
#include <cctype>

bool ArgumentParser::parseArguments(int argc, char** argv,
                                    std::string& cloudFile,
                                    float& resolution,
                                    std::string& downsampleMethod,
                                    std::string& clusterMethod,
                                    std::string& outputCloudFile)
{
    // 預設解析度
    resolution = 0.1f;
    // 清空方法參數
    downsampleMethod = "";
    clusterMethod = "";
    // 預設 outputCloudFile 為空，表示不儲存結果
    outputCloudFile = "";

    // 從 argv[1] 開始解析參數
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--cloudfile") {
            if (i + 1 < argc) {
                cloudFile = argv[++i];
            } else {
                std::cerr << "Error: --cloudfile 需要一個檔案路徑" << std::endl;
                return false;
            }
        } else if (arg == "--method") {
            if (i + 1 < argc) {
                downsampleMethod = argv[++i];
                // 轉為小寫
                for (auto &c : downsampleMethod) {
                    c = std::tolower(c);
                }
                if (downsampleMethod != "octree" && downsampleMethod != "voxelgrid") {
                    std::cerr << "Error: --method 必須是 'octree' 或 'voxelgrid'" << std::endl;
                    return false;
                }
            } else {
                std::cerr << "Error: --method 需要一個值 (octree 或 voxelgrid)" << std::endl;
                return false;
            }
        } else if (arg == "--cluster") {
            if (i + 1 < argc) {
                clusterMethod = argv[++i];
                for (auto &c : clusterMethod) {
                    c = std::tolower(c);
                }
                if (clusterMethod != "euclidean" && clusterMethod != "dbscan") {
                    std::cerr << "Error: --cluster 必須是 'euclidean' 或 'dbscan'" << std::endl;
                    return false;
                }
            } else {
                std::cerr << "Error: --cluster 需要一個值 (euclidean 或 dbscan)" << std::endl;
                return false;
            }
        } else if (arg == "--resolution" || arg == "--r") {
            if (i + 1 < argc) {
                try {
                    resolution = std::stof(argv[++i]);
                    if (resolution <= 0) {
                        throw std::invalid_argument("解析度必須是正數");
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: 解析度錯誤: " << e.what() << std::endl;
                    return false;
                }
            } else {
                std::cerr << "Error: " << arg << " 需要一個數值" << std::endl;
                return false;
            }
        } else if (arg == "--o" || arg == "--output") {
            if (i + 1 < argc) {
                outputCloudFile = argv[++i];
                std::cout << "Parsed output file: " << outputCloudFile << std::endl; // 加入除錯印出
            } else {
                std::cerr << "Error: " << arg << " 需要一個完整的檔案路徑（含檔名與副檔名）" << std::endl;
                return false;
            }
        } else {
            std::cerr << "Warning: 未知的參數: " << arg << std::endl;
        }
    }

    // 檢查必要參數
    if (cloudFile.empty()) {
        std::cerr << "Error: 必須提供 --cloudfile <PCD檔案路徑>" << std::endl;
        return false;
    }
    if (downsampleMethod.empty()) {
        std::cerr << "Error: 必須提供 --method <octree|voxelgrid>" << std::endl;
        return false;
    }
    if (clusterMethod.empty()) {
        std::cerr << "Error: 必須提供 --cluster <euclidean|dbscan>" << std::endl;
        return false;
    }

    return true;
}
