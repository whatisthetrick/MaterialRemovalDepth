#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h> // 用于 removeNaNFromPointCloud
#include <Eigen/Dense>
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <string>
#include <direct.h> // 用于 _mkdir（Windows）

using namespace pcl;
using namespace std;

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "Usage: " << argv[0] << " D:\\point_cloud_00000.pcd" << endl;
        return -1;
    }

    string filename = argv[1];

    cout << "[Step1] 程序已启动" << endl;
    cout << "[Step1] 收到路径参数: " << filename << endl;

    // 读取点云
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());
    if (io::loadPCDFile(filename, *cloud) == -1) {
        PCL_ERROR("无法加载点云文件！\n");
        return -1;
    }
    cout << "[Step1] 点云读取成功，点数: " << cloud->size() << endl;

    // 下采样
    PointCloud<PointXYZRGB>::Ptr cloud_filtered(new PointCloud<PointXYZRGB>());
    VoxelGrid<PointXYZRGB> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.5f, 0.5f, 0.5f);
    vg.filter(*cloud_filtered);
    cout << "[Step2] 下采样后点数: " << cloud_filtered->size() << endl;

    // RANSAC平面拟合
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    PointIndices::Ptr inliers(new PointIndices);
    SACSegmentation<PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setDistanceThreshold(1.0); // 调整平面拟合精度
    seg.setMaxIterations(1000); // 提高拟合稳定性
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        cout << "[Error] 未检测到主平面" << endl;
        return -1;
    }
    cout << "[Step3] 主平面拟合完成，法向量: ("
        << coefficients->values[0] << ", "
        << coefficients->values[1] << ", "
        << coefficients->values[2] << ")" << endl;

    // 计算旋转矩阵，将法向量对齐到Z轴
    Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Eigen::Vector3f target_axis(0.0, 0.0, 1.0); // 目标Z轴
    Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(plane_normal, target_axis);
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();

    // 旋转点云
    PointCloud<PointXYZRGB>::Ptr cloud_aligned(new PointCloud<PointXYZRGB>());
    transformPointCloud(*cloud_filtered, *cloud_aligned, transform);
    cout << "[Step4] 点云坐标系已矫正" << endl;

    // 提取 Z ∈ [440.12, 442.50] 区域点云
    PointCloud<PointXYZRGB>::Ptr z_filtered(new PointCloud<PointXYZRGB>());
    PassThrough<PointXYZRGB> pass;
    pass.setInputCloud(cloud_aligned);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(440.12f, 442.50f); // 固定范围
    pass.filter(*z_filtered);
    cout << "[Step5] 提取区域点数 (Z 440.12~442.50): " << z_filtered->size() << endl;

    // 聚类分出焊缝
    search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>());
    tree->setInputCloud(z_filtered);

    vector<PointIndices> cluster_indices;
    EuclideanClusterExtraction<PointXYZRGB> ec;
    ec.setClusterTolerance(1.0); // 调整焊缝间距阈值
    ec.setMinClusterSize(2000);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(z_filtered);
    ec.extract(cluster_indices);

    cout << "[Step6] 聚类完成，检测到焊缝数量: " << cluster_indices.size() << endl;

    // 创建 D:\weld_data 文件夹
    string folder_path = "D:\\";
    _mkdir(folder_path.c_str()); // Windows 创建文件夹

    // 可视化
    visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("焊缝检测"));
    viewer->addPointCloud(cloud_aligned, "aligned_cloud");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR, 0.7, 0.7, 0.7, "aligned_cloud");

    int weld_id = 1;
    for (const auto& indices : cluster_indices) {
        PointCloud<PointXYZRGB>::Ptr weld_cloud(new PointCloud<PointXYZRGB>());
        for (int idx : indices.indices)
            weld_cloud->push_back((*z_filtered)[idx]);

        // 计算焊缝尺寸
        PointXYZRGB weld_min_pt, weld_max_pt;
        getMinMax3D(*weld_cloud, weld_min_pt, weld_max_pt);

        float length = weld_max_pt.x - weld_min_pt.x;
        float width = weld_max_pt.y - weld_min_pt.y;
        float height = weld_max_pt.z - weld_min_pt.z;

        cout << "========== 焊缝 #" << weld_id << " ==========" << endl;
        cout << "长: " << length << " mm" << endl;
        cout << "宽: " << width << " mm" << endl;
        cout << "高: " << height << " mm" << endl;

        // ⭐ 导出焊缝 (relative_x, relative_y, relative_z)
        string xyz_filename = folder_path + "weld_" + to_string(weld_id) + "_relative_xyz.csv";
        ofstream xyz_out(xyz_filename);
        if (!xyz_out.is_open()) {
            cerr << "[Error] 无法创建 " << xyz_filename << " 文件" << endl;
        }
        else {
            xyz_out << "relative_x,relative_y,relative_z" << endl; // 表头
            for (const auto& p : weld_cloud->points) {
                float relative_x = p.x - weld_min_pt.x;
                float relative_y = p.y - weld_min_pt.y;
                float relative_z = p.z - weld_min_pt.z;
                xyz_out << relative_x << "," << relative_y << "," << relative_z << endl;
            }
            xyz_out.close();
            cout << "✅ 已导出焊缝 #" << weld_id << " 的 (relative_x, relative_y, relative_z) 到 " << xyz_filename << endl;
        }

        // 着色显示焊缝
        for (auto& p : weld_cloud->points) {
            p.r = rand() % 256; p.g = rand() % 256; p.b = rand() % 256; // 随机颜色
        }
        string cloud_name = "weld_" + to_string(weld_id);
        viewer->addPointCloud(weld_cloud, cloud_name);
        viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_name);

        // ⭐ 在焊缝质心添加编号标注
        Eigen::Vector4f centroid;
        compute3DCentroid(*weld_cloud, centroid); // 计算焊缝质心
        string text_id = "#" + to_string(weld_id);
        viewer->addText3D(text_id, PointXYZ(centroid[0], centroid[1], centroid[2]),
            8.0, 1.0, 0.0, 0.0, "text_" + to_string(weld_id));

        weld_id++;
    }

    viewer->addCoordinateSystem(50.0);
    viewer->setBackgroundColor(0, 0, 0);
    cout << "[Step7] 可视化已启动..." << endl;

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}