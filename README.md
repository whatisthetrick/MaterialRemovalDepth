# MaterialRemovalDepth
MaterialRemovalDepth（MRD）
想要得到清根的深度，即材料去除深度MRD。
步骤：
1、用深度相机拍摄3d点云进行处理，文件point_cloud_00000.pcd；
2、下采样减少点云数量、RANSAC平面拟合、旋转点云使z轴垂直于平面，方便后续的坐标值计算；
3、通过测试，选定z=（440.12，442.50）区间，聚类并可视化，该区间为感兴趣区域，即清根区域；
4、算出z=（440.12，442.50）区间内Xmax、Xmin、Ymax、Ymin、Zmax、Zmin，并将X-Xmin、Y-Ymin、Z-Zmin导出为csv文件；
（以上均用C++生成，下面的部分用matlab生成）
5、导出的文件手动处理筛选，因为使用金刚砂轮打磨，清根中间的部分为材料去除最深处，处理完的数据见清根数据.zip；
6、用matlab画出箱线图，选取平均值作为材料去除深度，文件display.m。
