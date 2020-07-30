/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");   //创建车，在render中有car的结构体，依次为位置，车的维度，颜色和名称
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;       //创建了一个vector容器
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)                //使用pcl中的可视化进行渲染
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer,lidar->position,inputCloud);
    renderPointCloud(viewer,inputCloud,"inputCloud");
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first,1.0,3,30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
 
    	std::cout<<"Cluster size";
    	pointProcessor.numPoints(cluster);
    	renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);

		Box box = pointProcessor.BoundingBox(cluster);
    	renderBox(viewer,box,clusterId);

    	++clusterId;
    }
  
}

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessorI,pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
	//ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();  //Here I is the intensity
	//pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
	//renderPointCloud(viewer,inputCloud,"inputCloud");

	//pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud,0.5f,Eigen::Vector4f(-20.,-5.,-100.,1),Eigen::Vector4f(20.,5.,100.,1));
	//renderPointCloud(viewer,filterCloud,"filterCloud");

	inputCloud = pointProcessorI.FilterCloud(inputCloud,0.3,Eigen::Vector4f (-10,-5,-2,1),Eigen::Vector4f(30,8,1,1));  //对点云进行过滤
	std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.SegmentPlane(inputCloud,25,0.3);  //点云分割，点云、最大迭代次数和距离容忍度作为参数
	//renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
	renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(1,0,0));   //渲染分割后的点云平面，第一个为绿色，二为红色，三为蓝色

	//clustering
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first,0.53,10,500);  //障碍物点云，采用KdTree将其进行欧式聚类
	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};

	for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
	{
		std::cout<<"Cluster size";
		pointProcessorI.numPoints(cluster);   //返回聚类的size
		renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);  //使用三种颜色对不同的聚类进行渲染

		Box box = pointProcessorI.BoundingBox(cluster);   
		renderBox(viewer,box,clusterId);   //渲染箱体
		++clusterId;
	}

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;  //摄像机的位置、相机朝向(x,y)
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;    //相机位置，相机朝向（x,z）
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;   
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);      //建立空间直角坐标系
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); //创建一个窗口，这个窗口由智能指针创建，所以在它的生命周期结束前窗口都不会被销毁
    CameraAngle setAngle = XY;      //调用render中的枚举变量
    initCamera(setAngle, viewer);   //初始化相机
   	//cityBlock(viewer);

   	ProcessPointClouds<pcl::PointXYZI> pointProcessorI;   //实例化一个名为pointProcessorI的类，	其数据类型为XYZI的点类型，其中X,Y,Z为三维坐标信息，I为intensity

   	//boost::filesystem库的核心类是path类,屏蔽了不同文件系统的差异,使用了可移植的POSIX语法提供了通用的目录和路径表示
	std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");  //读取文件中的pcd类型数据
	auto streamIterator = stream.begin();  //auto为关键字，其能够自动推断类型

	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;   //创建一个点云指针

    while (!viewer->wasStopped ())
    {
    	 // Clear viewer
  		viewer->removeAllPointClouds();
  		viewer->removeAllShapes();    //清空操作

  		// Load pcd and run obstacle detection process
  		inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());   //加载点云
  		cityBlock(viewer, pointProcessorI, inputCloudI);

 		streamIterator++;
  		if(streamIterator == stream.end())
    		streamIterator = stream.begin();

  		viewer->spinOnce();
    } 
}