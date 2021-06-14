
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
//#pragma once
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <chrono>

#define RENDERBOX true
#define VIEW false

std::vector<Box> CityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZI>);
    filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3f, Eigen::Vector4f(-7, -5, -3, 1), Eigen::Vector4f(12, 7, 5, 1));

    // TODO: replace filtering with your RANSAC
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    if (VIEW)
    {
        renderPointCloud(viewer, segmentCloud.first, "obstacles", Color(1, 0, 1));
        renderPointCloud(viewer, segmentCloud.second, "plane", Color(0, 1, 0));
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr diff(new pcl::PointCloud<pcl::PointXYZI>());

    for (int i = 0; i <= segmentCloud.first->points.size(); i++)
    {
        bool flag = true;
        for (int j = i + 1; j <= segmentCloud.second->points.size(); j++)
        {

            if (
                (segmentCloud.first->points[i].x != segmentCloud.second->points[j].x) && (segmentCloud.first->points[i].y != segmentCloud.second->points[j].y) && (segmentCloud.first->points[i].z != segmentCloud.second->points[j].z))
            {
                flag = false;
            }
        }

        if (flag)
        {
            diff->points.push_back(segmentCloud.first->points[i]);
        }
    }

    //renderPointCloud(viewer, diff, "diff", Color(1, 1, 0));

    // // // TODO: replace filtering with your Euclidean
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(diff, 0.53, 12, 300);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 1), Color(1, 1, 0), Color(0, 0, 1)};

    std::vector<Box> boxes;
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //pointProcessorI->numPoints(cluster);
        if (VIEW)
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        ++clusterId;
        if (RENDERBOX)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            boxes.push_back(box);
            if (VIEW)
                renderBox(viewer, box, clusterId);
        }
    }

    return boxes;
}

std::vector<Box>PlainCityBlock(ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZI>);
    filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3f, Eigen::Vector4f(-7, -5, -3, 1), Eigen::Vector4f(12, 7, 5, 1));

    // TODO: replace filtering with your RANSAC
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr diff(new pcl::PointCloud<pcl::PointXYZI>());

    for (int i = 0; i <= segmentCloud.first->points.size(); i++)
    {
        bool flag = true;
        for (int j = i + 1; j <= segmentCloud.second->points.size(); j++)
        {

            if (
                (segmentCloud.first->points[i].x != segmentCloud.second->points[j].x) && (segmentCloud.first->points[i].y != segmentCloud.second->points[j].y) && (segmentCloud.first->points[i].z != segmentCloud.second->points[j].z))
            {
                flag = false;
            }
        }

        if (flag)
        {
            diff->points.push_back(segmentCloud.first->points[i]);
        }
    }

    //renderPointCloud(viewer, diff, "diff", Color(1, 1, 0));

    // // // TODO: replace filtering with your Euclidean
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(diff, 0.53, 12, 300);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 1), Color(1, 1, 0), Color(0, 0, 1)};

    std::vector<Box> boxes;
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //pointProcessorI->numPoints(cluster);
        ++clusterId;
        if (RENDERBOX)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            boxes.push_back(box);
        }
    }

    return boxes;
}


float getAverage(float a, float b)
{
    return (a + b) / 2;
}
//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
        break;
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    if (VIEW) pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = FPS;
    //initCamera(setAngle, viewer);

    // //project caller
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    float x, y;

    // while (!viewer->wasStopped())
    
    // this condition can be controlled later by ROS
    while (true)
    {
        auto t_start = std::chrono::high_resolution_clock::now();

        // if (VIEW)
        // {
        //     // Clear viewer
        //     viewer->removeAllPointClouds();
        //     viewer->removeAllShapes();
        // }
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        // run the main function that perform clustering
        // if(VIEW){
        //     std::vector<Box> frame = CityBlock(viewer, pointProcessorI, inputCloudI);
        // }else{
        std::vector<Box> frame = PlainCityBlock(pointProcessorI, inputCloudI);

        // to be sent to ROS
        for (auto i : frame)
        {
            // float points (the centre) that are sent to ROS
            x = getAverage(i.x_min, i.x_max);
            y = getAverage(i.y_min, i.y_max);
            //cout << x << " , " << y;
        }

        //cout << endl;

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        // if (VIEW) viewer->spinOnce();
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        cout << elapsed_time_ms << endl;
    }
}
