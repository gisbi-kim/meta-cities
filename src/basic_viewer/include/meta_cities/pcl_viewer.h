#ifndef _PCL_VIEWER_H_
#define _PCL_VIEWER_H_
// original ref: http://pointclouds.org/documentation/tutorials/pcl_visualizer.php

#include <iostream>
#include <thread>
#include <vector>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std::chrono_literals;
using ViewerPtr = pcl::visualization::PCLVisualizer::Ptr;
using XYZConstPtr = pcl::PointCloud<pcl::PointXYZ>::ConstPtr;
using VectorInt = std::vector<int>;

using std::cout;
using std::endl;

namespace pcl_viewer 
{

void Foo (void)
{
    cout << "_PCL_VIEWER_H_" << endl;
} // Function: Foo


ViewerPtr genPCLViewer (void)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    return viewer;
} // Function: genPCLViewer


void simpleVis (ViewerPtr _viewer, XYZConstPtr _cloud, VectorInt _bg_color = {0, 0, 0})
{
    // set viewer properties
    _viewer->setBackgroundColor (_bg_color.at(0), _bg_color.at(1), _bg_color.at(2));
    _viewer->addPointCloud<pcl::PointXYZ> (_cloud, "sample cloud");
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    _viewer->addCoordinateSystem (1.0);
    _viewer->initCameraParameters ();

    // hold the window
    while (!_viewer->wasStopped ())
    {
        _viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }
} // Function: simpleVis



} // Namespace: pcl_viewer
#endif
