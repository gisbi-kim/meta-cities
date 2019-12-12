#include <iostream>

#include <pcl/pcl_config.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <meta_cities/pcl_viewer.h>

using std::cout;
using std::endl;


auto main (int argc, char** argv) -> int
{
  std::cout << "Im using PCL version: " << PCL_VERSION << std::endl;
  pcl_viewer::Foo();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

  // gen input point cloud 
  cloud_in->width    = 5; // num points
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (std::size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  // disp the input point cloud 
  std::cout << "Saved " << cloud_in->points.size () << " data points to input:" << std::endl;
  for (std::size_t i = 0; i < cloud_in->points.size (); ++i) 
  {
    std::cout << "    " << cloud_in->points[i].x << " " << cloud_in->points[i].y << " " << cloud_in->points[i].z << std::endl;
  }

  // viz the input point cloud 
  ViewerPtr viewer = pcl_viewer::genPCLViewer();
  pcl_viewer::simpleVis(viewer, cloud_in);

  // copy input point cloud to 2nd point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_out = *cloud_in;
  std::cout << "size:" << cloud_out->points.size() << std::endl;

  // transform the 2nd point cloud 
  for (std::size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
  }
  std::cout << "Transformed " << cloud_in->points.size () << " data points:" << std::endl;

  // disp the transformed point cloud
  for (std::size_t i = 0; i < cloud_out->points.size (); ++i)
  {
    std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
  }
  
  // registering the 2nd to 1st
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  return (0);
}
