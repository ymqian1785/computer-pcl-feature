
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>


void estimate_Normal( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, 
					 pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_normals ){

						 PCL_INFO("estimate_Normal:\n");
						 pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;
						 pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZRGB>());

						 ne.setInputCloud(cloud);
						 ne.setSearchMethod(tree_n);
						 ne.setRadiusSearch(0.05);
						 ne.compute(*cloud_normals);

						 // Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
						 for(size_t i = 0; i<cloud_normals->points.size(); ++i)
						 {
							 cloud_normals->points[i].x = cloud->points[i].x;
							 cloud_normals->points[i].y = cloud->points[i].y;
							 cloud_normals->points[i].z = cloud->points[i].z;
						 }
						 PCL_INFO(" After estimate_Normal!\n");
}

int main(int, char** argv)
{

    std::string filename = "E://shijie database//downsampled//down-a06-s10-e01.pcd";
	std::cout << "Reading " << filename << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) == -1) // load the file
	{
		PCL_ERROR ("Couldn't read file");
		return -1;
	}
	std::cout << "points: " << cloud->points.size () <<std::endl;


	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
	estimate_Normal(cloud, cloud_normals );



  // Create the VFH estimation class, and pass the input dataset+normals to it
  pcl::VFHEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::VFHSignature308> vfh;
  vfh.setInputCloud (cloud);
  vfh.setInputNormals (cloud_normals);
  // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  vfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

  // Compute the features
  vfh.compute (*vfhs);

  // vfhs->points.size () should be of size 1*
  pcl::io::savePCDFileASCII("I://data//down-vfh-4//VFH4-a06-e10-01.pcd", *vfhs);

  // Plotter object. 
  
    
  pcl::visualization::PCLPlotter plotter;
  // We need to set the size of the descriptor beforehand.
  plotter.addFeatureHistogram(*vfhs, 308); //设置的很坐标长度，该值越大，则显示的越细致
  plotter.plot();

  
}