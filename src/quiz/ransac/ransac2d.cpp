/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	const int cloud_size = cloud->points.size();
	
	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		
		// Randomly sample subset and fit line
		int rndIndex1 = rand()%cloud_size;
		float x1 = cloud->points[rndIndex1].x;
		float y1 = cloud->points[rndIndex1].y;

		// pick second different point
		int rndIndex2 = rand()%cloud_size;
		while(rndIndex2 == rndIndex1)
			rndIndex2 = rand()%cloud_size;
		float x2 = cloud->points[rndIndex2].x;
		float y2 = cloud->points[rndIndex2].y;

		// Random line parameters
		float a = (y1 - y2);
		float b = (x2 - x1);
		float c = (x1 * y2 - x2 * y1);

		// Measure distance between every point and fitted line
		for(int i = 0; i < cloud->points.size(); i++)
		{
			// If distance is smaller than threshold count it as inlier
			float distance = fabs(a * cloud->points[i].x + b * cloud->points[i].y + c) / sqrt (a * a + b * b);
			if(distance < distanceTol)
				inliers.insert(i);
		}

		if(inliers.size() > inliersResult.size())
			inliersResult = inliers; // New max
	}

	// Returns indicies of inliers from fitted line with most inliers
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceToP)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while(maxIterations--)
	{
		std::unordered_set<int> inliers;

		// Randomly pick a random plane
		while(inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		auto itr = inliers.begin();
		float x1 = cloud->points[*itr].x;
		float y1 = cloud->points[*itr].y;
		float z1 = cloud->points[*itr++].z;
		float x2 = cloud->points[*itr].x;
		float y2 = cloud->points[*itr].y;
		float z2 = cloud->points[*itr++].z;
		float x3 = cloud->points[*itr].x;
		float y3 = cloud->points[*itr].y;
		float z3 = cloud->points[*itr++].z;
	
		// Equation of a Plane through Three Points
		float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		float d = - a * x1 - b * y1 - c * z1;

		// Measure distance between every point and a plane
		for(int i = 0; i < cloud->points.size(); i++)
		{
			// If distance is smaller than threshold count it as inlier
			float distance = fabs(a * cloud->points[i].x + b * cloud->points[i].y + c * cloud->points[i].z + d) / sqrt (a * a + b * b + c * c);
			if(distance < distanceToP)
				inliers.insert(i);
		}

		if(inliers.size() > inliersResult.size())
			inliersResult = inliers; // New max
	}

	// Returns indicies of inliers from fitted plane with most inliers
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(0,0,1));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
