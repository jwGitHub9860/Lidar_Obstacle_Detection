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
	std::unordered_set<int> inliersResult;	//holds BEST inliers
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	while (maxIterations--)		// run for max number of iterations (while "max iterations > 0")
	{
		// Randomly pick two points

		std::unordered_set<int> inliers;		// pick unique values ONLY
		while (inliers.size() < 2)
		{
			inliers.insert(rand()%(cloud->points.size()));	// randomly inserts a point
		}
		float x_1, y_1, x_2, y_2;

		auto itr = inliers.begin();		// points at beginning of inliers
		x_1 = cloud->points[*itr].x;	//(cloud)->{points}[dereferences pointer -> INDEX into (cloud)].x value for {points}
		y_1 = cloud->points[*itr].y;
		itr++;
		x_2 = cloud->points[*itr].x;
		y_2 = cloud->points[*itr].y;

		// (y1 - y2)x + (x2 - x1)y + (x1 * y2 - x2 * y1) = 0
		float A = (y_1 - y_2);
		float B = (x_2 - x_1);
		float C = (x_1*y_2 - x_2*y_1);

		for (int index = 0; index < cloud->points.size(); index++)	// iterates through all points
		{
			if (inliers.count(index) > 0)	// checks if point isn't ALREADY accounted for
			{
				continue;
			}
			pcl::PointXYZ point = cloud->points[index];		// calculate distance & see if within threshold
			float x_3 = point.x;	// show x value
			float y_3 = point.y;	// show y value

			float d = fabs(A*x_3 + B*y_3 + C) / sqrt(A*A + B*B);	// Distance:	d = |Ax + By + C| / sqrt(A^2 + B^2)

			if (d <= distanceTol)
			{
				inliers.insert(index);
			}
		}
		
		if (inliers.size() > inliersResult.size())	// checks size of inliers
		{
			inliersResult = inliers;
		}
		
	}
	

	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;	//holds BEST inliers
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	while (maxIterations--)		// run for max number of iterations (while "max iterations > 0")
	{
		// Randomly pick two points

		std::unordered_set<int> inliers;		// pick unique values ONLY
		while (inliers.size() < 2)
		{
			inliers.insert(rand()%(cloud->points.size()));	// randomly inserts a point
		}
		float x_1, y_1, z_1, x_2, y_2, z_2, x_3, y_3, z_3;

		auto itr = inliers.begin();		// points at beginning of inliers
		x_1 = cloud->points[*itr].x;	//(cloud)->{points}[dereferences pointer -> INDEX into (cloud)].x value for {points}
		y_1 = cloud->points[*itr].y;
		z_1 = cloud->points[*itr].z;
		itr++;
		x_2 = cloud->points[*itr].x;
		y_2 = cloud->points[*itr].y;
		z_2 = cloud->points[*itr].z;
		itr++;
		x_3 = cloud->points[*itr].x;
		y_3 = cloud->points[*itr].y;
		z_3 = cloud->points[*itr].z;

		// v_1 = < x2 − x1, y2 − y1, z2 − z1 >
		float v_1_coor_1 = x2 - x1;
		float v_1_coor_2 = y2 - y1;
		float v_1_coor_3 = z2 - z1;

		// v_2 = < x3 − x1, y3 − y1, z3 − z1 >
		float v_2_coor_1 = x3 - x1;
		float v_2_coor_2 = y3 - y1;
		float v_2_coor_3 = z3 - z1;

		// v1 X v2 = < (y2 − y1)(z3 − z1) - (z2 − z1)(y3 − y1), (z2 − z1)(x3 − x1) - (x2 − x1)(z3 − z1), (x2 − x1)(y3 − y1) - (y2 − y1)(x3 − x1) >
		float i = (y2 - y1)(z3 - z1) - (z2 - z1)(y3 - y1);
		float j = (z2 - z1)(x3 - x1) - (x2 - x1)(z3 - z1);
		float k = (x2 - x1)(y3 - y1) - (y2 - y1)(x3 - x1);

		// ix + jy + kz - (ix1 + jy1 + kz1) = 0
		float A = i;
		float B = j;
		float C = k;
		float D = - (ix1 + jy1 + kz1);

		for (int index = 0; index < cloud->points.size(); index++)	// iterates through all points
		{
			if (inliers.count(index) > 0)	// checks if point isn't ALREADY accounted for
			{
				continue;
			}
			pcl::PointXYZ point = cloud->points[index];		// calculate distance & see if within threshold
			float x_3 = point.x;	// show x value
			float y_3 = point.y;	// show y value
			float z_3 = point.y;	// show z value

			float d = fabs(A*x_3 + B*y_3 + C*z_3 + D) / sqrt(A*A + B*B + C*C);	// Distance:	d = |Ax + By + Cz + D| / sqrt(A^2 + B^2 + C^2)

			if (d <= distanceTol)
			{
				inliers.insert(index);
			}
		}
		
		if (inliers.size() > inliersResult.size())	// checks size of inliers
		{
			inliersResult = inliers;
		}
		
	}
	

	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 0, 0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		//pcl::PointXYZ point = cloud->points[index];
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();		// Create data
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
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
