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

	auto startTime = std::chrono::steady_clock::now();
	
	// TODO: Fill in this function

	// For max iterations

    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indices of inliers from fitted line with most inliers
    int iterationNum = 0;

    while (iterationNum < maxIterations) {
        std::unordered_set<int> inliers;
        while (inliers.size() < 2) {
            inliers.insert(rand() % cloud->points.size());
        }
        auto it = inliers.begin();
        int idx1 = *it;
        it++;
        int idx2 = *it;
        float x1 = cloud->points[idx1].x, y1 = cloud->points[idx1].y;
        float x2 = cloud->points[idx2].x, y2 = cloud->points[idx2].y;

        float a = y1 - y2;
        float b = x2 - x1;
        float c = x1 * y2 - x2 * y1;
        float dist = 0.0;    // distance from point to a line
        for (int i = 0; i < cloud->points.size(); ++i) {
            if (i == idx1 || i == idx2) continue;

            pcl::PointXYZ point = cloud->points[i];
            float x3 = point.x, y3 = point.y;
            dist = fabs(a * x3 + b * y3 + c) / sqrt(pow(a, 2) + pow(b, 2));
            if (dist <= distanceTol) {
                inliers.insert(i);
            }
        }
        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
        iterationNum++;
    }

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took: " << elapsedTime.count() << " milliseconds." << std::endl;

	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxInterations, float distanceTol) {
    std::unordered_set<int> inliersResult;

    auto startTime = std::chrono::steady_clock::now();

    while (maxInterations--) {
        std::unordered_set<int> inliers;
        while (inliers.size() < 3) inliers.insert(rand() % cloud->points.size());
        auto it = inliers.begin();
        int idx1 = *it;
        it++;
        int idx2 = *it;
        it++;
        int idx3 = *it;
        float x1 = cloud->points[idx1].x, y1 = cloud->points[idx1].y, z1 = cloud->points[idx1].z;
        float x2 = cloud->points[idx2].x, y2 = cloud->points[idx2].y, z2 = cloud->points[idx2].z;
        float x3 = cloud->points[idx3].x, y3 = cloud->points[idx3].y, z3 = cloud->points[idx3].z;

        float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        float d = -(a * x1 + b * y1 + c * z1);

        for (int i = 0; i < cloud->points.size(); ++i) {
            if (inliers.find(i) != inliers.end()) continue;

            pcl::PointXYZ point = cloud->points[i];
            float x4 = point.x, y4 = point.y, z4 = point.z;
            float dist = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);
            if (dist <= distanceTol) {
                inliers.insert(i);
            }
        }
        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC 3D took: " << elapsedTime.count() << " milliseconds." << std::endl;

    return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
//	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
    std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

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
