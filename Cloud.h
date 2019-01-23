#pragma once

#include <iostream>
#include <fstream>
#include <ostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/core.hpp>

class Cloud {
public:
	Cloud();
	Cloud(std::string in_path, bool SkipFirst=true, bool skipA=true);
	Cloud(Cloud const& cloud);

	int readCloudXYZRGBAIFromTxt(bool SkipFirst, bool skipA);

	int computeKdTree();
	int computeDensity(float radius);
	int computeNormals();

	int savePredictedLabels(cv::Mat matPredictedLabels);

	int printFullCloud(std::ostream &flux);

//protected:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr XYZRGBACloud;
	pcl::PointCloud<pcl::Intensity>::Ptr intensityCloud;
	pcl::PointCloud<pcl::Normal>::Ptr normalsCloud;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtreeCloud;
	std::vector<int> density;
	std::vector<int> label;
	std::string path;

};