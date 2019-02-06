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
#include <boost/filesystem.hpp>


class Cloud {
public:
	Cloud();
	Cloud(std::string in_path, bool SkipFirst=true, bool skipA=true);
	Cloud(Cloud const& cloud);

	void readCloudXYZRGBAIFromTxt(bool SkipFirst, bool skipA);

	void computeKdTree();
	void computeDensity(float radius);
	void computeNormals();

	void savePredictedLabels(cv::Mat matPredictedLabels);
	void Cloud::savePredictedLabels(std::vector<int> vecPredictedLabels);

	void saveTxt(std::string projectPath);
	void printFullCloud(std::ostream &flux);

	//getter
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr XYZRGBACloud();
	pcl::PointCloud<pcl::Intensity>::Ptr intensityCloud();
	pcl::PointCloud<pcl::Normal>::Ptr normalsCloud();
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtreeCloud();
	std::vector<int> density();
	std::vector<int> label();
	std::string path();

	//setter
	void setXYZRGBACloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr XYZRGBACloud);
	void setIntensityCloud(pcl::PointCloud<pcl::Intensity>::Ptr intensityCloud);
	void setNormalsCloud(pcl::PointCloud<pcl::Normal>::Ptr normalsCloud);
	void setKdtreeCloud(pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtreeCloud);
	void setDensity(std::vector<int> density);
	void setLabel(std::vector<int> label);
	void setPath(std::string path);
	   
protected:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr m_XYZRGBACloud;
	pcl::PointCloud<pcl::Intensity>::Ptr m_intensityCloud;
	pcl::PointCloud<pcl::Normal>::Ptr m_normalsCloud;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr m_kdtreeCloud;
	std::vector<int> m_density;
	std::vector<int> m_label;
	std::string m_path;
	};