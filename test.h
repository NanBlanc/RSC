#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


void testOpenCV() {
	cv::Mat img = cv::imread("C:/programmation/dev/code/RSC/lena.png");
	cv::namedWindow("image", cv::WINDOW_NORMAL);
	cv::imshow("image", img);
	cv::waitKey(0);
}

void testPCL() {
	pcl::PointCloud<pcl::PointXYZ> cloud;

	//fill in the cloud data
	cloud.width = 50;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width*cloud.height);
	for (size_t i = 0; i < cloud.size(); i++) {
		cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	pcl::io::savePCDFileASCII("C:/Users/hobbe/3D Objects/test/mycloud.pcd", cloud);

}
