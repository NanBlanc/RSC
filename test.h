#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <liblas/liblas.hpp>
#include "fonctions.h"

int ReadLasFileIntensite(const std::string & fichier_in,
	std::deque<double>& dequeX,
	std::deque<double>& dequeY,
	std::deque<double>& dequeZ,
	std::deque<double>& dequeIntensite)
{
	dequeX.clear();
	dequeY.clear();
	dequeZ.clear();
	dequeIntensite.clear();
	//const clock_t t0 = clock();

	std::ifstream ifs(fichier_in.c_str(), std::ios::in | std::ios::binary);
	if (ifs.fail())
	{

		std::cerr << "erreur a l ouverture du flux pour " << fichier_in << std::endl;
		return 1;
	}

	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();

	unsigned long int nb_point = header.GetPointRecordsCount();

	dequeX.resize(nb_point);
	dequeY.resize(nb_point);
	dequeZ.resize(nb_point);
	dequeIntensite.resize(nb_point);

	unsigned long int compteur = 0;
	while (reader.ReadNextPoint())
	{
		if (compteur > nb_point)
		{
			std::cerr << compteur << ";" << nb_point << std::endl;
			std::cerr << "erreur entete du fichier corrompu" << std::endl;
			return 1;
		}
		liblas::Point const& p = reader.GetPoint();

		//std::cout << p.GetX() << ", " << p.GetY() << ", " << p.GetZ() << "\n";
		dequeX[compteur] = p.GetX();
		dequeY[compteur] = p.GetY();
		dequeZ[compteur] = p.GetZ();
		// P.GetIntensity
		compteur++;
	}

	if (compteur != nb_point)
	{
		std::cerr << compteur << ";" << nb_point << std::endl;
		std::cerr << "erreur entete du fichier corrompu" << std::endl;
		return 1;
	}

	//const clock_t t1 = clock();
	//std::cerr << "temps lecture                  : " << (t1-t0)/static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;

	//FIN DU PROGRAMME READLASFILEINTENSITE
	return 0;
}

void testLibLas() {
	std::string las = "C:/Users/hobbe/Desktop/RSC_test/road_sub.las";

	std::deque<double> dequeX;
	std::deque<double> dequeY;
	std::deque<double> dequeZ;
	std::deque<double> dequeI;

	ReadLasFileIntensite(las, dequeX, dequeY, dequeZ, dequeI);
	std::cout << dequeX[0] << " ; " << dequeY[0] << " ; "<< dequeZ[0] << " ; " << dequeI[0]<<"\n";

}

void testOpenCV() {
	cv::Mat img = cv::imread("C:/programmation/dev/code/RSC/lena.png");
	cv::namedWindow("image", cv::WINDOW_NORMAL);
	cv::imshow("image", img);
	cv::waitKey(0);
}

void testPCL() {
	pcl::PointCloud<PointXYZRGBI> cloud;

	//fill in the cloud data
	cloud.width = 50;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width*cloud.height);
	for (size_t i = 0; i < cloud.size(); i++) {
		cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].intensity = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].r = (uint8_t)1;
		cloud.points[i].g = 2;
		cloud.points[i].b = 3;
	}
	pcl::io::savePCDFileASCII("C:\\Users\\hobbe\\Desktop\\RSC_test\\mycloud.pcd", cloud);

}
