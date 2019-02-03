#pragma once
#include "Cloud.h" //WARNING :: NEED TO BE BEFOERE #include <windows.h> or PCL and FLANN CRASH DURING COMPILATION

#include <iostream>
#include <string>

#include <stdio.h>
#include <boost/filesystem.hpp>
#include <vector>


struct InArgs{
	std::string projectPath="";

	bool train = true;
	bool saveModel = false;
	std::string savePath="";

	bool predict = true;
	bool loadModel = false;
	std::string loadPath="";
	 
};



class Tools {
public:
	static InArgs argParser(int argc, char** argv);
	static void exitRSC(std::string message);
	static void printHelp();
	static void createFolder(const char * path);
	static void readDirectory(const std::string& name, std::vector<std::string>& v);
	static void readDirectoryExtension(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret);
	static void easterEgg();
	static void readCloudsInFolder(const std::string& folderPath, std::vector<Cloud>& outPtrClouds);
	static int getFileNumberInFolder(const std::string& folderPath);
private :
	struct path_leaf_string
	{
		std::string operator()(const boost::filesystem::directory_entry& entry) const
		{
			return entry.path().leaf().string();
		}
	};
};

//enum AlgoType { RF, SIG_SVM };
//
//cv::Mat formatInput(Cloud const& cloud);
//cv::Ptr <cv::ml::StatModel> createModels(AlgoType AT);
//
//struct PointXYZIRGBC
//{
//	PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
//	PCL_ADD_INTENSITY;
//	PCL_ADD_RGB;
//	float n;
//	int c;
//	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
//} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment
//
//POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRGBC,           // here we assume a XYZ + "test" (as fields)
//(float, x, x)
//(float, y, y)
//(float, z, z)
//(float, intensity, intensity)
//(float, rgb, rgb)
//(float, n, n)
//(int, c, c)
//)
//
//struct PointXYZRGBI
//{
//	PCL_ADD_POINT4D;
//	int r;
//	int g;
//	int b;
//	PCL_ADD_INTENSITY;
//	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//} EIGEN_ALIGN16;
//
//
//POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBI,
//(float, x, x)
//(float, y, y)
//(float, z, z)
//(int, r, r)
//(int, g, g)
//(int, b, b)
//(float, intensity, intensity)
//)
//
//std::ostream &operator<<(std::ostream &flux, PointXYZRGBI const& pt);
//
//int readCloudFromTxt(const std::string & fichier_in, pcl::PointCloud<PointXYZRGBI>& cloud, bool SkipFirst = true);
//int readCloudFromTxt(const std::string & fichier_in, pcl::PointCloud<pcl::PointXYZ>& cloud, bool SkipFirst = true, int skipNumber=0);
//int readCloudXYZRGBAIFromTxt(const std::string & fichier_in, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PtrCloudXYZRGBA, pcl::PointCloud<pcl::Intensity>::Ptr PtrCloudIntensity, bool SkipFirst = true, bool skipA = false);
//int ReadLas(const std::string & fichier_in, pcl::PointCloud<PointXYZRGBI>& cloud);
//int ReadLas(const std::string & fichier_in, pcl::PointCloud<pcl::PointXYZ>& cloud);
//int computeDensity(pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PtrCloud, float radius, std::vector<int>& density);
//
//#include <vector>
//#include <string>
//#include <opencv2/opencv.hpp>
//class Point{
//public:
//	//constructeurs
//	Point(double X, double Y, double Z, double I, int R, int G, int B, int N,
//		double normale, double densite, double planarite, int nbRetour, int numRetour, int classe);
//
//	//setter
//	void setX(double X);
//	void setY(double Y);
//	void setZ(double Z);
//	void setI(double I);
//
//	void setR(int R);
//	void setG(int G);
//	void setB(int B);
//	void setN(int N);
//	void setNormale(double normale);
//	void setDensite(double densite);
//	void setPlanarite(double planarite);
//	void setNbRetour(int nbRetour);
//	void setNumRetour(int numRetour);
//	void setClasse(int classe);
//
//	//getter
//	double getX();
//	double getY();
//	double getZ();
//	double getI();
//	int getR();
//	int getG();
//	int getB();
//	int getN();
//	double getNormale();
//	double getDensite();
//	double getPlanarite();
//	int getNbRetour();
//	int getNumRetour();
//	int getClasse();
//
//protected:
//	double m_X;
//	double m_Y;
//	double m_Z;
//	double m_I;
//	int m_R;
//	int m_G;
//	int m_B;
//	int m_N;
//	double m_normale;
//	double m_densite;
//	double m_planarite;
//	int m_nbRetour;
//	int m_numRetour;
//	int m_classe;
//};
//
//
//class Nuage {
//public:
//	//constructeurs
//	Nuage(std::vector<Point>& listePoints);
//	Nuage(std::string lien);
//
//	//fonctions
//	void setPointClass(int indicePoint, int classe);
//	void setAllPointClass(int classe);
//	void applyClassificationResults(cv::OutputArray& resultats);
//	int exportNuage(std::string lienExport="", std::string extention="");
//	void showPoints(int nb=0);
//
//	//setter
//	void setPoints(std::vector<Point>& points);
//	void setClasse(std::vector<Point>& points);
//
//	//getter
//	std::vector<Point> getPoints();
//	int getClasse();
//	bool getIsOneClass();
//	std::string getLien();
//
//protected:
//	std::vector<Point> m_points;
//	int m_classe;
//	void checkIsOneClass();
//	bool m_isOneClass;
//	std::string m_lien;
//};