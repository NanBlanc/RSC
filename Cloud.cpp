#include "Cloud.h"

Cloud::Cloud()
{
	XYZRGBACloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	intensityCloud = pcl::PointCloud<pcl::Intensity>::Ptr(new pcl::PointCloud<pcl::Intensity>);
	normalsCloud = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	kdtreeCloud = pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	density = std::vector<int>();
	label = std::vector<int>();
	path = "";
}
Cloud::Cloud(std::string in_path, bool SkipFirst, bool skipA)
{
	XYZRGBACloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	intensityCloud = pcl::PointCloud<pcl::Intensity>::Ptr(new pcl::PointCloud<pcl::Intensity>);
	normalsCloud = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	kdtreeCloud = pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	density = std::vector<int>();
	label = std::vector<int>();
	path = in_path;

	readCloudXYZRGBAIFromTxt(SkipFirst, skipA);
}
Cloud::Cloud(Cloud const& cloud)
{
	XYZRGBACloud = cloud.XYZRGBACloud;
	intensityCloud = cloud.intensityCloud;
	normalsCloud = cloud.normalsCloud;
	kdtreeCloud = cloud.kdtreeCloud;
	density = cloud.density;
	label = cloud.label;
	path = cloud.path;
}

int Cloud::readCloudXYZRGBAIFromTxt(bool SkipFirst, bool skipA) 
{
	XYZRGBACloud->is_dense = false;
	intensityCloud->is_dense = false;
	//1.ouverture du flux du fichier de centroide
	std::ifstream flux_in(path.c_str(), std::ios::in);

	if (flux_in.fail())
	{
		std::cerr << "erreur au chargement du nuage " << std::endl;
		return 1;
	}
	flux_in.precision(12);


	//2.recuperation des données
	std::string lstrli;
	int compteur = 0;


	while (std::getline(flux_in, lstrli))
	{


		if (lstrli == "")
			continue;
		if (SkipFirst == true && compteur == 0)
		{
			SkipFirst = false;
			continue;
		}

		std::istringstream iss;
		iss.str(lstrli);
		iss.precision(12);

		pcl::PointXYZRGBA pt;
		pcl::Intensity pti;
		std::string trash;
		int tempInt;
		double tempDouble;

		iss >> pt.x;
		iss >> pt.y;
		iss >> pt.z;

		iss >> tempInt;
		pt.r = tempInt;
		//std::cerr << "tempInt r" << tempInt << "\n";
		iss >> tempInt;
		pt.g = tempInt;
		iss >> tempInt;
		pt.b = tempInt;
		if (!skipA)
		{
			iss >> tempInt;
			pt.a = tempInt;
		}
		
		iss >> tempDouble;
		pti.intensity = tempDouble;
		//std::cerr << "tempDouble " << tempDouble << "\n";

		XYZRGBACloud->points.push_back(pt);
		intensityCloud->points.push_back(pti);

		//if (compteur == 0)

		//	std::cout << pt;

		compteur++;
	}

	flux_in.close();


	return 0;
}

int Cloud::computeKdTree()
{
	kdtreeCloud->setInputCloud(XYZRGBACloud); //this builds the kdtree
	return 0;
}
int Cloud::computeDensity(float radius)
{
	//pre-allocate the neighbor index and distance vector and density
	if (this->kdtreeCloud->getInputCloud() == nullptr) {
		//std::cerr << "\nWARNING : FCT COMPUTE DENSITY : NO INPUT CLOUD IN KDTREE\n";
		this->computeKdTree();
	}

	std::vector<int> pointIdx;
	std::vector<float> pointSquaredD;
	density.reserve(XYZRGBACloud->size());
	//search

	for (int i = 0; i < XYZRGBACloud->size(); i++)
	{
		kdtreeCloud->radiusSearch(XYZRGBACloud->points[i], radius, pointIdx, pointSquaredD);
		//if (i < 2)
		//{
		//	for (int j = 0; j < 10; j++)
		//	{
		//		std::cerr << "i: " << i << "j: " << j << " : " << pointIdx[j] << ", " << pointSquaredD[j] << "\n";
		//	}
		//}
		density[i] = pointIdx.size();
	}
	return 0;
}
int Cloud::computeNormals()
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	ne.setInputCloud(XYZRGBACloud);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).

	ne.setSearchMethod(kdtreeCloud);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(1);
	//set view point
	ne.setViewPoint(812.994, 599.005, 27.857);
	// Compute the features
	ne.compute(*normalsCloud);
	//for (int i = 0; i < 10; i++)
	//{
	//	std::cout << "curv : " << cloud_normals->points[i].curvature << ", normal : " << cloud_normals->points[i].normal << ", x: " << cloud_normals->points[i].normal_x << ", y:" << cloud_normals->points[i].normal_y << ", z:" << cloud_normals->points[i].normal_z << "\n";
	//}
	return 0;
}

int Cloud::savePredictedLabels(cv::Mat matPredictedLabels) {
	for (int i = 0; i < matPredictedLabels.rows; i++)// matPredictedLabels.rows
	{
		this->label.push_back(matPredictedLabels.at<float>(i, 0));
	}
	return 0;
}

int Cloud::savePredictedLabels(std::vector<int> vecPredictedLabels) {
	for (int i = 0; i < vecPredictedLabels.size(); i++)// matPredictedLabels.rows
	{
		this->label.push_back(vecPredictedLabels[i]);
	}
	return 0;
}

int Cloud::printFullCloud(std::ostream &flux)
{
	flux.precision(12);
	for (int i = 0; i < this->XYZRGBACloud->size(); i++)//this->XYZRGBACloud->size()
	{
		//flux << "1";
		flux << this->XYZRGBACloud->points[i].x << " " << this->XYZRGBACloud->points[i].y << " " << this->XYZRGBACloud->points[i].z << " "
			<< (int)this->XYZRGBACloud->points[i].r << " " << (int)this->XYZRGBACloud->points[i].g << " " << (int)this->XYZRGBACloud->points[i].b << " "
			<< (int)this->XYZRGBACloud->points[i].a << " " << this->intensityCloud->points[i].intensity << " " << this->normalsCloud->points[i].normal_x << " "
			<< this->normalsCloud->points[i].normal_y << " " << this->normalsCloud->points[i].normal_z << " " << this->label[i] << "\n";
	}
	return 0;
}

void Cloud::saveTxt(std::string projectPath)
{
	boost::filesystem::path p(this->path);
	std::string cloudName = p.filename().string();
	std::string outPath = projectPath + "\\out_clouds";
	std::cerr << "OUT::" << outPath << "\\" << cloudName << "\n";


	std::ofstream output(outPath +"\\"+cloudName, std::ios::out | std::ios::trunc);
	output << "//X Y Z R G B A Intensity NX NY NZ Label\n";
	this->printFullCloud(output);
}