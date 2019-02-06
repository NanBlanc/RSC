#include "Cloud.h"

Cloud::Cloud()
{
	m_XYZRGBACloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	m_intensityCloud = pcl::PointCloud<pcl::Intensity>::Ptr(new pcl::PointCloud<pcl::Intensity>);
	m_normalsCloud = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	m_kdtreeCloud = pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	m_density = std::vector<int>();
	m_label = std::vector<int>();
	m_path = "";
}
Cloud::Cloud(std::string in_path, bool SkipFirst, bool skipA)
{
	m_XYZRGBACloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	m_intensityCloud = pcl::PointCloud<pcl::Intensity>::Ptr(new pcl::PointCloud<pcl::Intensity>);
	m_normalsCloud = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	m_kdtreeCloud = pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGBA>);
	m_density = std::vector<int>();
	m_label = std::vector<int>();
	m_path = in_path;

	readCloudXYZRGBAIFromTxt(SkipFirst, skipA);
}
Cloud::Cloud(Cloud const& cloud)
{
	m_XYZRGBACloud = cloud.m_XYZRGBACloud;
	m_intensityCloud = cloud.m_intensityCloud;
	m_normalsCloud = cloud.m_normalsCloud;
	m_kdtreeCloud = cloud.m_kdtreeCloud;
	m_density = cloud.m_density;
	m_label = cloud.m_label;
	m_path = cloud.m_path;
}

void Cloud::readCloudXYZRGBAIFromTxt(bool SkipFirst, bool skipA) 
{
	m_XYZRGBACloud->is_dense = false;
	m_intensityCloud->is_dense = false;
	//1.ouverture du flux du fichier de centroide
	std::ifstream flux_in(m_path.c_str(), std::ios::in);

	if (flux_in.fail())
	{
		std::cerr << "erreur au chargement du nuage " << std::endl;
		return;
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

		m_XYZRGBACloud->points.push_back(pt);
		m_intensityCloud->points.push_back(pti);

		//if (compteur == 0)

		//	std::cout << pt;

		compteur++;
	}

	flux_in.close();


	return;
}

void Cloud::computeKdTree()
{
	m_kdtreeCloud->setInputCloud(m_XYZRGBACloud); //this builds the kdtree
	return;
}
void Cloud::computeDensity(float radius)
{
	//pre-allocate the neighbor index and distance vector and density
	if (this->m_kdtreeCloud->getInputCloud() == nullptr) {
		//std::cerr << "\nWARNING : FCT COMPUTE DENSITY : NO INPUT CLOUD IN KDTREE\n";
		this->computeKdTree();
	}

	std::vector<int> pointIdx;
	std::vector<float> pointSquaredD;
	m_density.reserve(m_XYZRGBACloud->size());
	//search

	for (int i = 0; i < m_XYZRGBACloud->size(); i++)
	{
		m_kdtreeCloud->radiusSearch(m_XYZRGBACloud->points[i], radius, pointIdx, pointSquaredD);
		//if (i < 2)
		//{
		//	for (int j = 0; j < 10; j++)
		//	{
		//		std::cerr << "i: " << i << "j: " << j << " : " << pointIdx[j] << ", " << pointSquaredD[j] << "\n";
		//	}
		//}
		m_density[i] = pointIdx.size();
	}
	return;
}
void Cloud::computeNormals()
{
	//pre-allocate the neighbor index and distance vector and density
	if (this->m_kdtreeCloud->getInputCloud() == nullptr) {
		//std::cerr << "\nWARNING : FCT COMPUTE DENSITY : NO INPUT CLOUD IN KDTREE\n";
		this->computeKdTree();
	}
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	ne.setInputCloud(m_XYZRGBACloud);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).

	ne.setSearchMethod(m_kdtreeCloud);

	// Use all neighbors in a sphere of radius 3cm
	ne.setKSearch(25);
	//set view point
	ne.setViewPoint(812.994, 599.005, 27.857);
	// Compute the features
	ne.compute(*m_normalsCloud);
	//for (int i = 0; i < 10; i++)
	//{
	//	std::cout << "curv : " << cloud_normals->points[i].curvature << ", normal : " << cloud_normals->points[i].normal << ", x: " << cloud_normals->points[i].normal_x << ", y:" << cloud_normals->points[i].normal_y << ", z:" << cloud_normals->points[i].normal_z << "\n";
	//}
	return;
}

void Cloud::savePredictedLabels(cv::Mat matPredictedLabels) {
	for (int i = 0; i < matPredictedLabels.rows; i++)// matPredictedLabels.rows
	{
		this->m_label.push_back(matPredictedLabels.at<float>(i, 0));
	}
	return;
}

void Cloud::savePredictedLabels(std::vector<int> vecPredictedLabels) {
	for (int i = 0; i < vecPredictedLabels.size(); i++)// matPredictedLabels.rows
	{
		this->m_label.push_back(vecPredictedLabels[i]);
	}
	return;
}

void Cloud::printFullCloud(std::ostream &flux)
{
	flux.precision(12);
	for (int i = 0; i < this->m_XYZRGBACloud->size(); i++)//this->XYZRGBACloud->size()
	{
		//flux << "1";
		flux << this->m_XYZRGBACloud->points[i].x << " " << this->m_XYZRGBACloud->points[i].y << " " << this->m_XYZRGBACloud->points[i].z << " "
			<< (int)this->m_XYZRGBACloud->points[i].r << " " << (int)this->m_XYZRGBACloud->points[i].g << " " << (int)this->m_XYZRGBACloud->points[i].b << " "
			<< (int)this->m_XYZRGBACloud->points[i].a << " " << this->m_intensityCloud->points[i].intensity << " " << this->m_normalsCloud->points[i].normal_x << " "
			<< this->m_normalsCloud->points[i].normal_y << " " << this->m_normalsCloud->points[i].normal_z << " " << this->m_label[i] << "\n";
	}
	return;
}

void Cloud::saveTxt(std::string projectPath)
{
	boost::filesystem::path p(this->m_path);
	std::string cloudName = p.filename().string();
	std::string outPath = projectPath + "\\out_clouds";
	std::cerr << "Saving " << outPath << "\\" << cloudName << " ... ";


	clock_t t0 = clock();
	std::ofstream output(outPath +"\\"+cloudName, std::ios::out | std::ios::trunc);
	output << "//X Y Z R G B A Intensity NX NY NZ Label\n";
	this->printFullCloud(output);
	clock_t t1 = clock();
	std::cerr << (t1 - t0) / static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;
}


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Cloud::XYZRGBACloud() { return m_XYZRGBACloud; }
pcl::PointCloud<pcl::Intensity>::Ptr Cloud::intensityCloud() { return m_intensityCloud; }
pcl::PointCloud<pcl::Normal>::Ptr Cloud::normalsCloud() { return m_normalsCloud; }
pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr Cloud::kdtreeCloud() { return m_kdtreeCloud; }
std::vector<int> Cloud::density() { return m_density; }
std::vector<int> Cloud::label() { return m_label; }
std::string Cloud::path() { return m_path; }

//setter
void Cloud::setXYZRGBACloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr XYZRGBACloud) { m_XYZRGBACloud = XYZRGBACloud; }
void Cloud::setIntensityCloud(pcl::PointCloud<pcl::Intensity>::Ptr intensityCloud) { m_intensityCloud = intensityCloud; }
void Cloud::setNormalsCloud(pcl::PointCloud<pcl::Normal>::Ptr normalsCloud) { m_normalsCloud = normalsCloud; }
void Cloud::setKdtreeCloud(pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtreeCloud) { m_kdtreeCloud = kdtreeCloud; }
void Cloud::setDensity(std::vector<int> density) { m_density = density; }
void Cloud::setLabel(std::vector<int> label) { m_label = label; }
void Cloud::setPath(std::string path) { m_path = path; }