#include "fonctions.h"
#include <windows.h> //for create directory

// When passing char arrays as parameters they must be pointers
InArgs Tools::argParser(int argc, char** argv) {
	InArgs input;
	if (argc < 2)
		exitRSC("Welcome on RSC, Please select a running mode : \n\t - all\n\t - train\n\t - classify\n\t - help\n");
	if (argc < 3 && (std::string(argv[1]) == "-all" || std::string(argv[1]) == "-train" || std::string(argv[1]) == "-classify"))
		exitRSC("ERROR : Missing Project Path, Please read help [-h] or [-help]");

	if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "-help")
	{
		Tools::printHelp();
		exitRSC("");
	}

	input.projectPath = argv[2];
	if (std::string(argv[1]) == "-all")
	{
		if (argc > 2) {
			for (int i = 3; i < argc; i++) { /* We will iterate over argv[] to get the parameters stored inside.* Note that we're starting on 1 because we don't need to know the * path of the program, which is stored in argv[0] */
				if (std::string(argv[i]) == "-s") {
					input.savePath = input.projectPath + "/models";
					input.saveModel = true;
				}
				else
					exitRSC("ERROR : invalid option for -all, Please read help [-h] or [-help]");
			}
		}
	}
	else if (std::string(argv[1]) == "-train")
	{
		input.predict = false;
		input.saveModel = true;
		input.savePath = input.projectPath + "\\models";
		if (argc > 3) 
			exitRSC("ERROR : invalid option for -train, Please read help [-h] or [-help]");
	}
	else if (std::string(argv[1]) == "-classify")
	{
		input.train = false;
		input.loadModel = true;
		if (argc > 3) {
			for (int i = 3; i < argc; i++) {
				if (std::string(argv[i]) == "-m") {
					if (i + 1 < argc) {
						input.loadPath = argv[i + 1];
						i++;
					}
					else 
						exitRSC("ERROR : invalid option for - classify, Please read help[-h] or [-help]");
				
				}
				else
					exitRSC("ERROR : invalid option for -classify, Please read help [-h] or [-help]");
			}
		}
		else {
			std::cerr << "Warning : Default path for model, First one will be selcted, Please read help [-h] or [-help]\n";
			boost::filesystem::path p(input.projectPath + "\\models");
			std::vector<boost::filesystem::path> vecModels;
			readDirectoryExtension(p, ".yml", vecModels);
			if (vecModels.size() == 0)
				exitRSC("ERROR : No Models found");
			input.loadPath = input.projectPath + "\\models\\" +vecModels[0].string();
			std::cerr << "Warning : Default model load : " << input.loadPath << "\n";
		}
	}
	else
	{
		exitRSC("Select a running mode:\n\t-all\n\t-train\n\t-classify\n\t-help\n");
	}
	return input;
}

void Tools::exitRSC(std::string message) {
	std::cerr << message;
	std::cerr << "---\\-\\-\\-\\-\\END/-/-/-/-/-/---\n";
	Sleep(2000);
	exit(0);
}
void Tools::printHelp() {
	std::cerr << "RSC Program help menu:\n";
	std::cerr << "1-RSC call\n";
	std::cerr << "2-Project directory structure\n";
	std::cerr << "3-Informations\n";
	std::cerr << "--------------\n";
	std::cerr << "1- RSC call :\n";
	std::cerr << "3 possible modes : \n";
	std::cerr << "-all : train and classify accordingly to data in project path\n";
	std::cerr << "\tmandatory args :\n";
	std::cerr << "\t\tproject path\n";
	std::cerr << "\toptional args : \n";
	std::cerr << "\t\t-s : save trained model in [projectPath]/models\n";
	std::cerr << "\texemple : [RSCpath] -all C:/desktop/Project -s\n";
	std::cerr << "-train : train a model and save it accordingly to data in project path\n";
	std::cerr << "\tmandatory args :\n";
	std::cerr << "\t\tproject path\n";
	std::cerr << "-classify : classify data with a given model\n";
	std::cerr << "\tmandatory args :\n";
	std::cerr << "\t\tproject path\n";
	std::cerr << "\toptional args : \n";
	std::cerr << "\t\t-m : model path \n";
	std::cerr << "\t\t /!\\ : if not given, will load the first model in [ProjectPath]/models\n";
	std::cerr << "-h : show help... help-ception !?\n";
	std::cerr << "--------------\n";
	std::cerr << "2-Project directory structure :\n";
	std::cerr << "MyProject\n";
	std::cerr << "\tGround_truth\n";
	std::cerr << "\t\tClass_1\n";
	std::cerr << "\t\t\tCloud_1\n";
	std::cerr << "\t\t\tCloud_2\n";
	std::cerr << "\t\t\tCloud...\n";
	std::cerr << "\t\tClass 2\n";
	std::cerr << "\t\t\tCloud_1\n";
	std::cerr << "\t\t\tCloud...\n";
	std::cerr << "\t\tClass... [optional : 2 Class at least]\n";
	std::cerr << "\tModels [optional : if -all or -train ; automatically create when saving model]\n";
	std::cerr << "\tIn_clouds [optional if only -train]\n";
	std::cerr << "\tOut_clouds [optional : automatically create if -all or -classify\n";
	std::cerr << "--------------\n";
	std::cerr << "3-Informations :\n";
	std::cerr << "Written by :\n";
	std::cerr << "\tOlivier Stocker\n";
	std::cerr << "\tstocker.olivier@gmail.com\n";
	std::cerr << "\n";
	exitRSC("");
}

void Tools::createFolder(const char * path)
{
	if (!CreateDirectory(path, NULL))
	{
		return;
	}
}



void Tools::readDirectory(const std::string& name, std::vector<std::string>& v)
{
	boost::filesystem::path p(name);
	boost::filesystem::directory_iterator start(p);
	boost::filesystem::directory_iterator end;
	std::transform(start, end, std::back_inserter(v), path_leaf_string());
}




void Tools::readDirectoryExtension(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret)
{
	if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return;

	boost::filesystem::recursive_directory_iterator it(root);
	boost::filesystem::recursive_directory_iterator endit;
	while (it != endit)
	{
		if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ext) ret.push_back(it->path().filename());
		++it;
	}
}

void Tools::readCloudsInFolder(const std::string& folderPath, std::vector<Cloud>& outPtrClouds)
{
	std::vector<boost::filesystem::path> inCloudsPath;
	Tools::readDirectoryExtension(folderPath, ".txt", inCloudsPath);
	for (int i = 0; i < inCloudsPath.size(); i++) {
		outPtrClouds.push_back(Cloud(folderPath + "\\" + inCloudsPath[i].string()));
	}
}

int Tools::getFileNumberInFolder(const std::string& folderPath)
{
	std::vector<std::string> v;
	Tools::readDirectory(folderPath, v);
	return v.size();
}


	//if (argc < 5) { // Check the value of argc. If not enough parameters have been passed, inform user and exit.
	//	std::cout << "Usage is -in <infile> -out <outdir>\n"; // Inform the user of how to use the program
	//	std::cin.get();
	//	exit(0);
	//}
	//else { // if we got enough parameters...
	//	char* myFile, myPath, myOutPath;
	//	std::cout << argv[0];
	//	for (int i = 1; i < argc; i++) { /* We will iterate over argv[] to get the parameters stored inside.
	//									  * Note that we're starting on 1 because we don't need to know the
	//									  * path of the program, which is stored in argv[0] */
	//		if (i + 1 != argc) // Check that we haven't finished parsing already
	//			if (argv[i] == "-f") {
	//				// We know the next argument *should* be the filename:
	//				myFile = argv[i + 1];
	//			}
	//			else if (argv[i] == "-p") {
	//				myPath = argv[i + 1];
	//			}
	//			else if (argv[i] == "-o") {
	//				myOutPath = argv[i + 1];
	//			}
	//			else {
	//				std::cout << "Not enough or invalid arguments, please try again.\n";
	//				Sleep(2000);
	//				exit(0);
	//			}
	//		std::cout << argv[i] << " ";
	//	}
	//	//... some more code
	//	std::cin.get();
	//	return 0;
	//}











//
//cv::Mat formatInput(Cloud const& cloud) 
//{
//	int nbparam = 7;
//
//	cv::Mat feature_row = cv::Mat::ones(1, nbparam, CV_32F);
//	cv::Mat features;
//
//	for(int i =0;i<cloud.XYZRGBACloud->size();i++)
//	{
//		
//		feature_row.at<float>(0, 0) = cloud.XYZRGBACloud->points[i].r;
//		feature_row.at<float>(0, 1) = cloud.XYZRGBACloud->points[i].g;
//		feature_row.at<float>(0, 2) = cloud.XYZRGBACloud->points[i].b;
//		feature_row.at<float>(0, 3) = cloud.intensityCloud->points[i].intensity;
//		feature_row.at<float>(0, 4) = cloud.normalsCloud->points[i].normal_x;
//		feature_row.at<float>(0, 5) = cloud.normalsCloud->points[i].normal_y;
//		feature_row.at<float>(0, 6) = cloud.normalsCloud->points[i].normal_z;
//
//		features.push_back(feature_row);
//	}
//
//	return features;
//}
//
//cv::Ptr<cv::ml::StatModel> createModels(AlgoType AT)
//{
//	cv::Ptr<cv::ml::StatModel> modelPtr;
//
//	switch (AT) {
//	case AlgoType::RF: 
//	{
//		auto random_forest = cv::ml::RTrees::create(); 
//
//		auto criter = cv::TermCriteria();
//		criter.type = CV_TERMCRIT_EPS + CV_TERMCRIT_ITER;
//		criter.epsilon = 1e-8;
//		criter.maxCount = 5000;
//
//		// parameters for random forest
//		random_forest->setMaxCategories(2);
//		random_forest->setMaxDepth(3000);
//		random_forest->setMinSampleCount(1);
//		random_forest->setTruncatePrunedTree(false);
//		random_forest->setUse1SERule(false);
//		random_forest->setUseSurrogates(false);
//		random_forest->setPriors(cv::Mat());
//		random_forest->setTermCriteria(criter);
//		random_forest->setCVFolds(1);
//
//		modelPtr = random_forest;
//	}
//		break;
//	case AlgoType::SIG_SVM: 
//	{
//		auto sigmoid_svm = cv::ml::SVM::create();
//
//		auto criter_svm = cv::TermCriteria();
//		criter_svm.type = CV_TERMCRIT_EPS;
//		criter_svm.epsilon = 1e-10;
//
//		// parameters for rbf support vector machines
//		sigmoid_svm->setC(100);
//		sigmoid_svm->setTermCriteria(criter_svm);
//		sigmoid_svm->setCoef0(0.3);
//		sigmoid_svm->setKernel(sigmoid_svm->SIGMOID);
//		sigmoid_svm->setGamma(0.1);
//		sigmoid_svm->setType(sigmoid_svm->C_SVC);
//
//		modelPtr = sigmoid_svm;
//	}
//		break;
//	}
//
//	return modelPtr;
//}
//
//
//
//std::ostream &operator<<(std::ostream &flux, PointXYZRGBI const& pt)
//{
//	flux << pt.x << ", " << pt.y << ", " << pt.z << ", " << pt.r << ", " << pt.g << ", " << pt.b << ", " << pt.intensity << "\n";
//	return flux;
//}
//
//
//
//int readCloudFromTxt(const std::string & fichier_in, pcl::PointCloud<PointXYZRGBI>& cloud, bool SkipFirst) {
//
//	cloud.is_dense = false;
//
//	//1.ouverture du flux du fichier de centroide
//	std::ifstream flux_in(fichier_in.c_str(), std::ios::in);
//
//	if (flux_in.fail())
//	{
//		std::cerr << "erreur au chargement du nuage " << std::endl;
//		return 1;
//	}
//	flux_in.precision(12);
//
//
//	//2.recuperation des données
//	std::string lstrli;
//	int compteur = 0;
//
//
//	while (std::getline(flux_in, lstrli))
//	{
//
//
//		if (lstrli == "")
//			continue;
//		if (SkipFirst == true && compteur == 0)
//		{
//			SkipFirst = false;
//			continue;
//		}
//
//		std::istringstream iss;
//		iss.str(lstrli);
//		iss.precision(12);
//
//		PointXYZRGBI pt;
//
//		iss >> pt.x;
//		iss >> pt.y;
//		iss >> pt.z;
//		iss >> pt.r;
//		iss >> pt.g;
//		iss >> pt.b;
//		iss >> pt.intensity;
//
//		cloud.points.push_back(pt);
//		if(compteur==0)
//
//			std::cout << pt;
//
//		compteur++;
//	}
//
//	flux_in.close();
//
//	
//	return 0;
//}
//int readCloudFromTxt(const std::string & fichier_in, pcl::PointCloud<pcl::PointXYZ>& cloud, bool SkipFirst, int skipNumber) {
//
//	cloud.is_dense = false;
//
//	//1.ouverture du flux du fichier de centroide
//	std::ifstream flux_in(fichier_in.c_str(), std::ios::in);
//
//	if (flux_in.fail())
//	{
//		std::cerr << "erreur au chargement du nuage " << std::endl;
//		return 1;
//	}
//	flux_in.precision(12);
//
//
//	//2.recuperation des données
//	std::string lstrli;
//	int compteur = 0;
//
//
//	while (std::getline(flux_in, lstrli))
//	{
//
//
//		if (lstrli == "")
//			continue;
//		if (SkipFirst == true && compteur == 0)
//		{
//			SkipFirst = false;
//			continue;
//		}
//
//		std::istringstream iss;
//		iss.str(lstrli);
//		iss.precision(12);
//
//		pcl::PointXYZ pt;
//		std::string trash;
//		iss >> pt.x;
//		iss >> pt.y;
//		iss >> pt.z;
//		for (int i = 0; i < skipNumber; i++)
//		{
//			iss >> trash;
//		}
//
//
//		cloud.points.push_back(pt);
//		//if (compteur == 0)
//
//		//	std::cout << pt;
//
//		compteur++;
//	}
//
//	flux_in.close();
//
//
//	return 0;
//}
//int readCloudXYZRGBAIFromTxt(const std::string & fichier_in, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PtrCloudXYZRGBA, pcl::PointCloud<pcl::Intensity>::Ptr PtrCloudIntensity, bool SkipFirst, bool skipA) {
//	PtrCloudXYZRGBA->is_dense = false;
//	PtrCloudIntensity->is_dense = false;
//	//1.ouverture du flux du fichier de centroide
//	std::ifstream flux_in(fichier_in.c_str(), std::ios::in);
//
//	if (flux_in.fail())
//	{
//		std::cerr << "erreur au chargement du nuage " << std::endl;
//		return 1;
//	}
//	flux_in.precision(12);
//
//
//	//2.recuperation des données
//	std::string lstrli;
//	int compteur = 0;
//
//
//	while (std::getline(flux_in, lstrli))
//	{
//
//
//		if (lstrli == "")
//			continue;
//		if (SkipFirst == true && compteur == 0)
//		{
//			SkipFirst = false;
//			continue;
//		}
//
//		std::istringstream iss;
//		iss.str(lstrli);
//		iss.precision(12);
//
//		pcl::PointXYZRGBA pt;
//		pcl::Intensity pti;
//		std::string trash;
//		int tempInt;
//		double tempDouble;
//
//		iss >> pt.x;
//		iss >> pt.y;
//		iss >> pt.z;
//		iss >> tempInt;
//	
//		pt.r = tempInt;
//		iss >> tempInt;
//		pt.g = tempInt;
//		iss >> tempInt;
//		pt.b = tempInt;
//		if (!skipA)
//		{
//			iss >> tempInt;
//			pt.b = tempInt;
//		}
//		iss >> tempDouble;
//		pti.intensity = tempDouble;
//		//std::cerr << "tempDouble " << tempDouble << "\n";
//
//		PtrCloudXYZRGBA->points.push_back(pt);
//		PtrCloudIntensity->points.push_back(pti);
//
//		//if (compteur == 0)
//
//		//	std::cout << pt;
//
//		compteur++;
//	}
//
//	flux_in.close();
//
//
//	return 0;
//}
//int ReadLas(const std::string & fichier_in, pcl::PointCloud<pcl::PointXYZ>& cloud)
//{
//	//const clock_t t0 = clock();
//	std::cout << "XJDJDJDJDJ\n";
//	std::ifstream ifs(fichier_in.c_str(), std::ios::in | std::ios::binary);
//	if (ifs.fail())
//	{
//
//		std::cerr << "erreur a l ouverture du flux pour " << fichier_in << std::endl;
//		return 1;
//	}
//
//	liblas::ReaderFactory f;
//	liblas::Reader reader = f.CreateWithStream(ifs);
//	liblas::Header const& header = reader.GetHeader();
//
//	unsigned long int nb_point = header.GetPointRecordsCount();
//
//	std::cout << "nb_point : " << nb_point << "\n";
//	cloud.width = nb_point;
//	cloud.height = 1;
//	cloud.is_dense = false;
//	cloud.points.resize(cloud.width*cloud.height);
//
//	unsigned long int compteur = 0;
//	while (reader.ReadNextPoint())
//	{
//		if (compteur > nb_point)
//		{
//			std::cerr << compteur << ";" << nb_point << std::endl;
//			std::cerr << "erreur entete du fichier corrompu" << std::endl;
//			return 1;
//		}
//		liblas::Point const& p = reader.GetPoint();
//
//		if (compteur < 10)
//			std::cout << p.GetX() << ", " << p.GetY() << ", " << p.GetZ() << "\n";
//
//		cloud.points[compteur].x = static_cast<float>(p.GetX());
//		cloud.points[compteur].y = (float)p.GetY();
//		cloud.points[compteur].z = (float)p.GetZ();
//
//
//		if (compteur == 0)
//			std::cout << cloud.points[0].x << ", " << cloud.points[0].y << ", " << cloud.points[0].z << "\n";
//		// P.GetIntensity
//		compteur++;
//	}
//
//	if (compteur != nb_point)
//	{
//		std::cerr << compteur << ";" << nb_point << std::endl;
//		std::cerr << "erreur entete du fichier corrompu" << std::endl;
//		return 1;
//	}
//
//	//const clock_t t1 = clock();
//	//std::cerr << "temps lecture                  : " << (t1-t0)/static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;
//
//	//FIN DU PROGRAMME READLASFILEINTENSITE
//	return 0;
//}
//int ReadLas(const std::string & fichier_in, pcl::PointCloud<PointXYZRGBI>& cloud)
//{
//	//const clock_t t0 = clock();
//	std::cout << "XJDJDJDJDJ\n";
//	std::ifstream ifs(fichier_in.c_str(), std::ios::in | std::ios::binary);
//	if (ifs.fail())
//	{
//
//		std::cerr << "erreur a l ouverture du flux pour " << fichier_in << std::endl;
//		return 1;
//	}
//
//	liblas::ReaderFactory f;
//	liblas::Reader reader = f.CreateWithStream(ifs);
//	liblas::Header const& header = reader.GetHeader();
//
//	unsigned long int nb_point = header.GetPointRecordsCount();
//
//		std::cout << "nb_point : " << nb_point  << "\n";
//	cloud.width = nb_point;
//	cloud.height = 1;
//	cloud.is_dense = false;
//	cloud.points.resize(cloud.width*cloud.height);
//	unsigned long int compteur = 0;
//	while (reader.ReadNextPoint())
//	{
//		if (compteur > nb_point)
//		{
//			std::cerr << compteur << ";" << nb_point << std::endl;
//			std::cerr << "erreur entete du fichier corrompu" << std::endl;
//			return 1;
//		}
//		liblas::Point const& p = reader.GetPoint();
//
//		if (compteur<10)
//			std::cout << p.GetX() << ", " << p.GetY() << ", " << p.GetZ() << "\n";
//
//		cloud.points[compteur].x = p.GetX();
//		cloud.points[compteur].y = p.GetY();
//		cloud.points[compteur].z = p.GetZ();
//		cloud.points[compteur].intensity = p.GetIntensity();
//		cloud.points[compteur].r = (int)p.GetColor().GetRed();
//		cloud.points[compteur].g = (int)p.GetColor().GetGreen();
//		cloud.points[compteur].b = (int)p.GetColor().GetBlue();
//
//		if (compteur == 1)
//			std::cout << cloud.points[0].x << ", " << cloud.points[0].y << ", " << cloud.points[0].z << cloud.points[0].intensity << ", " << cloud.points[0].r << ", " << cloud.points[0].g << ", " << cloud.points[0].b << "\n";
//		// P.GetIntensity
//		compteur++;
//	}
//
//	if (compteur != nb_point)
//	{
//		std::cerr << compteur << ";" << nb_point << std::endl;
//		std::cerr << "erreur entete du fichier corrompu" << std::endl;
//		return 1;
//	}
//
//	//const clock_t t1 = clock();
//	//std::cerr << "temps lecture                  : " << (t1-t0)/static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;
//
//	//FIN DU PROGRAMME READLASFILEINTENSITE
//	return 0;
//}
//int computeDensity(pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PtrCloud, float radius, std::vector<int>& density) {
//	//pre-allocate the neighbor index and distance vector and density
//	if (kdtree->getInputCloud() == nullptr) {
//		//std::cerr << "\nWARNING : FCT COMPUTE DENSITY : NO INPUT CLOUD IN KDTREE\n";
//		kdtree->setInputCloud(PtrCloud);
//	}
//	std::vector<int> pointIdx;
//	std::vector<float> pointSquaredD;
//	density.reserve(PtrCloud->size());
//	//search
//	for (int i = 0; i < PtrCloud->size(); i++)
//	{
//		kdtree->radiusSearch(PtrCloud->points[i], radius, pointIdx, pointSquaredD);
//		//for (int i = 0; i < pointIdx.size(); i++)
//		//{
//		//	std::cerr << i << " : " << pointIdx[i] << "\n";
//		//}
//		density[i] = pointIdx.size();
//	}
//	for (int i = 0; i < 10; i++)
//	{
//		std::cerr << i << " : " << density[i] << "\n";
//	}
//	return 0;
//}
////classe Point
////constructeur
//Point::Point(double X, double Y, double Z, double I, int R, int G, int B, int N,
//	double normale, double densite, double planarite, int nbRetour, int numRetour, int classe) :
//	m_X(X), m_Y(Y), m_Z(Z), m_I(I), m_R(R), m_G(G), m_B(B), m_N(N), m_normale(normale),
//	m_densite(densite), m_planarite(planarite), m_nbRetour(nbRetour), m_numRetour(numRetour), m_classe(classe) {}
//
////setter
//void Point::setX(double X) { m_X = X; }
//void Point::setY(double Y) { m_Y = Y; }
//void Point::setZ(double Z) { m_Z = Z; }
//void Point::setI(double I) { m_I = I; }
//void Point::setR(int R) { m_R = R; }
//void Point::setG(int G) { m_G = G; }
//void Point::setB(int B) { m_B = B; }
//void Point::setN(int N) { m_N = N; }
//void Point::setNormale(double normale) { m_normale = normale; }
//void Point::setDensite(double densite) { m_densite = densite; }
//void Point::setPlanarite(double planarite) { m_planarite = planarite; }
//void Point::setNbRetour(int nbRetour) { m_nbRetour = nbRetour; }
//void Point::setNumRetour(int numRetour) { m_numRetour = numRetour; }
//void Point::setClasse(int classe) { m_classe = classe; }
//
////getter
//double Point::getX() { return m_X; }
//double Point::getY() { return m_Y; }
//double Point::getZ() { return m_Z; }
//double Point::getI() { return m_I; }
//int Point::getR() { return m_R; }
//int Point::getG() { return m_G; }
//int Point::getB() { return m_B; }
//int Point::getN() { return m_N; }
//double Point::getNormale() { return m_normale; }
//double Point::getDensite() { return m_densite; }
//double Point::getPlanarite() { return m_planarite; }
//int Point::getNbRetour() { return m_nbRetour; }
//int Point::getNumRetour() { return m_numRetour; }
//int Point::getClasse() { return m_classe; }
//
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
////classe nuage
////constructeur
//Nuage::Nuage(std::vector<Point>& listePoints): m_points(listePoints) {
//	m_classe = 0;
//	m_isOneClass = false;
//	this->checkIsOneClass();
//	m_lien = "";
//}
//Nuage::Nuage(std::string lien): m_lien(lien) {
//	Point a(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//	std::vector<Point> b;
//	b.push_back(a);
//	m_points = b;
//	m_classe = 0;
//	m_isOneClass = false;
//	checkIsOneClass();
//	
//	//std::cout << "gfghjhghgg" << "\n";
//	//pcl::PointCloud<PointXYZIRGB>::Ptr cloud (new pcl::PointCloud<PointXYZIRGB>);
//	//if(pcl::io::loadPCDFile<PointXYZIRGB>(lien,*cloud)==-1)
//	//{
//	//	std::string alert = "/!\\FAIL TO READ FILE : " + lien + "\n";
//	//	std::cout << alert;
//	//}
//	//std::cout << "width: " <<cloud->width << "\n";
//	//std::cout << "height: " << cloud->height << "\n";
//	//std::cout << "pointvalue X: " << cloud->at(0).x;
//
//	//float c[4];
//	//c[0] = 1;
//	//std::cout << "c : " << c[0] << "\n";
//
//
//}
//
////fonctions publiques
//void Nuage::setPointClass(int indicePoint, int classe) { return; }
//void Nuage::setAllPointClass(int classe) { return; }
//void Nuage::applyClassificationResults(cv::OutputArray& resultats) { return; }
//int Nuage::exportNuage(std::string lienExport, std::string extention) { return 0; }
//void Nuage::showPoints(int nb) {
//	if (nb == 0 || nb > m_points.size())
//		nb = this->m_points.size();
//
//	for (int i = 0; i < nb; i++)
//	{
//		std::cout << "indice: " << i << " X: " << m_points[i].getX() << " Y: " << m_points[i].getY() << " Z: " << m_points[i].getZ() << " I: " << m_points[i].getI()
//			<< " R: " << m_points[i].getR() << " G: " << m_points[i].getG() << " B: " << m_points[i].getB() << " N: " << m_points[i].getN() << " Classe: " << m_points[i].getClasse() << "\n";
//	}
//}
////fonction privée
//void Nuage::checkIsOneClass() {
//	if(m_points.size()==0) { 
//		this->m_isOneClass = false; 
//		return;
//	}
//	int classePoint1 = m_points[0].getClasse();
//	for each (Point pt in m_points)
//	{
//		if (pt.getClasse() != classePoint1) { this->m_isOneClass = false; }
//		else { this->m_isOneClass = true; }
//	}
//	return;
//}
//
////setter
//void Nuage::setPoints(std::vector<Point>& points){}
//void Nuage::setClasse(std::vector<Point>& points){}
//
////getter
//std::vector<Point> Nuage::getPoints() { return m_points; }
//int Nuage::getClasse() { return m_classe; }
//bool Nuage::getIsOneClass() { return m_isOneClass; }
//std::string Nuage::getLien() { return m_lien; }

void Tools::easterEgg() {
	std::cerr << "                                 *//((((((((((########%%%%%%%%%%%%%%%%%%%%%####%#//***..,                            \n";
	std::cerr << "                               ,/(((##(##((######%%%%%%%%%%%%%%%%%%%%%%%%%%##%%%##///***,,.                          \n";
	std::cerr << "                          ..,,,/(####((########%%%##%%%%%%%%%#%%%%%%%%%%%%###%%%%%%##((/***,,,.*,                    \n";
	std::cerr << "                        ....,,/*/#############%%%((#%#########%##%%%%%%%%%%##%%%%%%%%%%%%%%%%%%%##//(                \n";
	std::cerr << "                      ....../,,(#%%########%%%%%((#########%%%%%%%%%%%%&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##(****       \n";
	std::cerr << "                     .,,,,./((##%%%%%%%%%%%%%%%%%%%%%%%%&%%%&&&&&&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#####(****.\n";
	std::cerr << "                    ,*,..,*(##%%%%#(***********//////////////(((((((((((((((((/((((#%%%%%%%%%%%%##%%%%%%%%###########\n";
	std::cerr << "                    ,,.,*(((#*,****////*//*/******/////////////////////////////(((((((######%#####(***,//############\n";
	std::cerr << "                  ..,,/(((/****/*/(((((((((((((///////////////////////////(((((((((((###(##########(**.     */(######\n";
	std::cerr << "                  *,/((((**///((((((((###########(((/////////////////////((((((((((((#############(((/.            /#\n";
	std::cerr << "                  **(#(///((((((((((((#############((((((/(////////////(((((((((((((((((((((/(((####((/              \n";
	std::cerr << "                  **##((/((///////((((((((#########((((((((((((((((((((((((((((((((((((/(////////((((((*,            \n";
	std::cerr << "                  */((/////*///(((####################((((((((((((((((((((((((((((((((####%%%#####(((((/,,,,,        \n";
	std::cerr << "               .***(#%%%%##(((((((((########%%%%%%##########(((((((((((((((((%%%%%#((((((((((///////((#####(*,,,.    \n";
	std::cerr << "        **/##%%%%%%#//(*//((((####%%%############%%%%%%%%%%&%%%%%%%%%%%%%%%%%(((((((#########((((///**,   /########(/\n";
	std::cerr << "        %%%(%#%%%(    (//(((###########%%###########%%%%%%&&%%##%%%%%%%%%%#((((#################((//**.    ,/########\n";
	std::cerr << "        /#%%%%%#/     /*/(######(////(((((((#########%%%%#(((((((((((%%%%#((#######(((///////((((((//*     ,(((####*,\n";
	std::cerr << "          *#%%%###( ,*,*/((###////(%%%%%%%%(##########%%%%((((///(((#%%%%(((#####((#%%%%%%#///*((((//    (####/###,,,\n";
	std::cerr << "           *%%#/.*(##***//****///(#%%%%%%%%##########(%%%%#/*******/#%%%#(((#######%%%%%%%%#(((//*///**##(,   *(#(,  \n";
	std::cerr << "            /%#/    ./*,,,**/////((((((((((########(/(%%%/*,..,,,,,**/%%%(((((((######((((////////****,       /(#,,  \n";
	std::cerr << "             (%#.   ****,****//(((((((((((((((((/*//(%%#/*,...,,,,,,**/#%%////(((((((((((((///////****,      *(#*,   \n";
	std::cerr << "              /%#   *,*****,*****////////////*****/#%%(/**,,..,,,,,*****(%%(/////////////////////*****,     *(#,     \n";
	std::cerr << "              **#%(**,****,,,,,,,,,**************/%%%(/**,,..,,,,,,,*****/#%%(/////////////////********,  *(#*       \n";
	std::cerr << "                **#%#/******,,,,,,*************#%%##(/**,,,...,,,,,,,*****/(##%%///////////////*********/##,,        \n";
	std::cerr << "                 /(,,,%%(*,*********/////*,#%%#(((((/*,,,,....,,,*,,,******//(((##%%#//////////*****(#%/*,,,         \n";
	std::cerr << "                  ,,,,,****/##%%%%%%%%%%%####((((((*,,,,..,,,,*************////((((((((#########//*******,,,         \n";
	std::cerr << "                  .,,,,*****////(((((((((((((((((***,,,,,********************////((((((((((//////********//          \n";
	std::cerr << "                  ,,,,,******////////////(((((******/////////////////**********/((///////////////********##          \n";
	std::cerr << "                   ,********/////////////((((/////////////////////////////*****//////////////////*******,%%          \n";
	std::cerr << "                   ,*******/////////////////(((((##%%%(/////////////**(%%##(////////////////////*********,.          \n";
	std::cerr << "                   ******////////(((((/////////((#######(/////////*/((((((((/(//////////////////*********(           \n";
	std::cerr << "                   ****///////(((((((((////////((((((####(((/////////(((((((/////////////////////*******/,           \n";
	std::cerr << "                   **////////((######(((((((////((((((#(#((((((((((((((((((((((((/////////////////******(            \n";
	std::cerr << "                   (*///////((##%######(((((((((((((((#####((((((((((((((((((((((((((((((((((((///*****/             \n";
	std::cerr << "                   %///////((#########(##((((((((##############((((#####(##((((((((((((((((((((//******/             \n";
	std::cerr << "                   %*(//////(((###%%%%####################%%%%%%%%%%##########((((((((######((//******(              \n";
	std::cerr << "                    ,(#(((/(((((((########((((((((((((((((###########(((((((((/////((######((//**//////              \n";
	std::cerr << "                      (((((((((((((((((((((((((((((((((((((((((((((((((////////////((((((((////////(((               \n";
	std::cerr << "                      /##(((((((((((((/////////(((((((###(((((((((((((((/////////////////////////((((                \n";
	std::cerr << "                       /(%####(((((((((/////////(((((((((((((#(((((((((((////////////////////(((###(                 \n";
	std::cerr << "                        (#%%%#####((((((((///////(((((((((((((((((((((////////////////////(((#####(                  \n";
	std::cerr << "                          (%%%%%##(#((((((((((((((((((((((((((((((((((//////////////////(((####(#(                   \n";
	std::cerr << "                             %%%%%###((((((((((((#(#((((((((((((((((((////////((((((((((((#####(/                    \n";
	std::cerr << "                              (#%%%%%###(((((((((((((((((((((###(((((((((((((((((((((((#######(#                     \n";
	std::cerr << "                               /(#%%%##%####(((((((((((((((#((##((((((((((((((((#############(#                      \n";
	std::cerr << "                                 (##%%%%########################((((((((((((##############(((                        \n";
	std::cerr << "                                   (#%%%%%########################(#(((##%%%%%#%###########                          \n";
	std::cerr << "                                     (%%%%%%%%#%%%##%%%%%###%%%%%####%%%%%%%%%%%#%%%####(                            \n";
	std::cerr << "                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%##                               \n";
	std::cerr << "                                         .##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%#                                  \n";
	std::cerr << "                                               ##%%%%%%%%%%%%%%%%%%%%%%##%%####%                                     \n";
	std::cerr << "                                                        ////////*                                                    \n";
	std::cerr << "                                                                                                                             \n";
	std::cerr << "         @@@@@      @@@@@@@@         @@@@@@@@#   @@@@@      @@@@@@@@    @@@@@@@@@   #@@@@@@@@   @@@@@% %@@@@@@@@.@@@@        \n";
	std::cerr << "         @@@@@      @@@@@@@@       *@@@@@&@@@@@  @@@@@      @@@@@@@@   @@@@@@@@@@& &@@@@@@@@@@  @@@@@% %@@@@@@@@.@@@@        \n";
	std::cerr << "         @@@@@      @@@@@@@@*      @@@@@% @@@@@  @@@@@     #@@@@@@@@.  @@@@@ @@@@@ @@@@@ %@@@@. @@@@@% %@@@@@    @@@         \n";
	std::cerr << "         @@@@@     %@@@@@@@@@      @@@@@% @@@@@  @@@@@     @@@@@@@@@@  @@@@@@@     /@@@@@@/     @@@@@% %@@@@@,,,             \n";
	std::cerr << "         @@@@@     @@@@@(@@@@      @@@@@%        @@@@@     @@@@@@@@@@   @@@@@@@@&    @@@@@@@@   @@@@@% %@@@@@@@@             \n";
	std::cerr << "         @@@@@     @@@@% @@@@,     @@@@@% &&&&&  @@@@@     @@@@/%@@@@      @@@@@@@     ,@@@@@@* @@@@@% %@@@@@(((             \n";
	std::cerr << "         @@@@@    .@@@@@@@@@@@     @@@@@% @@@@@  @@@@@    /@@@@@@@@@@* @@@@@ @@@@@ &@@@@ .@@@@@ @@@@@% %@@@@@                \n";
	std::cerr << "         @@@@@@@@ @@@@@@@@@@@@     #@@@@& @@@@@  @@@@@@@@ @@@@@@@@@@@@ @@@@@ @@@@@ /@@@@. @@@@@ @@@@@% %@@@@@                \n";
	std::cerr << "         @@@@@@@@ @@@@@, @@@@@      @@@@@@@@@@   @@@@@@@@ @@@@@  @@@@@ *@@@@@@@@@%  @@@@@@@@@@  @@@@@% %@@@@@                \n";
	std::cerr << "                                       (@@%.                              ,&@@*        (@@%.                                 \n";
	std::cerr << "                                                                                                                             \n";
	std::cerr << "                                                                                                                             \n";
	std::cerr << "                                                                                                                             \n";
	std::cerr << "                                                                                                                             \n";
	std::cerr << "  @@@@@@@.   @@@@ @@@@@@@@%  .@@@@@@@   @@@@@@@@@@@       %@@@@@@%   ,@@@@@      @@@@@@@     &@@@@@@&     @@@@@@@,   @@@@@@@@\n";
	std::cerr << "@@@@@@@@@@@  @@@@ @@@@@@@@% @@@@@@@@@@@ @@@@@@@@@@@     .@@@@@@@@@@. ,@@@@@     .@@@@@@@&   @@@@@@@@@@. &@@@@@@@@@@  @@@@@@@@\n";
	std::cerr << "@@@@@ @@@@@* @@@, @@@@@     @@@@@ @@@@@    @@@@@        @@@@@ ,@@@@@ ,@@@@@     &@@@@@@@@  (@@@@* @@@@& @@@@@ @@@@@  @@@@@   \n";
	std::cerr << "@@@@@ @@@@@# ,,,  @@@@@     @@@@@@         @@@@@        @@@@@ .@@@@@ ,@@@@@     @@@@&@@@@  ,@@@@@#      @@@@@@       @@@@@   \n";
	std::cerr << "@@@@@ #####/      @@@@@@@@.  @@@@@@@@      @@@@@        @@@@@ .##### ,@@@@@     @@@@,@@@@%  %@@@@@@@#    @@@@@@@@    @@@@@@@@\n";
	std::cerr << "@@@@@             @@@@@@@@.    @@@@@@@@    @@@@@        @@@@@        ,@@@@@    *@@@@ @@@@@     @@@@@@@*    #@@@@@@@  @@@@@@@@\n";
	std::cerr << "@@@@@ @@@@@#      @@@@@     @@@@@ @@@@@    @@@@@        @@@@@ ,@@@@@ ,@@@@@    @@@@@@@@@@@ .@@@@%.@@@@@ @@@@@ @@@@@, @@@@@   \n";
	std::cerr << "@@@@@ @@@@@,      @@@@@     @@@@@ @@@@@.   @@@@@        @@@@@ ,@@@@@ ,@@@@@    @@@@@@@@@@@( @@@@% @@@@@ @@@@@ %@@@@* @@@@@   \n";
	std::cerr << "@@@@@@@@@@@       @@@@@@@@@ #@@@@@@@@@@    @@@@@         @@@@@@@@@@  ,@@@@@@@@ @@@@@ ,@@@@@ @@@@@@@@@@@  @@@@@@@@@@  @@@@@@@@\n";
	std::cerr << "  @@@@@@@         @@@@@@@@@   @@@@@@@.     @@@@@          %@@@@@@/   ,@@@@@@@@%@@@@@  @@@@@  .@@@@@@@     %@@@@@@(   @@@@@@@@\n";
	std::cerr << "                                                                                                                             \n";
	std::cerr << "                                                                                                                             \n";
}