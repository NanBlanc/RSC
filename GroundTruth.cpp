#include "GroundTruth.h"

GroundTruth::GroundTruth()
{
	m_ptrsCloud = std::vector<Cloud>();
	m_cloudLabel = std::vector<int>();
	m_labelName = std::map<int, std::string>();
	m_projectPath = std::string();
}

GroundTruth::GroundTruth(std::string projectPath):m_projectPath(projectPath)
{
	//creating empty containers of GT
	m_ptrsCloud = std::vector<Cloud>();
	m_cloudLabel = std::vector<int>();
	m_labelName = std::map<int, std::string>();

	//creating temporary containers
	std::vector<std::string> gtFolderPath;
	std::vector<std::vector<boost::filesystem::path>> gtCloudsPath;

	//creating GT path
	std::string gtPath = projectPath + "\\Ground_Truth";
	std::cerr << gtPath << "\n";

	//reading what GT folder have in it, and saving names into gtFolderPath
	Tools::readDirectory(gtPath, gtFolderPath);
	gtCloudsPath.resize(gtFolderPath.size());

	//for each folder in 
	for (int i = 0; i < gtFolderPath.size(); i++) {
		m_labelName.insert(std::pair<int,std::string>(i+1, gtFolderPath[i]));
		std::cerr << gtPath + "\\" + gtFolderPath[i] << "\n";
		Tools::readDirectoryExtension(gtPath + "\\" + gtFolderPath[i], ".txt", gtCloudsPath[i]);
		for (int j = 0; j < gtCloudsPath[i].size(); j++) {
			this->addCloud(Cloud(gtPath + "\\" + gtFolderPath[i] + "\\" + gtCloudsPath[i][j].string()), i+1);
			std::cerr << gtPath + "\\" + gtFolderPath[i] + "\\" + gtCloudsPath[i][j].string() << "\n";
		}
	}

}

GroundTruth::GroundTruth(GroundTruth const& GT)
{
	m_ptrsCloud = GT.m_ptrsCloud;
	m_cloudLabel = GT.m_cloudLabel;
	m_labelName = GT.m_labelName;
	m_projectPath = GT.m_projectPath;
}

int GroundTruth::addCloud(Cloud cloud2Add, int label)
{
	m_ptrsCloud.push_back(cloud2Add);
	m_cloudLabel.push_back(label);
	return 0;
}