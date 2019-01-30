#include "GroundTruth.h"

GroundTruth::GroundTruth(std::string projectPath):m_projectPath(projectPath)
{
	m_ptrsCloud = std::vector<Cloud>();
	m_cloudLabel= std::vector<int>();
}

int GroundTruth::addCloud(Cloud cloud2Add, int label)
{
	m_ptrsCloud.push_back(cloud2Add);
	m_cloudLabel.push_back(label);
	return 0;
}