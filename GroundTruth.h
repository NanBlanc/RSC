#pragma once

#include <vector>
#include <string>

#include "Cloud.h"
#include "fonctions.h"

class GroundTruth {
public:
	GroundTruth();
	GroundTruth(std::string projectPath);
	GroundTruth(GroundTruth const& GT);

	void addCloud(Cloud cloud2Add, int label);

	//getter
	std::vector<Cloud> ptrsCloud();
	std::vector<int> cloudLabel();
	std::map<int, std::string> labelName();
	std::string projectPath();

	//setter
	void setPtrsCloud(std::vector<Cloud> ptrsCloud);
	void setCloudLabel(std::vector<int> cloudLabel);
	void setLabelName(std::map<int, std::string> labelName);
	void setProjectPath(std::string projectPath);

protected:
	std::vector<Cloud> m_ptrsCloud;
	std::vector<int> m_cloudLabel;
	std::map<int,std::string> m_labelName;
	std::string m_projectPath;
};