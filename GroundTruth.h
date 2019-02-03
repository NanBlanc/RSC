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

	int addCloud(Cloud cloud2Add, int label);

//protected
	std::vector<Cloud> m_ptrsCloud;
	std::vector<int> m_cloudLabel;
	std::map<int,std::string> m_labelName;
	std::string m_projectPath;
};