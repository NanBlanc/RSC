#pragma once

#include <vector>
#include <string>

#include "Cloud.h"

class GroundTruth {
public:

	GroundTruth(std::string projectPath);

	int addCloud(Cloud cloud2Add, int label);

//protected
	std::vector<Cloud> m_ptrsCloud;
	std::vector<int> m_cloudLabel;
	std::string m_projectPath;
};