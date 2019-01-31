#pragma once

#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/dnn.hpp>
#include <omp.h>  // include OpenMP

#include "Cloud.h"
#include "GroundTruth.h"

class Classifier {
public:
	enum AlgoType { RF, SIG_SVM };


	Classifier(GroundTruth GT, AlgoType algoType);

	int predict(cv::Mat& inputPredict, std::vector<int>& outputPredict);
	int predict(Cloud& cloud2predict, std::vector<int>& outputPredict);
	int train();

	int load();
	int save;


	//protected:
	cv::Mat formatInput(Cloud const& cloud);
	cv::Ptr<cv::ml::RTrees> createModels(AlgoType AT);

	AlgoType m_algoType;
	cv::Ptr<cv::ml::RTrees> m_model;
	cv::Ptr<cv::ml::TrainData> m_trainData;
	cv::Mat testResponse;
};