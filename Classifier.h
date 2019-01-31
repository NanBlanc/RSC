#pragma once

#include <iostream>
#include <fstream>
#include <ostream>
#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/dnn.hpp>
#include <omp.h>  // include OpenMP

#include "Cloud.h"
#include "GroundTruth.h"

class Classifier {
public:
	enum AlgoType { RF, SIG_SVM };


	Classifier(GroundTruth GT, AlgoType algoType, double splitRatio,bool loaded = false);

	int predict(cv::Mat& inputPredict, std::vector<int>& outputPredict);
	int predict(Cloud& cloud2predict, std::vector<int>& outputPredict);
	int train();
	int test();

	int load(std::string loadPath);
	int save(std::string savePath);

	int printClassifierDescriptor(std::ostream &flux);

	//protected:
	int computeClassifierDescriptor(std::vector<int> error_output);
	cv::Mat formatInput(Cloud const& cloud);
	cv::Ptr<cv::ml::RTrees> createModels(AlgoType AT);

	AlgoType m_algoType;
	cv::Ptr<cv::ml::RTrees> m_model;
	cv::Ptr<cv::ml::TrainData> m_trainData;
	cv::Mat testResponse;

	cv::Mat confusionMatrix;
	double overallPrecision;
	double kappaIndice;
	cv::Mat	dimensionImportance;

};