#pragma once

#include <iostream>
#include <fstream>
#include <ostream>
#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/dnn.hpp>
#include <omp.h>  // include OpenMP
#include <cmath>

#include "Cloud.h"
#include "GroundTruth.h"

class Classifier {
public:
	enum AlgoType { RF, SIG_SVM };


	Classifier(GroundTruth GT, AlgoType algoType, InArgs flags, double splitRatio = 0.2);

	void predict(cv::Mat& inputPredict, std::vector<int>& outputPredict);
	void predict(Cloud& cloud2predict, std::vector<int>& outputPredict);
	void predict(std::vector<Cloud>& InClouds, std::vector<std::vector<int>>& vecOutputPredict);
	void train();
	void test();

	void load(std::string loadPath);
	void save(std::string savePath);

	void printClassifierDescriptor(std::ostream &flux);

	//getter
	AlgoType algoType();
	cv::Ptr<cv::ml::RTrees> model();
	cv::Ptr<cv::ml::TrainData> trainData();
	cv::Mat testResponse();
	cv::Mat confusionMatrix();
	double overallPrecision();
	double kappaIndice();
	cv::Mat	dimensionImportance();

	//setter
	void setAlgoType(AlgoType algoType);
	void setModel(cv::Ptr<cv::ml::RTrees> model);
	void setTrainData(cv::Ptr<cv::ml::TrainData> trainData);
	void setTestResponse(cv::Mat testResponse);
	void setConfusionMatrix(cv::Mat confusionMatrix);
	void setOverallPrecision(double overallPrecision);
	void setKappaIndice(double kappaIndice);
	void setDimensionImportance(cv::Mat	dimensionImportance);

protected:
	void computeClassifierDescriptor(std::vector<int> error_output);
	cv::Mat formatInput(Cloud cloud);
	cv::Ptr<cv::ml::RTrees> createModels(AlgoType AT);

	AlgoType m_algoType;
	cv::Ptr<cv::ml::RTrees> m_model;
	cv::Ptr<cv::ml::TrainData> m_trainData;
	cv::Mat m_testResponse;

	cv::Mat m_confusionMatrix;
	double m_overallPrecision;
	double m_kappaIndice;
	cv::Mat	m_dimensionImportance;

};