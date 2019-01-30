#include "Classifier.h"

Classifier::Classifier(GroundTruth GT, AlgoType algoType) : m_algoType(algoType)
{
	cv::Mat tempMat;
	std::vector<int> labels;
	for (int i = 0; i < GT.m_ptrsCloud.size(); i++)
	{
		tempMat.push_back(formatInput(GT.m_ptrsCloud[i]));
		for(int j=0; j < GT.m_ptrsCloud[i].XYZRGBACloud->size();j++) //creation of 1 label per point in each cloud
			labels.push_back(GT.m_cloudLabel[i]);
	}
	std::cerr << tempMat.row(0) << "\n";
	m_trainData= cv::ml::TrainData::create(tempMat, 0, labels);
	m_model= createModels(AlgoType::RF);
}

cv::Mat Classifier::formatInput(Cloud const& cloud)
{
	int nbparam = 7;

	cv::Mat feature_row = cv::Mat::ones(1, nbparam, CV_32F);
	cv::Mat features;


	for (int i = 0; i < cloud.XYZRGBACloud->size(); i++)
	{

		feature_row.at<float>(0, 0) = cloud.XYZRGBACloud->points[i].r;
		feature_row.at<float>(0, 1) = cloud.XYZRGBACloud->points[i].g;
		feature_row.at<float>(0, 2) = cloud.XYZRGBACloud->points[i].b;
		feature_row.at<float>(0, 3) = cloud.intensityCloud->points[i].intensity;
		feature_row.at<float>(0, 4) = cloud.normalsCloud->points[i].normal_x;
		feature_row.at<float>(0, 5) = cloud.normalsCloud->points[i].normal_y;
		feature_row.at<float>(0, 6) = cloud.normalsCloud->points[i].normal_z;

		features.push_back(feature_row);
	}

	return features;
}


cv::Ptr<cv::ml::StatModel> Classifier::createModels(AlgoType AT)
{
	cv::Ptr<cv::ml::StatModel> modelPtr;

	switch (AT) {
	case AlgoType::RF:
	{
		auto random_forest = cv::ml::RTrees::create();

		auto criter = cv::TermCriteria();
		criter.type = CV_TERMCRIT_EPS + CV_TERMCRIT_ITER;
		criter.epsilon = 1e-8;
		criter.maxCount = 5000;

		// parameters for random forest
		random_forest->setMaxCategories(2);
		random_forest->setMaxDepth(3000);
		random_forest->setMinSampleCount(1);
		random_forest->setTruncatePrunedTree(false);
		random_forest->setUse1SERule(false);
		random_forest->setUseSurrogates(false);
		random_forest->setPriors(cv::Mat());
		random_forest->setTermCriteria(criter);
		random_forest->setCVFolds(1);

		modelPtr = random_forest;
	}
	break;
	case AlgoType::SIG_SVM:
	{
		auto sigmoid_svm = cv::ml::SVM::create();

		auto criter_svm = cv::TermCriteria();
		criter_svm.type = CV_TERMCRIT_EPS;
		criter_svm.epsilon = 1e-10;

		// parameters for rbf support vector machines
		sigmoid_svm->setC(100);
		sigmoid_svm->setTermCriteria(criter_svm);
		sigmoid_svm->setCoef0(0.3);
		sigmoid_svm->setKernel(sigmoid_svm->SIGMOID);
		sigmoid_svm->setGamma(0.1);
		sigmoid_svm->setType(sigmoid_svm->C_SVC);

		modelPtr = sigmoid_svm;
	}
	break;
	}

	return modelPtr;
}


int Classifier::train()
{
	m_model->train(m_trainData);
	return 0;
}

int Classifier::predict(Cloud& cloud2predict, std::vector<int>& outputPredict)
{
	cv::Mat inputPredict = formatInput(cloud2predict);
	predict(inputPredict, outputPredict);
	return 0;
}

int Classifier::predict(cv::Mat& inputPredict, std::vector<int>& outputPredict)
{
	outputPredict.resize(inputPredict.rows);
	
#pragma omp parallel for
	for (int i = 0; i < inputPredict.rows; i++) 
	{
		cv::Mat iRow, oRow;
		iRow = inputPredict.row(i);
		m_model->predict(iRow, oRow);
		//std::cerr << "oRow" << oRow << "\n";
		outputPredict[i]=oRow.at<float>(0,0);
	}
	return 0;
}