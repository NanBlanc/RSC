#include "Classifier.h"

Classifier::Classifier(GroundTruth GT, Classifier::AlgoType algoType, InArgs flags, double splitRatio ) : m_algoType(algoType)
{
	if(flags.loadModel)
	{
		//std::cerr << "aaaaaaa: " << flags.loadPath << "\n";
		this->m_model = cv::Ptr<cv::ml::RTrees>();
		this->load(flags.loadPath);
	}
	else
	{
		cv::Mat tempMat;
		std::vector<int> labels;
		for (int i = 0; i < GT.ptrsCloud().size(); i++)
		{
			tempMat.push_back(formatInput(GT.ptrsCloud()[i]));
			for (int j = 0; j < GT.ptrsCloud()[i].XYZRGBACloud()->size(); j++) //creation of 1 label per point in each cloud
				labels.push_back(GT.cloudLabel()[i]);

		}
		//std::cerr << tempMat.row(0) << "\n";
		//for (int i = 0; i < labels.size(); i++)
		//	std::cerr << labels[i] << ", ";
		this->m_trainData = cv::ml::TrainData::create(tempMat, 0, labels);
		this->m_trainData->setTrainTestSplitRatio(splitRatio, true); //train size = 0.2*TOTAL
		this->m_testResponse = m_trainData->getTestResponses();
		//std::cerr << "test labels2" << this->testResponse << "\n";
		this->m_model = createModels(AlgoType::RF);
	}
}

cv::Mat Classifier::formatInput(Cloud cloud)
{
	int nbparam = 7;

	cv::Mat feature_row = cv::Mat::ones(1, nbparam, CV_32F);
	cv::Mat features;


	for (int i = 0; i < cloud.XYZRGBACloud()->size(); i++)
	{

		feature_row.at<float>(0, 0) = cloud.XYZRGBACloud()->points[i].r;
		feature_row.at<float>(0, 1) = cloud.XYZRGBACloud()->points[i].g;
		feature_row.at<float>(0, 2) = cloud.XYZRGBACloud()->points[i].b;
		feature_row.at<float>(0, 3) = cloud.intensityCloud()->points[i].intensity;
		feature_row.at<float>(0, 4) = cloud.normalsCloud()->points[i].normal_x;
		feature_row.at<float>(0, 5) = cloud.normalsCloud()->points[i].normal_y;
		feature_row.at<float>(0, 6) = cloud.normalsCloud()->points[i].normal_z;

		features.push_back(feature_row);
	}

	return features;
}


cv::Ptr<cv::ml::RTrees> Classifier::createModels(AlgoType AT)
{
	//cv::Ptr<cv::ml::StatModel> modelPtr;
	cv::Ptr<cv::ml::RTrees> modelPtr;

	switch (AT) {
	case AlgoType::RF:
	{
		auto random_forest = cv::ml::RTrees::create();

		auto criter = cv::TermCriteria();
		criter.type = CV_TERMCRIT_EPS + CV_TERMCRIT_ITER;
		criter.epsilon = 1e-8;
		criter.maxCount = 5000;

		// parameters for random forest
		random_forest->setMaxCategories(15);
		random_forest->setMaxDepth(25);
		random_forest->setMinSampleCount(5);
		random_forest->setTruncatePrunedTree(false);
		random_forest->setUse1SERule(false);
		random_forest->setUseSurrogates(false);
		random_forest->setPriors(cv::Mat());
		random_forest->setTermCriteria(criter);
		random_forest->setCVFolds(1);
		random_forest->setCalculateVarImportance(true);

		modelPtr = random_forest;
	}
	break;
	//case AlgoType::SIG_SVM:
	//{
	//	auto sigmoid_svm = cv::ml::SVM::create();

	//	auto criter_svm = cv::TermCriteria();
	//	criter_svm.type = CV_TERMCRIT_EPS;
	//	criter_svm.epsilon = 1e-10;

	//	// parameters for rbf support vector machines
	//	sigmoid_svm->setC(100);
	//	sigmoid_svm->setTermCriteria(criter_svm);
	//	sigmoid_svm->setCoef0(0.3);
	//	sigmoid_svm->setKernel(sigmoid_svm->SIGMOID);
	//	sigmoid_svm->setGamma(0.1);
	//	sigmoid_svm->setType(sigmoid_svm->C_SVC);

	//	modelPtr = sigmoid_svm;
	//}
	//break;
	}

	return modelPtr;
}


void Classifier::train()
{
	m_model->train(m_trainData);

	return;
}

void Classifier::test()
{
	std::vector<int> error_output;
	auto error = this->m_model->calcError(this->m_trainData, true, error_output);

	computeClassifierDescriptor(error_output);
	return;
}

void Classifier::computeClassifierDescriptor(std::vector<int> error_output)
{
	std::vector<int> label = this->m_trainData->getClassLabels();
	int nb_label = label.size();
	cv::Mat confusion = cv::Mat::zeros(nb_label + 1, nb_label + 1, CV_32F);

	std::vector<int> responses = this->m_testResponse;
	for (int i = 0; i < error_output.size(); i++)
	{
		//remplissage par ligne
		//vrai positif
		if (error_output[i] == responses[i])
			confusion.at<float>(responses[i] - 1, responses[i] - 1) += 1;

		//Vrai négatif
		else if (error_output[i] != responses[i])
			confusion.at<float>(error_output[i] - 1, responses[i] - 1) += 1;
	}

	double totalSum = 0;
	double niiSum = 0;
	double lxcSum = 0;
	for (int i = 0; i < nb_label; i++)
	{

		//précision utilisateur
		double sumU = 0;
		for (int j = 0; j < nb_label; j++) {
			sumU += confusion.at<float>(i, j);
		}
		confusion.at<float>(i, nb_label) = confusion.at<float>(i, i) / sumU;
		//précision prodocteur
		double sumP = 0;
		for (int j = 0; j < nb_label; j++) {
			sumP += confusion.at<float>(j, i);
		}
		confusion.at<float>(nb_label, i) = confusion.at<float>(i, i) / sumP;
		//precision total
		niiSum += confusion.at<float>(i, i);
		totalSum += sumU;
		lxcSum += sumU * sumP;
		//std::cerr << "lxc : " << lxcSum << " " << sumU << " " << sumP << "\n";
	}
	confusion.at<float>(nb_label, nb_label) = totalSum;
	
	//saving data
	double n = nb_label * nb_label;
	this->m_confusionMatrix = confusion;
	this->m_overallPrecision = niiSum / totalSum;
	this->m_kappaIndice = std::abs(n*niiSum - lxcSum) / (n*n - lxcSum);
	this->m_dimensionImportance = this->m_model->getVarImportance();

	return;
}

void Classifier::printClassifierDescriptor(std::ostream &flux)
{
	flux << "Confusion Matrix : \n";
	flux << this->m_confusionMatrix << "\n";
	flux << "Overall Precision : ";
	flux << this->m_overallPrecision << "\n";
	flux << "Kappa Indice : ";
	flux << this->m_kappaIndice << "\n";
	flux << "Attribute Importance : \n";
	flux << "R;G;B;I;NX;NY;NZ\n";
	flux << this->m_dimensionImportance.t() << "\n";
	return;
}

void Classifier::predict(std::vector<Cloud>& ptrInClouds, std::vector<std::vector<int>>& vecOutputPredict)
{
	
	for (int i = 0; i < ptrInClouds.size(); i++)
	{
		std::vector<int> outputPredict;
		predict(ptrInClouds[i], outputPredict);
		vecOutputPredict.push_back(outputPredict);
	}
	return;
}

void Classifier::predict(Cloud& cloud2predict, std::vector<int>& outputPredict)
{
	cv::Mat inputPredict = formatInput(cloud2predict);
	predict(inputPredict, outputPredict);
	return;
}

void Classifier::predict(cv::Mat& inputPredict, std::vector<int>& outputPredict)
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
	return;
}

void Classifier::load(std::string loadPath)
{
	this->m_model=cv::ml::StatModel::load<cv::ml::RTrees>(loadPath);
	return;
}
void Classifier::save(std::string savePath)
{
	Tools::createFolder(savePath.c_str());
	int modelNumber = Tools::getFileNumberInFolder(savePath);
	std::string modelPath = savePath + "\\RF_model_" + std::to_string(modelNumber);
	this->m_model->save(modelPath+".yml");
	std::ofstream output(modelPath+"_descriptor.txt", std::ios::out | std::ios::trunc);
	this->printClassifierDescriptor(output);
	return;
}

//getter
Classifier::AlgoType Classifier::algoType() { return m_algoType; }
cv::Ptr<cv::ml::RTrees> Classifier::model() { return m_model; }
cv::Ptr<cv::ml::TrainData> Classifier::trainData() { return m_trainData; }
cv::Mat Classifier::testResponse() { return m_testResponse; }
cv::Mat Classifier::confusionMatrix() { return m_confusionMatrix; }
double Classifier::overallPrecision() { return m_overallPrecision; }
double Classifier::kappaIndice() { return m_kappaIndice; }
cv::Mat	Classifier::dimensionImportance() { return m_dimensionImportance; }

//setter
void Classifier::setAlgoType(AlgoType algoType) { m_algoType = algoType; }
void Classifier::setModel(cv::Ptr<cv::ml::RTrees> model) { m_model = model; }
void Classifier::setTrainData(cv::Ptr<cv::ml::TrainData> trainData) { m_trainData = trainData; }
void Classifier::setTestResponse(cv::Mat testResponse) { m_testResponse = testResponse; }
void Classifier::setConfusionMatrix(cv::Mat confusionMatrix) { m_confusionMatrix = confusionMatrix; }
void Classifier::setOverallPrecision(double overallPrecision) { m_overallPrecision = overallPrecision; }
void Classifier::setKappaIndice(double kappaIndice) { m_kappaIndice = kappaIndice; }
void Classifier::setDimensionImportance(cv::Mat	dimensionImportance) { m_dimensionImportance = dimensionImportance; }