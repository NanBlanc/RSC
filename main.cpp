#include "Cloud.h"

//part2
#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/dnn.hpp>
#include <omp.h>  // include OpenMP

#include "fonctions.h"
#include "Classifier.h"
#include "GroundTruth.h"

#include <cmath>

int main(int argc, char **argv)
{
	std::cerr << "START\n";
	std::string repos_proj = argv[1];
	std::cerr.precision(12);
	//testLibLas();
	//testPCL();
	//testOpenCV();

	//std::vector<Cloud> clouds;
	//clouds.push_back(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/road_sub2.txt"));
	//clouds.push_back(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/vertite_terrain/1/road_sub2_class_bat.txt"));
	//clouds.push_back(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/vertite_terrain/2/road_sub2_class_veget.txt"));
	//clouds.push_back(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/vertite_terrain/3/road_sub2_class_sol.txt"));

	//load clouds
	Cloud cloud2Predict("C:/Users/hobbe/Desktop/RSC_test/Projet_test/road_sub2.txt");

	GroundTruth GT("");
	GT.addCloud(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/vertite_terrain/1/road_sub2_class_bat.txt"), 1);
	GT.addCloud(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/vertite_terrain/2/road_sub2_class_veget.txt"), 2);
	GT.addCloud(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/vertite_terrain/3/road_sub2_class_sol.txt"), 3);
	//GT.addCloud(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/vertite_terrain/1/road_sub2_class_bat_50pt.txt"), 1);
	//GT.addCloud(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/vertite_terrain/2/road_sub2_class_veget_15pt.txt"), 2);
	//GT.addCloud(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/vertite_terrain/3/road_sub2_class_sol_15pt.txt"), 3);



	//compute normals
	cloud2Predict.computeDensity(1);
	cloud2Predict.computeNormals();

	for (int i = 0; i < GT.m_ptrsCloud.size(); i++)
	{
		GT.m_ptrsCloud[i].computeDensity(1);
		GT.m_ptrsCloud[i].computeNormals();
		std::cerr << GT.m_ptrsCloud[i].XYZRGBACloud->points[0] << GT.m_ptrsCloud[i].intensityCloud->points[0].intensity << GT.m_ptrsCloud[i].normalsCloud->points[0] << "\n";
	}


	//openCV
	//classifier
	Classifier C_RF(GT,Classifier::AlgoType::RF);


	//train
	std::cerr << "Training ... ";
	clock_t t0 = clock();
	C_RF.train();
	clock_t t1 = clock();
	std::cerr << (t1 - t0) / static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;

	//std::cerr << "total size: " << C_RF.m_trainData->getSamples().rows << "\n";
	//std::cerr << "subset train size: " << C_RF.m_trainData->getTrainSamples().rows << "\n";
	//std::cerr << "subset test size: " << C_RF.m_trainData->getTestSamples().rows << "\n";
	std::vector<int> error_output;
	auto error = C_RF.m_model->calcError(C_RF.m_trainData, true, error_output);

	int nb_label = GT.m_ptrsCloud.size();
	std::vector<int> label=C_RF.m_trainData->getClassLabels();
	cv::Mat confusion=cv::Mat::zeros(nb_label+1,nb_label+1, CV_32F);

	std::vector<int> reponse = C_RF.testResponse;
	for (int i = 0; i < error_output.size(); i++) 
	{
		//remplissage par ligne
		//vrai positif
		if (error_output[i] == reponse[i])
			confusion.at<float>(reponse[i] - 1, reponse[i] - 1) += 1;
		
		//Vrai négatif
		else if (error_output[i] != reponse[i])
			confusion.at<float>(error_output[i] - 1, reponse[i] - 1) += 1;
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
		std::cerr<<"lxc : "<< lxcSum <<" "<< sumU <<" "<<sumP<<"\n";
	}
	double overallPrecision = niiSum / totalSum;
	double n = nb_label * nb_label;
	double kappa = std::abs(n*niiSum - lxcSum) / (n*n - lxcSum);
	std::cerr << "overallPrecision " << niiSum << " " << totalSum << " " << overallPrecision << " "<<kappa<<"\n";
	std::cerr << "confusion matx : \n" << confusion << "\n";




	//std::cerr << "mat eroor: " << error_output << "\n";
	cv::Mat	varImp = C_RF.m_model->getVarImportance();
	std::cerr << "mat importance: " << varImp << "\n";


	//predict
	std::vector<int> predict_output;

	std::cerr << "Prediction ... ";
	t0 = clock();
	C_RF.predict(cloud2Predict, predict_output);
	t1 = clock();
	std::cerr << (t1 - t0) / static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;


	//out
	cloud2Predict.savePredictedLabels(predict_output);
	std::ofstream output("C:/Users/hobbe/Desktop/RSC_test/Projet_test/road_sub2_predict_rf_new.txt", std::ios::out | std::ios::trunc);
	output << "//X Y Z R G B A Intensity NX NY NZ Label\n";
	cloud2Predict.printFullCloud(output);


	//std::cerr << error;
	std::cerr << "END\n";
	return 0;
}