#include "Cloud.h"

//part2
#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/dnn.hpp>
#include <omp.h>  // include OpenMP

#include "fonctions.h"
#include "Classifier.h"
#include "GroundTruth.h"

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


	//compute normals
	cloud2Predict.computeDensity(1);
	cloud2Predict.computeNormals();

	for (int i = 0; i < GT.m_ptrsCloud.size(); i++)
	{
		GT.m_ptrsCloud[i].computeDensity(1);
		GT.m_ptrsCloud[i].computeNormals();
		std::cerr << GT.m_ptrsCloud[i].XYZRGBACloud->points[0] << GT.m_ptrsCloud[i].intensityCloud->points[0].intensity << GT.m_ptrsCloud[i].normalsCloud->points[0] << "\n";
	}

	std::cerr << "aaaaaaaaa\n";
	//openCV
	//classifier
	Classifier C_RF(GT,Classifier::AlgoType::RF);
	std::cerr << "bbbbbbbb\n";
	C_RF.m_trainData->setTrainTestSplitRatio(0.2, true);
	std::cerr << "cccccccccccc\n";
	//train
	std::cerr << "Training ... ";
	clock_t t0 = clock();
	C_RF.train();
	clock_t t1 = clock();
	std::cerr << (t1 - t0) / static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;

	
	//cv::Mat error_output;
	std::vector<int> predict_output;

	//auto error = model->calcError(trainData, true, error_output);

	//predict
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