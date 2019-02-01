
#include "Cloud.h"
#include "Classifier.h"
#include "GroundTruth.h"
#include "fonctions.h"


int main(int argc, char **argv)
{
	std::cerr << "---/-/-/-/-/RSC\\-\\-\\-\\-\\-\\---\n";
	//testLibLas();
	//testPCL();
	//testOpenCV();
	clock_t t0;
	clock_t t1;

	std::cerr.precision(12);

	InArgs flags = Tools::ArgParser(argc, argv);
	//std::cerr << "falgs : ";
	//std::cerr << flags.projectPath << "," << flags.train << "," << flags.saveModel << ","
	//	<< flags.savePath << "," << flags.predict << "," << flags.loadModel << "," << flags.loadPath << ",\n";



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
	Classifier C_RF(GT, Classifier::AlgoType::RF, 0.2,flags.loadModel);

	//train
	if (flags.train) {
		std::cerr << "Training ... ";
		t0 = clock();
		C_RF.train();
		t1 = clock();
		std::cerr << (t1 - t0) / static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;

		//test
		std::cerr << "Testing ... ";
		t0 = clock();
		C_RF.test();
		t1 = clock();
		std::cerr << (t1 - t0) / static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;

		C_RF.printClassifierDescriptor(std::cerr);
	}

	//saving
	if (flags.saveModel) {
		std::cerr << "Saving ... ";
		t0 = clock();
		C_RF.save("C:/Users/hobbe/Desktop/RSC_test/Projet_test/models2.yml");
		t1 = clock();
		std::cerr << (t1 - t0) / static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;
	}


	//predict
	if (flags.predict) {
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
	}

	//std::cerr << error;
	std::cerr << "---/-/-/-/-/END\\-\\-\\-\\-\\-\\---\n";
	return 0;
}