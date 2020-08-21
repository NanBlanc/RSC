
#include "Cloud.h"
#include "Classifier.h"
#include "GroundTruth.h"
#include "fonctions.h"

//pdal test



int main(int argc, char **argv)
{
	std::cerr << "---/-/-/-/-/RSC Version 1.0\\-\\-\\-\\-\\-\\---\n";
	//testLibLas();
	//testPCL();
	//testOpenCV();
	clock_t t0;
	clock_t t1;

	std::cerr.precision(12);

	InArgs flags = Tools::argParser(argc, argv);
	std::cerr << "flags : (0 = false, 1 = true)\n";
	std::cerr << "Project : "<<flags.projectPath << "\nTrain : " << flags.train << "; save model : " << flags.saveModel << ", path : "
		<< flags.savePath << "\nPredict : " << flags.predict << "; load model : " << flags.loadModel << ", path : " << flags.loadPath << "\n";

	//test pdal


	//load and compute att of GT
	GroundTruth GT;
	if (flags.train) 
	{
		GT = GroundTruth(flags.projectPath);
		//std::map<int, std::string>::iterator it = GT.m_labelName.begin();
		//std::cout << "GT.m_labelName contains:\n";
		//for (it = GT.m_labelName.begin(); it != GT.m_labelName.end(); ++it)
		//	std::cout << it->first << " => " << it->second << '\n';

		//compute geom att
		std::cerr << "Computing Ground_Truth attributs ... ";
		t0 = clock();
		for (int i = 0; i < GT.ptrsCloud().size(); i++)
		{
			//GT.m_ptrsCloud[i].computeDensity(1);
			GT.ptrsCloud()[i].computeNormals();
			//std::cerr << GT.m_ptrsCloud[i].path << "\n" << GT.m_ptrsCloud[i].XYZRGBACloud->points[0] << GT.m_ptrsCloud[i].intensityCloud->points[0].intensity << GT.m_ptrsCloud[i].normalsCloud->points[0] << "\n";
		}
		t1 = clock();
		std::cerr << (t1 - t0) / static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;
	}

	//load and compute att of clouds to classify
	std::vector<Cloud> ptrInClouds;
	if (flags.predict) {
		//load clouds to classify
		std::string inCloudsFolderPath = flags.projectPath + "\\in_clouds";
		Tools::readCloudsInFolder(inCloudsFolderPath, ptrInClouds);

		//compute geom attribute
		std::cerr << "Computing In_Clouds attributs ... ";
		t0 = clock();
		for (int i = 0; i < ptrInClouds.size(); i++)
		{
			//ptrInClouds[i].computeDensity(1);
			ptrInClouds[i].computeNormals();
			//std::cerr << ptrInClouds[i].path << "\n" << ptrInClouds[i].XYZRGBACloud->points[0] << ptrInClouds[i].intensityCloud->points[0].intensity << ptrInClouds[i].normalsCloud->points[0] << "\n";
		}
		t1 = clock();
		std::cerr << (t1 - t0) / static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;
	}

	//openCV
	//classifier
	Classifier C_RF(GT, Classifier::AlgoType::RF,flags,0.2);

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
		std::cerr << "Saving model ... ";
		t0 = clock();
		C_RF.save(flags.savePath);
		t1 = clock();
		std::cerr << (t1 - t0) / static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;
	}



	//predict
	if (flags.predict) {
		std::vector<std::vector<int>> predict_output;

		std::cerr << "Prediction ... ";
		t0 = clock();
		C_RF.predict(ptrInClouds, predict_output);
		t1 = clock();
		std::cerr << (t1 - t0) / static_cast<float>(CLOCKS_PER_SEC) << "s" << std::endl;

		//out
		Tools::createFolder(std::string(flags.projectPath + "\\out_clouds").c_str());
		std::cerr << "Out_Clouds : \n";
		for (int i = 0; i < ptrInClouds.size(); i++) {
			//apply results
			ptrInClouds[i].savePredictedLabels(predict_output[i]);
			//print out
			ptrInClouds[i].saveTxt(flags.projectPath);
		}

	}

	std::cerr << "---\\-\\-\\-\\-\\END/-/-/-/-/-/---\n";
	return 0;
}