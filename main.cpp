#include "Cloud.h"

//part2
#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/dnn.hpp>

#include "fonctions.h"

int main(int argc, char **argv)
{
	std::string repos_proj = argv[1];
	std::cerr.precision(12);
	//testLibLas();
	//testPCL();
	//testOpenCV();
	std::vector<Cloud> clouds;
	clouds.push_back(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/road_sub2.txt"));
	clouds.push_back(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/vertite_terrain/1/road_sub2_class_bat.txt"));
	clouds.push_back(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/vertite_terrain/2/road_sub2_class_veget.txt"));
	clouds.push_back(Cloud("C:/Users/hobbe/Desktop/RSC_test/Projet_test/vertite_terrain/3/road_sub2_class_sol.txt"));

	std::vector<cv::Mat> Mats;


	for (int i = 0; i < clouds.size(); i++)
	{
		clouds[i].computeDensity(1);
		clouds[i].computeNormals();
		std::cerr << clouds[i].XYZRGBACloud->points[0] << clouds[i].intensityCloud->points[0].intensity << clouds[i].normalsCloud->points[0] << "\n";
		Mats.push_back(formatInput(clouds[i]));
		std::cerr << Mats[i].row(0) << "\n";
		std::cerr << "sizemat : " << Mats[i].size() << "\n";

	}


	//openCV

	//cv::Mat labels;
	//cv::Mat labels_row = cv::Mat::ones(1, 1, CV_32S);
	std::vector<int> labels;
	cv::Mat trainMat;
	for (int i = 1; i < clouds.size(); i++) 
	{
		for (int j = 0; j < Mats[i].rows; j++)
		{
			labels.push_back(i);
			std::cerr << i << "\n";
		}
		trainMat.push_back(Mats[i]);
	}
	std::cerr << "sizemat train : " << trainMat.size() << "\n";
	std::cerr << "sizemat label : " << labels.size() << "\n";

	std::cerr << "eeeeeeeeee\n";
	cv::Ptr<cv::ml::StatModel> model = createModels(AlgoType::SIG_SVM);
	std::cerr << "dddddddddd\n";
	cv::Ptr<cv::ml::TrainData> trainData = cv::ml::TrainData::create(trainMat, 0, labels);
	std::cerr << "cccccccccc\n";
	trainData->setTrainTestSplitRatio(0.2, true);
	model->train(trainData);
	std::cerr << "aaaaaaaaaaaa\n";
	//auto y_predict_error = cv::OutputArray(labels);
	//y_predict_error.clear();
	//auto y_predict = cv::OutputArray(labels);
	//y_predict.clear();

	cv::Mat error_output;
	cv::Mat predict_output;

	auto error = model->calcError(trainData, true, error_output);
	model->predict(Mats[0], predict_output);
	std::cerr << "bbbbbbbb\n";
	std::ofstream output("C:/Users/hobbe/Desktop/RSC_test/Projet_test/road_sub2_predict_svm.txt", std::ios::out | std::ios::trunc);
	output << "//X Y Z R G B A Intensity NX NY NZ Label\n";

	for (int i = 0; i < Mats[0].rows; i++) {
		output << clouds[0].XYZRGBACloud->points[i].x << " " << clouds[0].XYZRGBACloud->points[i].y << " " << clouds[0].XYZRGBACloud->points[i].z << " "
			<< (int)clouds[0].XYZRGBACloud->points[i].r << " " << (int)clouds[0].XYZRGBACloud->points[i].g << " " << (int)clouds[0].XYZRGBACloud->points[i].b << " "
			<< (int)clouds[0].XYZRGBACloud->points[i].a << " " << clouds[0].intensityCloud->points[i].intensity << " " << clouds[0].normalsCloud->points[i].normal_x << " "
			<< clouds[0].normalsCloud->points[i].normal_y << " " << clouds[0].normalsCloud->points[i].normal_z << " " << predict_output.at<float>(i,0) << "\n";
	}
	return 0;
}


//int main(int argc, char **argv)
//{
//	std::setprecision(12);
//	//testLibLas();
//	//testPCL();
//	//testOpenCV();
//	PointXYZRGBI pt;
//	std::cout << typeid(pt.x).name() << "\n";
//	std::cout << typeid(pt.y).name() << "\n";
//	std::cout << typeid(pt.z).name() << "\n";
//	std::cout << typeid(pt.r).name() << "\n";
//	std::cout << typeid(pt.g).name() << "\n";
//	std::cout << typeid(pt.b).name() << "\n";
//	std::cout << typeid(pt.intensity).name() << "\n";
//
//	//open cloud
//	pcl::PointCloud<PointXYZRGBI>::Ptr PtrCloud(new pcl::PointCloud<PointXYZRGBI>);
//	readCloudFromTxt("C:/Users/hobbe/Desktop/RSC_test/Projet_test/road_sub2.txt", *PtrCloud, true);
//	//ReadLas("C:\\Users\\hobbe\\Desktop\\RSC_test\\road_sub.las", cloud);
//
//	std::cout << " pt0: " << PtrCloud->points[0];
//
//
//
//	//create geometric attribute//
//	//normals
//
//	// Create the normal estimation class, and pass the input dataset to it
//	pcl::NormalEstimation<PointXYZRGBI, pcl::Normal> ne;
//	ne.setInputCloud(PtrCloud);
//	// Create an empty kdtree representation, and pass it to the normal estimation object.
//// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//	pcl::search::KdTree<PointXYZRGBI>::Ptr tree(new pcl::search::KdTree<PointXYZRGBI>());
//	ne.setSearchMethod(tree);
//	// Output datasets
//	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//	// Use all neighbors in a sphere of radius 3cm
//	ne.setRadiusSearch(0.03);
//	//set view point
//	ne.setViewPoint(812.994, 599.005, 27.857);
//	// Compute the features
//	ne.compute(*cloud_normals);
//	std::cout << "curv : " << cloud_normals->points[0].curvature << ", normal : " << cloud_normals->points[0].normal << ", x: " << cloud_normals->points[0].normal_x << ", y:" << cloud_normals->points[0].normal_y << ", z:" << cloud_normals->points[0].normal_z << "\n";
//
//	//create opencv input
//
//
//	//open 
//
//
//
//	return 0;
//}
//


//// Example : random forest (tree) learning
//// usage: prog training_data_file testing_data_file
//
//// For use with test / training datasets : opticaldigits_ex
//
//// Author : Toby Breckon, toby.breckon@cranfield.ac.uk
//
//// Copyright (c) 2011 School of Engineering, Cranfield University
//// License : LGPL - http://www.gnu.org/licenses/lgpl.html
//
//#include <cv.h>       // opencv general include file
//#include <ml.h>		  // opencv machine learning include file
//
//using namespace cv; // OpenCV API is in the C++ "cv" namespace
//
//#include <stdio.h>
//
///******************************************************************************/
//// global definitions (for speed and ease of use)
//
//#define NUMBER_OF_TRAINING_SAMPLES 3823
//#define ATTRIBUTES_PER_SAMPLE 64
//#define NUMBER_OF_TESTING_SAMPLES 1797
//
//#define NUMBER_OF_CLASSES 10
//
//// N.B. classes are integer handwritten digits in range 0-9
//
///******************************************************************************/
//
//// loads the sample database from file (which is a CSV text file)
//
//int read_data_from_csv(const char* filename, Mat data, Mat classes,
//	int n_samples)
//{
//	float tmp;
//
//	// if we can't read the input file then return 0
//	FILE* f = fopen(filename, "r");
//	if (!f)
//	{
//		printf("ERROR: cannot read file %s\n", filename);
//		return 0; // all not OK
//	}
//
//	// for each sample in the file
//
//	for (int line = 0; line < n_samples; line++)
//	{
//
//		// for each attribute on the line in the file
//
//		for (int attribute = 0; attribute < (ATTRIBUTES_PER_SAMPLE + 1); attribute++)
//		{
//			if (attribute < 64)
//			{
//
//				// first 64 elements (0-63) in each line are the attributes
//
//				fscanf(f, "%f,", &tmp);
//				data.at<float>(line, attribute) = tmp;
//				// printf("%f,", data.at<float>(line, attribute));
//
//			}
//			else if (attribute == 64)
//			{
//
//				// attribute 65 is the class label {0 ... 9}
//
//				fscanf(f, "%f,", &tmp);
//				classes.at<float>(line, 0) = tmp;
//				// printf("%f\n", classes.at<float>(line, 0));
//
//			}
//		}
//	}
//
//	fclose(f);
//
//	return 1; // all OK
//}
//
///******************************************************************************/
//
//int main(int argc, char** argv)
//{
//	// lets just check the version first
//
//	printf("OpenCV version %s (%d.%d.%d)\n",
//		CV_VERSION,
//		CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_SUBMINOR_VERSION);
//
//	// define training data storage matrices (one for attribute examples, one
//	// for classifications)
//
//	Mat training_data = Mat(NUMBER_OF_TRAINING_SAMPLES, ATTRIBUTES_PER_SAMPLE, CV_32FC1);
//	Mat training_classifications = Mat(NUMBER_OF_TRAINING_SAMPLES, 1, CV_32FC1);
//
//	//define testing data storage matrices
//
//	Mat testing_data = Mat(NUMBER_OF_TESTING_SAMPLES, ATTRIBUTES_PER_SAMPLE, CV_32FC1);
//	Mat testing_classifications = Mat(NUMBER_OF_TESTING_SAMPLES, 1, CV_32FC1);
//
//	// define all the attributes as numerical
//	// alternatives are CV_VAR_CATEGORICAL or CV_VAR_ORDERED(=CV_VAR_NUMERICAL)
//	// that can be assigned on a per attribute basis
//
//	Mat var_type = Mat(ATTRIBUTES_PER_SAMPLE + 1, 1, CV_8U);
//	var_type.setTo(Scalar(CV_VAR_NUMERICAL)); // all inputs are numerical
//
//	// this is a classification problem (i.e. predict a discrete number of class
//	// outputs) so reset the last (+1) output var_type element to CV_VAR_CATEGORICAL
//
//	var_type.at<uchar>(ATTRIBUTES_PER_SAMPLE, 0) = CV_VAR_CATEGORICAL;
//
//	double result; // value returned from a prediction
//
//	// load training and testing data sets
//
//	if (read_data_from_csv(argv[1], training_data, training_classifications, NUMBER_OF_TRAINING_SAMPLES) &&
//		read_data_from_csv(argv[2], testing_data, testing_classifications, NUMBER_OF_TESTING_SAMPLES))
//	{
//		// define the parameters for training the random forest (trees)
//
//		float priors[] = { 1,1,1,1,1,1,1,1,1,1 };  // weights of each classification for classes
//		// (all equal as equal samples of each digit)
//
//		CvRTParams params = CvRTParams(25, // max depth
//			5, // min sample count
//			0, // regression accuracy: N/A here
//			false, // compute surrogate split, no missing data
//			15, // max number of categories (use sub-optimal algorithm for larger numbers)
//			priors, // the array of priors
//			false,  // calculate variable importance
//			4,       // number of variables randomly selected at node and used to find the best split(s).
//			100,	 // max number of trees in the forest
//			0.01f,				// forrest accuracy
//			CV_TERMCRIT_ITER | CV_TERMCRIT_EPS // termination cirteria
//		);
//
//		// train random forest classifier (using training data)
//
//		printf("\nUsing training database: %s\n\n", argv[1]);
//		cv::ml::RTrees* rtree = new cv::ml::RTrees;
//		
//		rtree->train(training_data, CV_ROW_SAMPLE, training_classifications,
//			Mat(), Mat(), var_type, Mat(), params);
//
//		// perform classifier testing and report results
//
//		Mat test_sample;
//		int correct_class = 0;
//		int wrong_class = 0;
//		int false_positives[NUMBER_OF_CLASSES] = { 0,0,0,0,0,0,0,0,0,0 };
//
//		printf("\nUsing testing database: %s\n\n", argv[2]);
//
//		for (int tsample = 0; tsample < NUMBER_OF_TESTING_SAMPLES; tsample++)
//		{
//
//			// extract a row from the testing matrix
//
//			test_sample = testing_data.row(tsample);
//
//			// run random forest prediction
//
//			result = rtree->predict(test_sample, Mat());
//
//			printf("Testing Sample %i -> class result (digit %d)\n", tsample, (int)result);
//
//			// if the prediction and the (true) testing classification are the same
//			// (N.B. openCV uses a floating point decision tree implementation!)
//
//			if (fabs(result - testing_classifications.at<float>(tsample, 0))
//				>= FLT_EPSILON)
//			{
//				// if they differ more than floating point error => wrong class
//
//				wrong_class++;
//
//				false_positives[(int)result]++;
//
//			}
//			else
//			{
//
//				// otherwise correct
//
//				correct_class++;
//			}
//		}
//
//		printf("\nResults on the testing database: %s\n"
//			"\tCorrect classification: %d (%g%%)\n"
//			"\tWrong classifications: %d (%g%%)\n",
//			argv[2],
//			correct_class, (double)correct_class * 100 / NUMBER_OF_TESTING_SAMPLES,
//			wrong_class, (double)wrong_class * 100 / NUMBER_OF_TESTING_SAMPLES);
//
//		for (int i = 0; i < NUMBER_OF_CLASSES; i++)
//		{
//			printf("\tClass (digit %d) false postives 	%d (%g%%)\n", i,
//				false_positives[i],
//				(double)false_positives[i] * 100 / NUMBER_OF_TESTING_SAMPLES);
//		}
//
//
//		// all matrix memory free by destructors
//
//
//		// all OK : main returns 0
//
//		return 0;
//	}
//
//	// not OK : main returns -1
//
//	return -1;
//}
///******************************************************************************/