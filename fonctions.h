#pragma once
#include "Cloud.h" //WARNING :: NEED TO BE BEFOERE #include <windows.h> or PCL and FLANN CRASH DURING COMPILATION

#include <iostream>
#include <string>

#include <stdio.h>
#include <boost/filesystem.hpp>
#include <vector>


struct InArgs{
	std::string projectPath="";

	bool train = true;
	bool saveModel = false;
	std::string savePath="";

	bool predict = true;
	bool loadModel = false;
	std::string loadPath="";
	 
};



class Tools {
public:
	static InArgs argParser(int argc, char** argv);
	static void exitRSC(std::string message);
	static void printHelp();
	static void createFolder(const char * path);
	static void readDirectory(const std::string& name, std::vector<std::string>& v);
	static void readDirectoryExtension(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret);
	static void easterEgg();
	static void readCloudsInFolder(const std::string& folderPath, std::vector<Cloud>& outPtrClouds);
	static int getFileNumberInFolder(const std::string& folderPath);
private :
	struct path_leaf_string
	{
		std::string operator()(const boost::filesystem::directory_entry& entry) const
		{
			return entry.path().leaf().string();
		}
	};
};

