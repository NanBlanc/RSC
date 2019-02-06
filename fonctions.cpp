#include "fonctions.h"
#include <windows.h> //for create directory

// When passing char arrays as parameters they must be pointers
InArgs Tools::argParser(int argc, char** argv) {
	InArgs input;
	if (argc < 2)
		exitRSC("Welcome on RSC, Please select a running mode : \n\t -all\n\t -train\n\t -predict\n\t -help\n");
	if (argc < 3 && (std::string(argv[1]) == "-all" || std::string(argv[1]) == "-train" || std::string(argv[1]) == "-predict"))
		exitRSC("ERROR : Missing Project Path, Please read help [-h] or [-help]");
	if (std::string(argv[1]) == "-vivi") {easterEgg(); exitRSC("");} //don't ask any questions
	if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "-help")
	{
		Tools::printHelp();
		exitRSC("");
	}
	

	input.projectPath = argv[2];
	if (std::string(argv[1]) == "-all")
	{
		if (argc > 2) {
			for (int i = 3; i < argc; i++) { /* We will iterate over argv[] to get the parameters stored inside.* Note that we're starting on 1 because we don't need to know the * path of the program, which is stored in argv[0] */
				if (std::string(argv[i]) == "-s") {
					input.savePath = input.projectPath + "/models";
					input.saveModel = true;
				}
				else
					exitRSC("ERROR : invalid option for -all, Please read help [-h] or [-help]");
			}
		}
	}
	else if (std::string(argv[1]) == "-train")
	{
		input.predict = false;
		input.saveModel = true;
		input.savePath = input.projectPath + "\\models";
		if (argc > 3) 
			exitRSC("ERROR : invalid option for -train, Please read help [-h] or [-help]");
	}
	else if (std::string(argv[1]) == "-predict")
	{
		input.train = false;
		input.loadModel = true;
		if (argc > 3) {
			for (int i = 3; i < argc; i++) {
				if (std::string(argv[i]) == "-m") {
					if (i + 1 < argc) {
						input.loadPath = argv[i + 1];
						i++;
					}
					else 
						exitRSC("ERROR : invalid option for - predict, Please read help[-h] or [-help]");
				
				}
				else
					exitRSC("ERROR : invalid option for -predict, Please read help [-h] or [-help]");
			}
		}
		else {
			std::cerr << "Warning : Default path for model, First one will be selcted, Please read help [-h] or [-help]\n";
			boost::filesystem::path p(input.projectPath + "\\models");
			std::vector<boost::filesystem::path> vecModels;
			readDirectoryExtension(p, ".yml", vecModels);
			if (vecModels.size() == 0)
				exitRSC("ERROR : No Models found");
			input.loadPath = input.projectPath + "\\models\\" +vecModels[0].string();
			std::cerr << "Warning : Default model load : " << input.loadPath << "\n";
		}
	}
	else
	{
		exitRSC("Select a running mode:\n\t-all\n\t-train\n\t-predict\n\t-help\n");
	}
	return input;
}

void Tools::exitRSC(std::string message) {
	std::cerr << message;
	std::cerr << "---\\-\\-\\-\\-\\END/-/-/-/-/-/---\n";
	Sleep(2000);
	exit(0);
}
void Tools::printHelp() {
	std::cerr << "RSC Program help menu:\n";
	std::cerr << "1-RSC call\n";
	std::cerr << "2-Project directory structure\n";
	std::cerr << "3-Informations\n";
	std::cerr << "--------------\n";
	std::cerr << "1- RSC call :\n";
	std::cerr << "3 possible modes : \n";
	std::cerr << "-all : train and predict accordingly to data in project path\n";
	std::cerr << "\tmandatory args :\n";
	std::cerr << "\t\tproject path\n";
	std::cerr << "\toptional args : \n";
	std::cerr << "\t\t-s : save trained model in [projectPath]/models\n";
	std::cerr << "\texemple : [RSCpath] -all C:/desktop/Project -s\n";
	std::cerr << "-train : train a model and save it accordingly to data in project path\n";
	std::cerr << "\tmandatory args :\n";
	std::cerr << "\t\tproject path\n";
	std::cerr << "-predict : predict data with a given model\n";
	std::cerr << "\tmandatory args :\n";
	std::cerr << "\t\tproject path\n";
	std::cerr << "\toptional args : \n";
	std::cerr << "\t\t-m : model path \n";
	std::cerr << "\t\t /!\\ : if not given, will load the first model in [ProjectPath]/models\n";
	std::cerr << "-h : show help... help-ception !?\n";
	std::cerr << "--------------\n";
	std::cerr << "2-Project directory structure :\n";
	std::cerr << "MyProject\n";
	std::cerr << "\tGround_truth\n";
	std::cerr << "\t\tClass_1\n";
	std::cerr << "\t\t\tCloud_1\n";
	std::cerr << "\t\t\tCloud_2\n";
	std::cerr << "\t\t\tCloud...\n";
	std::cerr << "\t\tClass 2\n";
	std::cerr << "\t\t\tCloud_1\n";
	std::cerr << "\t\t\tCloud...\n";
	std::cerr << "\t\tClass... [optional : 2 Class at least]\n";
	std::cerr << "\tModels [optional : if -all or -train ; automatically create when saving model]\n";
	std::cerr << "\tIn_clouds [optional if only -train]\n";
	std::cerr << "\tOut_clouds [optional : automatically create if -all or -predict\n";
	std::cerr << "--------------\n";
	std::cerr << "3-Informations :\n";
	std::cerr << "Written by :\n";
	std::cerr << "\tOlivier Stocker\n";
	std::cerr << "\tstocker.olivier@gmail.com\n";
	std::cerr << "\n";
	exitRSC("");
}

void Tools::createFolder(const char * path)
{
	if (!CreateDirectory(path, NULL))
	{
		return;
	}
}



void Tools::readDirectory(const std::string& name, std::vector<std::string>& v)
{
	boost::filesystem::path p(name);
	boost::filesystem::directory_iterator start(p);
	boost::filesystem::directory_iterator end;
	std::transform(start, end, std::back_inserter(v), path_leaf_string());
}




void Tools::readDirectoryExtension(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret)
{
	if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return;

	boost::filesystem::recursive_directory_iterator it(root);
	boost::filesystem::recursive_directory_iterator endit;
	while (it != endit)
	{
		if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ext) ret.push_back(it->path().filename());
		++it;
	}
}

void Tools::readCloudsInFolder(const std::string& folderPath, std::vector<Cloud>& outPtrClouds)
{
	std::vector<boost::filesystem::path> inCloudsPath;
	Tools::readDirectoryExtension(folderPath, ".txt", inCloudsPath);
	std::cerr << "In_Clouds :\n";
	for (int i = 0; i < inCloudsPath.size(); i++) {
		std::cerr << inCloudsPath[i] << "\n";
		outPtrClouds.push_back(Cloud(folderPath + "\\" + inCloudsPath[i].string()));
	}
}

int Tools::getFileNumberInFolder(const std::string& folderPath)
{
	std::vector<std::string> v;
	Tools::readDirectory(folderPath, v);
	return v.size();
}





















































































































































void Tools::easterEgg() {
	std::cerr << "                                 *//((((((((((########%%%%%%%%%%%%%%%%%%%%%####%#//***..,                            \n";
	std::cerr << "                               ,/(((##(##((######%%%%%%%%%%%%%%%%%%%%%%%%%%##%%%##///***,,.                          \n";
	std::cerr << "                          ..,,,/(####((########%%%##%%%%%%%%%#%%%%%%%%%%%%###%%%%%%##((/***,,,.*,                    \n";
	std::cerr << "                        ....,,/*/#############%%%((#%#########%##%%%%%%%%%%##%%%%%%%%%%%%%%%%%%%##//(                \n";
	std::cerr << "                      ....../,,(#%%########%%%%%((#########%%%%%%%%%%%%&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##(****       \n";
	std::cerr << "                     .,,,,./((##%%%%%%%%%%%%%%%%%%%%%%%%&%%%&&&&&&%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#####(****.\n";
	std::cerr << "                    ,*,..,*(##%%%%#(***********//////////////(((((((((((((((((/((((#%%%%%%%%%%%%##%%%%%%%%###########\n";
	std::cerr << "                    ,,.,*(((#*,****////*//*/******/////////////////////////////(((((((######%#####(***,//############\n";
	std::cerr << "                  ..,,/(((/****/*/(((((((((((((///////////////////////////(((((((((((###(##########(**.     */(######\n";
	std::cerr << "                  *,/((((**///((((((((###########(((/////////////////////((((((((((((#############(((/.            /#\n";
	std::cerr << "                  **(#(///((((((((((((#############((((((/(////////////(((((((((((((((((((((/(((####((/              \n";
	std::cerr << "                  **##((/((///////((((((((#########((((((((((((((((((((((((((((((((((((/(////////((((((*,            \n";
	std::cerr << "                  */((/////*///(((####################((((((((((((((((((((((((((((((((####%%%#####(((((/,,,,,        \n";
	std::cerr << "               .***(#%%%%##(((((((((########%%%%%%##########(((((((((((((((((%%%%%#((((((((((///////((#####(*,,,.    \n";
	std::cerr << "        **/##%%%%%%#//(*//((((####%%%############%%%%%%%%%%&%%%%%%%%%%%%%%%%%(((((((#########((((///**,   /########(/\n";
	std::cerr << "        %%%(%#%%%(    (//(((###########%%###########%%%%%%&&%%##%%%%%%%%%%#((((#################((//**.    ,/########\n";
	std::cerr << "        /#%%%%%#/     /*/(######(////(((((((#########%%%%#(((((((((((%%%%#((#######(((///////((((((//*     ,(((####*,\n";
	std::cerr << "          *#%%%###( ,*,*/((###////(%%%%%%%%(##########%%%%((((///(((#%%%%(((#####((#%%%%%%#///*((((//    (####/###,,,\n";
	std::cerr << "           *%%#/.*(##***//****///(#%%%%%%%%##########(%%%%#/*******/#%%%#(((#######%%%%%%%%#(((//*///**##(,   *(#(,  \n";
	std::cerr << "            /%#/    ./*,,,**/////((((((((((########(/(%%%/*,..,,,,,**/%%%(((((((######((((////////****,       /(#,,  \n";
	std::cerr << "             (%#.   ****,****//(((((((((((((((((/*//(%%#/*,...,,,,,,**/#%%////(((((((((((((///////****,      *(#*,   \n";
	std::cerr << "              /%#   *,*****,*****////////////*****/#%%(/**,,..,,,,,*****(%%(/////////////////////*****,     *(#,     \n";
	std::cerr << "              **#%(**,****,,,,,,,,,**************/%%%(/**,,..,,,,,,,*****/#%%(/////////////////********,  *(#*       \n";
	std::cerr << "                **#%#/******,,,,,,*************#%%##(/**,,,...,,,,,,,*****/(##%%///////////////*********/##,,        \n";
	std::cerr << "                 /(,,,%%(*,*********/////*,#%%#(((((/*,,,,....,,,*,,,******//(((##%%#//////////*****(#%/*,,,         \n";
	std::cerr << "                  ,,,,,****/##%%%%%%%%%%%####((((((*,,,,..,,,,*************////((((((((#########//*******,,,         \n";
	std::cerr << "                  .,,,,*****////(((((((((((((((((***,,,,,********************////((((((((((//////********//          \n";
	std::cerr << "                  ,,,,,******////////////(((((******/////////////////**********/((///////////////********##          \n";
	std::cerr << "                   ,********/////////////((((/////////////////////////////*****//////////////////*******,%%          \n";
	std::cerr << "                   ,*******/////////////////(((((##%%%(/////////////**(%%##(////////////////////*********,.          \n";
	std::cerr << "                   ******////////(((((/////////((#######(/////////*/((((((((/(//////////////////*********(           \n";
	std::cerr << "                   ****///////(((((((((////////((((((####(((/////////(((((((/////////////////////*******/,           \n";
	std::cerr << "                   **////////((######(((((((////((((((#(#((((((((((((((((((((((((/////////////////******(            \n";
	std::cerr << "                   (*///////((##%######(((((((((((((((#####((((((((((((((((((((((((((((((((((((///*****/             \n";
	std::cerr << "                   %///////((#########(##((((((((##############((((#####(##((((((((((((((((((((//******/             \n";
	std::cerr << "                   %*(//////(((###%%%%####################%%%%%%%%%%##########((((((((######((//******(              \n";
	std::cerr << "                    ,(#(((/(((((((########((((((((((((((((###########(((((((((/////((######((//**//////              \n";
	std::cerr << "                      (((((((((((((((((((((((((((((((((((((((((((((((((////////////((((((((////////(((               \n";
	std::cerr << "                      /##(((((((((((((/////////(((((((###(((((((((((((((/////////////////////////((((                \n";
	std::cerr << "                       /(%####(((((((((/////////(((((((((((((#(((((((((((////////////////////(((###(                 \n";
	std::cerr << "                        (#%%%#####((((((((///////(((((((((((((((((((((////////////////////(((#####(                  \n";
	std::cerr << "                          (%%%%%##(#((((((((((((((((((((((((((((((((((//////////////////(((####(#(                   \n";
	std::cerr << "                             %%%%%###((((((((((((#(#((((((((((((((((((////////((((((((((((#####(/                    \n";
	std::cerr << "                              (#%%%%%###(((((((((((((((((((((###(((((((((((((((((((((((#######(#                     \n";
	std::cerr << "                               /(#%%%##%####(((((((((((((((#((##((((((((((((((((#############(#                      \n";
	std::cerr << "                                 (##%%%%########################((((((((((((##############(((                        \n";
	std::cerr << "                                   (#%%%%%########################(#(((##%%%%%#%###########                          \n";
	std::cerr << "                                     (%%%%%%%%#%%%##%%%%%###%%%%%####%%%%%%%%%%%#%%%####(                            \n";
	std::cerr << "                                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%##                               \n";
	std::cerr << "                                         .##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#%%%%%#                                  \n";
	std::cerr << "                                               ##%%%%%%%%%%%%%%%%%%%%%%##%%####%                                     \n";
	std::cerr << "                                                        ////////*                                                    \n";
	std::cerr << "                                                                                                                             \n";
	std::cerr << "         @@@@@      @@@@@@@@         @@@@@@@@#   @@@@@      @@@@@@@@    @@@@@@@@@   #@@@@@@@@   @@@@@% %@@@@@@@@.@@@@        \n";
	std::cerr << "         @@@@@      @@@@@@@@       *@@@@@&@@@@@  @@@@@      @@@@@@@@   @@@@@@@@@@& &@@@@@@@@@@  @@@@@% %@@@@@@@@.@@@@        \n";
	std::cerr << "         @@@@@      @@@@@@@@*      @@@@@% @@@@@  @@@@@     #@@@@@@@@.  @@@@@ @@@@@ @@@@@ %@@@@. @@@@@% %@@@@@    @@@         \n";
	std::cerr << "         @@@@@     %@@@@@@@@@      @@@@@% @@@@@  @@@@@     @@@@@@@@@@  @@@@@@@     /@@@@@@/     @@@@@% %@@@@@,,,             \n";
	std::cerr << "         @@@@@     @@@@@(@@@@      @@@@@%        @@@@@     @@@@@@@@@@   @@@@@@@@&    @@@@@@@@   @@@@@% %@@@@@@@@             \n";
	std::cerr << "         @@@@@     @@@@% @@@@,     @@@@@% &&&&&  @@@@@     @@@@/%@@@@      @@@@@@@     ,@@@@@@* @@@@@% %@@@@@(((             \n";
	std::cerr << "         @@@@@    .@@@@@@@@@@@     @@@@@% @@@@@  @@@@@    /@@@@@@@@@@* @@@@@ @@@@@ &@@@@ .@@@@@ @@@@@% %@@@@@                \n";
	std::cerr << "         @@@@@@@@ @@@@@@@@@@@@     #@@@@& @@@@@  @@@@@@@@ @@@@@@@@@@@@ @@@@@ @@@@@ /@@@@. @@@@@ @@@@@% %@@@@@                \n";
	std::cerr << "         @@@@@@@@ @@@@@, @@@@@      @@@@@@@@@@   @@@@@@@@ @@@@@  @@@@@ *@@@@@@@@@%  @@@@@@@@@@  @@@@@% %@@@@@                \n";
	std::cerr << "                                       (@@%.                              ,&@@*        (@@%.                                 \n";
	std::cerr << "                                                                                                                             \n";
	std::cerr << "                                                                                                                             \n";
	std::cerr << "                                                                                                                             \n";
	std::cerr << "                                                                                                                             \n";
	std::cerr << "  @@@@@@@.   @@@@ @@@@@@@@%  .@@@@@@@   @@@@@@@@@@@       %@@@@@@%   ,@@@@@      @@@@@@@     &@@@@@@&     @@@@@@@,   @@@@@@@@\n";
	std::cerr << "@@@@@@@@@@@  @@@@ @@@@@@@@% @@@@@@@@@@@ @@@@@@@@@@@     .@@@@@@@@@@. ,@@@@@     .@@@@@@@&   @@@@@@@@@@. &@@@@@@@@@@  @@@@@@@@\n";
	std::cerr << "@@@@@ @@@@@* @@@, @@@@@     @@@@@ @@@@@    @@@@@        @@@@@ ,@@@@@ ,@@@@@     &@@@@@@@@  (@@@@* @@@@& @@@@@ @@@@@  @@@@@   \n";
	std::cerr << "@@@@@ @@@@@# ,,,  @@@@@     @@@@@@         @@@@@        @@@@@ .@@@@@ ,@@@@@     @@@@&@@@@  ,@@@@@#      @@@@@@       @@@@@   \n";
	std::cerr << "@@@@@ #####/      @@@@@@@@.  @@@@@@@@      @@@@@        @@@@@ .##### ,@@@@@     @@@@,@@@@%  %@@@@@@@#    @@@@@@@@    @@@@@@@@\n";
	std::cerr << "@@@@@             @@@@@@@@.    @@@@@@@@    @@@@@        @@@@@        ,@@@@@    *@@@@ @@@@@     @@@@@@@*    #@@@@@@@  @@@@@@@@\n";
	std::cerr << "@@@@@ @@@@@#      @@@@@     @@@@@ @@@@@    @@@@@        @@@@@ ,@@@@@ ,@@@@@    @@@@@@@@@@@ .@@@@%.@@@@@ @@@@@ @@@@@, @@@@@   \n";
	std::cerr << "@@@@@ @@@@@,      @@@@@     @@@@@ @@@@@.   @@@@@        @@@@@ ,@@@@@ ,@@@@@    @@@@@@@@@@@( @@@@% @@@@@ @@@@@ %@@@@* @@@@@   \n";
	std::cerr << "@@@@@@@@@@@       @@@@@@@@@ #@@@@@@@@@@    @@@@@         @@@@@@@@@@  ,@@@@@@@@ @@@@@ ,@@@@@ @@@@@@@@@@@  @@@@@@@@@@  @@@@@@@@\n";
	std::cerr << "  @@@@@@@         @@@@@@@@@   @@@@@@@.     @@@@@          %@@@@@@/   ,@@@@@@@@%@@@@@  @@@@@  .@@@@@@@     %@@@@@@(   @@@@@@@@\n";
	std::cerr << "                                                                                                                             \n";
	std::cerr << "                                                                                                                             \n";
}