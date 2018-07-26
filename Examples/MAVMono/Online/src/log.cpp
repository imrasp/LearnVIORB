//
// Created by rasp on 1/23/18.
//
#include "log.h"

Log::Log(std::string name){
    //initialize Record folder
    boost::filesystem::path dir(name);
    boost::filesystem::path dir2(name + "/cam0");

    if (!(boost::filesystem::exists(dir))) {
        std::cout << "Doesn't Exists" << std::endl;
        if (boost::filesystem::create_directory(dir))
            std::cout << "....Successfully Created " << name << "Directory!" << std::endl;
    }
    if (!(boost::filesystem::exists(dir2))) {
        std::cout << "Doesn't Exists" << std::endl;
        if (boost::filesystem::create_directory(dir2))
            std::cout << "....Successfully Created " << name << "/cam0/" << " Directory!" << std::endl;
    }
}

Log::~Log(){

}