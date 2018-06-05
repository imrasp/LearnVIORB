//
// Created by rasp on 1/23/18.
//
#include "log.h"

Log::Log(std::string name){
    //initialize Record folder
    boost::filesystem::path dir("./" + name);
    boost::filesystem::path dir2("./" + name + "/cam0");

    if (!(boost::filesystem::exists(dir))) {
        std::cout << "Doesn't Exists" << std::endl;
        if (boost::filesystem::create_directory(dir))
            std::cout << "....Successfully Created " << "./record_data/" << " Directory!" << std::endl;
    }
    if (!(boost::filesystem::exists(dir2))) {
        std::cout << "Doesn't Exists" << std::endl;
        if (boost::filesystem::create_directory(dir2))
            std::cout << "....Successfully Created " << "./record_data/cam0/" << " Directory!" << std::endl;
    }
}

Log::~Log(){

}

void Log::position_stream(double lat, double lon, double alt, double timestamp){
//    nlohmann::json drone_position = {{"misison_id", 1}, {"time", 1518017782}, {"lat", lat},{"lon", lon}, {"alt", alt}};
//    auto r = cpr::Post(cpr::Url{"127.0.0.1:3000/current_pos"},
//                       cpr::Body{drone_position.dump()} //,
////                       cpr::Header{{"Content-Type", "text/plain"}}
//    );
}