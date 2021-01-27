#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;
// helper function for main.cpp
// it takes a string of file path and read the data into a vector of bytes
static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    // us fstream to open the file in the binary mode, 
    std::ifstream is{path, std::ios::binary | std::ios::ate}; // ios::ate 打开文件定位到文件尾（at end），但是不能写文件
    if( !is )
        return std::nullopt;
    
    // because the input stream (is) is in the end of the file, the size of the input stream can be get using is.tellg()
    auto size = is.tellg();
    // create a vector of bytes with the same size of the input stream
    std::vector<std::byte> contents(size);    
    
    // input stream seeks back the the begining of the file
    is.seekg(0); 
    // the entire file is read into the content vector
    is.read((char*)contents.data(), size); 

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv) // argument count and argument vector
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            // search for the -f file flag and store the file name into osm_data_file
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    // read file into osm_data
    std::vector<std::byte> osm_data;
  
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, 10, 10, 90, 90};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
