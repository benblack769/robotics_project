#include <iostream>
#include <cassert>
#include <cstring>
#include <streambuf>
#include <fstream>
#include "json.hpp"

using json = nlohmann::json;
struct Point{
    double x,y;
};
using EdgeList = std::vector<uint32_t>;
using Graph = std::vector<EdgeList>;
using WeightMap = std::vector<std::array<double,5>>;
using WeightAlloc = std::vector<double>;
using WeightMaps = std::vector<WeightMap>;
using WeightPop = std::vector<WeightMaps>;

std::string read_file(std::string filename){
    std::ifstream t(filename);
    std::string str((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
    return str;
}
void parse_json(std::string filename){
    auto parent_json = json::parse(read_file(filename));
    //std::string new_filename =
    auto main_json = json::parse(read_file(filename));

    int count = 0;
    for(auto & arg : parent_json.at("points")){
        count++;
    }
    std::cout << count << "\n";
}
double dist(Point p1,Point p2){
    return abs(p1.x-p2.x)+abs(p1.y-p2.y);
}
Graph calc_pathing_graph(Graph & full_graph,std::vector<Point> & points){
    Graph small_graph(full_graph.size());

    assert(full_graph.size() == points.size());
    for(int n = 0; n < points.size(); n++){
        Point source = points[n];
        for(int e = 0; e < full_graph[n].size(); e++){
            Point dest = points[full_graph[n][e]];
            if(dist(source,dest) < 5.01){
                small_graph[n].push_back(e);
            }
        }
    }
    return small_graph;
}

#define arr_to_point(arr) Point{arr[0].get<double>(),arr[1].get<double>()}
int main(int argc, const char ** argv){
    assert(argc == 4 && "needs three arguments, the filename of the enviornment and of the visibility info graph and the full visiblity graph");

    std::string env_fname = argv[1];
    std::string vis_info_fname = argv[2];
    std::string vis_fname = argv[3];

    //parses first file
    auto env_json = json::parse(read_file(env_fname));
    auto guard_loc_j = env_json.at("guard_locations");
    auto agent_loc_j = env_json.at("agent_location");
    Point guard_loc = arr_to_point(guard_loc_j);//[0].get<double>();,guard_loc_j[1].get<double>()};
    Point agent_loc = arr_to_point(agent_loc_j);//[0].get<double>(),agent_loc_j[1].get<double>()};

    std::vector<Point> points;
    auto json_data = json::parse(read_file(vis_info_fname));
    for(auto & jval : json_data.at("points")){
        points.push_back(arr_to_point(jval));
    }
    auto full_vis_info = json::parse(read_file(vis_fname));
    Graph vis_graph;
    for(auto & node_info : full_vis_info){
        EdgeList edges;
        for(auto & edge : node_info){
            edges.push_back(edge.get<uint32_t>());
        }
        vis_graph.push_back(edges);
    }
    assert(vis_graph.size() == points.size());
    Graph pathing_graph = calc_pathing_graph(vis_graph,points);
    std::cout << pathing_graph.size() << "\n";

    //std::cout << points.size() << "\n";
    //std::cout << points[0].x << "\n";
}
