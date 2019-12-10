#include <iostream>
#include <cassert>
#include <cstring>
#include <streambuf>
#include <fstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include "json.hpp"
//#include "cnpy.h"

using json = nlohmann::json;
struct Point{
    double x,y;
};
using EdgeList = std::vector<uint32_t>;
using Graph = std::vector<EdgeList>;

using WeightAlloc = std::vector<double>;
using WeightAllocs = std::vector<WeightAlloc>;

using WeightMap = std::vector<std::array<double,5>>;
using WeightMaps = std::vector<WeightMap>;
using WeightPop = std::vector<WeightMaps>;

using PointRewards = std::vector<uint32_t>;

using randgen = std::default_random_engine;
using DenseGraph = std::vector<std::vector<char>>;
using MoveDistMap = std::vector<std::vector<uint32_t>>;

int rand_int(randgen & gen, int maxsize){
    std::uniform_int_distribution<int> distribution(0,maxsize-1);
    return distribution(gen);
}
struct Path{
    std::vector<uint32_t> travel_points;
    std::vector<uint32_t> dest_markers;
    //average reward is total_reward/age
    int total_reward=0;
    int age=0;
};
constexpr int NUM_PATHS = 200;
using PathCollection = std::vector<Path>;


std::string read_file(std::string filename){
    std::ifstream t(filename);
    std::string str((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
    return str;
}
double dist(Point p1,Point p2){
    return std::abs(p1.x-p2.x)+std::abs(p1.y-p2.y);
}
Graph calc_pathing_graph(Graph & full_graph,std::vector<Point> & points,int target_dist){
    Graph small_graph(full_graph.size());

    assert(full_graph.size() == points.size());
    for(size_t n = 0; n < points.size(); n++){
        Point source = points[n];
        for(size_t e : full_graph[n]){
            Point dest = points[e];
            if(dist(source,dest) < target_dist+0.01){
                small_graph[n].push_back(e);
            }
        }
    }
    return small_graph;
}
size_t nearest_point(Point p,std::vector<Point> & points){
    size_t neari = -1;
    double near_dist = 1e20;
    for(size_t i = 0; i < points.size(); i++){
        double cur_dist = dist(p,points[i]);
        if(near_dist > cur_dist){
            near_dist = cur_dist;
            neari = i;
        }
    }
    return neari;
}
Graph parse_graph(json & json){
    Graph graph;
    for(auto & node_info : json){
        EdgeList edges;
        for(auto & edge : node_info){
            edges.push_back(edge.get<uint32_t>());
        }
        graph.push_back(edges);
    }
    return graph;
}
std::vector<uint32_t> move_dist_calculator(Graph & move_graph,size_t start){
    size_t graph_size = move_graph.size();
    std::vector<uint32_t> dists(graph_size,-1);
    std::vector<char> has_visited(graph_size,false);
    EdgeList cur_nodes;
    cur_nodes.push_back(start);
    size_t depth = 0;
    while(cur_nodes.size()){
        EdgeList next_nodes;
        for(uint32_t n : cur_nodes){
            if(!has_visited[n]){
                has_visited[n] = true;
                dists[n] = depth;
                for(uint32_t e : move_graph[n]){
                    next_nodes.push_back(e);
                }
            }
        }
        cur_nodes.swap(next_nodes);
        depth++;
    }
    return dists;
}
MoveDistMap move_dist_calculator(Graph & move_graph){
    size_t graph_size = move_graph.size();
    MoveDistMap dists(graph_size);
    for(size_t n = 0; n < graph_size; n++){
        dists[n] = move_dist_calculator(move_graph,n);
    }
    return dists;
}
void bfs(Graph & move_graph,size_t start,size_t end,EdgeList & travel_points,size_t max_adds){
    size_t graph_size = move_graph.size();
    std::vector<char> has_visited(graph_size,false);
    EdgeList cur_nodes;
    cur_nodes.push_back(start);
    size_t num_adds = 0;
    while(cur_nodes.size()){
        EdgeList next_nodes;
        for(uint32_t n : cur_nodes){
            if(!has_visited[n]){
                has_visited[n] = true;
                travel_points.push_back(n);
                num_adds++;
                if(n == end || num_adds >= max_adds){
                    return;
                }
                for(uint32_t e : move_graph[n]){
                    next_nodes.push_back(e);
                }
            }
        }
        cur_nodes.swap(next_nodes);
    }
    assert(false && "cannot reach end of path!");
}
void generate_path(Path & path,size_t path_length,size_t src_point,Graph & move_graph){
    size_t graph_size = move_graph.size();
    size_t cur_point = src_point;
    while(path_length < path.travel_points.size()){
        size_t rand_point = rand()%graph_size;
        bfs(move_graph,cur_point,rand_point,path.travel_points,path_length-path.travel_points.size());
        path.dest_markers.push_back(path.travel_points.back());
        cur_point = path.travel_points.back();
    }
}
void generate_paths(PathCollection & paths,size_t path_length,Graph & move_graph,size_t src_point){
    for(Path & path : paths){
        generate_path(path,path_length,src_point,move_graph);
    }
}
DenseGraph compute_vispairs(Graph & graph){
    size_t graph_size = graph.size();
    DenseGraph res(graph_size,std::vector<char>(graph_size,false));
    for(size_t n = 0; n <graph.size(); n++){
        for(size_t e : graph[n]){
            res[n][e] = true;
        }
    }
    return res;
}
PointRewards evaluate_theif_rewards(Path & thief,EdgeList & rewards,DenseGraph & vis_graph){
    PointRewards t_rew;
    for(uint32_t n : thief.travel_points){
        uint32_t cur_reward = 0;
        for(uint32_t ri = 0; ri < rewards.size(); ){
            uint32_t rp = rewards[ri];
            if(vis_graph[n][rp]){
                rewards.erase(rewards.begin()+ri);
                cur_reward++;
            }
            else{
                ri++;
            }
        }
        t_rew.push_back(cur_reward);
    }
    return t_rew;
}
std::vector<PointRewards> all_theif_rewards(PathCollection & theif_paths,EdgeList & rewards,DenseGraph & vis_graph){
    std::vector<PointRewards> res(theif_paths.size());
    for(size_t i = 0; i < theif_paths.size(); i++){
        res[i] = evaluate_theif_rewards(theif_paths[i],rewards,vis_graph);
    }
    return res;
}
void evaluate_rewards(Path & guard,Path & thief,PointRewards & theif_rewards,DenseGraph & vis_graph){
    size_t travel_size = guard.travel_points.size();
    assert(travel_size == thief.travel_points.size());
    size_t n = 0;
    int agent_reward = 0;
    for(; n <travel_size; n++){
        if(vis_graph[guard.travel_points[n]][thief.travel_points[n]]){
            break;
        }
        else{
            agent_reward += theif_rewards[n];
        }
    }
    /*for(;n <travel_size; n++){
        guard_reward += theif_rewards[n];
    }*/
    guard.total_reward -= agent_reward;
    guard.age++;
    thief.total_reward += agent_reward;
    thief.age++;
}
void regen_path(EdgeList & travel_path, EdgeList & path_markers,size_t start_loc,Graph & move_graph){
    size_t cur_point = start_loc;
    for(uint32_t dest_marker : path_markers){
        bfs(move_graph,cur_point,dest_marker,travel_path,1000000);
        cur_point = travel_path.back();
    }
}
uint32_t compute_length(EdgeList & path,MoveDistMap & dist_map){
    uint32_t dist = 0;
    for(size_t i = 0; i < path.size()-1; i++){
        dist += dist_map[path[i]][path[i+1]];
    }
    return dist;
}
bool mutate_path(Path & path,Graph & move_graph,MoveDistMap & dist_map){
    size_t path_length = path.travel_points.size();
    size_t graph_size = move_graph.size();
    if(rand()%3 == 0){
        //erase from end of path
        size_t elim_end_path = rand()%(path.dest_markers.size()-1);
        path.dest_markers.erase(path.dest_markers.begin()+elim_end_path,path.dest_markers.end());
        size_t start_loc = path.travel_points.front();
        path.travel_points.clear();
        regen_path(path.travel_points,path.dest_markers,start_loc,move_graph);
        generate_path(path,path_length,path.travel_points.back(),move_graph);
        return true;
    }
    else{
        //erase from middle of path
        size_t start_erase = rand()%(path.dest_markers.size()-1);
        size_t end_erase = start_erase+rand()%(path.dest_markers.size()-start_erase);

        size_t replace_size = rand()%(2*(end_erase - start_erase));

        EdgeList start_path(path.dest_markers.begin(),path.dest_markers.begin()+start_erase);
        EdgeList end_path(path.dest_markers.begin()+end_erase,path.dest_markers.end());

        size_t len_start = compute_length(start_path,dist_map);
        size_t len_end = compute_length(end_path,dist_map);

        //regenerate start and end of path
        size_t start_loc = path.travel_points.front();
        for(int tries = 0; tries < 10; tries++){
            size_t start_middle = path.dest_markers[start_erase];
            size_t end_middle = path.dest_markers[end_erase];
            EdgeList middle_path(replace_size);
            size_t dist_middle = 0;
            for(size_t i = 0; i < replace_size; i++){
                middle_path[i] = rand()%graph_size;
                if(i > 0){
                    dist_middle += dist_map[middle_path[i-1]][middle_path[i]];
                }
            }
            dist_middle += dist_map[middle_path[start_middle]][middle_path[0]];
            dist_middle += dist_map[middle_path[replace_size-1]][end_middle];
            if(len_start + len_end + dist_middle < path_length){
                path.dest_markers.clear();
                path.dest_markers.insert(path.dest_markers.begin(),start_path.begin(),start_path.end());
                path.dest_markers.insert(path.dest_markers.begin(),middle_path.begin(),middle_path.end());
                path.dest_markers.insert(path.dest_markers.begin(),end_path.begin(),end_path.end());
                path.travel_points.clear();
                regen_path(path.travel_points,path.dest_markers,start_loc,move_graph);
                generate_path(path,path_length,path.travel_points.back(),move_graph);
                return true;
            }
        }
        return false;
    }
}
void clear_rewards(PathCollection & paths){
    for(Path & p : paths){
        p.age = 0;
        p.total_reward = 0;
    }
}
int count_rewards(PathCollection & paths){
    int sum = 0;
    for(Path & p : paths){
        sum += p.total_reward;
    }
    return sum;
}
void add_paths(PathCollection & paths,Graph & move_graph,MoveDistMap & dist_map,int num_to_add){
    size_t path_len = paths.size();
    for(int i = 0; i < num_to_add; i++){
        Path new_path = paths[rand()%path_len];
        while(!mutate_path(new_path,move_graph,dist_map));
        paths.push_back(new_path);
    }
}
struct SortVal{
    size_t idx;
    int val;
    bool operator < (SortVal sv)const{
        return val > sv.val;
    }
};
void elim_pathcollection(PathCollection & paths,size_t res_size){
    std::vector<SortVal> sortl;
    for(size_t i = 0; i < paths.size(); i++){
        sortl.push_back(SortVal{i,paths[i].total_reward});
    }
    std::sort(sortl.begin(),sortl.end());
    PathCollection new_paths;
    for(size_t i = 0; i < res_size; i++){
        new_paths.push_back(std::move(paths[sortl[i].idx]));
    }
    paths.swap(new_paths);
}
void compete_paths(Graph & move_graph,
                   Graph & vis_graph,
                   EdgeList & reward_points,
                   size_t guard_start,
                   size_t theif_start){
    const size_t PATH_LENGTH = 1000;
    const size_t NUM_PATHS = 100;
    const size_t NUM_ITERS = 10000000;
    const size_t ADD_PATHS = 5;
    PathCollection guard_paths(NUM_PATHS);
    PathCollection theif_paths(NUM_PATHS);
    generate_paths(guard_paths,PATH_LENGTH,move_graph,guard_start);
    generate_paths(theif_paths,PATH_LENGTH,move_graph,theif_start);
    DenseGraph dense_vis_graph = compute_vispairs(vis_graph);
    MoveDistMap dist_maps = move_dist_calculator(move_graph);
    for(size_t i = 0; i < NUM_ITERS; i++){
        //evaluate theif rewards
        std::vector<PointRewards> theif_rewards = all_theif_rewards(theif_paths,reward_points,dense_vis_graph);

        add_paths(guard_paths,move_graph,dist_maps,ADD_PATHS);
        add_paths(theif_paths,move_graph,dist_maps,ADD_PATHS);
        clear_rewards(guard_paths);
        clear_rewards(theif_paths);
        for(size_t t = 0; t < theif_paths.size(); t++){
            for(size_t g = 0; g < guard_paths.size(); g++){
                evaluate_rewards(guard_paths[g],theif_paths[t],theif_rewards[t],dense_vis_graph);
            }
        }
        elim_pathcollection(guard_paths,NUM_PATHS);
        elim_pathcollection(theif_paths,NUM_PATHS);
        std::cout << count_rewards(theif_paths) << std::endl;
    }
}


#define arr_to_point(arr) Point{arr[0].get<double>(),arr[1].get<double>()}
int main(int argc, const char ** argv){
    assert(argc == 3 && "needs 2 arguments, the filename of the enviornment and of the full visiblity graph");

    std::string env_fname = argv[1];
    std::string full_vis_fname = argv[2];

    //parses first file
    auto env_json = json::parse(read_file(env_fname));
    auto guard_loc_j = env_json.at("guard_locations");
    auto agent_loc_j = env_json.at("agent_location");
    std::string agent_outpath = env_json.at("agent_weightmap").get<std::string>();
    std::string guard_outpath = env_json.at("guard_weightmap").get<std::string>();
    std::string map_fname = env_json.at("map_fname").get<std::string>();
    std::string vis_info_fname = env_json.at("adjacency_list").get<std::string>();
    Point guard_loc = arr_to_point(guard_loc_j);//[0].get<double>();,guard_loc_j[1].get<double>()};
    Point agent_loc = arr_to_point(agent_loc_j);//[0].get<double>(),agent_loc_j[1].get<double>()};

    std::vector<Point> points;
    auto json_data = json::parse(read_file(vis_info_fname));
    for(auto & jval : json_data.at("points")){
        points.push_back(arr_to_point(jval));
    }

    size_t guard_idx = nearest_point(guard_loc,points);
    size_t agent_idx = nearest_point(agent_loc,points);

    EdgeList reward_vals;
    auto json_data2 = json::parse(read_file(map_fname));
    for(auto & jval : json_data2.at("reward_points")){
        Point rew_p = arr_to_point(jval);
        size_t rew_i = nearest_point(rew_p,points);
        reward_vals.push_back(rew_i);
    }

    auto full_vis_info = json::parse(read_file(full_vis_fname));
    Graph vis_graph;
    for(auto & node_info : full_vis_info){
        EdgeList edges;
        for(auto & edge : node_info){
            edges.push_back(edge.get<uint32_t>());
        }
        vis_graph.push_back(edges);
    }
    assert(vis_graph.size() == points.size());
    int guard_linesight = env_json.at("guard_linesight").get<int>();
    Graph pathing_graph = calc_pathing_graph(vis_graph,points,5);
    Graph pruned_vis_graph = calc_pathing_graph(vis_graph,points,guard_linesight);
    //WeightAllocs vis_graph_binmaps = graph_to_binmap(vis_graph);
    std::cout << pathing_graph.size() << "\n";
    compete_paths(
        pruned_vis_graph,
        pathing_graph,
        reward_vals,//TODO:
        agent_idx,
        guard_idx
    );
    //std::cout << points.size() << "\n";
    //std::cout << points[0].x << "\n";
}
