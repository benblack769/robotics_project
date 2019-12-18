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
    int x,y;
};
using PointList = std::vector<Point>;
using EdgeList = std::vector<uint32_t>;
using Graph = std::vector<EdgeList>;

using PointRewards = std::vector<uint32_t>;

using DenseGraph = std::vector<std::vector<char>>;
using MoveDistMap = std::vector<std::vector<uint32_t>>;

using PathSubset = std::vector<uint8_t>;

struct PtrPair{
    void * v1;
    void * v2;
    bool operator == (PtrPair o)const{
        return v1 == o.v1 && v2 == o.v2;
    }
};
/*
using randgen = std::default_random_engine;
int rand_int(randgen & gen, int maxsize){
    std::uniform_int_distribution<int> distribution(0,maxsize-1);
    return distribution(gen);
}
*/
struct Path{
    std::vector<uint32_t> travel_points;
    std::vector<uint32_t> dest_markers;
    //average reward is total_reward/age
    int total_reward=0;
    int age=0;
};
constexpr int NUM_PATHS = 200;
using PathCollection = std::vector<Path>;
bool validate_path(const Path & path,const PointList& points);
namespace std{
    template<>
    struct hash<PtrPair>{
        size_t operator()(const PtrPair & P)const{
            return hash<void*>()(P.v1) ^ hash<void*>()(P.v2);
        }
    };
}
struct ValPair{
    int v1;
    int v2;
};

std::string read_file(std::string filename){
    std::ifstream t(filename);
    std::string str((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
    return str;
}
int dist(Point p1,Point p2){
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
size_t nearest_point(Point p,PointList & points){
    size_t neari = -1;
    int near_dist = 1000000000;
    for(size_t i = 0; i < points.size(); i++){
        int cur_dist = dist(p,points[i]);
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
    constexpr uint32_t NULL_DIST = -1;
    std::vector<uint32_t> dists(graph_size,NULL_DIST);
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
    for(auto v : dists){
        assert(v != NULL_DIST);
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
    constexpr uint32_t NULL_PARENT = uint32_t(-1);
    size_t graph_size = move_graph.size();
    std::vector<uint32_t> parents(graph_size,NULL_PARENT);
    parents[start] = start;
    EdgeList cur_nodes;
    cur_nodes.push_back(start);
    size_t depth = 0;
    while(cur_nodes.size()){
        EdgeList next_nodes;
        for(uint32_t n : cur_nodes){
            if(n == end){
                break;
            }
            for(uint32_t e : move_graph[n]){
                if(parents[e] == NULL_PARENT){
                    next_nodes.push_back(e);
                    parents[e] = n;
                }
            }
        }
        depth++;
        cur_nodes.swap(next_nodes);
    }
    EdgeList new_edges;
    uint32_t cur_node = end;
    while(cur_node != start){
        new_edges.push_back(cur_node);
        cur_node = parents[cur_node];
    }
    std::reverse(new_edges.begin(),new_edges.end());
    if(new_edges.size() > max_adds){
        new_edges.resize(max_adds);
    }
    travel_points.insert(travel_points.end(),new_edges.begin(),new_edges.end());
}
void generate_path(Path & path,size_t path_length,size_t src_point,Graph & move_graph){
    size_t graph_size = move_graph.size();
    size_t cur_point = src_point;
    while(path_length > path.travel_points.size()){
        size_t rand_point = rand()%graph_size;
        bfs(move_graph,cur_point,rand_point,path.travel_points,path_length-path.travel_points.size());
        if(path.travel_points.size()){
            path.dest_markers.push_back(path.travel_points.back());
            cur_point = path.travel_points.back();
        }
    }
    assert(path.dest_markers.size() > 1);
}
void generate_paths(PathCollection & paths,size_t path_length,Graph & move_graph,size_t src_point){
    for(Path & path : paths){
        path.dest_markers = std::vector<uint32_t>();
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
PointRewards evaluate_theif_rewards(Path & thief,EdgeList rewards,DenseGraph & vis_graph){
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
void add_theif_rewards(std::vector<PointRewards> &cur_rewards,PathCollection & theif_paths,EdgeList reward_points,DenseGraph & vis_graph){
    for(size_t i = cur_rewards.size(); i < theif_paths.size(); i++){
        cur_rewards.push_back(evaluate_theif_rewards(theif_paths[i],reward_points,vis_graph));
    }
}
ValPair evaluate_rewards(Path & guard,Path & thief,PointRewards & theif_rewards,DenseGraph & vis_graph){
    size_t travel_size = guard.travel_points.size();
    if(travel_size != thief.travel_points.size()){
        int x = 0;
    }
    assert(travel_size == thief.travel_points.size());
    assert(travel_size == theif_rewards.size());
    int agent_reward = 0;
    size_t n = 0;
    for(; n < travel_size; n++){
        if(vis_graph[guard.travel_points[n]][thief.travel_points[n]]){
            break;
        }
        else{
            agent_reward += theif_rewards[n];
        }
    }
    int guard_reward = 0;
    for(;n <travel_size; n++){
        guard_reward += theif_rewards[n];
    }
    return ValPair{guard_reward,agent_reward};
}
void add(Path & guard, Path & thief,ValPair val){
    guard.total_reward += val.v1;
    guard.age++;
    thief.total_reward += val.v2;
    thief.age++;
}
void sub(Path & guard, Path & thief,ValPair val){
    guard.total_reward -= val.v1;
    guard.age--;
    thief.total_reward -= val.v2;
    thief.age--;
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
bool mutate_path(Path & path,Graph & move_graph,PointList & points,MoveDistMap & dist_map,size_t start_loc){
    size_t path_length = path.travel_points.size();
    size_t graph_size = move_graph.size();
    if(rand()%3 == 0){
        //erase from end of path
        size_t elim_end_path = std::max(int(0),rand()%((int)path.dest_markers.size())-1);
        path.dest_markers.erase(path.dest_markers.begin()+elim_end_path,path.dest_markers.end());
        path.travel_points.clear();
        regen_path(path.travel_points,path.dest_markers,start_loc,move_graph);
        size_t start_gen = path.travel_points.size() ? path.travel_points.back() : start_loc;
        generate_path(path,path_length,start_gen,move_graph);
        assert(path.dest_markers.size() > 1);
        assert(path.travel_points.size() == path_length);
        assert(validate_path(path,points));
        return true;
    }
    else{
        //erase from middle of path
        size_t start_erase = std::max(size_t(1),rand()%(path.dest_markers.size()));
        size_t end_erase = start_erase+rand()%(path.dest_markers.size()-start_erase);

        size_t replace_size = 1+rand()%(5+(end_erase - start_erase));

        EdgeList start_path(path.dest_markers.begin(),path.dest_markers.begin()+start_erase);
        EdgeList end_path(path.dest_markers.begin()+end_erase,path.dest_markers.end());

        size_t len_start = compute_length(start_path,dist_map) + dist_map[start_loc][start_path.front()];
        size_t len_end = compute_length(end_path,dist_map);

        //regenerate start and end of path
        for(int tries = 0; tries < 10; tries++){
            EdgeList middle_path(replace_size);
            size_t dist_middle = 0;
            for(size_t i = 0; i < replace_size; i++){
                middle_path[i] = rand()%graph_size;
                if(i > 0){
                    dist_middle += dist_map[middle_path[i-1]][middle_path[i]];
                }
            }
            dist_middle += dist_map[start_path.back()][middle_path.front()];
            dist_middle += dist_map[middle_path.back()][end_path.front()];
            if(len_start + len_end + dist_middle < path_length){
                path.dest_markers.clear();
                path.dest_markers.insert(path.dest_markers.end(),start_path.begin(),start_path.end());
                path.dest_markers.insert(path.dest_markers.end(),middle_path.begin(),middle_path.end());
                path.dest_markers.insert(path.dest_markers.end(),end_path.begin(),end_path.end());

                path.travel_points.clear();
                regen_path(path.travel_points,path.dest_markers,start_loc,move_graph);
                generate_path(path,path_length,path.travel_points.back(),move_graph);
                //if(!validate_path(path,points)){
                    std::vector<Point> path_points;
                    for(uint32_t tp : path.travel_points){
                        path_points.push_back(points[tp]);
                    }
                    for(size_t i = 1; i < path.travel_points.size(); i++){
                        if(dist(path_points[i],path_points[i-1]) > 5){
                            assert(false && "failed path validation\n");
                        }
                    }
                //}
                assert(validate_path(path,points));
                assert(path.dest_markers.size() > 1);
                assert(path.travel_points.size() == path_length);
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
int count_age(PathCollection & paths){
    int sum = 0;
    for(Path & p : paths){
        sum += p.age;
    }
    return sum;
}
bool validate_path(const Path & path,const PointList& points){
    if(path.travel_points.size() == 0){
        return true;
    }
    Point prevp = points[path.travel_points[0]];
    for(uint32_t n : path.travel_points){
        Point newp = points[n];
        if(dist(prevp,newp) > 5){
            return false;
        }
        prevp = newp;
    }
    return true;
}
bool validate_paths(const PathCollection & paths,const PointList& points){
    for(const Path & path : paths){
        if(!validate_path(path,points)){
            return false;
        }
    }
    return true;
}
void add_paths(PathCollection & paths,Graph & move_graph,PointList & points,
               MoveDistMap & dist_map,int num_to_add,int start_loc){
    size_t path_len = paths.size();
    for(int i = 0; i < num_to_add; i++){
        Path new_path = paths[rand()%path_len];
        new_path.age = 0;
        new_path.total_reward = 0;
        while(!mutate_path(new_path,move_graph,points,dist_map,start_loc));
        paths.push_back(new_path);
    }
}
template<class Ty>
void keep_indexes(std::vector<Ty> & paths,PathSubset & indexes){
    std::vector<Ty> new_paths;
    assert(indexes.size() == paths.size());
    for(size_t i = 0; i < indexes.size(); i++){
        if(indexes[i]){
            new_paths.push_back(std::move(paths[i]));
        }
    }
    paths.swap(new_paths);
}
PathSubset find_best(PathCollection & paths,size_t res_size){
    struct SortVal{
        uint32_t idx;
        int val;
        bool operator < (SortVal sv)const{
            return val > sv.val;
        }
    };
    int tot_rew = count_rewards(paths);
    int reward_noise = tot_rew / 10;
    std::vector<SortVal> sortl(paths.size());
    for(uint32_t i = 0; i < paths.size(); i++){
        sortl[i] = SortVal{i,paths[i].total_reward+rand()%(1+reward_noise)};
    }
    std::sort(sortl.begin(),sortl.end());
    PathSubset res(paths.size(),false);
    for(size_t i = 0; i < res_size; i++){
        res[sortl[i].idx] = true;
    }
    return res;
}
void save_paths(PathCollection & paths,std::string filename){
    std::vector<std::vector<uint32_t>> ppaths;
    for(Path & p : paths){
        ppaths.push_back(p.travel_points);
    }
    json j_list(ppaths);
    std::ofstream file(filename);
    if(!file){
        throw std::runtime_error("file " +filename+"failed to open");
    }
    file << j_list.dump() << '\n';
}
void compete_paths(Graph & move_graph,
                    Graph & vis_graph,
                    Graph & rew_graph,
                   PointList & points,
                   EdgeList & reward_points,
                   size_t guard_start,
                   size_t theif_start,
                    std::string name){
    const size_t PATH_LENGTH = 500;
    const size_t NUM_PATHS = 2000;
    const size_t NUM_ITERS = 10000000;
    const size_t ADD_PATHS = 20;
    PathCollection guard_paths(NUM_PATHS,Path{});
    PathCollection theif_paths(NUM_PATHS,Path{});
    std::cout << "initted1" << std::endl;
    generate_paths(guard_paths,PATH_LENGTH,move_graph,guard_start);
    generate_paths(theif_paths,PATH_LENGTH,move_graph,theif_start);
    DenseGraph dense_vis_graph = compute_vispairs(vis_graph);
    DenseGraph dense_rew_graph = compute_vispairs(rew_graph);
    MoveDistMap dist_maps = move_dist_calculator(move_graph);
    std::cout << "initted2" << std::endl;
    int save_count = 0;
    std::vector<PointRewards> theif_point_rewards;
    add_theif_rewards(theif_point_rewards,theif_paths,reward_points,dense_rew_graph);
    for(size_t g = 0; g < guard_paths.size(); g++){
        for(size_t t = 0; t < theif_paths.size(); t++){
            ValPair eval = evaluate_rewards(guard_paths[g],theif_paths[t],theif_point_rewards[t],dense_vis_graph);
            add(guard_paths[g],theif_paths[t],eval);
        }
    }

    for(size_t i = 0; i < NUM_ITERS; i++){
        //evaluate theif rewards
        add_paths(guard_paths,move_graph,points,dist_maps,ADD_PATHS,guard_start);
        add_paths(theif_paths,move_graph,points,dist_maps,ADD_PATHS,theif_start);
        add_theif_rewards(theif_point_rewards,theif_paths,reward_points,dense_rew_graph);
        //clear_rewards(guard_paths);
        //clear_rewards(theif_paths);
        for(size_t t = NUM_PATHS; t < NUM_PATHS+ADD_PATHS; t++){
            for(size_t g = 0; g < guard_paths.size(); g++){
                ValPair eval = evaluate_rewards(guard_paths[g],theif_paths[t],theif_point_rewards[t],dense_vis_graph);
                add(guard_paths[g],theif_paths[t],eval);
            }
        }
        for(size_t g = NUM_PATHS; g < NUM_PATHS+ADD_PATHS; g++){
            for(size_t t = 0; t < NUM_PATHS; t++){
                ValPair eval = evaluate_rewards(guard_paths[g],theif_paths[t],theif_point_rewards[t],dense_vis_graph);
                add(guard_paths[g],theif_paths[t],eval);
            }
        }
        assert(validate_paths(guard_paths,points));
        assert(validate_paths(theif_paths,points));
        std::cout << "evaluated" << std::endl;
        PathSubset best_guards = find_best(guard_paths,NUM_PATHS);
        PathSubset best_thiefs = find_best(theif_paths,NUM_PATHS);

        for(size_t t = 0; t < theif_paths.size(); t++){
            for(size_t g = 0; g < guard_paths.size(); g++){
                if(!best_guards[g] || !best_thiefs[t]){
                    ValPair eval = evaluate_rewards(guard_paths[g],theif_paths[t],theif_point_rewards[t],dense_vis_graph);
                    sub(guard_paths[g],theif_paths[t],eval);
                }
            }
        }

        keep_indexes(guard_paths,best_guards);
        keep_indexes(theif_paths,best_thiefs);
        keep_indexes(theif_point_rewards,best_thiefs);
        for(size_t t = 0; t < theif_paths.size(); t++){
            assert(theif_paths[t].age == guard_paths.size());
        }
        for(size_t g = 0; g < guard_paths.size(); g++){
            assert(guard_paths[g].age == theif_paths.size());
        }
        //elim_pathcollection(guard_paths,NUM_PATHS);
        //elim_pathcollection(theif_paths,NUM_PATHS);
        std::cout << count_rewards(guard_paths)/double(count_age(guard_paths)) << "  "
                    << count_rewards(theif_paths)/double(count_age(guard_paths)) << "\n";
        if(i == (1 << save_count)*8){
            std::cout << "saved\n";
            std::string agent_fpath = "wm_img_dir/"+name+"_agent.weightmap.json."+std::to_string(save_count);
            std::string guard_fpath = "wm_img_dir/"+name+"_guard.weightmap.json."+std::to_string(save_count);
            save_paths(guard_paths,guard_fpath);
            save_paths(theif_paths,agent_fpath);
            save_count++;
        }
    }
}


#define arr_to_point(arr) Point{arr[0].get<int>(),arr[1].get<int>()}
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

    PointList points;
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
    int reward_collect_rad = env_json.at("reward_collect_radius").get<int>();
    Graph pathing_graph = calc_pathing_graph(vis_graph,points,5);
    Graph pruned_vis_graph = calc_pathing_graph(vis_graph,points,guard_linesight);
    Graph pruned_rew_graph = calc_pathing_graph(vis_graph,points,reward_collect_rad);
    //WeightAllocs vis_graph_binmaps = graph_to_binmap(vis_graph);
    std::cout << pathing_graph.size() << "\n";
    compete_paths(
        pathing_graph,
        pruned_vis_graph,
        pruned_rew_graph,
        points,
        reward_vals,//TODO:
        agent_idx,
        guard_idx,
        env_fname
    );
    //std::cout << points.size() << "\n";
    //std::cout << points[0].x << "\n";
}
