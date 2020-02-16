#include <iostream>
#include <cassert>
#include <cstring>
#include <streambuf>
#include <fstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <ctime>
#include <thread>
#include <random>
#include <omp.h>
#include "json.hpp"

size_t numHWThreads = std::thread::hardware_concurrency();

using json = nlohmann::json;
struct Point{
    int x=0,y=0;
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

using randgen = std::default_random_engine;
int rand_int(randgen & gen, int maxsize){
    std::uniform_int_distribution<int> distribution(0,maxsize-1);
    return distribution(gen);
}
int rand(randgen & gen){
    return rand_int(gen,1<<30);
}
struct Path{
    std::vector<uint32_t> travel_points;
    std::vector<uint32_t> dest_markers;
    //average reward is total_reward/age
};
struct PathReward{
    int total_reward=0;
    int age=0;
};
using PathRewards = std::vector<PathReward>;
//constexpr int NUM_PATHS = 200;
using PathCollection = std::vector<Path>;
bool validate_path(const Path & path,const PointList& points,size_t start_loc);
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
    assert(t && "file did not open properly");
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
            if(dist(source,dest) <= target_dist){
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
void bfs(const Graph & move_graph,size_t start,size_t end,EdgeList & travel_points,size_t max_adds){
    constexpr uint32_t NULL_PARENT = uint32_t(-1);
    size_t graph_size = move_graph.size();
    std::vector<uint32_t> parents(graph_size,NULL_PARENT);
    parents[start] = start;
    EdgeList cur_nodes;
    cur_nodes.push_back(start);
    EdgeList next_nodes;
    size_t depth = 0;
    while(cur_nodes.size()){
        next_nodes.resize(0);
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
void generate_path(Path & path,randgen & gen,size_t path_length,size_t src_point,const Graph & move_graph){
    size_t graph_size = move_graph.size();
    size_t cur_point = src_point;
    while(path_length > path.travel_points.size()){
        size_t rand_point = rand(gen)%graph_size;
        bfs(move_graph,cur_point,rand_point,path.travel_points,path_length-path.travel_points.size());
        if(path.travel_points.size()){
            path.dest_markers.push_back(path.travel_points.back());
            cur_point = path.travel_points.back();
        }
    }
    assert(path.dest_markers.size() > 1);
}
void generate_paths(PathCollection & paths,randgen & gen,size_t path_length,const Graph & move_graph,size_t src_point){
    for(Path & path : paths){
        path.dest_markers = std::vector<uint32_t>();
        generate_path(path,gen,path_length,src_point,move_graph);
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
PointRewards evaluate_theif_rewards(const Path & thief,EdgeList rewards,const DenseGraph & vis_graph){
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
PathRewards rand_rewards(){

}

void add_theif_rewards(std::vector<PointRewards> &cur_rewards,const PathCollection & theif_paths,EdgeList reward_points,const DenseGraph & vis_graph){
    for(size_t i = cur_rewards.size(); i < theif_paths.size(); i++){
        cur_rewards.push_back(evaluate_theif_rewards(theif_paths[i],reward_points,vis_graph));
    }
}
ValPair evaluate_rewards(const Path & guard,const Path & thief,const PointRewards & theif_rewards,const DenseGraph & vis_graph){
    size_t travel_size = guard.travel_points.size();
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
void add(PathReward & guard, PathReward & thief,ValPair val){
    guard.total_reward += val.v1;
    guard.age++;
    thief.total_reward += val.v2;
    thief.age++;
}
void sub(PathReward & guard, PathReward & thief,ValPair val){
    guard.total_reward -= val.v1;
    guard.age--;
    thief.total_reward -= val.v2;
    thief.age--;
}
void regen_path(EdgeList & travel_path, EdgeList & path_markers,size_t start_loc,const Graph & move_graph){
    size_t cur_point = start_loc;
    for(uint32_t dest_marker : path_markers){
        bfs(move_graph,cur_point,dest_marker,travel_path,1000000);
        cur_point = travel_path.back();
    }
}
uint32_t compute_length(const EdgeList & path,const MoveDistMap & dist_map){
    uint32_t dist = 0;
    for(size_t i = 0; i < path.size()-1; i++){
        dist += dist_map[path[i]][path[i+1]];
    }
    return dist;
}
bool mutate_path(Path & path,randgen & gen,const Graph & move_graph,const PointList & points,const MoveDistMap & dist_map,size_t start_loc){
    size_t path_length = path.travel_points.size();
    size_t graph_size = move_graph.size();
    if(rand(gen)%3 == 0){
        //erase from end of path
        size_t elim_end_path = std::max(int(0),rand(gen)%((int)path.dest_markers.size())-1);
        path.dest_markers.erase(path.dest_markers.begin()+elim_end_path,path.dest_markers.end());
        path.travel_points.clear();
        regen_path(path.travel_points,path.dest_markers,start_loc,move_graph);
        size_t start_gen = path.travel_points.size() ? path.travel_points.back() : start_loc;
        generate_path(path,gen,path_length,start_gen,move_graph);
        assert(path.dest_markers.size() > 1);
        assert(path.travel_points.size() == path_length);
        assert(validate_path(path,points,start_loc));
        return true;
    }
    else{
        size_t elim_end_path = rand(gen)%(path.dest_markers.size()/4+1);
        path.dest_markers.erase(path.dest_markers.end()-elim_end_path,path.dest_markers.end());
        //erase from middle of path
        size_t start_erase = std::max(size_t(1),rand(gen)%(path.dest_markers.size()));
        size_t end_erase = start_erase+rand(gen)%(path.dest_markers.size()-start_erase);

        size_t replace_size = 1+rand(gen)%(5+(end_erase - start_erase));

        EdgeList start_path(path.dest_markers.begin(),path.dest_markers.begin()+start_erase);
        EdgeList end_path(path.dest_markers.begin()+end_erase,path.dest_markers.end());

        size_t len_start = compute_length(start_path,dist_map) + dist_map[start_loc][start_path.front()];
        size_t len_end = compute_length(end_path,dist_map);

        //regenerate start and end of path
        for(int tries = 0; tries < 10; tries++){
            EdgeList middle_path(replace_size);
            size_t dist_middle = 0;
            for(size_t i = 0; i < replace_size; i++){
                middle_path[i] = rand(gen)%graph_size;
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
                generate_path(path,gen,path_length,path.travel_points.back(),move_graph);
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
                assert(validate_path(path,points,start_loc));
                assert(path.dest_markers.size() > 1);
                assert(path.travel_points.size() == path_length);
                return true;
            }
        }
        return false;
    }
}
int count_rewards(const PathRewards & paths){
    int sum = 0;
    for(PathReward p : paths){
        sum += p.total_reward;
    }
    return sum;
}
int count_age(const PathRewards & paths){
    int sum = 0;
    for(PathReward p : paths){
        sum += p.age;
    }
    return sum;
}
bool validate_path(const Path & path,const PointList& points,size_t start_loc){
    if(path.travel_points.size() == 0){
        return true;
    }
    Point prevp = points[start_loc];
    for(uint32_t n : path.travel_points){
        Point newp = points[n];
        if(dist(prevp,newp) > 5){
            return false;
        }
        prevp = newp;
    }
    return true;
}
bool validate_paths(const PathCollection & paths,const PointList& points,size_t start_loc){
    for(const Path & path : paths){
        if(!validate_path(path,points,start_loc)){
            return false;
        }
    }
    return true;
}
void add_paths(PathCollection & paths,randgen & gen,const Graph & move_graph,const PointList & points,
               const MoveDistMap & dist_map,int num_to_add,int start_loc){
    size_t path_len = paths.size();
    for(int i = 0; i < num_to_add; i++){
        Path new_path = paths[rand(gen)%path_len];
        while(!mutate_path(new_path,gen,move_graph,points,dist_map,start_loc));
        paths.push_back(new_path);
    }
}
template<class Ty>
void keep_indexes(std::vector<Ty> & paths,PathSubset & indexes){
    std::vector<Ty> new_paths;
    new_paths.reserve(indexes.size());
    assert(indexes.size() == paths.size());
    for(size_t i = 0; i < indexes.size(); i++){
        if(indexes[i]){
            new_paths.push_back(std::move(paths[i]));
        }
    }
    paths.swap(new_paths);
}
PathSubset random_subset(randgen & gen,size_t size,size_t count){
    PathSubset res(size,false);
    for(size_t i = 0; i < count; i++){
        int idx;
        do{
            idx = rand_int(gen,size);
        }while(res[idx]);
        res[idx] = true;
    }
    return res;
}
PathSubset find_best(const PathRewards & rewards,size_t res_size){
    struct SortVal{
        uint32_t idx;
        int val;
        bool operator < (SortVal sv)const{
            return val > sv.val;
        }
    };
    res_size = std::min(res_size,rewards.size());
    int tot_rew = count_rewards(rewards);
    //int reward_noise = tot_rew / 10;
    std::vector<SortVal> sortl(rewards.size());
    for(uint32_t i = 0; i < rewards.size(); i++){
        sortl[i] = SortVal{i,rewards[i].total_reward};//+rand()%(1+reward_noise)};
    }
    std::sort(sortl.begin(),sortl.end());
    PathSubset res(rewards.size(),false);
    for(size_t i = 0; i < res_size; i++){
        res[sortl[i].idx] = true;
    }
    return res;
}
void save_paths(PathCollection & paths,std::vector<Point> & plist,std::string filename){
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
struct ConstantGameInfo{
    Graph move_graph;
    DenseGraph dense_vis_graph;
    DenseGraph dense_rew_graph;
    MoveDistMap dist_maps;
    PointList points;
    EdgeList reward_points;
    size_t guard_start;
    size_t thief_start;
};
using response_fn_ty = void (const PathCollection &,
                         const PathRewards &,
                     const PathCollection &,
                     const ConstantGameInfo &,
                    randgen &,
                     size_t,
                     size_t,
                     size_t,
                     PathCollection &,
                     PathRewards &);
void best_guard_response(const PathCollection & old_guard_paths,
                         const PathRewards & old_guard_rewards,
                     const PathCollection & theif_paths,
                     const ConstantGameInfo & gi,
                     randgen & gen,
                     size_t NUM_PATHS,
                     size_t NUM_ITERS,
                     size_t RET_SIZE,
                     PathCollection & new_guard_paths,
                     PathRewards & new_guard_rewards){

     //initialize guard paths with current best mixture for faster convergence
    PathCollection guard_paths = old_guard_paths;
    PathRewards guard_rewards = old_guard_rewards;
    if(guard_paths.size() > NUM_PATHS){
        PathSubset best_old_guards = random_subset(gen,old_guard_paths.size(),NUM_PATHS);//find_best(guard_rewards,NUM_PATHS);
        keep_indexes(guard_paths,best_old_guards);
        keep_indexes(guard_rewards,best_old_guards);
    }

    PathReward dummy_thief_rew;
    std::vector<PointRewards> theif_point_rewards;
    add_theif_rewards(theif_point_rewards,theif_paths,gi.reward_points,gi.dense_rew_graph);

    if(guard_paths.size() == 0){
        size_t PATH_LENGTH = theif_paths[0].travel_points.size();
        guard_paths.resize(NUM_PATHS);
        generate_paths(guard_paths,gen,PATH_LENGTH,gi.move_graph,gi.guard_start);
        guard_rewards.resize(NUM_PATHS);

        for(size_t g = old_guard_paths.size(); g < NUM_PATHS; g++){
            for(size_t t = 0; t < theif_paths.size(); t++){
                ValPair eval = evaluate_rewards(guard_paths[g],theif_paths[t],theif_point_rewards[t],gi.dense_vis_graph);
                add(guard_rewards[g],dummy_thief_rew,eval);
            }
        }
    }
    if(guard_paths.size() < NUM_PATHS){
        add_paths(guard_paths,gen,gi.move_graph,gi.points,gi.dist_maps,NUM_PATHS - guard_paths.size(),gi.guard_start);
        guard_rewards.resize(NUM_PATHS);

        for(size_t g = old_guard_paths.size(); g < NUM_PATHS; g++){
            for(size_t t = 0; t < theif_paths.size(); t++){
                ValPair eval = evaluate_rewards(guard_paths[g],theif_paths[t],theif_point_rewards[t],gi.dense_vis_graph);
                add(guard_rewards[g],dummy_thief_rew,eval);
            }
        }
    }
     const size_t ADD_PATHS = 5;
     for(size_t i = 0; i < NUM_ITERS; i++){
         //evaluate theif rewards
         add_paths(guard_paths,gen,gi.move_graph,gi.points,gi.dist_maps,ADD_PATHS,gi.guard_start);
         guard_rewards.resize(NUM_PATHS+ADD_PATHS);
         for(size_t g = NUM_PATHS; g < NUM_PATHS+ADD_PATHS; g++){
             for(size_t t = 0; t < theif_paths.size(); t++){
                 ValPair eval = evaluate_rewards(guard_paths[g],theif_paths[t],theif_point_rewards[t],gi.dense_vis_graph);
                 add(guard_rewards[g],dummy_thief_rew,eval);
             }
         }
         //assert(validate_paths(guard_paths,gi.points,gi.guard_start));
         PathSubset best_guards = find_best(guard_rewards,NUM_PATHS);

         keep_indexes(guard_paths,best_guards);
         keep_indexes(guard_rewards,best_guards);
         for(size_t g = 0; g < guard_paths.size(); g++){
             assert(guard_rewards[g].age == int(theif_paths.size()));
         }
     }
     new_guard_paths = guard_paths;
     new_guard_rewards = guard_rewards;
     PathSubset best_new_guards = find_best(new_guard_rewards,RET_SIZE);
     keep_indexes(new_guard_paths,best_new_guards);
     keep_indexes(new_guard_rewards,best_new_guards);
}
void best_thief_response(const PathCollection & old_thief_paths,
                         const PathRewards & old_thief_rewards,
                     const PathCollection & guard_paths,
                     const ConstantGameInfo & gi,
                     randgen & gen,
                     size_t NUM_PATHS,
                     size_t NUM_ITERS,
                     size_t RET_SIZE,
                     PathCollection & new_thief_paths,
                     PathRewards & new_thief_rewards){

    //initialize guard paths with current best mixture for faster convergence
    PathCollection thief_paths = old_thief_paths;
    PathRewards thief_rewards = old_thief_rewards;
    if(thief_paths.size() > NUM_PATHS){
        PathSubset best_old_thiefs = random_subset(gen,old_thief_rewards.size(),NUM_PATHS);//find_best(guard_rewards,NUM_PATHS);
        //PathSubset best_old_thiefs = find_best(old_thief_rewards,NUM_PATHS);
        keep_indexes(thief_paths,best_old_thiefs);
        keep_indexes(thief_rewards,best_old_thiefs);
    }

    PathReward dummy_guard_rew;
    std::vector<PointRewards> theif_point_rewards;
    if(thief_paths.size() == 0){
        size_t PATH_LENGTH = guard_paths[0].travel_points.size();
        thief_paths.resize(NUM_PATHS);
        generate_paths(thief_paths,gen,PATH_LENGTH,gi.move_graph,gi.thief_start);
        add_theif_rewards(theif_point_rewards,thief_paths,gi.reward_points,gi.dense_rew_graph);
        thief_rewards.resize(NUM_PATHS);

        for(size_t t = old_thief_paths.size(); t < NUM_PATHS; t++){
            for(size_t g = 0; g < guard_paths.size(); g++){
                ValPair eval = evaluate_rewards(guard_paths[g],thief_paths[t],theif_point_rewards[t],gi.dense_vis_graph);
                add(dummy_guard_rew,thief_rewards[t],eval);
            }
        }
    }
    if(thief_paths.size() < NUM_PATHS){
        add_paths(thief_paths,gen,gi.move_graph,gi.points,gi.dist_maps,NUM_PATHS - thief_paths.size(),gi.thief_start);
        add_theif_rewards(theif_point_rewards,thief_paths,gi.reward_points,gi.dense_rew_graph);
        thief_rewards.resize(NUM_PATHS);

        for(size_t t = old_thief_paths.size(); t < NUM_PATHS; t++){
            for(size_t g = 0; g < guard_paths.size(); g++){
                ValPair eval = evaluate_rewards(guard_paths[g],thief_paths[t],theif_point_rewards[t],gi.dense_vis_graph);
                add(dummy_guard_rew,thief_rewards[t],eval);
            }
        }
    }
     const size_t ADD_PATHS = 5;
     for(size_t i = 0; i < NUM_ITERS; i++){
         //evaluate theif rewards
         add_paths(thief_paths,gen,gi.move_graph,gi.points,gi.dist_maps,ADD_PATHS,gi.thief_start);
         add_theif_rewards(theif_point_rewards,thief_paths,gi.reward_points,gi.dense_rew_graph);
         thief_rewards.resize(NUM_PATHS+ADD_PATHS);
         for(size_t t = NUM_PATHS; t < NUM_PATHS+ADD_PATHS; t++){
             for(size_t g = 0; g < guard_paths.size(); g++){
                 ValPair eval = evaluate_rewards(guard_paths[g],thief_paths[t],theif_point_rewards[t],gi.dense_vis_graph);
                 add(dummy_guard_rew,thief_rewards[t],eval);
             }
         }
         //assert(validate_paths(thief_paths,gi.points,gi.thief_start));
         PathSubset best_thiefs = find_best(thief_rewards,NUM_PATHS);

         keep_indexes(thief_paths,best_thiefs);
         keep_indexes(thief_rewards,best_thiefs);
         keep_indexes(theif_point_rewards,best_thiefs);
         for(size_t t = 0; t < thief_paths.size(); t++){
             assert(thief_rewards[t].age == guard_paths.size());
         }
     }
     new_thief_paths = thief_paths;
     new_thief_rewards = thief_rewards;
     PathSubset best_new_thiefs = find_best(new_thief_rewards,RET_SIZE);
     keep_indexes(new_thief_paths,best_new_thiefs);
     keep_indexes(new_thief_rewards,best_new_thiefs);
}
void parallel_response_fn(PathCollection & old_paths,
                          PathRewards & old_rewards,
                      PathCollection & guard_paths,
                      ConstantGameInfo & gi,
                      size_t NUM_PATHS,
                      size_t NUM_ITERS,
                      size_t RET_SIZE,
                      size_t num_par_execs,
                      PathCollection & new_paths,
                      PathRewards & new_rewards,
                      response_fn_ty fn){
    std::random_device true_rand;
    std::vector<randgen> gen;
    for(size_t i = 0; i < numHWThreads; i++){
        gen.emplace_back(true_rand());
    }
    new_paths.clear();
    new_rewards.clear();
    PathCollection & all_paths = new_paths;
    PathRewards & all_rewards = new_rewards;
    #pragma omp parallel for num_threads(numHWThreads)
    for(size_t i = 0; i < num_par_execs; i++){
        int tid = omp_get_thread_num();
        randgen & thisgen = gen.at(tid);
        PathCollection response;
        PathRewards rewards;
        fn(old_paths,old_rewards,guard_paths,gi,thisgen,NUM_PATHS,NUM_ITERS,RET_SIZE,response,rewards);
#pragma omp critical
        {
        all_paths.insert(all_paths.end(),response.begin(),response.end());
        all_rewards.insert(all_rewards.end(),rewards.begin(),rewards.end());
        }
    }

    PathSubset best_vals = find_best(all_rewards,RET_SIZE);
    keep_indexes(all_paths,best_vals);
    keep_indexes(all_rewards,best_vals);
}

double total_reward(PathCollection & guard_paths,
                 PathRewards & rewards,
                 PathCollection & thief_paths,
                 ConstantGameInfo & gi,
                 response_fn_ty fn){

    const size_t NUM_RESPONSE_ITERS = 2000;
    const size_t NUM_RESPONSE_PATHS = 400;
    const size_t NUM_RETS = 10;
    const size_t NUM_PAR_RESP = std::max(size_t(12),numHWThreads);
    PathCollection responses;
    PathRewards response_rews;
    parallel_response_fn(
                guard_paths,
                rewards,
                thief_paths,
                gi,
                NUM_RESPONSE_PATHS,
                NUM_RESPONSE_ITERS,
                NUM_RETS,
                NUM_PAR_RESP,
                responses,
                response_rews,
                fn
                );
    int tot_rew = 0;
    for(PathReward & rew : response_rews){
        tot_rew += rew.total_reward;
    }
    return tot_rew/double(NUM_RETS*response_rews[0].age);
}

void compete_paths(ConstantGameInfo & gi,
                    std::string name){

    std::random_device true_rand;
    randgen gen(true_rand());

    const size_t PATH_LENGTH = 400;
    //const size_t NUM_PATHS = 450;
    const size_t NUM_START_PATHS = 1;
    const size_t NUM_ITERS = 10000000;
    const size_t ADD_PATHS = 1;
    PathCollection guard_paths(NUM_START_PATHS,Path{});
    PathCollection theif_paths(NUM_START_PATHS,Path{});
    PathRewards guard_rewards(NUM_START_PATHS,PathReward{});
    PathRewards thief_rewards(NUM_START_PATHS,PathReward{});
    std::cout << "initted1" << std::endl;
    std::cout << "initted2" << std::endl;
    int save_count = 0;
    std::vector<PointRewards> theif_point_rewards;

    generate_paths(guard_paths,gen,PATH_LENGTH,gi.move_graph,gi.guard_start);
    generate_paths(theif_paths,gen,PATH_LENGTH,gi.move_graph,gi.thief_start);
    add_theif_rewards(theif_point_rewards,theif_paths,gi.reward_points,gi.dense_rew_graph);
    for(size_t g = 0; g < guard_paths.size(); g++){
        for(size_t t = 0; t < theif_paths.size(); t++){
            ValPair eval = evaluate_rewards(guard_paths[g],theif_paths[t],theif_point_rewards[t],gi.dense_vis_graph);
            add(guard_rewards[g],thief_rewards[t],eval);
        }
    }


    std::string report_fname = "wm_img_dir/"+name+"_report.csv";
    std::ofstream report_file(report_fname);
    report_file << "row_idx,num_updates,guard_rew,thief_rew,guard_add_response,thief_add_response,guard_report_reward,thief_report_reward" << std::endl;

    for(size_t iter_n = 0; iter_n < NUM_ITERS; iter_n++){
        //evaluate theif rewards

        const size_t PAR_RESPONSES = numHWThreads;
        const size_t NUM_RESPONSE_PATHS = 1000/PAR_RESPONSES;
        const size_t NUM_RESPONSE_ITERS = 10*NUM_RESPONSE_PATHS;
        PathCollection thief_responses;
        PathRewards thief_response_rews;
        PathCollection theif_start;
        PathRewards theif_rew_start;
        if(iter_n > 200){
            theif_start = theif_paths;
            theif_rew_start = thief_rewards;
        }
        parallel_response_fn(
                    theif_start,
                    theif_rew_start,
                    guard_paths,
                    gi,
                    NUM_RESPONSE_PATHS,
                    NUM_RESPONSE_ITERS,
                    ADD_PATHS,
                    PAR_RESPONSES,
                    thief_responses,
                    thief_response_rews,
                    best_thief_response
                    );
        theif_paths.insert(theif_paths.end(),thief_responses.begin(),thief_responses.end());

        thief_rewards.resize(theif_paths.size());
        add_theif_rewards(theif_point_rewards,theif_paths,gi.reward_points,gi.dense_rew_graph);

        for(size_t t = theif_paths.size()-ADD_PATHS; t < theif_paths.size(); t++){
            for(size_t g = 0; g < guard_paths.size(); g++){
                ValPair eval = evaluate_rewards(guard_paths[g],theif_paths[t],theif_point_rewards[t],gi.dense_vis_graph);
                add(guard_rewards[g],thief_rewards[t],eval);
            }
        }
        assert(validate_paths(theif_paths,gi.points,gi.thief_start));



        PathCollection guard_responses;
        PathRewards guard_response_rews;
        PathCollection guard_start;
        PathRewards guard_rew_start;
        if(iter_n > 200){
            guard_start = guard_paths;
            guard_rew_start = guard_rewards;
        }
        parallel_response_fn(
                    guard_start,
                    guard_rew_start,
                    theif_paths,
                    gi,
                    NUM_RESPONSE_PATHS,
                    NUM_RESPONSE_ITERS,
                    ADD_PATHS,
                    PAR_RESPONSES,
                    guard_responses,
                    guard_response_rews,
                    best_guard_response
                    );
        guard_paths.insert(guard_paths.end(),guard_responses.begin(),guard_responses.end());
        guard_rewards.resize(guard_paths.size());

        for(size_t g = guard_paths.size()-ADD_PATHS; g < guard_paths.size(); g++){
            for(size_t t = 0; t < theif_paths.size(); t++){
                ValPair eval = evaluate_rewards(guard_paths[g],theif_paths[t],theif_point_rewards[t],gi.dense_vis_graph);
                add(guard_rewards[g],thief_rewards[t],eval);
            }
        }
        assert(validate_paths(guard_paths,gi.points,gi.guard_start));


        for(size_t t = 0; t < theif_paths.size(); t++){
            assert(thief_rewards[t].age == int(guard_paths.size()));
        }
        for(size_t g = 0; g < guard_paths.size(); g++){
            assert(guard_rewards[g].age == int(theif_paths.size()));
        }

        double guard_cur_reward = count_rewards(guard_rewards)/double(count_age(guard_rewards));
        double thief_cur_reward = count_rewards(thief_rewards)/double(count_age(thief_rewards));
        double guard_response_rew = count_rewards(guard_response_rews)/double(count_age(guard_response_rews));
        double thief_response_rew = count_rewards(thief_response_rews)/double(count_age(thief_response_rews));
        std::cout << guard_cur_reward << "  "
                    << thief_cur_reward << "     "
                    << guard_response_rew << "  "
                     << thief_response_rew <<  std::endl;

        if(iter_n == size_t(pow(double(save_count+1),1.5)*4)){
            std::cout << "saved started" << std::endl;
            std::string agent_fpath = "wm_img_dir/"+name+"_agent.weightmap.json."+std::to_string(save_count);
            std::string guard_fpath = "wm_img_dir/"+name+"_guard.weightmap.json."+std::to_string(save_count);
            save_paths(guard_paths,gi.points,guard_fpath);
            save_paths(theif_paths,gi.points,agent_fpath);
            double guard_rew = total_reward(guard_paths,guard_rewards,theif_paths,gi,best_guard_response);
            double thief_rew = total_reward(theif_paths,thief_rewards,guard_paths,gi,best_thief_response);
            std::cout << "estimated guard reward: "<< guard_rew << std::endl;
            std::cout << "estimated thief reward: "<< thief_rew << std::endl;
            size_t num_updates = iter_n * ADD_PATHS;
            report_file << save_count << ',' << num_updates << ','
                        << guard_cur_reward << "," << thief_cur_reward << ','
                        << guard_response_rew << "," << thief_response_rew << ','
                     << guard_rew << "," << thief_rew << std::endl;
             std::cout << "saved ended" << std::endl;

            save_count++;
        }
        //std::cout << double(cur_time)/ tot_time << "\n";

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

    ConstantGameInfo game_info{
        .move_graph = pathing_graph,
        .dense_vis_graph = compute_vispairs(pruned_vis_graph),
        .dense_rew_graph = compute_vispairs(pruned_rew_graph),
        .dist_maps = move_dist_calculator(pathing_graph),
        .points=points,
        .reward_points=reward_vals,
        .guard_start=guard_idx,
        .thief_start=agent_idx,
    };
    compete_paths(
        game_info,
        env_fname
    );
    //std::cout << points.size() << "\n";
    //std::cout << points[0].x << "\n";
}
