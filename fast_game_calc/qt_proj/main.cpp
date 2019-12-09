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

using WeightAlloc = std::vector<double>;
using WeightAllocs = std::vector<WeightAlloc>;

using WeightMap = std::vector<std::array<double,5>>;
using WeightMaps = std::vector<WeightMap>;
using WeightPop = std::vector<WeightMaps>;

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
        for(size_t e = 0; e < full_graph[n].size(); e++){
            Point dest = points[full_graph[n][e]];
            if(dist(source,dest) < target_dist+0.01){
                small_graph[n].push_back(e);
            }
        }
    }
    return small_graph;
}
void update_weights(WeightAlloc & weight,WeightMap & move_map,Graph & move_graph){
   size_t graph_size = weight.size();
    //theif steps forward
    WeightAlloc next_weights(graph_size,0.0);
    for(size_t n = 0; n < graph_size; n++){
        double add_prob = weight[n];
        for(size_t j = 0; j < move_graph[n].size(); j++){
            size_t e = move_graph[n][j];
            double moveprob = move_map[n][j];
            next_weights[e] += moveprob * add_prob;
        }
    }
    weight.swap(next_weights);
}
//int col_rew = 0;
//int for_pass = 0;
void forward_pass(WeightMaps & theif, size_t theif_start,
        WeightMaps & guard, size_t guard_start,
        WeightAlloc & rewards,
        Graph & vis_graph,
        Graph & move_graph,
        WeightAllocs & guard_posses,
        WeightAllocs & theif_posses,
        WeightAllocs & end_rewards,
        double & total_reward
    ){
    total_reward = 0;
    size_t graph_size = theif[0].size();
    assert(graph_size == guard[0].size() &&
         graph_size == vis_graph.size()  &&
         graph_size == move_graph.size() &&
         graph_size == rewards.size());

    size_t time_steps = theif.size();
    end_rewards.assign(time_steps,WeightAlloc(graph_size,0.0));
    assert(time_steps == guard.size() &&
            time_steps == end_rewards.size() &&
            time_steps == theif_posses.size());// &&
            //time_steps == guard_posses.size() );

    //end_probs.assign(graph_size,0.0);
    WeightAlloc theif_probs(graph_size,0.0);
    theif_probs[theif_start] = 1.0;

    WeightAlloc guard_probs(graph_size,0.0);
    guard_probs[guard_start] = 1.0;

    for(size_t t = 0; t < time_steps; t++){
        //guard elmininate seen theif positions
        for(size_t n = 0; n < graph_size; n++){
            double guard_prob = 0;
            for(uint32_t e : vis_graph[n]){
                guard_prob += guard_probs[e];
            }
            theif_probs[n] -= theif_probs[n] * guard_prob;
        }
        //theif sees reward
        for(size_t n = 0; n < graph_size; n++){
            //double theif_prob = 0;
            for(uint32_t e : vis_graph[n]){
                double this_reward = theif_probs[e] * rewards[n];
                end_rewards[t][e] += this_reward;
                total_reward += this_reward;
            }
        }

        theif_posses[t] = theif_probs;
        guard_posses[t] = guard_probs;
        update_weights(theif_probs,theif[t],move_graph);
        update_weights(guard_probs,guard[t],move_graph);
        //std::cout << "t" << t << std::endl;
    }
}
void update_based_off_rew(WeightAlloc & rewards,WeightMap & move_map,Graph & move_graph,double update_weight){
    size_t graph_size = rewards.size();
    //updates move weights based off expected future reward
    for(size_t n = 0; n < graph_size; n++){
        size_t maxi = -1;
        double max_rew = -1;
        for(size_t j = 0; j < move_graph[n].size(); j++){
            size_t e = move_graph[n][j];
            double rew = rewards[e];
            if(max_rew < rew){
                max_rew = rew;
                maxi = j;
            }
        }
        for(double & weight  : move_map[n]){
            weight *= (1.0-update_weight);
        }
        move_map[n][maxi] += update_weight;
    }
}
void backward_pass(WeightMaps & theif,
        WeightMaps & guard,
        WeightAlloc & rewards,
        Graph & vis_graph,
        Graph & move_graph,
        const WeightAllocs & guard_posses,
        const WeightAllocs & theif_posses,
        const WeightAllocs & end_rewards,
        const double update_weight,
        double & removed_rew
    ){
    //assert(total_reward > 0 && "algorithm is logically not possible when total reward is zero");
    //double inv_total_reward = 1.0/total_reward;
    removed_rew = 0;
    size_t graph_size = theif[0].size();
    assert(graph_size == guard[0].size() &&
         graph_size == vis_graph.size()  &&
         graph_size == move_graph.size() &&
         graph_size == rewards.size());

    size_t time_steps = theif.size();
    assert(time_steps == guard.size() &&
        time_steps == end_rewards.size());

    WeightAlloc future_reward_col = end_rewards[time_steps-1];
    WeightAlloc guard_rem_rew(graph_size,0.0);

    for(size_t t = time_steps-1; t > 0; t--){
        //calculate the amount of reward that is removed by a guard being in a location
        for(size_t n = 0; n < graph_size; n++){
            for(uint32_t e : vis_graph[n]){
                guard_rem_rew[n] += future_reward_col[e] * theif_posses[t][e];
            }
            //guard_rem_rew[n] *= guard_posses[t][n];
            removed_rew += guard_rem_rew[n];
        }
        WeightAlloc tmp_fut_rew = future_reward_col;
        WeightAlloc tmp_gd_rew = guard_rem_rew;
        //update future reward collected map
        update_weights(future_reward_col,theif[t-1],move_graph);
        update_weights(guard_rem_rew,guard[t-1],move_graph);
        //theif updates move weights based off expected future reward
        update_based_off_rew(tmp_fut_rew,theif[t-1],move_graph,update_weight);
        update_based_off_rew(tmp_gd_rew,guard[t-1],move_graph,update_weight);

        for(size_t n = 0; n < graph_size; n++){
            for(uint32_t e : vis_graph[n]){
                future_reward_col[n] += end_rewards[t-1][e];
            }
        }
    }
}
WeightMaps uniform(Graph & pathing_graph,size_t time){
    WeightMaps res(time,WeightMap(pathing_graph.size()));
    for(size_t t = 0; t < time; t++){
        for(size_t n = 0; n < pathing_graph.size(); n++){
            size_t edge_len = pathing_graph[n].size();
            // for(size_t e = 0; e < edge_len; e++){
            //     res[t][n][e] = 1.0/edge_len;
            // }
            double sum = 0;
            for(size_t e = 0; e < edge_len; e++){
                res[t][n][e] = rand();
                sum += res[t][n][e];
            }
            for(size_t e = 0; e < edge_len; e++){
                res[t][n][e] /= sum;
            }
        }
    }
    return res;
}
void print_output(WeightMaps & map,std::string filename){
    json res(map);
    std::ofstream file(filename);
    file << res.dump() << std::endl;
}
void step_update(
        WeightMaps & theif, size_t theif_start,
        WeightMaps & guard, size_t guard_start,
        WeightAlloc & rewards,
        Graph & vis_graph,
        Graph & move_graph,
        double update_weight,
        size_t num_steps){
    for(size_t step = 0; step < num_steps; step++){
        WeightAllocs guard_posses(theif.size(),WeightAlloc(vis_graph.size(),0.0));
        WeightAllocs theif_posses(theif.size(),WeightAlloc(vis_graph.size(),0.0));
        WeightAllocs end_posses(theif.size(),WeightAlloc(vis_graph.size(),0.0));
        double tot_rew = 0;
        double removed_rew = 0;
        forward_pass(
            theif,theif_start,
            guard,guard_start,
            rewards,
            vis_graph,
            move_graph,
            guard_posses,
            theif_posses,
            end_posses,
            tot_rew
        );
        backward_pass(
            theif,
            guard,
            rewards,
            vis_graph,
            move_graph,
            guard_posses,
            theif_posses,
            end_posses,
            update_weight,
            removed_rew
        );
        std::cout << "step: " << step << ", reward: " << tot_rew << ", removed reward: " << removed_rew << std::endl;
        if(step % 15 == 5){
            std::cout << "started save, do not interrup!" << std::endl;
            print_output(theif,"agent.weightmap.json");
            print_output(guard,"guard.weightmap.json");
            std::cout << "finished save." << std::endl;
        }
    }
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
WeightAllocs graph_to_binmap(Graph & graph){
    WeightAllocs map(graph.size(),WeightAlloc(graph.size(),0.0));

    for(size_t n = 0; n < graph.size(); n++){
        for(size_t e : graph[n]){
            map[n][e] = 1.0;
        }
    }
    return map;
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

    std::vector<double> reward_vals(points.size(),0.0);
    auto json_data2 = json::parse(read_file(map_fname));
    for(auto & jval : json_data2.at("reward_points")){
        Point rew_p = arr_to_point(jval);
        size_t rew_i = nearest_point(rew_p,points);
        reward_vals[rew_i] = 1.0;
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

    size_t time_steps = 1000;
    size_t update_steps = 5000;
    double update_weight = 0.07;
    auto theif_weights = uniform(pathing_graph,time_steps);
    auto guard_weights = uniform(pathing_graph,time_steps);
    std::cout << pathing_graph.size() << "\n";
    step_update(
        theif_weights,agent_idx,
        guard_weights,guard_idx,
        reward_vals,//TODO:
        pruned_vis_graph,
        pathing_graph,
        update_weight,
        update_steps
        );
    //std::cout << points.size() << "\n";
    //std::cout << points[0].x << "\n";
}
