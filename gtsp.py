import subprocess
import tempfile
import json
import time
import numpy as np
import scipy.sparse
from static_pathing import dikstras_dists

def adj_graph_to_distance_matrix(graph,node_weights):
    init_graph_weights = 1e7*np.ones([len(graph),len(graph)],dtype=np.float64)
    for n,edges in enumerate(graph):
        for e in edges:
            init_graph_weights[n,e] = node_weights[n]
    #print("set weights",flush=True)
    #print(init_graph_weights.shape,flush=True)
    shortest_paths = init_graph_weights#scipy.sparse.csgraph.floyd_warshall(init_graph_weights)

    return shortest_paths

def remapping(keep_nodes):
    return {i:n for i,n in enumerate(keep_nodes)}

def reduce_distance_matrix(dist_mat,keep_nodes):
    np_idx = np.asarray(keep_nodes,dtype=np.int64)
    rows = dist_mat[np_idx]
    cols = rows[:,np_idx]
    return cols

def padback_dist_mat(dist_mat,pad_amt,pad_val):
    orig_len = len(dist_mat)
    new_len = len(dist_mat)+pad_amt
    cat1v = pad_val*np.ones([orig_len,pad_amt])
    cat2v = pad_val*np.ones([pad_amt,new_len])
    dist_mat = np.concatenate([dist_mat,cat1v],axis=1)
    dist_mat = np.concatenate([dist_mat,cat2v],axis=0)
    return dist_mat

def dist_mat_to_str(dist_mat,int_factor):
    intified = (dist_mat * int_factor).astype(np.int64)
    return "\n".join(" ".join(str(r) for r in row)+" " for i,row in enumerate(intified))

#def add_dist_mat(dist_mat,num_to_add):
def gtsp_sets_str(set_list):
    return "\n".join(str(i+1)+" "+" ".join(str(r+1) for r in row)+" -1 " for i,row in enumerate(set_list))

def get_translated_sets(translator,start,reward_idxs,max_reward_benefit):
    start_set = translator[start]
    reward_sets = []
    for ridx in reward_idxs:
        set = 1

def get_dist_mat(points,adj_graph,weights,goals):
    dist_mat = np.zeros([len(goals),len(goals)],dtype=np.int64)
    for i in range(len(goals)):
        dists = dikstras_dists(goals[i],weights,points,adj_graph,goals)
        for j in range(len(goals)):
            dist = dists[j]
            if i == j:
                dist = 99999999
            dist_mat[i,j] = dist
    return dist_mat


def get_path(gtsp,points,adj_graph,weights,start_idx,reward_idxs,max_reward_benefit):
    all_reward_idxs = [start_idx]+reward_idxs
    dist_mat = get_dist_mat(points,adj_graph,weights,all_reward_idxs)
    reduced = dist_mat#reduce_distance_matrix(dist_mat,reamapped)
    rew_adds = [len(all_reward_idxs)+i for i in range(len(reward_idxs))]
    reduced_offset = padback_dist_mat(reduced,len(rew_adds)+1,max_reward_benefit)
    LAST_EL = 1+len(reward_idxs)*2
    reduced_offset[0,LAST_EL] = reduced_offset[LAST_EL,0] = max_reward_benefit/4
    gtsp_sets = [[0],[LAST_EL]]+[[1+i,1+i+len(reward_idxs)] for i in range(len(reward_idxs))]
    print(gtsp_sets)
    #reamapped = remapping(reward_idxs)
    instance = produce_instance_from_graph(reduced_offset,gtsp_sets)
    soln = gtsp.solve_instance(instance)
    #print(soln)
    ordering = parse_output(soln)
    startidx = ordering.index(0)
    zerofirst = ordering[startidx:]+ordering[:startidx]
    goal_ordering = [i-1 for i in zerofirst[1:]]
    #removes added nodes
    removed_null = [i for i in goal_ordering if i < 1+len(reward_idxs)]
    print(removed_null)
    return removed_null

def parse_output(tour_text):
    for line in tour_text.strip().split("\n"):
        if "[" in line:
            startidx = line.index("[")
            endidx = line.index("]")
            pylist = json.loads(line[startidx:endidx+1])
            m1list = [i-1 for i in pylist]
            return m1list

def produce_instance_from_graph(edge_weights,gtsp_sets):
    return '''
NAME : g
TYPE : GTSP
COMMENT : None
DIMENSION : {dim}
GTSP_SETS : {nsets}
EDGE_WEIGHT_TYPE : EXPLICIT
EDGE_WEIGHT_FORMAT : FULL_MATRIX
EDGE_WEIGHT_SECTION
{edge_weights}
GTSP_SET_SECTION
{gtsp_sets}
EOF
    '''.format(
        dim=len(edge_weights),
        nsets=len(gtsp_sets),
        edge_weights=dist_mat_to_str(edge_weights,1),
        gtsp_sets=gtsp_sets_str(gtsp_sets)
    )

class GTSP:
    def __init__(self):
        self.proc = subprocess.Popen(["julia","-q"],stdin=subprocess.PIPE,stdout=subprocess.PIPE,bufsize=-1)
        self.proc.stdin.write(b"import GLNS\n")

    def solve_instance(self,instance_text):
        with tempfile.NamedTemporaryFile(mode="w") as inFile:
            inFile.write(instance_text)
            inFile.flush()
            with tempfile.NamedTemporaryFile(mode="w") as outFile:
                self.proc.stdin.write ('GLNS.solver("{}", mode="fast",output="{}")\n'.format(inFile.name,outFile.name).encode("utf-8"))
                #self.proc.stdin.write(b'println("hello world")\n')
                self.proc.stdin.flush()
                while True:
                    line = self.proc.stdout.readline()
                    #print(line,flush=True)
                    if b"Tour Ordering" in line:
                        time.sleep(0.2)
                        with open(outFile.name) as outFile:
                            return outFile.read()


if __name__=="__main__":
    tsp_solver = GTSP()
    visibilty_info = json.load(open("enviornments/house.graph.json"))
    #res_graph = adj_graph_to_distance_matrix(visibilty_info["adj_list"],visibilty_info['counts'])
    #np.save("house.npy",res_graph)
    #print(res_graph)
    rewards = [5000,421,8214,6281]
    start = 3219
    for i in range(10):
        out = get_path(tsp_solver,visibilty_info["points"],visibilty_info["adj_list"],visibilty_info['counts'],start,rewards,500000)
        print(out)
