import subprocess
import tempfile
import numpy as np
#import scipy.sparse

def adj_graph_to_distance_matrix(graph,node_weights):
    init_graph_weights = 1e20*np.ones([len(graph),len(graph)],dtype=np.float64)
    for n,edges in enumerate(graph):
        for e in edges:
            init_graph_weights[n,e] = node_weights[n]

    shortest_paths = scipy.sparse.csgraph.floyd_warshall(init_graph_weights)

    return shortest_paths

def remapping(keep_nodes):
    return {i:n for i,n in enumerate(keep_nodes)}

def reduce_distance_matrix(dist_mat,keep_nodes):
    np_idx = np.asarray(keep_nodes,dtype=np.int64)
    rows = dist_mat[np_idx]
    cols = rows[:,np_idx]
    return cols

def dist_mat_to_str(dist_mat,int_factor):
    intified = (dist_mat * int_factor).astype(np.int64)
    return "\n".join(" ".join(row) for row in intified)

#def add_dist_mat(dist_mat,num_to_add):


def get_translated_sets(translator,start,reward_idxs,max_reward_benefit):
    start_set = translator[start]
    reward_sets = []
    for ridx in reward_idxs:
        set = 1

def produce_instance_from_graph(graph,start,reward_idxs,max_reward_benefit):
    return '''
NAME : g
TYPE : GTSP
COMMENT : None
DIMENTION : {dim}
GTSP_SETS : {nsets}
EDGE_WEIGHT_TYPE : EXPLICIT
EDGE_WEIGHT_FORMAT : FULL_MATRIX
EDGE_WEIGHT_SECTION
{edge_weights}
GTSP_SET_SECTION
{gtsp_sets}
EOF
    '''

class GTSP:
    def __init__(self):
        self.proc = subprocess.Popen(["julia","-q"],stdin=subprocess.PIPE,stdout=subprocess.PIPE,bufsize=0)
        self.proc.stdin.write(b"import GLNS\n")
        #self.proc.stdout.read(7)

    def solve_instance(self,instance_text):
        with tempfile.NamedTemporaryFile(mode="w") as inFile:
            inFile.write(instance_text)
            with tempfile.NamedTemporaryFile(mode="r") as outFile:
                self.proc.stdin.write('GLNS.solver("{}", mode="fast",output="{}")\n'.format(inFile.name,outFile.name).encode("utf-8"))
                self.proc.stdin.write(b'println("hello world")\n')
                self.proc.stdout.readline()
                return outFile.read()


if __name__=="__main__":
    tsp_solver = GTSP()
    tsp_solver.solve_instance("bad instance")
