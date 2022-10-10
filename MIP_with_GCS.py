import gurobipy as gp

from helper import peter_plot_path, generate_distances, adjacencyMatrix, generateComponent

from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components

import matplotlib.pyplot as plt

import sys

sys.path.insert(1, r'C:\Users\o_dav\Dropbox\2022Sem2\math3205\project\math3205project\test')

from load_test import load_test
 
test = load_test('A')['A-n37-k5']
#test = load_test('P')['P-n16-k8']
# test = load_test('P')['P-n19-k2']

# default data
drone_num = 2
drone_speeds = 2.5#[1, 1.5]
a = [1, 1.5]
drone_battery = B_l = [100, 130] # max flight time
truck_speed = 1
truck_service_time = 10
#drone_flight_duration = 100
drone_service_time = 5
L = range(len(a))
EPS = 0.8
b = [1,1]

# alternate drone data (from powerpoint)
drone_battery = B_l = [100, 50]
a = [0.4, 0.2]
b = [0.4, 0.2]

def MIP(test):
    """Takes in a dictionary of nodes and a source node and returns the optimal solution"""  
    # s is first node (source depot)
    # t is last node (sink depot)
    N = test['N']
    N_s = test['Ns']
    N_t = test['Nt']
    N_st = test['Nst']

    # generate arcs
    A = [(i,j) for i in N_s for j in N_t if N_s[i] != N_t[j]]
    M = len(A)+1

    # generate arc distances
    D = generate_distances(A, N_st)

    # generate truck wait time!!!!!!!!!!!!!!!!!!!!!
    T_v = {(i,j): truck_service_time + D[i,j]/truck_speed for (i,j) in A}

    # generate drone battery usage, includes service time
    b_l = {(i,j,l): 2*D[i,j]/drone_speeds + drone_service_time*b[l] for i in N_s for j in N for l in L if i != j}

    # total required delivery time
    tau_l = {(i,j,l): a[l]*D[i,j]*2/drone_speeds + drone_service_time*b[l] for i in N_s for j in N for l in L if i != j}

    # !!!! model !!!!
    m = gp.Model("MIP_implementation")

    # variables
    X = {(i,j): m.addVar(vtype=gp.GRB.BINARY) for (i,j) in A} # if truck travels from i to j
    H = {(i,j,l): m.addVar(vtype=gp.GRB.BINARY) for i in N_s for j in N for l in L} # if drone l travels from i to j
    W = {i: m.addVar(vtype=gp.GRB.CONTINUOUS) for i in N_s} # Wait time at node i

    # set objective (subtracking service time for back to depot)
    m.setObjective(gp.quicksum(T_v[i,j]*X[i,j] for (i,j) in A) + gp.quicksum(W[i] for i in N_s)- truck_service_time, gp.GRB.MINIMIZE)

    # constraints
    departFromDepot = m.addConstr(gp.quicksum(X["s",j] for j in N) == 1) # truck must depart from depot

    returnToDepot = m.addConstr(gp.quicksum(X[i,"t"] for i in N) == 1) # truck must return to depot


    flowConservation = {i:
        m.addConstr(gp.quicksum(X[i,j] for j in N_t if j != i) == 
                    gp.quicksum(X[j,i] for j in N_s if j != i))
        for i in N}

    visitEachNode = {j: 
        m.addConstr(gp.quicksum(X[i,j] for i in N_s if i != j) + 
                    gp.quicksum(H[i,j,l] for i in N_s for l in L if i != j) == 1)
        for j in N} 

    droneDispatch = {i:
        m.addConstr(M*gp.quicksum(X[i,j] for j in N_t if N_t[j] != N_s[i]) >= 
                    gp.quicksum(H[i,j,l] for j in N for l in L if j != i))
        for i in N_s}

    batteryConsumption = {l:
        m.addConstr(gp.quicksum(H[i,j,l]*b_l[i,j,l] for i in N_s for j in N if i !=j) <= B_l[l])
        for l in L}

    waitTime = {(i,l):
        m.addConstr(W[i] >= gp.quicksum(H[i,j,l]*tau_l[i,j,l] for j in N if j != i))
        for i in N_s for l in L}

    # additionalPConstr = m.addConstr(gp.quicksum(X[i,j] for i in N for j in N_t if j != i)==25)
    def Callback(model,where):
    # GCS Separation (Algorithm 1 from the paper)
        if where==gp.GRB.Callback.MIPSOL:
            XV = model.cbGetSolution(X)
            HV = model.cbGetSolution(H)
            AA = [(i, j) for (i, j) in A if XV[i,j]>0]
            #print(AA)
            #print(list(N_st))
            GG = csr_matrix(adjacencyMatrix(list(N_st), AA))
            #plt.imshow(GG.toarray())
            # Find all SCC on G
            _, label = connected_components(GG,directed=True,connection='strong')
            #print(_)
            #print(label)
            S = generateComponent(label, N_st)
            #print(S)
            #peter_plot_path(XV, HV, N, N_s, N_st, A)
            # C = []
            for s in S:
                for k in s:
                    deltaS = [(i, j) for (i, j) in A if i in s and j not in s]
                    deltaK = [(i, j) for (i, j) in A if i==k and j!=k]
                    v = sum(XV[i,j] for (i,j) in deltaK)- \
                        sum(XV[i,j] for (i,j) in deltaS)
                    if v > EPS:
                        model.cbLazy(gp.quicksum(X[i,j] for (i,j) in deltaS)>=
                                    gp.quicksum(X[i,j] for (i,j) in deltaK))

    m.setParam('LazyConstraints', 1)
    m.optimize(Callback)

    X_vals = {(i,j): X[i,j].x for (i,j) in A}
    H_vals = {(i,j,l): H[i,j,l].x for i in N_s for j in N for l in L}

    if False:
        print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        print(f"battery usage: {[sum([H_vals[i,j,l]*b_l[i,j,l] for i in N_s for j in N if i != j]) for l in L]}")
        print(f"truck path: {[(i,j) for (i,j) in A if X_vals[i,j] > 0.9]}")
        print(f"drone decisions: {[(i,j,l) for i in N_s for j in N for l in L if H_vals[i,j,l] > 0.9]}")
    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")

    return X_vals, H_vals, N, N_s, N_st, A


if __name__ == "__main__":
    X_vals, H_vals, N, N_s, N_st, A = MIP(test) # Best objective 2.015393143872e+02, 174 (modding eqn) vs 177.2
    peter_plot_path(X_vals, H_vals, N, N_s, N_st, A)

    # X_vals, H_vals, N, N_s, N_st, A = MIP(a_n37_k5,a_n37_k5_s) # Best objective 
    # peter_plot_path(X_vals, H_vals, N, N_s, N_st, A)
