import gurobipy as gp
import math
from helper import plot_path, peter_plot_path, generate_distances, dist_betw_arcs

# data
# P-n16-k8 augerat problem
p_n16_k8= {
"n0": (37, 52),
"n1": (49, 49),
"n2": (52, 64),
"n3": (31, 62),
"n4": (52, 33),
"n5": (42, 41),
"n6": (52, 41),
"n7": (57, 58),
"n8": (62, 42),
"n9": (42, 57),
"n10": (27, 68),
"n11": (43, 67),
"n12": (58, 48),
"n13": (58, 27),
"n14": (37, 69)
}

p_n16_k8_s = (30, 40)


# default data
drone_num = 2
drone_speeds = 2.5#[1, 1.5]
a = [1, 1.5]
drone_battery = B_l = [100, 100] # max flight time
truck_speed = 1
truck_service_time = 10
drone_flight_duration = 100
drone_service_time = 5
# M = 1000000
L = range(len(a))





# s is first node (source depot)
# t is last node (sink depot)
def MIP(N, s):
    """Takes in a dictionary of nodes and a source node and returns the optimal solution"""  
    N_s = N.copy()
    N_t = N.copy()
    N_st = N.copy()
    t = s
    N_t['t'] = t
    N_s['s'] = s
    N_st['s'] = s
    N_st['t'] = t

    # generate arcs
    A = [(i,j) for i in N_s for j in N_t if N_s[i] != N_t[j]]
    M = len(A)+1

    # generate truck wait time!!!!!!!!!!!!!!!!!!!!!
    T_v = {(i,j): truck_service_time + generate_distances(A, N_st)[A.index((i,j))]/truck_speed for (i,j) in A}

    # generate drone delivery times (battery usage), includes service time !!!!!!!!!!!!!!!!!
    b_l = {(i,j,l): a[l]*((generate_distances(A, N_st)[A.index((i,j))]*2/drone_speeds) + drone_service_time) for i in N_s for j in N for l in L if i != j}

    # total required ddelivery time
    tau_l = {(i,j,l): (generate_distances(A, N_st)[A.index((i,j))]*2)/(drone_speeds*a[l]) + drone_service_time for i in N_s for j in N for l in L if i != j}
    # !!!! model !!!!
    m = gp.Model("MIP_implementation")

    # create variables
    X = {(i,j): m.addVar(vtype=gp.GRB.BINARY) for (i,j) in A} # if truck travels from i to j
    H = {(i,j,l): m.addVar(vtype=gp.GRB.BINARY) for i in N_s for j in N for l in L} # if drone l travels from i to j
    V = {i: m.addVar(vtype=gp.GRB.CONTINUOUS) for i in N_st} # visiting order of truck at node i
    W = {i: m.addVar(vtype=gp.GRB.CONTINUOUS) for i in N_s} # Wait time at node i

    # set objective
    m.setObjective(gp.quicksum(T_v[i,j]*X[i,j] for (i,j) in A) + gp.quicksum(W[i] for i in N_s), gp.GRB.MINIMIZE)

    # constraints
    departFromDepot = m.addConstr(gp.quicksum(X["s",j] for j in N) == 1) # truck must depart from depot

    returnToDepot = m.addConstr(gp.quicksum(X[i,"t"] for i in N) == 1) # truck must return to depot


    flowConservation = {i:
        m.addConstr(gp.quicksum(X[i,j] for j in N_t if j != i) == 
                    gp.quicksum(X[j,i] for j in N_s if j != i))
        for i in N}

    # !!! think this might be wrong !!!
    # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    subTourElimination = {(i,j): 
        m.addConstr(V[i] - V[j] <= M*(1-X[i,j])-1) 
        for (i,j) in A}

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

    initConstr = m.addConstr(V['s']==0)
        
    # m.addConstr(V["s"]==0)

    m.optimize()
    X_vals = {(i,j): X[i,j].x for (i,j) in A}
    H_vals = {(i,j,l): H[i,j,l].x for i in N_s for j in N for l in L}
    V_vals = {i: V[i].x for i in N_st}
    if True:
        print(V_vals)
        print(f"battery usage: {[sum([H_vals[i,j,l]*b_l[i,j,l] for i in N_s for j in N if i != j]) for l in L]}") # print out battery usage
        print(f"truck path: {[(i,j) for (i,j) in A if X_vals[i,j] > 0.9]}")
        print(f"drone decisions{[(i,j,l) for i in N_s for j in N for l in L if H_vals[i,j,l] > 0.9]}")
        print(f"order: {[V_vals[i] for i in V_vals]}")
    return X_vals, H_vals, V_vals, N, N_s, N_st, A


if __name__ == "__main__":
    #MIP(node_distance, drones)
    # print(N, N_s, N_t,N_st)
    # print(distances[:20])
    # print([T_v[i,j] for (i,j) in A[:20]])
    # print([b_l[i,j,l] for (i,j) in A[:20] for l in L])
    # print([tau_l[i,j,l] for (i,j) in A[:20] for l in L])
    # print(V)
    # print([X["s",j] for j in N])
    # print([X[i,"t"] for i in N])
    X_vals, H_vals, V_vals, N, N_s, N_st, A = MIP(p_n16_k8,p_n16_k8_s)
    peter_plot_path(X_vals, H_vals, N, N_s, N_st, A)
    # Plot truck routs
