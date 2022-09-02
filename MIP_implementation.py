import gurobipy as gp
import math
import matplotlib.pyplot as plt

# data
# P-n16-k8 augerat problem
N = 16

N= {
"n2 ": (37, 52),
"n3 ": (49, 49),
"n4 ": (52, 64),
"n5 ": (31, 62),
"n6 ": (52, 33),
"n7 ": (42, 41),
"n8 ": (52, 41),
"n9 ": (57, 58),
"n10": (62, 42),
"n11": (42, 57),
"n12": (27, 68),
"n13": (43, 67),
"n14": (58, 48),
"n15": (58, 27),
"n16": (37, 69)
}

s = t = [30, 40]

N_s = N.copy()
N_t = N.copy()
N_st = N.copy()
N_t['t'] = t
N_s['s'] = s
N_st['s'] = s
N_st['t'] = t


# default data
drone_num = 2
drone_speeds = [2.5, 2.5]
drone_battery = B = [100, 100]
truck_speed = 1
truck_service_time = 10
drone_flight_duration = 100
drone_service_time = 5
M = 1000000
L = range(len(drone_speeds))

# generate arcs
A = [(i,j) for i in N_s for j in N_t if i != j]

# generate arc distances
def generate_distances(all_arcs):
    """distances correspond to arc distance from i to j in arc (i,j)"""
    distances = []
    for x in all_arcs:
        distances.append(math.sqrt((N_st[x[0]][0] - N_st[x[1]][0])**2 + (N_st[x[0]][1] - N_st[x[1]][1])**2))
    return distances

def dist_betw_arcs(arc1, arc2):
    """generates distance between arcs"""
    return math.sqrt((arc1[0] - arc2[0])**2 + (arc1[1] - arc2[1])**2)

# generate truck wait time
T_v = {(i,j): truck_service_time + dist_betw_arcs(N_st[i], N_st[j])/truck_speed for (i,j) in A}

# generate battery usages for drones 
b_l = {(i,j,l): generate_distances(A)[A.index((i,j))]/drone_speeds[l] for (i,j) in A for l in L}

# generate drone delivery times, includes service time
tau_l = {(i,j,l): generate_distances(A)[A.index((i,j))]/drone_speeds[l] + drone_service_time for (i,j) in A for l in L}

def plot_path():
    pass

def MIP(N, N_s, N_t, N_st, A):
    # s is first node (source)
    # t is last node (sink)

    # create model
    m = gp.Model("MIP_implementation")

    # create variables
    X = {(i,j): m.addVar(vtype=gp.GRB.BINARY) for (i,j) in A} # if truck travels from i to j
    H = {(i,j,l): m.addVar(vtype=gp.GRB.BINARY) for (i,j) in A for l in L} # if drone l travels from i to j
    V = {i: m.addVar(vtype=gp.GRB.CONTINUOUS) for i in N_st} # visiting order of truck at node i
    W = {i: m.addVar(vtype=gp.GRB.CONTINUOUS) for i in N_s} # Wait time at node i

    # set objective
    m.setObjective(gp.quicksum(T_v[i,j]*X[i,j] for (i,j) in A) + gp.quicksum(W[i] for i in N_s), gp.GRB.MINIMIZE)

    # constraints
    departFromDepot = m.addConstr(gp.quicksum(X["s",j] for j in N) == 1) # truck must depart from depot

    returnToDepot = m.addConstr(gp.quicksum(X[i,"t"] for i in N) == 1) # truck must return to depot

    flowConservation = {i:
        m.addConstr(gp.quicksum(X[i,j] for j in N_t if j != i) - gp.quicksum(X[j,i] for j in N_s if j != i) == 0)
        for i in N} 
    
    subTourElimination = {(i,j): 
        m.addConstr(V[i] - V[j] <= M*(1-X[i,j])-1)
        for (i,j) in A}

    visitEachNode = {j: 
        m.addConstr(gp.quicksum(X[i,j] for i in N_s if i != j) + gp.quicksum(H[i,j,l] for i in N_s for l in L if i != j) == 1)
        for j in N}
    
    droneDispatch = {i:
        m.addConstr(M*gp.quicksum(X[i,j] for j in N_t if j != i) >= gp.quicksum(H[i,j,l] for j in N for l in L if j != i))
        for i in N_s}

    batteryConsumption = {l:
        m.addConstr(gp.quicksum(H[i,j,l]*b_l[i,j,l] for (i,j) in A) <= B[l])
        for l in L}

    waitTime = {(i,l):
        m.addConstr(W[i] >= gp.quicksum(H[i,j,l]*tau_l[i,j,l] for j in N if j != i))
        for i in N_s for l in L}

    m.addConstr(V["s"]==0)

    m.optimize()

if __name__ == "__main__":
    #MIP(node_distance, drones)
    # print(N, N_s, N_t,N_st)
    # print(distances[:20])
    MIP(N, N_s, N_t, N_st, A)
