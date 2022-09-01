import gurobipy as gp
import math

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


print(N_s == N_t)
# default data
drone_num = 2
drone_speeds = [2.5, 2.5]
truck_speed = 1
truck_service_time = 10
drone_flight_duration = 100
drone_service_time = 5


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
    return math.sqrt((arc1[0] - arc2[0])**2 + (arc1[1] - arc2[1])**2)

# generate truck wait time
T_v = {(i,j): truck_service_time + dist_betw_arcs(N_st[i], N_st[j])/truck_speed for (i,j) in A}


def MIP(N, N_s, N_t, N_st, A, dist):
    # s is first node (source)
    # t is last node (sink)
    L = range(len(drone_speeds))

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
    departFromDepot = m.addConstr(gp.quicksum(X[0,] for j in N) == 1) # truck must depart from depot

    returnToDepot = m.addConstr(gp.quicksum(X[i,len(N_s)] for i in N) == 1) # truck must return to depot

    flowConservation = {i:
        m.addConstr(gp.quicksum(X[i+1,j+1] for j in N_t if j != i) - gp.quicksum(X[j+1,i+1] for j in N_t if j != i) == 0)
        for i in N} 
    
    # subTourElimination = {(i,j):
    # }

    m.optimize()

if __name__ == "__main__":
    #MIP(node_distance, drones)
    # print(N, N_s, N_t,N_st)
    distances = generate_distances(A)
    print(A[:100])
    # print(distances[:20])
    MIP(N, N_s, N_t, N_st, A, distances)
