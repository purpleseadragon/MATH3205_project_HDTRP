import gurobipy as gp
import math
import matplotlib.pyplot as plt

# data
# P-n16-k8 augerat problem
N= {
"n0 ": (37, 52),
"n1 ": (49, 49),
"n2 ": (52, 64),
"n3 ": (31, 62),
"n4 ": (52, 33),
"n5 ": (42, 41),
"n6 ": (52, 41),
"n7 ": (57, 58),
"n8": (62, 42),
"n9": (42, 57),
"n10": (27, 68),
"n11": (43, 67),
"n12": (58, 48),
"n13": (58, 27),
"n14": (37, 69)
}

s = t = (30, 40)

N_s = N.copy()
N_t = N.copy()
N_st = N.copy()
N_t['t'] = t
N_s['s'] = s
N_st['s'] = s
N_st['t'] = t


# default data
drone_num = 2
drone_speeds = [1, 1.5]
drone_battery = B_l = [100, 100] # max flight time
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

# generate drone delivery times (battery usage), includes service time 
b_l = {(i,j,l): (generate_distances(A)[A.index((i,j))]*2) + drone_service_time for i in N_s for j in N for l in L if i != j}

# total required ddelivery time
tau_l = {(i,j,l): (generate_distances(A)[A.index((i,j))]*2)/drone_speeds[l] + drone_service_time for i in N_s for j in N for l in L if i != j}


# s is first node (source depot)
# t is last node (sink depot)

# !!!! model !!!!
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

# cant flow in and out of depot from same node !!! added because my subtour elim wasnt working !!!
noFlowAlongSameArc = {(i,j): 
    m.addConstr(X[i,j] + X[j,i] <= 1) 
    for (i,j) in A if i != "s" and j != "t"}

depotNoSameFlow = {i:
    m.addConstr(X["s",i] + X[i,"t"] <= 1)
    for i in N if i != "s" and i != "t"}

flowConservation = {i:
    m.addConstr(gp.quicksum(X[i,j] for j in N_t if j != i)  == gp.quicksum(X[j,i] for j in N_s if j != i))
    for i in N}

# !!! think this might be wrong !!!
subTourElimination = {(i,j): 
    m.addConstr(V[i] - V[j] <= M*(1-X[i,j]))
    for (i,j) in A}

visitEachNode = {j: 
    m.addConstr(gp.quicksum(X[i,j] for i in N_s if i != j) + gp.quicksum(H[i,j,l] for i in N_s for l in L if i != j) == 1)
    for j in N} 

droneDispatch = {i:
    m.addConstr(M*gp.quicksum(X[i,j] for j in N_t if j != i) >= gp.quicksum(H[i,j,l] for j in N for l in L if j != i))
    for i in N_s}

batteryConsumption = {l:
    m.addConstr(gp.quicksum(H[i,j,l]*b_l[i,j,l] for i in N_s for j in N if i !=j) <= B_l[l])
    for l in L}

waitTime = {(i,l):
    m.addConstr(W[i] >= gp.quicksum(H[i,j,l]*tau_l[i,j,l] for j in N if j != i))
    for i in N_s for l in L}

m.addConstr(V["s"]==0)

m.optimize()
X_vals = {(i,j): X[i,j].x for (i,j) in A}
H_vals = {(i,j,l): H[i,j,l].x for (i,j) in A for l in L}
V_vals = {i: V[i].x for i in N_st}


def plot_path(X,H):
    """Takes X and H variables and plots the truck and drone routes respectively"""
    # get path
    path = []
    for (i,j) in A:
        if X[i,j] > 0.9:
            path.append((i,j))
    # get drone path
    drone_path = []
    for (i,j) in A:
        for l in L:
            if H[i,j,l] > 0.9:
                drone_path.append((i,j))
    # plot path
    for (i,j) in path:
        plt.plot([N_st[i][0], N_st[j][0]], [N_st[i][1], N_st[j][1]], 'k')
    for (i,j) in drone_path:
        plt.plot([N_st[i][0], N_st[j][0]], [N_st[i][1], N_st[j][1]], 'r')
    for k in N_s:
        plt.plot(N_s[k][0], N_s[k][1], 'g',marker="o")
        plt.text(N_s[k][0], N_s[k][1], k, fontsize=12, horizontalalignment="right")
    plt.show()


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
    print(V_vals)
    print([sum([H_vals[i,j,l]*b_l[i,j,l] for i in N_s for j in N if i != j]) for l in L]) # print out battery usage
    print([(i,j) for (i,j) in A if X_vals[i,j] > 0.9])
    print([(i,j,l) for (i,j) in A for l in L if H_vals[i,j,l] > 0.9])
    plot_path(X_vals, H_vals)