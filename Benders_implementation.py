import gurobipy as gp
import math
from helper import plot_path, peter_plot_path, generate_distances, dist_betw_arcs, find_sets

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

# A-n37-k5
a_n37_k5 = {
'n0 ':(59, 46),
'n1 ':(96 ,42),
'n2 ':(47 ,61),
'n3 ':(26 ,15),
'n4 ':(66 ,6 ),
'n5 ':(96 ,7 ),
'n6 ':(37 ,25),
'n7 ':(68 ,92),
'n8 ':(78 ,84),
'n9 ':(82 ,28),
'n10':(93, 90),
'n11':(74, 42),
'n12':(60, 20),
'n13':(78, 58),
'n14':(36, 48),
'n15':(45, 36),
'n16':(73, 57),
'n17':(10, 91),
'n18':(98, 51),
'n19':(92, 62),
'n20':(43, 42),
'n21':(53, 25),
'n22':(78, 65),
'n23':(72, 79),
'n24':(37, 88),
'n25':(16, 73),
'n26':(75, 96),
'n27':(11, 66),
'n28':(9 ,49 ),
'n29':(25, 72),
'n30':(8 ,68 ),
'n31':(12, 61),
'n32':(50, 2 ),
'n33':(26, 54),
'n34':(18, 89),
'n35':(22, 53)
}

a_n37_k5_s = (38, 46)

# default data
eps = 0.0001
drone_num = 2
drone_speeds = 2.5#[1, 1.5]
a = [1, 1.5]
drone_battery = B_l = [100, 100] # max flight time
truck_speed = 1
truck_service_time = 10
drone_flight_duration = 100
drone_service_time = 5
L = range(len(a))


N = p_n16_k8
s = p_n16_k8_s
if False:
    N = a_n37_k5 
    s = a_n37_k5_s 
# s is first node (source depot)
# t is last node (sink depot)
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

# generate arc distances
D = generate_distances(A, N_st)

# generate truck wait time!!!!!!!!!!!!!!!!!!!!!
T_v = {(i,j): truck_service_time + D[i,j]/truck_speed for (i,j) in A}

# generate drone delivery times (battery usage), includes service time !!!!!!!!!!!!!!!!! -> why would service time affect drone battery usage?
b_l = {(i,j,l): a[l]*((D[i,j]*2)/(drone_speeds) + drone_service_time) for i in N_s for j in N for l in L if i != j}

# total required delivery time
tau_l = {(i,j,l): (D[i,j]*2)/(drone_speeds*a[l]) + drone_service_time for i in N_s for j in N for l in L if i != j}

# !!!! model !!!!
m = gp.Model("MIP_implementation")

# variables
X = {(i,j): m.addVar(vtype=gp.GRB.BINARY) for (i,j) in A} # if truck travels from i to j
H = {(i,j,l): m.addVar(vtype=gp.GRB.BINARY) for i in N_s for j in N for l in L} # if drone l travels from i to j
W = {i: m.addVar(vtype=gp.GRB.CONTINUOUS) for i in N_s} # Wait time at node i

# set objective
m.setObjective(gp.quicksum(T_v[i,j]*X[i,j] for (i,j) in A) + gp.quicksum(W[i] for i in N_s), gp.GRB.MINIMIZE)

# constraints
departFromDepot = m.addConstr(gp.quicksum(X["s",j] for j in N) == 1) # truck must depart from depot

returnToDepot = m.addConstr(gp.quicksum(X[i,"t"] for i in N) == 1) # truck must return to depot

noFlowAlongSameArc = {(i,j): 
    m.addConstr(X[i,j] + X[j,i] <= 1) 
    for (i,j) in A if i != "s" and j != "t"}

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


def Callback(model, where):
    if where==gp.GRB.Callback.MIPSOL:
        XV = model.cbGetSolution(X)
        HV = model.cbGetSolution(H)
        sets = find_sets(XV, A)
        # if length of sets is larger than 1 then the solution is infeasible
        if len(sets) > 1:
            for sub in sets[1:]:
                len_subtour = len(sub)
                print(sets, "|||", sub)
                model.cbLazy(gp.quicksum(X[i,j] for (i,j) in sub) <= len_subtour-eps)
        peter_plot_path(XV, HV, N, N_s, N_st, A)

m.setParam('LazyConstraints', 1)
#m.setParam('outputflag', 0)
m.optimize(Callback)

X_vals = {(i,j): X[i,j].x for (i,j) in A}
H_vals = {(i,j,l): H[i,j,l].x for i in N_s for j in N for l in L}


if True:
    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    #print(f"battery usage: {[sum([H_vals[i,j,l]*b_l[i,j,l] for i in N_s for j in N if i != j]) for l in L]}")
    print(f"truck path: {[(i,j) for (i,j) in A if X_vals[i,j] > 0.9]}")
    print(f"drone decisions: {[(i,j,l) for i in N_s for j in N for l in L if H_vals[i,j,l] > 0.9]}")
print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")

if __name__ == "__main__":
    # Best objective 2.015393143872e+02, 174 (modding eqn) vs 177.2
    peter_plot_path(X_vals, H_vals, N, N_s, N_st, A)
    print(f"subtours {find_sets(X_vals, A)}")