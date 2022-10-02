import gurobipy as gp

from helper import peter_plot_path, generate_distances

from load_test_peter import load_test

# load tests 
#test1 = load_test('P')['P-n16-k8']
test2 = load_test('A')['A-n37-k5']

# default data
drone_num = 2
drone_speeds = 2.5#[1, 1.5]
a = [1, 1.5]
drone_battery = B_l = [100, 130] # max flight time
truck_speed = 1
truck_service_time = 10
drone_flight_duration = 100
drone_service_time = 5
L = range(len(a))
OBJBND = False

def MIP_route_length(test,  p):
    """returns the optimal solution, p is given route length and test is test data"""  
    # !!! maybe need to change to root relaxation version
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

    # generate drone delivery times (battery usage), includes service time !!!!!!!!!!!!!!!!! -> why would service time affect drone battery usage?
    b_l = {(i,j,l): a[l]*((D[i,j]*2)/(drone_speeds) + drone_service_time) for i in N_s for j in N for l in L if i != j}

    # total required delivery time
    tau_l = {(i,j,l): (D[i,j]*2)/(drone_speeds*a[l]) + drone_service_time for i in N_s for j in N for l in L if i != j}

    # !!!! model !!!!
    m = gp.Model("MIP_implementation")

    # variables
    X = {(i,j): m.addVar(vtype=gp.GRB.BINARY) for (i,j) in A} # if truck travels from i to j
    H = {(i,j,l): m.addVar(vtype=gp.GRB.BINARY) for i in N_s for j in N for l in L} # if drone l travels from i to j
    V = {i: m.addVar(vtype=gp.GRB.CONTINUOUS) for i in N_st} # visiting order of truck at node i
    W = {i: m.addVar(vtype=gp.GRB.CONTINUOUS) for i in N_s} # Wait time at node i

    # set objective (subtracking service time for back to depot)
    m.setObjective(gp.quicksum(T_v[i,j]*X[i,j] for (i,j) in A) + gp.quicksum(W[i] for i in N_s) - truck_service_time, gp.GRB.MINIMIZE)

    # constraints
    # Xrelax = {(i,j):
    #     [m.addConstr(0 <= X[i,j]),  m.addConstr(X[i,j]<= 1)]
    #     for (i,j) in A}

    # Hrelax = {(i,j,l):
    #     [m.addConstr(0 <= H[i,j,l]),  m.addConstr(H[i,j,l]<= 1)]
    #     for i in N_s for j in N for l in L}

    departFromDepot = m.addConstr(gp.quicksum(X["s",j] for j in N) == 1) # truck must depart from depot

    returnToDepot = m.addConstr(gp.quicksum(X[i,"t"] for i in N) == 1) # truck must return to depot


    flowConservation = {i:
        m.addConstr(gp.quicksum(X[i,j] for j in N_t if j != i) == 
                    gp.quicksum(X[j,i] for j in N_s if j != i))
        for i in N}

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

    additionalPConstr = m.addConstr(gp.quicksum(X[i,j] for i in N for j in N_t if j != i)==p)
    #m.setParam("outputflag", 0)
    #m.optimize()


    def Callback(model, where):
        global OBJBND
        if where == gp.GRB.Callback.MIPNODE:
            nodecount = model.cbGet(gp.GRB.Callback.MIPNODE_NODCNT)
            #print(model.cbGet(gp.GRB.Callback.MIPNODE_OBJBND))
            if nodecount > 0:
                OBJBND = model.cbGet(gp.GRB.Callback.MIPNODE_OBJBND)
                model.terminate()

    m.setParam('OutputFlag', 0)
    m.optimize(Callback)
    # X_vals = {(i,j): X[i,j].x for (i,j) in A}
    # H_vals = {(i,j,l): H[i,j,l].x for i in N_s for j in N for l in L}
    # V_vals = {i: V[i].x for i in N_st}

    if False:
        print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
        print(f"battery usage: {[sum([H_vals[i,j,l]*b_l[i,j,l] for i in N_s for j in N if i != j]) for l in L]}")
        print(f"truck path: {[(i,j) for (i,j) in A if X_vals[i,j] > 0.9]}")
        print(f"drone decisions: {[(i,j,l) for i in N_s for j in N for l in L if H_vals[i,j,l] > 0.9]}")
        print(f"order: {[V_vals[i] for i in V_vals]}")
    # print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    return OBJBND


def length_bounding(Z_hat, p_under, test):
    """finds lower bound of HDTRP"""
    N = test['N']
    len_N = len(N) # ???
    print(f"there are {len_N} nodes not including the source / sink node")
    p = p_under + round((len_N - p_under)/3)
    new_z = 0
    print(f"finding smallest possible masx route length")
    while True:
        Z_p = MIP_route_length(test, p)
        print(f"given p = {p}, Z_p={Z_p:.2f}")
        if Z_p <= Z_hat:
            break
        # elif p_under == p:
        #     break
        else:
            p = p_under + round((p-p_under+1)/3)
    p_hat = p
    ps_list = []
    for i in range(p_hat+1, len_N):
        ps_list.append(i)
    print(f"iterating through possible ps {ps_list} ")
    for p in ps_list:
        Z_p = MIP_route_length(test, p)
        if Z_p > Z_hat:
            p_upper = p - 1 # max truck route length
            print(f"the upper bound for truck route length is {p_upper}")
            break
    try:
        return p_upper
    except:
        raise NotImplementedError


if __name__ == "__main__":
    obj_val1 = MIP_route_length(test2, 23) 
    #obj_val2 = RootRelaxPp(test2, 23) 
    print(f"mine: {obj_val1}")

    # Result of Primal Heuristic: Z^=741.0981772209472, p_=20
    length_bounding(741.0981772209472, 20, test2)
    # X_vals, H_vals, V_vals, N, N_s, N_st, A = MIP(a_n37_k5,a_n37_k5_s) # Best objective 
    # peter_plot_path(X_vals, H_vals, N, N_s, N_st, A)