# import gurobipy as gp

# model = gp.read('<installdir>/examples/data/p0033.mps')
# model.optimize()
H = {(i,j,l): BMP.addVar(vtype=gp.GRB.BINARY) for i in N_s for j in N for l in L} # if drone l travels from i to j
    W = {i: BMP.addVar(vtype=gp.GRB.CONTINUOUS) for i in N_s} # Wait time at node i

visitEachNode = {j: 
        BMP.addConstr(gp.quicksum(X[i,j] for i in N_s if i != j) + 
                    gp.quicksum(H[i,j,l] for i in N_s for l in L if i != j) == 1)
        for j in N} 

    droneDispatch = {i:
        BMP.addConstr(M*gp.quicksum(X[i,j] for j in N_t if N_t[j] != N_s[i]) >= 
                    gp.quicksum(H[i,j,l] for j in N for l in L if j != i))
        for i in N_s}

    batteryConsumption = {l:
        BMP.addConstr(gp.quicksum(H[i,j,l]*b_l[i,j,l] for i in N_s for j in N if i !=j) <= B_l[l])
        for l in L}

    waitTime = {(i,l):
        BMP.addConstr(W[i] >= gp.quicksum(H[i,j,l]*tau_l[i,j,l] for j in N if j != i))
        for i in N_s for l in L}