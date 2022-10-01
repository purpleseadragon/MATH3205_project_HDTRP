import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import math
import numpy as np

# generate arc distances
def generate_distances(all_arcs, N_st):
    """distances correspond to arc distance from i to j in arc (i,j)"""
    distances = dict()
    for x in all_arcs:
        distances[x] = (math.sqrt((N_st[x[0]][0] - N_st[x[1]][0])**2 + (N_st[x[0]][1] - N_st[x[1]][1])**2))
    return distances

def dist_betw_arcs(arc1, arc2):
    """generates distance between arcs"""
    return math.sqrt((arc1[0] - arc2[0])**2 + (arc1[1] - arc2[1])**2)


def plot_path(X,H, N, N_s, N_st, A, L):
    """Takes X and H variables and plots the truck and drone routes respectively"""
    # get path
    path = []
    for (i,j) in A:
        if X[i,j] > 0.9:
            path.append((i,j))
    # get drone path
    drone_path = []
    for i in N_s:
        for j in N:
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

def peter_plot_path(X,H, N, N_s, N_st, A):
    """Peter's code for plotting"""
    verts = []
    for (i,j) in A:
        if X[i,j] > 0.9:
            #print(i,j)
            verts.append(tuple(N_st[i]))
            verts.append(tuple(N_st[j]))
            
    # print(verts)
    codes = []
    for i in range(int(len(verts)/2)):
        # print(i)
        codes.append(Path.MOVETO)
        codes.append(Path.LINETO) 
    # print(codes)
    path = Path(verts, codes)
    fig, ax = plt.subplots()
    patch = patches.PathPatch(path, color='blue', lw=2)
    ax.add_patch(patch)

    xs, ys = zip(*verts)
    ax.plot(xs, ys, 'x', lw=2, color='black', ms=10)

    # Plot drone 0 routs
    verts2 = []
    for i in N_s:
        for j in N:
            for l in [0]:
                if i!=j and H[i,j,l] > 0.9:
                    # print(i,j)
                    verts2.append(tuple(N_st[i]))
                    verts2.append(tuple(N_st[j]))
                    
    # print(verts)
    codes2 = []
    for i in range(int(len(verts2)/2)):
        # print(i)
        codes2.append(Path.MOVETO)
        codes2.append(Path.LINETO) 
    # print(codes)
    path2 = Path(verts2, codes2)
    patch2 = patches.PathPatch(path2, color='orange', lw=1)
    ax.add_patch(patch2)

    xs2, ys2 = zip(*verts2)
    ax.plot(xs2, ys2, '.', lw=2, color='orange', ms=8)

    # Plot drone 1 routs
    verts3 = []
    for i in N_s:
        for j in N:
            for l in [1]:
                if i!=j and H[i,j,l] > 0.9:
                    # print(i,j)
                    verts3.append(tuple(N_st[i]))
                    verts3.append(tuple(N_st[j]))
                    
    # print(verts)
    codes3 = []
    for i in range(int(len(verts3)/2)):
        # print(i)
        codes3.append(Path.MOVETO)
        codes3.append(Path.LINETO) 
    # print(codes)
    path3 = Path(verts3, codes3)
    patch3 = patches.PathPatch(path3, color='red', lw=1)
    ax.add_patch(patch3)

    xs3, ys3 = zip(*verts3)
    ax.plot(xs3, ys3, '.', lw=2, color='red', ms=8)

    for n in N_s:
        x, y = N_s[n]
        ax.text(x-1.5,y+2, n)

    ax.set_xlim(-5, 105)
    ax.set_ylim(-5, 105)
    # ax.set_xlim(20, 70)
    # ax.set_ylim(20, 80)
    plt.show()


def peter_plot_path2(X, N, N_s, N_st, A):
    """Peter's code for plotting"""
    verts = []
    for (i,j) in A:
        if X[i,j] > 0.9:
            #print(i,j)
            verts.append(tuple(N_st[i]))
            verts.append(tuple(N_st[j]))
            
    # print(verts)
    codes = []
    for i in range(int(len(verts)/2)):
        # print(i)
        codes.append(Path.MOVETO)
        codes.append(Path.LINETO) 
    # print(codes)
    path = Path(verts, codes)
    fig, ax = plt.subplots()
    patch = patches.PathPatch(path, color='blue', lw=2)
    ax.add_patch(patch)

    xs, ys = zip(*verts)
    ax.plot(xs, ys, 'x', lw=2, color='black', ms=10)

    for n in N_s:
        x, y = N_s[n]
        ax.text(x-1.5,y+2, n)

    ax.set_xlim(-5, 105)
    ax.set_ylim(-5, 105)
    # ax.set_xlim(20, 70)
    # ax.set_ylim(20, 80)
    plt.show()

def find_sets(X_vals, A):
    """finds subtours -> used for subtour lazy constraints"""
    all_tours = []
    all_tours.append(set())
    # first subtour made up of source and sink node tour
    for (i,j) in A:
        if X_vals[i,j] > 0.9:
            if i == "s":
                all_tours[0].add((i,j))
            elif j == "t":
                all_tours[0].add((i,j))

    # find additional subtours
    for (i,j) in A:
        if X_vals[i,j] > 0.9:
            added = False # boolean for if the arc has been added to subtour set yet
            for subtour in all_tours:
                for (y,z) in subtour:
                    if i == z or i == y or j == z or j == y:
                        subtour.add((i,j))
                        added = True
                        break
            if not added:
                all_tours.append(set())
                all_tours[-1].add((i,j))
    return all_tours

"""PETER"""
def adjacencyMatrix(V, E):
    # Construct an adjacency matrix for graph G(V, E)
    # V is a list of vertices (nodes) and E is a list of edges (arcs)
    M = np.zeros(shape=(len(V),len(V)))
    for (i,j) in E:
        M[V.index(i)][V.index(j)] = 1
    return M


"""PETER"""
def generateComponent(label, Nst):
    # collects components with same label together (i.e subroutes longer than len 1)
    S = {}
    for i in range(len(label)):
        if label[i] not in S:
            # print('opps')
            S[label[i]] = [list(Nst)[i]]
        else:
            # print('!!!!!!!!!!!!!!!!!!!!!!!!')
            S[label[i]].append(list(Nst)[i])
    # print(S)
    return list(S.values())
