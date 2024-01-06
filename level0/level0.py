#Load files

import json
file=open("C:\\21pw21-hackathon\Input data\level0.json")
inp=json.load(file)


loc_dist=[inp['restaurants']['r0']['neighbourhood_distance']]
loc_dist[0].insert(0,0)

#Collect all distances into a list called loc_dist

for neighbours in inp['neighbourhoods'].values():
    neighbour_dist=list()
    for dist in neighbours['distances']:
        neighbour_dist.append(dist)
    neighbour_dist.insert(0,0)
    loc_dist.append(neighbour_dist)
#print(loc_dist[0])

#TSP using dynamic programming method isnt time optimized 

'''def tsp_dynamic_programming(n, dist):
    dp = [[float('inf')] * n for _ in range(1 << n)]

    dp[1][0] = 0

    for mask in range(1, 1 << n):
        for i in range(n):
            if mask & (1 << i) != 0:
                for j in range(n):
                    if mask & (1 << j) != 0 and j != i:
                        dp[mask][i] = min(dp[mask][i], dp[mask ^ (1 << i)][j] + dist[j][i])

    mask = (1 << n) - 1
    min_cost = min(dp[mask][i] + dist[i][0] for i in range(1, n))

    return min_cost


n = len(loc_dist)


result = tsp_dynamic_programming(n, loc_dist)
print("The cost of the most efficient tour =", result)'''


def total_distance(tour, distances):
    total = 0
    n = len(tour)
    for i in range(n):
        total += distances[tour[i]][tour[(i + 1) % n]]
    return total


#Lin Kernighan method isnt time optimized either

'''def reverse_segment_if_better(tour, i, j, distances):
    n = len(tour)
    a, b = tour[i], tour[j]
    reverse_tour = tour[:i] + tour[i:j+1][::-1] + tour[j+1:]
    delta = (
        distances[a][b] +
        distances[tour[(i-1) % n]][tour[(j+1) % n]] -
        distances[tour[i % n]][tour[(j+1) % n]] -
        distances[tour[(i-1) % n]][b]
    )
    return delta < 0, reverse_tour

def lin_kernighan(tour, distances):
    improved = True
    iteration = 0
    max_iterations = 100  # Set a maximum number of iterations
    min_improvement = 1e-6  # Set a minimum improvement threshold

    while improved and iteration < max_iterations:
        improved = False
        for i in range(len(tour)):
            for j in range(i+2, len(tour)-1):
                reverse, new_tour = reverse_segment_if_better(tour, i, j, distances)
                if reverse:
                    tour = new_tour
                    improved = True

        iteration += 1

        # Calculate improvement in tour length
        initial_length = total_distance(tour, distances)
        improved_length = total_distance(new_tour, distances)
        improvement = initial_length - improved_length

        # Check if improvement is below the threshold
        if improvement > min_improvement:
            break

    return tour
'''

'''initial=[0]
visited=[0]
j=0
for i in range(len(loc_dist)):
    min_val=loc_dist[j].index(min(loc_dist[j]))
    if 
    initial.append(loc_dist[i].index(min(loc_dist[i])))'''
    
    
def nearest_neighbor_algorithm(distances):
    num_nodes = len(distances)
    visited = [False] * num_nodes
    visited[0]=True
    tour = [0]  # Start from node 0

    for _ in range(num_nodes - 1):
        current_node = tour[-1]
        min_distance = float('inf')
        nearest_node = -1

        for i in range(num_nodes):
            if not visited[i] and i != current_node:
                distance = distances[current_node][i]
                if distance < min_distance:
                    min_distance = distance
                    nearest_node = i

        tour.append(nearest_node)
        visited[nearest_node] = True

    tour.append(tour[0])  

    return tour



initial_tour = nearest_neighbor_algorithm(loc_dist)
#improved_tour = lin_kernighan(initial_tour, loc_dist)

print("Initial Tour:", initial_tour)
#print("Improved Tour:", improved_tour)
print("Initial Tour Length:", total_distance(initial_tour, loc_dist))
#print("Improved Tour Length:", total_distance(improved_tour, loc_dist))

out_dist=list()
for i in range(len(initial_tour)):
    if initial_tour[i]==0:
        out_dist.append("r0")
    else:
        out_dist.append("n"+str(initial_tour[i]-1))    

paths = {"v0":{'path':out_dist}}
print(paths)

with open("C:\\21pw21-hackathon\Output data\level0_output.json", "w") as outfile: 
    json.dump(paths, outfile)