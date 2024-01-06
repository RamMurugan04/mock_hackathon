#Load files

import json
file=open("C:\\21pw21-hackathon\Input data\level1a.json")
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

vehicle=inp['vehicles']['v0']

order_quantities = [inner_dict['order_quantity'] for inner_dict in inp['neighbourhoods'].values()]
order_quantities.insert(0,600)


'''
paths = list()
visited=[[0]*20]
i=0
while True:
    dist = loc_dist[0]
    dist.pop(0)
    restaurant_dist=dict()
    profit_ratio=dict()
    for i in range(len(dist)):
        restaurant_dist[i]= dist[i]
        profit_ratio[i]=dist[i]/order_quantities[i]
    profit_ratio=sorted(profit_ratio.items(), key=lambda x:x[1])
    cost=vehicle['capacity']
    path=[0]
    while cost>0:
        current_node=profit_ratio[0]
        if cost-order_quantities[current_node[0]]>0:
            cost-=order_quantities[current_node[0]]
            path.append(current_node[0])
            
        else:
            cost=0
        
    
    print(profit_ratio)
    #for i in range(len(restaurant_dist)):
        #profit_ratio[i]=restaurant_dist[i]/order_quantities[i]
    #profit_ratio=sorted(profit_ratio, key=lambda x:x[1])
    #print(profit_ratio)
    i+=1
'''
def nearest_neighbor_algorithm(distances):
    num_nodes = len(distances)
    visited = [False] * num_nodes
    visited[0] = True
    tour = [0]  # Start from node 0
    min_cost = 0

    for _ in range(num_nodes - 1):
        current_node = tour[-1]
        min_distance = float('inf')
        nearest_node = -1
        
        dist = distances[current_node]
        dist.pop(0)
        restaurant_dist=dict()
        profit_ratio=dict()
        for i in range(len(dist)):
            restaurant_dist[i]= dist[i]
            profit_ratio[i]=dist[i]/order_quantities[i]
        profit_ratio=sorted(profit_ratio.items(), key=lambda x:x[1])
        
        i=profit_ratio[0][0]
        while not visited[i] and i != current_node:
            distance = distances[current_node][i]
            nearest_node = i
            profit_ratio.pop(0)
            i=profit_ratio[0][0]

        tour.append(nearest_node)
        min_cost += min_distance
        visited[nearest_node] = True

    tour.append(tour[0])

    return tour

initial_tour = nearest_neighbor_algorithm(loc_dist)
print(initial_tour)
'''
def knapsack_recursive(weights, capacity, n):
    if n == 0 or capacity == 0:
        return 0

    if weights[n-1] > capacity:
        return knapsack_recursive(weights, capacity, n-1)
    else:
        include_item = weights[n-1] + knapsack_recursive(weights, capacity - weights[n-1], n-1)
        exclude_item = knapsack_recursive(weights, capacity, n-1)
        return max(include_item, exclude_item)

def find_optimal_paths(weights, dist_matrix):
    n = len(weights)

    # Calculate the TSP cost for the entire set of nodes
    tsp_cost = nearest_neighbor_algorithm(dist_matrix)

    # Sort nodes based on their weights in descending order
    sorted_indices = sorted(range(n), key=lambda k: weights[k], reverse=True)

    paths = []
    current_capacity = weights[sorted_indices[0]]

    for i in range(n):
        if i == 0:
            continue

        if current_capacity >= weights[sorted_indices[i]]:
            # Check if adding the node exceeds TSP cost
            tsp_cost_with_node = nearest_neighbor_algorithm(dist_matrix + [[0] * (i+1) for _ in range(n-i)])
            if tsp_cost_with_node <= tsp_cost:
                current_capacity -= weights[sorted_indices[i]]
                paths[-1].append(sorted_indices[i])
            else:
                current_capacity = weights[sorted_indices[i]]
                paths.append([sorted_indices[0], sorted_indices[i]])
        else:
            current_capacity = weights[sorted_indices[i]]
            paths.append([sorted_indices[0], sorted_indices[i]])

    return paths

# Example usage:

optimal_paths = find_optimal_paths(order_quantities, loc_dist)
print("Optimal Paths:", optimal_paths)



'''