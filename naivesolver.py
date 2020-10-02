import math
from sklearn.neighbors import NearestNeighbors
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


class NaiveSolver:
    """Capacited Vehicles Routing Problem (CVRP)."""
    demands = []
    vehicle_capacities = []
    routing = None
    manager = None
    NEIGHBORS = 0
    num_vehicles = 0
    customer_neighbour = None
    distance = None
    depot = 0

    def __init__(self, customers, vehicle_count, capacity):
        """Stores the data for the problem."""
        self.NEIGHBORS = len(customers)
        self.customers = customers
        self.create_adjacency_list()
        self.demands = list(map(lambda x: x.demand, customers))
        self.vehicle_capacities = [capacity for i in range(vehicle_count)]
        self.num_vehicles = vehicle_count
        self.depot = 0

        # Create the routing index manager.
        self.manager = pywrapcp.RoutingIndexManager(len(self.customers), self.num_vehicles, self.depot)

        # Create Routing Model.
        self.routing = pywrapcp.RoutingModel(self.manager)

        # Create and register a transit callback.
        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = self.manager.IndexToNode(from_index)
            to_node = self.manager.IndexToNode(to_index)
            if to_node not in self.customer_neighbour[from_node]:
                return 9999999
            return self.distance[from_node][self.customer_neighbour[from_node][to_node]]

        transit_callback_index = self.routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        self.routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Capacity constraint.
        def demand_callback(from_index):
            from_node = self.manager.IndexToNode(from_index)
            return self.demands[from_node]

        demand_callback_index = self.routing.RegisterUnaryTransitCallback(demand_callback)
        self.routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,
            self.vehicle_capacities,
            True,
            'Capacity')

    def create_adjacency_list(self):
        knn = NearestNeighbors(n_neighbors=self.NEIGHBORS, metric='euclidean')
        knn.fit(list(map(lambda x: [x.x, x.y], self.customers)))
        distance, indices = knn.kneighbors(list(map(lambda x: [x.x, x.y], self.customers)))
        self.customer_neighbour = list(map(lambda i: dict(list(zip(indices[i], range(len(indices[i]))))),
                                           range(len(self.customers))))
        self.distance = distance

    def solve(self):
        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.seconds = 120

        # Solve the problem.
        solution = self.routing.SolveWithParameters(search_parameters)

        # print("Solver status: ", self.routing.status())

        # Print solution on console.
        if solution:
            # self.print_solution(solution)
            return self.get_solution(solution)
        else:
            print("Huh? No solution!")

    def get_solution(self, solution):
        total_distance = 0
        vehicle_route = []
        for vehicle_id in range(self.num_vehicles):
            vehicle_route.append([])
            index = self.routing.Start(vehicle_id)
            route_distance = 0
            while not self.routing.IsEnd(index):
                node_index = self.manager.IndexToNode(index)
                vehicle_route[vehicle_id].append(node_index)
                previous_index = index
                index = solution.Value(self.routing.NextVar(index))
                route_distance += self.routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id)
            vehicle_route[vehicle_id].append(self.manager.IndexToNode(index))
            total_distance += route_distance
        return total_distance, vehicle_route

    def print_solution(self, solution):
        """Prints solution on console."""
        total_distance = 0
        total_load = 0
        for vehicle_id in range(self.num_vehicles):
            index = self.routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            route_distance = 0
            route_load = 0
            while not self.routing.IsEnd(index):
                node_index = self.manager.IndexToNode(index)
                route_load += self.demands[node_index]
                plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
                previous_index = index
                index = solution.Value(self.routing.NextVar(index))
                route_distance += self.routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id)
            plan_output += ' {0} Load({1})\n'.format(self.manager.IndexToNode(index),
                                                     route_load)
            plan_output += 'Distance of the route: {}m\n'.format(route_distance)
            plan_output += 'Load of the route: {}\n'.format(route_load)
            print(plan_output)
            total_distance += route_distance
            total_load += route_load
        print('Total distance of all routes: {}m'.format(total_distance))
        print('Total load of all routes: {}'.format(total_load))
