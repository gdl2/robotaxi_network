"""Simple Pickup Delivery Problem (PDP)."""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model(duration_matrix, pickup_dropoff_pairs, vehicle_start_locations, num_vehicles):
    """Stores the data for the problem."""
    data = {}
    data['duration_matrix'] = duration_matrix
    data['pickups_deliveries'] = pickup_dropoff_pairs
    data['num_vehicles'] = num_vehicles
    data['starts'] = vehicle_start_locations
    data['ends'] = [0]*num_vehicles
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    total_duration = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_duration = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_duration += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Duration of the route: {}m\n'.format(route_duration)
        print(plan_output)
        total_duration += route_duration
    print('Total Duration of all routes: {}m'.format(total_duration))

def solution_vehicle_routes(data, manager, routing, solution):
    vehicle_routes = {}
    for vehicle_id in range(data['num_vehicles']):
        vehicle_routes[vehicle_id] = {'route': [], 'duration': 0}
        route_duration = 0
        index = routing.Start(vehicle_id)
        # This will ignore the last node (which is 0)
        while not routing.IsEnd(index):
            vehicle_routes[vehicle_id]['route'].append(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_duration += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        vehicle_routes[vehicle_id]['duration'] = route_duration
    return vehicle_routes


from random import choice, sample
def create_random_duration_matrix(num_locations, start_time, end_time):
    # Generate random points in 2D coordinate plane
    list_points = []
    for i in range(num_locations):
        x, y = choice(range(start_time, end_time)), choice(range(start_time, end_time))
        list_points.append((x,y))

    # Return manhanttan distance matrix (1 unit of duration = 1 unit of distance)
    duration_matrix = []
    for i in range(num_locations):
        row = []
        for j in range(num_locations):
            xi, yi = list_points[i]
            xj, yj = list_points[j]
            duration = abs(xi-xj) + abs(yi-yj)
            row.append(duration)
        duration_matrix.append(row)

    # Add dummy depot by setting first row and first column to 0
    duration_matrix.insert(0, [0]*num_locations)
    for i in range(num_locations+1):
        duration_matrix[i].insert(0, 0)
    return duration_matrix

def create_random_pickup_dropoff_pairs(num_locations):
    # Generate random pairs without repetition (not including origin depot)
    lst = list(range(1, num_locations+1))
    pickup_dropoff_pairs = []
    while len(lst):
        pickup, dropoff = sample(lst, 2)
        pickup_dropoff_pairs.append([pickup, dropoff])
        lst.remove(pickup)
        lst.remove(dropoff)
    return pickup_dropoff_pairs

def main():
    num_locations = 10
    num_vehicles = round(num_locations/3)

    while True:
        # Instantiate the data problem.
        duration_matrix = create_random_duration_matrix(num_locations, 0, 15)
        duration_matrix = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 1151, 1122, 1873, 1336, 704, 964, 1761, 1311, 1105, 995, 828, 721, 1220], [0, 1161, 0, 918, 828, 270, 643, 1426, 1995, 556, 822, 379, 859, 605, 340], [0, 1126, 902, 0, 1235, 879, 1012, 1014, 1401, 633, 416, 814, 315, 622, 1096], [0, 1849, 835, 1210, 0, 587, 1331, 2003, 2401, 587, 920, 1028, 1392, 1269, 972], [0, 1348, 272, 841, 575, 0, 830, 1532, 1919, 474, 746, 519, 899, 789, 417], [0, 696, 622, 987, 1343, 806, 0, 1081, 1837, 798, 894, 428, 693, 483, 683], [0, 951, 1427, 960, 1977, 1488, 1097, 0, 1021, 1435, 1264, 1240, 765, 972, 1488], [0, 1776, 1936, 1384, 2401, 1912, 1859, 1013, 0, 1866, 1688, 1894, 1444, 1690, 2129], [0, 1329, 552, 649, 616, 466, 839, 1442, 1840, 0, 359, 597, 844, 708, 795], [0, 1108, 817, 417, 944, 793, 917, 1320, 1719, 342, 0, 722, 623, 519, 1010], [0, 1009, 383, 803, 1063, 519, 462, 1252, 1926, 575, 707, 0, 651, 382, 587], [0, 827, 862, 317, 1379, 891, 713, 763, 1444, 832, 626, 657, 0, 351, 1062], [0, 730, 622, 677, 1305, 798, 501, 980, 1691, 711, 563, 395, 398, 0, 807], [0, 1257, 331, 1119, 936, 405, 731, 1515, 2197, 792, 1024, 582, 1061, 791, 0]]
        pickup_dropoff_pairs = create_random_pickup_dropoff_pairs(num_locations)
        # CANNOT HAVE MULTIPLE TRIPS THAT START FROM THE
        pickup_dropoff_pairs = [[14, 7], [5, 5], [5, 5], [6, 10], [14, 13], [5, 11], [2, 11], [2, 4], [1, 8], [5, 5], [5, 9], [2, 14], [6, 3]]
        vehicle_start_locations = [0]*num_vehicles
        data = create_data_model(duration_matrix, pickup_dropoff_pairs, vehicle_start_locations, num_vehicles)

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(len(data['duration_matrix']),
                                               data['num_vehicles'], data['starts'], data['ends'])

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)


        # Define cost of each arc.
        def duration_callback(from_index, to_index):
            """Returns the duration between the two nodes."""
            # Convert from routing variable Index to duration matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data['duration_matrix'][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(duration_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Distance constraint.
        dimension_name = 'Duration'
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            3000,  # vehicle maximum travel duration
            True,  # start cumul to zero
            dimension_name)
        duration_dimension = routing.GetDimensionOrDie(dimension_name)
        duration_dimension.SetGlobalSpanCostCoefficient(100)

        # Define Transportation Requests.
        for request in data['pickups_deliveries']:
            pickup_index = manager.NodeToIndex(request[0])
            delivery_index = manager.NodeToIndex(request[1])
            routing.AddPickupAndDelivery(pickup_index, delivery_index)
            routing.solver().Add(
                routing.VehicleVar(pickup_index) == routing.VehicleVar(
                    delivery_index))
            # We are assuming single-occupancy vehicles:
            # The vehicle must drop off passenger right after picking them up
            routing.solver().Add(
                duration_dimension.CumulVar(pickup_index) ==
                duration_dimension.CumulVar(delivery_index) - duration_callback(pickup_index, delivery_index))
            # Passenger must have been picked up within the 15-minute time window
            routing.solver().Add(
                duration_dimension.CumulVar(pickup_index) <= 15)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.LOCAL_CHEAPEST_INSERTION)
        #search_parameters.local_search_metaheuristic = (
        #    routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.seconds = 20
        #search_parameters.log_search = True

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            print_solution(data, manager, routing, solution)
            print(solution_vehicle_routes(data, manager, routing, solution))
            return
        else:
            num_vehicles += 1
            print("PASS", num_vehicles)

from time import time
if __name__ == '__main__':
    start_time = time()
    main()
    print(time()-start_time, "seconds")
