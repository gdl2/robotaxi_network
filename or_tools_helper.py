from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model(duration_matrix, pickup_dropoff_pairs, num_vehicles, demands):
    """Stores the data for the problem."""
    data = {}
    data['duration_matrix'] = duration_matrix
    data['pickups_deliveries'] = pickup_dropoff_pairs
    data['num_vehicles'] = num_vehicles
    data['starts'] = [0]*num_vehicles #XXX CHANGE THIS XXX
    data['ends'] = [0]*num_vehicles
    data['demands'] = demands
    return data

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

def vrp_solver(duration_matrix, pickup_dropoff_pairs, demands):
    num_vehicles = max(round(len(pickup_dropoff_pairs)/2), 1)
    while True:
        """Entry point of the program."""
        # Instantiate the data problem.
        data = create_data_model(duration_matrix, pickup_dropoff_pairs, num_vehicles, demands)

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(len(data['duration_matrix']),
                                               data['num_vehicles'], data['starts'], data['ends'])

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)


        # Create and register a transit callback.
        def duration_callback(from_index, to_index):
            """Returns the duration between the two nodes."""
            # Convert from routing variable Index to duration matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data['duration_matrix'][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(duration_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Distance constraint.
        dimension_name = 'Duration'
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            120*60,  # vehicle maximum travel duration
            True,  # start cumul to zero
            dimension_name)
        duration_dimension = routing.GetDimensionOrDie(dimension_name)
        duration_dimension.SetGlobalSpanCostCoefficient(100)

        # Define Transportation Requests.
        for request in data['pickups_deliveries']:
            pickup_index = manager.NodeToIndex(request[0])
            delivery_index = manager.NodeToIndex(request[1])
            routing.AddPickupAndDelivery(pickup_index, delivery_index)
            # Make sure each vehicle drops off passenger before picking up someone else
            routing.solver().Add(routing.NextVar(pickup_index) == delivery_index)
            routing.solver().Add(
                routing.VehicleVar(pickup_index) == routing.VehicleVar(
                    delivery_index))
            # Passenger must have been picked up within the 15-minute time window
            routing.solver().Add(
                duration_dimension.CumulVar(pickup_index) <= 15*60)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.LOCAL_CHEAPEST_INSERTION)
        search_parameters.time_limit.seconds = 5

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            return solution_vehicle_routes(data, manager, routing, solution)
        else:
            print(num_vehicles, " PASS")
            num_vehicles += 1
