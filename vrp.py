from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# مصفوفة المسافات
distance_matrix = [
    [0, 3, 5, 8, 6, 10, 12],
    [3, 0, 4, 7, 5, 9, 11],
    [5, 4, 0, 3, 6, 7, 8],
    [8, 7, 3, 0, 4, 5, 6],
    [6, 5, 6, 4, 0, 8, 9],
    [10, 9, 7, 5, 8, 0, 4],
    [12, 11, 8, 6, 9, 4, 0]
]

# الطلبات
demands = [0, 200, 300, 250, 150, 400, 350]

capacity = 1000
num_vehicles = 2
depot = 0

names = [
    "Depot",
    "P1(Arafat)",
    "P2(Sebkha)",
    "P3(DarNaim)",
    "P4(Teyarett)",
    "P5(Toujounine)",
    "P6(Riyad)"
]

# إنشاء الموديل
manager = pywrapcp.RoutingIndexManager(
    len(distance_matrix),
    num_vehicles,
    depot
)

routing = pywrapcp.RoutingModel(manager)

# distance callback
def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return distance_matrix[from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(
    distance_callback
)

routing.SetArcCostEvaluatorOfAllVehicles(
    transit_callback_index
)

# demand callback
def demand_callback(from_index):
    from_node = manager.IndexToNode(from_index)
    return demands[from_node]

demand_callback_index = routing.RegisterUnaryTransitCallback(
    demand_callback
)

routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0,
    [capacity] * num_vehicles,
    True,
    "Capacity"
)

# إعدادات البحث
search_parameters = pywrapcp.DefaultRoutingSearchParameters()

search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)

# حل المشكلة
solution = routing.SolveWithParameters(search_parameters)

# عرض النتائج
if solution:
    print("===== Plan de distribution d’eau =====")

    total_distance = 0

    for vehicle_id in range(num_vehicles):

        index = routing.Start(vehicle_id)

        route = []
        route_load = 0
        route_distance = 0

        while not routing.IsEnd(index):

            node_index = manager.IndexToNode(index)

            route.append(names[node_index])

            route_load += demands[node_index]

            previous_index = index

            index = solution.Value(
                routing.NextVar(index)
            )

            route_distance += routing.GetArcCostForVehicle(
                previous_index,
                index,
                vehicle_id
            )

        route.append(names[manager.IndexToNode(index)])

        print(f"\nCamion {vehicle_id + 1}")
        print(" -> ".join(route))
        print("Charge :", route_load, "L")
        print("Distance :", route_distance, "km")

        total_distance += route_distance

    print("\nDistance totale :", total_distance, "km")

else:
    print("Pas de solution.")