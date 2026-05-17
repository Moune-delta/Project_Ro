from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# مصفوفة المسافات
distance_matrix = [
    [0, 10, 15, 20, 25],
    [10, 0, 35, 25, 30],
    [15, 35, 0, 30, 20],
    [20, 25, 30, 0, 15],
    [25, 30, 20, 15, 0]
]

# أسماء المدن
cities = ['A', 'B', 'C', 'D', 'E']

# عدد المدن
num_cities = 5

# مدينة البداية
depot = 0

# إنشاء الموديل
manager = pywrapcp.RoutingIndexManager(
    num_cities,
    1,
    depot
)

routing = pywrapcp.RoutingModel(manager)

# دالة حساب المسافة
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

# إعدادات البحث
search_parameters = pywrapcp.DefaultRoutingSearchParameters()

search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)

# حل المشكلة
solution = routing.SolveWithParameters(search_parameters)

# عرض النتائج
if solution:

    print("===== Solution TSP =====")

    index = routing.Start(0)

    route = []

    total_distance = 0

    while not routing.IsEnd(index):

        route.append(cities[manager.IndexToNode(index)])

        previous_index = index

        index = solution.Value(
            routing.NextVar(index)
        )

        total_distance += routing.GetArcCostForVehicle(
            previous_index,
            index,
            0
        )

    route.append(cities[manager.IndexToNode(index)])

    print(" -> ".join(route))

    print("Distance totale :", total_distance, "km")

else:
    print("Pas de solution.")