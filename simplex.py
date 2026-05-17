from ortools.linear_solver import pywraplp

solver = pywraplp.Solver.CreateSolver('GLOP')

x = solver.NumVar(0, solver.infinity(), 'x')
y = solver.NumVar(0, solver.infinity(), 'y')

solver.Add(x + 2 * y <= 8)
solver.Add(3 * x + y <= 9)

solver.Maximize(3 * x + 4 * y)

status = solver.Solve()

if status == pywraplp.Solver.OPTIMAL:
    print("===== Solution Optimale =====")
    print("x =", x.solution_value())
    print("y =", y.solution_value())
    print("Profit =", solver.Objective().Value())