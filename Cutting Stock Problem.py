import gurobipy as gp
from gurobipy import GRB
import numpy as np
import os

# Change to new directory
new_directory = r"D:\SYSPHD\Gurobi"
os.chdir(new_directory)

# Parameters
timber_type = [9, 14, 16]  # 木材原材料的长度
cost_type = [5, 9, 10]  # 木材原材料的单位成本
demand_length = [4, 5, 7]  # 需要木材的长度
demand_num = [30, 20, 40]  # 需要木材的数量

# Parameters of Initial Plan
unit_num = [[2, 0, 0], [0, 1, 0], [0, 0, 1]]  # 三种初始方案

cost_plan = []  # 初始方案的单位成本
for i in range(len(unit_num)):  # 判断初始方案的成本
    dot_product = np.dot(unit_num[i], demand_length)
    if dot_product <= timber_type[0]:
        cost_plan.append(cost_type[0])
    elif dot_product <= timber_type[1]:
        cost_plan.append(cost_type[1])
    elif dot_product <= timber_type[2]:
        cost_plan.append(cost_type[2])
    else:
        print("Wrong Plan")

# 输入三个参数, 方案矩阵的系数, 需要木材的数量, 方案的单位成本
def solve_Rmp(unit_num, demand_num, cost_plan):
    RMP = gp.Model("Restricted Master Problem")  # 建立模型

    # Decision Variable
    x = RMP.addVars(len(unit_num), vtype=GRB.CONTINUOUS, name='x')

    # Constraints
    constraints = []
    for i in range(len(demand_num)):  # 三次循环
        cons_expr = gp.quicksum(unit_num[j][i] * x[j] for j in range(len(unit_num))) >= demand_num[i]
        constr = RMP.addConstr(cons_expr, name=f'Demand{i} Constraint')
        constraints.append(constr)

    # Objective
    objective = gp.quicksum(cost_plan[i] * x[i] for i in range(len(x)))
    RMP.setObjective(objective, GRB.MINIMIZE)

    # Optimization
    RMP.optimize()

    # Return Results 
    if RMP.status == GRB.OPTIMAL:
        solution_x = [x[i].x for i in range(len(x))]  # 提取变量的最优解
        dual_values = [constr.Pi for constr in constraints]  # 提取约束的对偶值
        RMP_optimal = RMP.objVal  # 提取最优目标值
        return solution_x, RMP_optimal, dual_values
    else:
        print("No optimal solution found")
        return None

# 输入三个参数, RMP的对偶值, 需求木材的长度, 木材原材料的长度, 木材原材料的单位成本, 整数值num
def solve_Subproblem(dual_values, demand_length, timber_type, cost_type, num):
    Sub = gp.Model("Subproblem")

    # Decision Variable
    y = Sub.addVars(len(dual_values), vtype=GRB.INTEGER, name='y')

    # Sub Constraints
    sub_cons_expr = gp.quicksum(demand_length[i] * y[i] for i in range(len(demand_length))) \
                    <= timber_type[num]
    Sub.addConstr(sub_cons_expr)

    # Sub Objective
    subobjective = gp.quicksum(dual_values[i] * y[i] for i in range(len(dual_values))) - cost_type[num]
    Sub.setObjective(subobjective, GRB.MAXIMIZE)

    # Optimization
    Sub.optimize()

    # Reserve Results
    if Sub.status == GRB.OPTIMAL:
        solution_y = [int(y[i].x) for i in range(len(y))]  # 提取变量的最优解(系数列)
        suboptimal = Sub.objVal  # 提取最优目标值(检验数)
        return solution_y, suboptimal, cost_type[num]  # cost_type[num]是新方案的成本
    else:
        print("No optimal solution found")
        return None

# 获取加入RMP方案的系数列和成本系数
def get_new_pattern(list_solution_y, list_suboptimal, list_cost_type):
    max_suboptimal = max(list_suboptimal)
    if max_suboptimal <= 0:
        return None, None
    index = list_suboptimal.index(max_suboptimal)
    return list_solution_y[index], list_cost_type[index]

# 更新系数矩阵
def update_column(unit_num, new_column):
    unit_num.append(new_column)
    return unit_num

# 更新成本列表
def update_cost_plan(cost_plan, new_cost):
    cost_plan.append(new_cost)
    return cost_plan

# 问题大循环封装为函数
def iterative_optimization(unit_num, demand_num, cost_plan, timber_type, cost_type, \
                           demand_length, epsilon=1e-6):
    
    # Initial solve of RMP
    result = solve_Rmp(unit_num, demand_num, cost_plan)
    if result is None:
        return None
    solution_x, RMP_optimal, dual_values = result

    # Recording the iterations' results
    iteration_results = []

    reserve_RMP_optimal = 0
    process_RMP_optimal = RMP_optimal
    optimal_gap = abs(process_RMP_optimal - reserve_RMP_optimal)

    # Iterative optimization
    while optimal_gap > epsilon:
        reserve_RMP_optimal = process_RMP_optimal

        # Solve Subproblems
        list_solution_y = []
        list_suboptimal = []
        list_cost_type = []
        for i in range(len(timber_type)):
            result = solve_Subproblem(dual_values, demand_length, timber_type, cost_type, i)
            if result is None:
                return None
            solution_y, suboptimal, sub_cost = result
            list_solution_y.append(solution_y)
            list_suboptimal.append(suboptimal)
            list_cost_type.append(sub_cost)
        
        iteration_results.append({
            'solution_x': solution_x,
            'RMP_optimal': process_RMP_optimal,
            'dual_values': dual_values,
            'subproblems_optimal_values': list_suboptimal
        })
        
        # Get new column and update
        new_unit_column, new_unit_cost = get_new_pattern(list_solution_y, list_suboptimal, \
                                                         list_cost_type)

        # If no valid column, stop the loop
        if new_unit_column is None:
            break

        unit_num = update_column(unit_num, new_unit_column)
        cost_plan = update_cost_plan(cost_plan, new_unit_cost)

        # Solve updated RMP
        result = solve_Rmp(unit_num, demand_num, cost_plan)
        if result is None:
            return None
        solution_x, process_RMP_optimal, dual_values = result
        optimal_gap = abs(process_RMP_optimal - reserve_RMP_optimal)

    # Final results
    return solution_x, process_RMP_optimal, iteration_results, unit_num

# 调用迭代优化函数
final_solution_x, final_RMP_optimal, iteration_results, final_unit_num = iterative_optimization(
    unit_num, demand_num, cost_plan, timber_type, cost_type, demand_length)

# 输出最终结果
if final_solution_x is not None:
    print("Final RMP Optimal Solution:", final_solution_x)
    print("Final RMP Optimal Value:", final_RMP_optimal)

    # 输出每次迭代的决策变量取值，最优值和对偶变量
    for i, result in enumerate(iteration_results):
        print(f"Iteration {i+1}:")
        print(f"  Solution x: {result['solution_x']}")
        print(f"  RMP Optimal Value: {result['RMP_optimal']}")
        print(f"  Dual Values: {result['dual_values']}")
        print(f"  Subproblems Optimal Values: {result['subproblems_optimal_values']}")

    # 输出x值>0的系数和对应的解
    for i, x_value in enumerate(final_solution_x):
        if x_value > 0:
            print(f"Column {i}: {final_unit_num[i]}, Coefficient: {x_value}")
else:
    print("No optimal solution found.")
