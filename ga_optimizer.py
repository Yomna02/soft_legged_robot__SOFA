import numpy as np
import pandas as pd
from deap import base, creator, tools, algorithms 
from leg_simulate import run_simulation

# Load the CSV file
actual_dataset = pd.read_csv('/home/youmna/Documents/Soft Robot Grad Project/Soft Legged Robot - SOFA/leg_data.csv')

actual_input_disp = actual_dataset.iloc[:, :3]  # Selects all rows and the first three columns (l1, l2, l3)
actual_output_points = actual_dataset.iloc[:, -3:]  # Selects all rows and the last three columns (x, y, z)

print("Inputs (l1, l2, l3):\n", actual_input_disp.shape)
print("Outputs (x, y, z):\n", actual_output_points.shape)

num_rows = min(len(actual_input_disp), 1002)
for i in range(num_rows):
    print(actual_input_disp.iloc[i].tolist())
    print(actual_output_points.iloc[i].tolist())

# Problem definition
def fitness_function(individual):
    poisson_ratio, youngs_modulus = individual
    sim_output_points = []
    num_rows = min(len(actual_input_disp), 1000)

    for i in range(num_rows):
        sim_point = run_simulation(youngs_modulus, poisson_ratio, actual_input_disp.iloc[i].tolist())
        sim_output_points.append(sim_point)
    
    rms_error = calculate_rmse(sim_output_points, actual_output_points)
    return (rms_error,)

def calculate_rmse(points_a, points_b):
    points_a = np.array(points_a)
    points_b = np.array(points_b)
    
    if points_a.shape != points_b.shape:
        raise ValueError("Both lists must have the same shape and each point must have three coordinates (X, Y, Z)")
    
    squared_errors = np.square(points_a - points_b)
    mean_squared_errors = np.mean(np.sum(squared_errors, axis=1))
    rmse_value = np.sqrt(mean_squared_errors)
    return rmse_value

# Define genetic algorithm parameters
# creator.create("FitnessMin", base.Fitness, weights=(-1.0,))  # Minimize error
# creator.create("Individual", list, fitness=creator.FitnessMin)
# toolbox = base.Toolbox()
# toolbox.register("attr_float", np.random.uniform, 0.1, 100.0)  # Define range
# toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, n=2)
# toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# # GA operators
# toolbox.register("evaluate", fitness_function)
# toolbox.register("select", tools.selTournament, tournsize=3)
# toolbox.register("mate", tools.cxBlend, alpha=0.5)
# toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.2)

# # Run GA
# def main():
#     population = toolbox.population(n=50)
#     ngen, cxpb, mutpb = 100, 0.7, 0.2
#     algorithms.eaSimple(population, toolbox, cxpb, mutpb, ngen, verbose=True)
#     best = tools.selBest(population, k=1)[0]
#     return best

# best_params = main()
# print("Optimized Parameters:", best_params)
