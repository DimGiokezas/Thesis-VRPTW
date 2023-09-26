import os,math
from ortools.constraint_solver import pywrapcp,routing_enums_pb2
from tqdm import tqdm
import logging
from time import time
import json 
from pydantic import BaseModel
from typing import List

class Customer(BaseModel):
    """Represents the Customer entity used for the VRPTW"""
    id: int
    x: int
    y: int 
    demand: int 
    ready_time: int 
    due_date: int
    service_time: int 

class Vehicle(BaseModel):
    """Represents the Vehicle entity used for the VRPTW"""
    id: int 
    capacity: int

class Problem(BaseModel):
    """Represents the Problem used for the VRPTW """
    vehicles: List[Vehicle]
    customers: List[Customer]
    id: str # Instance name
    depot: int = 0
                
class Solution:
    """Generates the solution of the problem"""
    def __init__(self, problem:Problem, fsolution, model, manager):
        """Initiates the Solution class"""
        self.problem = problem
        self.fsolution = fsolution
        self.model = model
        self.manager = manager
        self.routes = list()
        self.total_driving_time = 0
        self.total_driving_service_time = 0
        logging.basicConfig(filename=f'{os.path.join("..", "Solutions", "logs",self.problem.id.replace(".txt", ".log"))}', filemode='w', format='%(asctime)s - %(message)s', level=logging.DEBUG)
        
    def print_solution(self):
        """Prints the solution of the problem to the console"""
        print(f"Objective: {self.fsolution.ObjectiveValue()}")
        time_dimension = self.model.GetDimensionOrDie('Time')
        for vehicle_id in range(len(self.problem.vehicles)):
            index = self.model.Start(vehicle_id)
            plan_output = f"Route for vehicle {vehicle_id}:\n"
            route_load = 0
            while not self.model.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
                plan_output += f'{self.manager.IndexToNode(index)}(Load({self.problem.customers[self.manager.IndexToNode(index)].demand})) -> '
                route_load += self.problem.customers[self.manager.IndexToNode(index)].demand
                index = self.fsolution.Value(self.model.NextVar(index))
            time_var = time_dimension.CumulVar(index)
            plan_output += f'{self.manager.IndexToNode(index)}\n'
            plan_output += f'Time of the route: {self.fsolution.Min(time_var)} seconds\n'
            plan_output += f'Load of the route: {route_load}\n'
            print(plan_output)
            self.total_driving_time += self.fsolution.Min(time_var)
        
        print('Total time of all routes: {0:.3f} seconds'.format(self.total_driving_time))   
        print(f"Cost evaluation: {self.evaluate_cost()}")
        
    def get_routes(self):
        """Get vehicle routes from a solution and store them in an list of lists"""
        for route_nbr in range(self.model.vehicles()):
            index = self.model.Start(route_nbr)
            route = [self.manager.IndexToNode(index)]
            while not self.model.IsEnd(index):
                index = self.fsolution.Value(self.model.NextVar(index))
                route.append(self.manager.IndexToNode(index))
            self.routes.append(route)
        return self.routes
    
    def evaluate_cost(self):
        """ Computes the cost of all the deliveries.
            Considers driving time to each client from depot and back, and service time of each client."""
        driving_time = 0
        service_time = 0

        time_dimension = self.model.GetDimensionOrDie('Time')
        for vehicle_id in range(len(self.problem.vehicles)):
            index = self.model.Start(vehicle_id)
            while not self.model.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
                index = self.fsolution.Value(self.model.NextVar(index))
                driving_time += self.fsolution.Min(time_var)
                service_time += self.problem.customers[self.manager.IndexToNode(index)].service_time
            time_var = time_dimension.CumulVar(index)
            driving_time += self.fsolution.Min(time_var)        
        self.total_driving_service_time = driving_time + service_time

        return self.total_driving_service_time
    
    def feasible(self):
        """ Check if the total capacity of the route is less or equal of the capacity of the vehicle.
            Check for the time of the route.
            Check for the time windows of the visited customers.
        """
        logging.info(f"~~~~~~ Instance: {str(self.problem.id)} ~~~~~")

        # Check for the capacity of each route 
        for vehicle_id in range(self.model.vehicles()):
            capacity_of_route = 0
            index = self.model.Start(vehicle_id)
            logging.info(f"Starting capacity check for route {vehicle_id}")
            while not self.model.IsEnd(index):
                capacity_of_route += self.problem.customers[self.manager.IndexToNode(index)].demand
                index = self.fsolution.Value(self.model.NextVar(index))

            if capacity_of_route <= self.problem.vehicles[0].capacity:
                logging.info(f"Capacity check for route {str(vehicle_id)} --> OK")
            else:
                logging.warning(f"Capacity check for route {str(vehicle_id)} --> NOT OK")
                return False
        
        # Check for the time windows of clients of each route
        time_dimension = self.model.GetDimensionOrDie('Time')
        for vehicle_id in range(len(self.problem.vehicles)):
            index = self.model.Start(vehicle_id)
            logging.info(f"Starting time window checks for route {vehicle_id}")
            while not self.model.IsEnd(index):
                if self.manager.IndexToNode(index) == self.problem.depot:
                    index = self.fsolution.Value(self.model.NextVar(index))
                else:
                    time_var = time_dimension.CumulVar(index)
                    time_window_arrive = self.fsolution.Min(time_var)

                    if self.problem.customers[self.manager.IndexToNode(index)].ready_time <= time_window_arrive <= self.problem.customers[self.manager.IndexToNode(index)].due_date:
                        logging.info(f"\tTime window of client {str(self.manager.IndexToNode(index))} met --> OK ")
                    else:
                        logging.warning(f"\tTime window of client {str(self.manager.IndexToNode(index-1))} not met --> NOT OK")
                        return False
                    index = self.fsolution.Value(self.model.NextVar(index))
            logging.info(f"Time window checks for route {vehicle_id} met --> OK")
        logging.info(f"~~ Cost of solution: {self.evaluate_cost()}")
        logging.info("\n")
        
        return True
    
    def save_to_file(self):
        """Saves the routes of the solution to a file"""
        data = {
            'routes': [],
            'cost': int
        }
        
        with open(os.path.join('..', 'Solutions', 'sols', self.problem.id.replace(".txt", ".json")), 'w') as f:
            for vehicle_id, customers in enumerate(self.routes):
                route = {
                    'vehicle': vehicle_id,
                    'route': customers
                }
                data['routes'].append(route)   
            data['cost'] = self.evaluate_cost()
            
            json.dump(data, f, indent=4)

class Callback:
    def __init__(self, data:Problem, mn): 
        self.data = data
        self.manager = mn

    def time_callback(self, from_index, to_index):
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        return self.data.G[from_node][to_node]['weight']
    
    def demand_callback(self, from_index):
        from_node = self.manager.IndexToNode(from_index)
        return self.data.customers[from_node].demand

def  vehicle_routing_solver(data:Problem):
    manager = pywrapcp.RoutingIndexManager(len(data.customers), len(data.vehicles), data.depot)
    model = pywrapcp.RoutingModel(manager)
    cb = Callback(data, manager)
    
    transit_callback_index = model.RegisterTransitCallback(cb.time_callback)
    demand_callback_index = model.RegisterUnaryTransitCallback(cb.demand_callback)

    model.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    model.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,
        [int(data.vehicles[i].capacity) for i in range(len(data.vehicles))],
        True,
        'Demand'
    )
    model.AddDimension(
        transit_callback_index,
        math.ceil(data.customers[data.depot].ready_time/2)+100000, 
        math.ceil(data.customers[data.depot].ready_time/2)+100000, 
        False,
        'Time'
    )
    
    # Add time-window constraint
    time_dimension = model.GetDimensionOrDie('Time')
    for customer_id in range(len(data.customers)):
        index = manager.NodeToIndex(customer_id)
        if customer_id == data.depot:
            time_dimension.SlackVar(index).SetValue(data.customers[data.depot].service_time)
            continue
        time_dimension.CumulVar(index).SetRange(data.customers[customer_id].ready_time, data.customers[customer_id].due_date)
        time_dimension.SlackVar(index).SetValue(data.customers[customer_id].service_time)
    
    # Add constraint from depot 
    for vehicle_id in range(len(data.vehicles)):
        index = model.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data.customers[data.depot].ready_time, data.customers[data.depot].due_date)
    
    for vehicle_id in range(len(data.vehicles)):
        model.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(model.Start(vehicle_id)))
        model.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(model.End(vehicle_id)))
        
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.time_limit.seconds = 300
    solution = model.SolveWithParameters(search_parameters)
    
    if solution:
        sol = Solution(data, solution, model, manager)
        sol.feasible()
        sol.print_solution()
        _ = sol.get_routes()
        sol.save_to_file()
    else:
        print("Solution failed to be constructed")

def solomon_to_json(input_file):
    """Function to convert dataset instance to json format for handling by Pydantic class objects"""
    data = {
        'vehicles': [],
        'customers': []
    }
    
    with open(input_file, 'r') as f:
        lines = f.readlines()
        vehicle_count, vehicle_capacity = lines[4].split()

        for line in lines[9:]:
            parts = line.split()
            customer = {
                'id': int(parts[0]),
                'x': int(parts[1]),
                'y': int(parts[2]),
                'demand': int(parts[3]),
                'ready_time': int(parts[4]),
                'due_date': int(parts[5]),
                'service_time': int(parts[6])
            }
            data['customers'].append(customer)
        
        for i in range(int(vehicle_count)):
            vehicle = {
                'id': i,
                'capacity': int(vehicle_capacity)
            }
            data['vehicles'].append(vehicle)
    
    output_file = input_file.replace('.txt', '.json')
    with open(output_file, 'w') as f:
        json.dump(data, f, indent=4)
    
    return output_file

path_to_datasets = os.path.join("..", "Datasets")

def scenario1():
    time_start = time()
    
    for instance in tqdm(os.listdir(path_to_datasets)):
        if instance.endswith(".txt"):
            dataset = os.path.join(path_to_datasets, instance)
            dataset_json = solomon_to_json(dataset)

            with open(dataset_json) as f:
                data = json.load(f)
            problem = Problem(vehicles=data['vehicles'], customers=data['customers'], id=dataset.split(f'{os.sep}')[-1])
            vehicle_routing_solver(problem)
    
    time_end = time()
    print(f"Time elapsed: {str(time_end - time_start)}")

def scenario2(instance = "C101.txt"):
    dataset = os.path.join(path_to_datasets, instance)
    dataset_json = solomon_to_json(dataset)

    with open(dataset_json) as f:
        data = json.load(f)
    problem = Problem(vehicles=data['vehicles'], customers=data['customers'], id=dataset.split(f'{os.sep}')[-1])
    vehicle_routing_solver(problem)
        
if __name__ == "__main__":
    # scenario1()
    scenario2()
    