import os,math
import tkinter as tk
from tkinter import filedialog
from itertools import product
import docplex.mp.model as mpx
import docplex.mp.context as cpx

class Customer:
    def __init__(self,customer_id,customer_xcoord,customer_ycoord,customer_demand,customer_ready_time,customer_due_date,customer_service_time):
        self.id=customer_id
        self.coordinates={"x":customer_xcoord,"y":customer_ycoord}
        self.demand=customer_demand
        self.ready_time=customer_ready_time
        self.due_date=customer_due_date
        self.service_time=customer_service_time
    
    def distance(self,other_customer):
        return math.sqrt(math.pow(other_customer.coordinates['x']-self.coordinates['x'], 2) + math.pow(other_customer.coordinates['y']-self.coordinates['y'], 2))
    def __str__(self):
        return f"{self.id},({self.coordinates['x']},{self.coordinates['y']}),{self.demand},{self.ready_time},{self.due_date},{self.service_time}"

class Problem:
    path_to_datasets = os.path.join("..", "Datasets")

    @staticmethod
    def change_path_to_datasets(ui=True,**args):
        if ui:
            root=tk.Tk()
            root.withdraw()
            folder_path=filedialog.askdirectory()
            if folder_path:
                Problem.path_to_datasets=folder_path
        else:
            if 'path' not in args:
                raise ValueError('Ui arg setted to false and no path value provided')

            folder_path=args['path']

    def __init__(self,dataset_name,skiprows=0):
        self.customers=list()
        self.vehicles=None
        self.capacity=None
        self.depot=None

        with open(os.path.join(self.path_to_datasets,dataset_name),'r') as reader:
            for row_counter,line in enumerate(reader):
                if row_counter==4:
                    self.vehicles, self.capacity = [int(x) for x in line.strip().split()]

                if row_counter<=skiprows:
                    continue
                
                if line.strip()=="": 
                    continue
                
                data = line.strip().split()
                
                if len(data)!=7: 
                    continue
                
                self.customers.append(Customer(int(data[0]),int(data[1]),int(data[2]),int(data[3]),int(data[4]),int(data[5]),int(data[6])))

        self.depot = self.customers[0]
        self.customers.append(self.depot)

        self.travel_time={i:{j:self.customers[i].distance(self.customers[j])+self.customers[i].service_time if i!=j else 0 for j in range(self.no_customers())} for i in range(self.no_customers())}

    def no_customers(self):
        return len(self.customers)
    
    def statistics(self):        
        avg_demand = 0.0
        avg_service_time = 0.0
        avg_time_window = 0.0
        for customer in self.customers:
            avg_demand += customer.demand
            avg_service_time += customer.service_time
            avg_time_window += (customer.due_date - customer.ready_time)
        avg_demand /= self.no_customers()
        avg_service_time /= self.no_customers()
        avg_time_window /= self.no_customers()     
        
        stdev_demand = 0.0
        stdev_service_time = 0.0
        stdev_time_window = 0.0
        temp_sum_demand = 0.0
        temp_sum_service_time = 0.0
        temp_sum_time_window = 0.0
        for customer in self.customers:
            temp_sum_demand += math.pow(customer.demand - avg_demand, 2)
            temp_sum_service_time += math.pow(customer.service_time - avg_service_time, 2)
            temp_sum_time_window += math.pow((customer.due_date - customer.ready_time) - avg_time_window, 2)
        stdev_demand += math.sqrt(temp_sum_demand, self.no_customers())   
        stdev_service_time += math.sqrt(temp_sum_service_time, self.no_customers())
        stdev_time_window += math.sqrt(temp_sum_time_window, self.no_customers())
    
def solve_vrptw_cplex(problem:Problem,timelimit):
    context = cpx.Context.make_default_context()
    context.cplex_parameters.threads = 4
    model = mpx.Model(name = 'VRPModel', context=context)
    
    C = range(1,problem.no_customers()-1)
    xvars = {(i,j,v):model.binary_var(name = f'c{str(i)}_c{str(j)}_v{str(v)}') for i in range(problem.no_customers()) for j in range(problem.no_customers()) for v in range(problem.vehicles)}
    service_time={(i,v):model.integer_var(lb=problem.customers[i].ready_time,ub=problem.customers[i].due_date) for i in range(problem.no_customers()) for v in range(problem.vehicles)}

    # 1. In each arc only a route should be applied
    for i in C:
        model.add(
            sum([
                xvars[(i,j,v)]
                for j in range(problem.no_customers())
                for v in range(problem.vehicles)
            ])==1
        )
    
    # 2. No customer can loop into itself
    for i in C:
        model.add(
            sum([
                xvars[(i,i,v)]
                for v in range(problem.vehicles)
            ])==0
        )

    # Depot and termination node constraints
    model.add(
        sum([
            xvars[(i,0,v)] 
            for i in range(problem.no_customers()) 
            for v in range(problem.vehicles)
            ])==0
        )
    
    model.add(
        sum([
            xvars[(problem.no_customers()-1,j,v)] 
            for j in range(problem.no_customers()) 
            for v in range(problem.vehicles)
            ])==0
        )

    # 3. Capacity constraint
    for v in range(problem.vehicles):
        model.add(  ## Addition 1/9/2023
            sum([
                xvars[(i,j,v)] * problem.customers[i].demand
                for i in range(problem.no_customers())
                for j in range(problem.no_customers())
            ])<=problem.capacity
        )
        
    # 4. All vehicles must start from depot
    for v in range(problem.vehicles):
        model.add(
            sum([
                xvars[(0,j,v)]
                for j in range(problem.no_customers())
            ])==1
        )
    
    # 5. Incoming and Outcoming vertices
    for v in range(problem.vehicles):
        for cid in C:
            model.add(
                sum([
                    xvars[(i,cid,v)]
                    for i in range(problem.no_customers())
                ])-sum([
                    xvars[(cid,j,v)]
                    for j in range(problem.no_customers())
                ])==0
            )

    # 6. Every vehicle should visit the arrival node
    for v in range(problem.vehicles):
        model.add(
            sum([
                xvars[(i,problem.no_customers()-1,v)]
                for i in range(problem.no_customers())
            ])==1
        )
    
    # 7. Time window constraint
    for i in range(problem.no_customers()):
        for j in range(problem.no_customers()):
            for v in range(problem.vehicles):
                model.add(
                    # service_time[(i,v)]+problem.travel_time[i][j]-K*(1-xvars[(i,j,v)])<=service_time[(j,v)]
                    xvars[(i,j,v)] * (service_time[(i,v)] + problem.travel_time[i][j] - service_time[(j,v)]) <= 0
                )

    # 8. Objective
    objective=sum([
        problem.customers[i].distance(problem.customers[j]) * xvars[(i,j,v)]
        for i in range(problem.no_customers())
        for j in range(problem.no_customers())
        for v in range(problem.vehicles)
    ])

    model.minimize(objective)
    model.print_information()
    model.parameters.timelimit = timelimit    
    solution_model = model.solve(log_output=True)
    
    solution={}
    if solution_model:
        for (i,j,v) in list(product(range(problem.no_customers()),range(problem.no_customers()),range(problem.vehicles))):
            if model.solution.get_value(xvars[(i,j,v)])==1:
                solution[(i,j)]=v
                
    return solution,model.objective_value

if __name__=='__main__':
    problem=Problem('c101.txt')
    
    print(solve_vrptw_cplex(problem,60))