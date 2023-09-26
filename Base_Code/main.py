from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Dict
from ortools_solver import scenario2
import json 
import os 

app = FastAPI()

class SolutionOutput(BaseModel): 
    routes: List[Dict[str, int | List[int]]]
    cost: int
    
path_to_solutions = os.path.join("..", "Solutions", "sols")

@app.get("/solve/{instance}")
async def solve_problem(instance: str):
    try:        
        scenario2(instance)
        print(instance)
        solution_file = os.path.join(path_to_solutions, instance.replace('.txt', '.json'))
        with open(solution_file) as f:
            solution_data = json.load(f)
        
        return SolutionOutput(routes=solution_data['routes'], cost=solution_data['cost'])
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
