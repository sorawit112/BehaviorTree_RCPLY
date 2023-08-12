import typing
from py_trees import common
from py_trees.behaviours import SuccessEveryN as pySuccessEveryN

commmon_status_mapping = {
    'success' : common.Status.SUCCESS,
    'running' : common.Status.RUNNING,
    'failure' : common.Status.FAILURE    
}
 

class TickCounter(pySuccessEveryN):
    def __init__(self, name: str, n: int):
        if isinstance(duration, str):
            try:
                duration = float(duration)
            except ValueError:
                raise ValueError(f"input string {duration} can't convert to type 'Float'")
        elif isinstance(duration, (int, float)):
            duration = int(duration)
        else:
            raise TypeError(f"duration type:{type(duration)} not supported")
        
        super().__init__(name, n)