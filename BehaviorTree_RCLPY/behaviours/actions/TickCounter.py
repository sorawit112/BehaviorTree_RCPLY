import typing
from py_trees import common
from py_trees.behaviours import TickCounter as pyTickCounter

commmon_status_mapping = {
    'success' : common.Status.SUCCESS,
    'running' : common.Status.RUNNING,
    'failure' : common.Status.FAILURE    
}
 

class TickCounter(pyTickCounter):
    def __init__(
        self, 
        name: str, 
        duration: typing.Union[str, int], 
        completion_status: typing.Union[str, common.Status]
    ):
        if isinstance(duration, str):
            try:
                duration = int(duration)
            except ValueError:
                raise ValueError(f"input string {duration} can't convert to type 'int'")
        elif isinstance(duration, (int, float)):
            duration = int(duration)
        else:
            raise TypeError(f"duration type:{type(duration)} not supported")
        
        if isinstance(completion_status, str):    
            try:
                completion_status = commmon_status_mapping[completion_status.lower().replace(' ', '')]
            except KeyError as e:
                raise KeyError(f"{e} can't remapping key:{completion_status} to {list(commmon_status_mapping.values())}")
        elif not isinstance(completion_status, common.Status):
            raise TypeError(f"eventually type:{type(completion_status)} not supported")
        
        super().__init__(name, duration, completion_status)