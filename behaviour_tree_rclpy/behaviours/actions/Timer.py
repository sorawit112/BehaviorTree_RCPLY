import typing
from py_trees import common
from py_trees.timers import Timer as pyTimer

commmon_status_mapping = {
    'success' : common.Status.SUCCESS,
    'running' : common.Status.RUNNING,
    'failure' : common.Status.FAILURE    
}
 

class Timer(pyTimer):
    def __init__(self, attr:dict={}):
        name: str = attr['name'] if 'name' in attr.keys() else 'Timer'
        duration: typing.Union[str, float] = attr['duration']
        if isinstance(duration, str):
            try:
                duration = float(duration)
            except ValueError:
                raise ValueError(f"input string {duration} can't convert to type 'Float'")
        elif isinstance(duration, int):
            duration = float(duration)
        elif not isinstance(duration, float):
            raise TypeError(f"duration type:{type(duration)} not supported")
        
        super().__init__(name, duration)