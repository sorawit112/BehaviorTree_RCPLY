import typing
from py_trees import common
from py_trees.behaviours import SuccessEveryN as pySuccessEveryN

commmon_status_mapping = {
    'success' : common.Status.SUCCESS,
    'running' : common.Status.RUNNING,
    'failure' : common.Status.FAILURE    
}
 

class SuccessEveryN(pySuccessEveryN):
    def __init__(self, attr:dict={}):
        name: str = attr['name'] if 'name' in attr.keys() else 'SuccessEveryN'
        n: typing.Union[str, int, float] = attr['n']
        if isinstance(n, str):
            try:
                n = int(n)
            except ValueError:
                raise ValueError(f"input string {n} can't convert to type 'int'")
        elif isinstance(n, (int, float)):
            n = int(n)
        else:
            raise TypeError(f"duration type:{type(n)} not supported")
        
        super().__init__(name, n)