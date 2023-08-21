import typing
from py_trees.common import Status
from py_trees.behaviour import Behaviour
from py_trees.decorators import Condition as pyCondition

commmon_status_mapping = {
    'success' : Status.SUCCESS,
    'running' : Status.RUNNING,
    'failure' : Status.FAILURE    
}

class Condition(pyCondition):
    def __init__(
        self, 
        child: Behaviour,
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'Condition'
        status: typing.Union[str, Status] = kwargs['status']
        if isinstance(status, str):    
            try:
                status = commmon_status_mapping[status.lower().replace(' ', '')]
            except KeyError as e:
                raise KeyError(f"{e} can't remapping key:{status} to {list(commmon_status_mapping.values())}")
        elif not isinstance(status, Status):
            raise TypeError(f"eventually type:{type(status)} not supported")
        
        super().__init__(name, child, status)