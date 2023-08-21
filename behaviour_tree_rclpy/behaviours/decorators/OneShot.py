import typing
from py_trees.common import OneShotPolicy
from py_trees.behaviour import Behaviour
from py_trees.decorators import OneShot as pyOneShot

class OneShot(pyOneShot):
    def __init__(
        self, 
        child: Behaviour,
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'OneShot'
        only_success: typing.Union[str, bool] = kwargs['only_success']
        if isinstance(only_success, str):
            if only_success.lower() in ['true', 'false']:
                only_success = bool(only_success)
            else:
                raise ValueError(f"input string {only_success} can't convert to Bool")
        elif not isinstance(only_success, bool):
            raise TypeError(f"only_success type:{type(only_success)} not supported")
        
        if only_success:
            policy = OneShotPolicy.ON_SUCCESSFUL_COMPLETION
        else:
            policy = OneShotPolicy.ON_COMPLETION
            
        super().__init__(name, child, policy)