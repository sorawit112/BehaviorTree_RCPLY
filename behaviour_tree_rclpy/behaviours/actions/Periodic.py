import typing
from py_trees.behaviours import Periodic as pyPeriodic

class Periodic(pyPeriodic):
    def __init__(self, **kwargs):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'Periodic'
        n: typing.Union[int, str] = kwargs['n']
        
        if isinstance(n, str):
            try:
                n = int(n)
            except ValueError:
                raise ValueError(f"input string {n} can't convert to type int")
        elif not isinstance(n, int):
            raise TypeError(f"n type:{type(n)} not supported")
        super().__init__(name, n)