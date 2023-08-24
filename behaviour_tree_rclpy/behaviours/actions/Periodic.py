import typing
from py_trees.behaviours import Periodic as pyPeriodic

class Periodic(pyPeriodic):
    def __init__(self, attr:dict={}):
        name: str = attr['name'] if 'name' in attr.keys() else 'Periodic'
        n: typing.Union[int, str] = attr['n']
        
        if isinstance(n, str):
            try:
                n = int(n)
            except ValueError:
                raise ValueError(f"input string {n} can't convert to type int")
        elif not isinstance(n, int):
            raise TypeError(f"n type:{type(n)} not supported")
        super().__init__(name, n)