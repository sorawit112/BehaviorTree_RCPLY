import typing
from py_trees.behaviour import Behaviour
from py_trees.decorators import Repeat as pyRepeat


class Repeat(pyRepeat):
    def __init__(
        self, 
        name: str, 
        child: Behaviour, 
        num_cycles: typing.Union[str, int]
    ):
        if isinstance(num_cycles, str):
            try:
                num_cycles = int(num_cycles)
            except ValueError:
                raise ValueError(f"input string {num_cycles} can't convert to type 'int'")
        elif isinstance(num_cycles, (int, float)):
            num_cycles = int(num_cycles)
        else:
            raise TypeError(f"num_cycles type:{type(num_cycles)} not supported")
        
        super().__init__(name, child, num_cycles)