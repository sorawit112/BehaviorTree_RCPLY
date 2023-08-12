import typing
from py_trees.behaviour import Behaviour
from py_trees.decorators import Timeout as pyTimeout


class Timeout(pyTimeout):
    def __init__(
        self, 
        name: str, 
        child: Behaviour, 
        msec: typing.Union[str, int],
    ):
        """
        Wrapper of py_trees.decorators.Timeout with use argument from groot

        Args:
            child: the child behaviour or subtree
            name: the decorator name
            msec: timeout length in milliseconds
        """
        
        if isinstance(msec, str):
            try:
                msec = float(msec)/1000.0
            except ValueError:
                raise ValueError(f"input string {msec} can't convert to type 'Float'")
        elif isinstance(msec, int):
            msec = float(msec)/1000.0
        elif not isinstance(msec, float):
            raise TypeError(f"msec type:{type(msec)} not supported")
        
        super().__init__(name, child, msec)