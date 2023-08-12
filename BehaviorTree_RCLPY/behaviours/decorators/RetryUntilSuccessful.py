import typing
from py_trees.behaviour import Behaviour
from py_trees.decorators import Retry

class RetryUntilSuccessful(Retry):
    def __init__(
        self, 
        name: str, 
        child: Behaviour, 
        num_attemps: typing.Union[str, int]
    ):
        """
        The Default groot := RetryUntilSuccessful
        
        Wrapper of py_trees.decorators.Retry with use argument num_attemps from groot as num_failures

        Args:
            child: the child behaviour or subtree
            name: the decorator name
            num_attemps: maximum retry or will return FAILURE
        """
        
        if isinstance(num_attemps, str):
            try:
                num_attemps = int(num_attemps)
            except ValueError:
                raise ValueError(f"input string {num_attemps} can't convert to type 'int'")
        elif isinstance(num_attemps, (int, float)):
            num_attemps = int(num_attemps)
        else:
            raise TypeError(f"num_attemps type:{type(num_attemps)} not supported")
        
        super().__init__(name, child, num_attemps)