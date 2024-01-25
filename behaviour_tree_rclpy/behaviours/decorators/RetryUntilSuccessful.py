import typing
from py_trees.behaviour import Behaviour
from py_trees.decorators import Retry

class RetryUntilSuccessful(Retry):
    def __init__(
        self, 
        child: Behaviour,
        attr:dict={},
    ):
        """
        The Default groot := RetryUntilSuccessful
        
        Wrapper of py_trees.decorators.Retry with use argument num_attemps from groot as num_failures

        Args:
            child: the child behaviour or subtree
            name: the decorator name
            num_attemps: maximum retry or will return FAILURE
        """
        name: str = attr['name'] if 'name' in attr.keys() else 'RetryUntilSuccessful'
        num_attempts: typing.Union[str, int] = attr['num_attempts']
        if isinstance(num_attempts, str):
            try:
                num_attempts = int(num_attempts)
            except ValueError:
                raise ValueError(f"input string {num_attempts} can't convert to type 'int'")
        elif isinstance(num_attempts, (int, float)):
            num_attempts = int(num_attempts)
        else:
            raise TypeError(f"num_attemps type:{type(num_attempts)} not supported")
        
        super().__init__(name, child, num_attempts)