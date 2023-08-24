
from py_trees.common import Status
from py_trees.behaviour import Behaviour
from py_trees.decorators import Condition

class KeepRunningUntilFailure(Condition):
    """
    def __init__(
        self, 
        name: str, 
        child: Behaviour, 
    ):sulates a behaviour and wait for child FAILURE
    This behaviour will tick with
    :data:`~py_trees.common.Status.RUNNING` while waiting and
    :data:`~py_trees.common.Status.SUCCESS` when child return FAILURE
    """

    def __init__(
        self, 
        child: Behaviour,
        attr:dict={},
    ):
        """
        The Default groot := KeepRunningUntilFailure
        wrapper of py_trees.decorators.Condition which set status as FAILURE

        Args:
            name: the decorator name
            child: the child to be decorated
        """
        name: str = attr['name'] if 'name' in attr.keys() else 'KeepRunningUntilFailure'
        super().__init__(name, child, Status.FAILURE)