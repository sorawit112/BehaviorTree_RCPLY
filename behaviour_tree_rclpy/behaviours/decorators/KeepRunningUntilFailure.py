
from py_trees.common import Status
from py_trees.behaviour import Behaviour
from py_trees.decorators import Condition

class KeepRunningUntilFailure(Condition):
    """
    Encadef __init__(
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
        **kwargs,
    ):
        """
        The Default groot := KeepRunningUntilFailure
        wrapper of py_trees.decorators.Condition which set status as FAILURE

        Args:
            name: the decorator name
            child: the child to be decorated
        """
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'KeepRunningUntilFailure'
        super().__init__(name, child, Status.FAILURE)