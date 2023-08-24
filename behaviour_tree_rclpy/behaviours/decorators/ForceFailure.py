from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.decorators import Decorator


class ForceFailure(Decorator):
    """any Status to FAILURE"""
    
    def __init__(
        self, 
        child: Behaviour,
        attr:dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'ForceFailure'
        super(ForceFailure, self).__init__(name, child)

    def update(self) -> Status:
        """
        Reflect :data:`~py_trees.common.Status.*` as :data:`~py_trees.common.Status.FAILURE`.

        Returns:
            the behaviour's new status :class:`~py_trees.common.Status`
        """
        
        self.feedback_message = "force failure" + (
            " [%s]" % self.decorated.feedback_message
            if self.decorated.feedback_message
            else ""
        )
        return Status.FAILURE