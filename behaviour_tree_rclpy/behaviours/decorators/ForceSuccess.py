from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.decorators import Decorator


class ForceSuccess(Decorator):
    """any Status to SUCCESS"""
    
    def __init__(
        self, 
        child: Behaviour,
        attr:dict={},
    ):
        name: str = attr['name'] if 'name' in attr.keys() else 'ForceSuccess'
        super().__init__(name, child)

    def update(self) -> Status:
        """
        Reflect :data:`~py_trees.common.Status.*` as :data:`~py_trees.common.Status.SUCCESS`.

        Returns:
            the behaviour's new status :class:`~py_trees.common.Status`
        """
        
        self.feedback_message = "force success" + (
            " [%s]" % self.decorated.feedback_message
            if self.decorated.feedback_message
            else ""
        )
        return Status.SUCCESS