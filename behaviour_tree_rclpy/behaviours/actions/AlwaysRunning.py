from py_trees.behaviour import Behaviour
from py_trees.common import Status

class AlwaysRunning(Behaviour):
    def __init__(self, **kwargs):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'AlwaysRunning'
        super(AlwaysRunning, self).__init__(name)

    def update(self) -> Status:
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.logger.debug(
            "%s.terminate(%s->%s)" % (self.__class__.__name__, self.status, new_status)
        )