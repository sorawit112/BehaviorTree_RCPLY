import typing
from py_trees import common
from py_trees.behaviour import Behaviour
from py_trees.composites import Composite

class Parallel(Composite):
    """
    Parallels enable a kind of spooky at-a-distance concurrency.

    .. graphviz:: dot/parallel.dot

    A parallel ticks every child every time the parallel is itself ticked.
    The parallelism however, is merely conceptual. The children have actually been
    sequentially ticked, but from both the tree and the parallel's purview, all
    children have been ticked at once.
    """

    def __init__(
        self, 
        children: typing.Optional[typing.List[Behaviour]],
        **kwargs,
    ):
        """
        Initialise the behaviour with name, policy and a list of children.

        Args:
            name: the composite behaviour name
            failure_threshold: minimum child failure to return FAILURE
            success_threshold: minimum child success to return SUCCESS
            children: list of children to add
        """
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'Parallel'
        failure_threshold: typing.Union[str, int] = kwargs['failure_threshold']
        success_threshold: typing.Union[str, int] = kwargs['success_threshold']
        
        if isinstance(failure_threshold, str):
            try:
                failure_threshold = int(failure_threshold)
            except ValueError:
                raise ValueError(f"input string {failure_threshold} can't convert to type 'int'")
        elif isinstance(failure_threshold, (int, float)):
            failure_threshold = int(failure_threshold)
        else:
            raise TypeError(f"duration type:{type(failure_threshold)} not supported")
        
        if isinstance(success_threshold, str):
            try:
                success_threshold = int(success_threshold)
            except ValueError:
                raise ValueError(f"input string {success_threshold} can't convert to type 'int'")
        elif isinstance(success_threshold, (int, float)):
            success_threshold = int(success_threshold)
        else:
            raise TypeError(f"duration type:{type(success_threshold)} not supported")
        
        super(Parallel, self).__init__(name, children)
        
        self.failure_threshold = failure_threshold
        self.success_threshold = success_threshold

    def tick(self) -> typing.Iterator[Behaviour]:
        """
        Tick over the children.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children

        Raises:
            RuntimeError: if the policy configuration was invalid
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)

        # reset
        if self.status != common.Status.RUNNING:
            self.logger.debug("%s.tick(): re-initialising" % self.__class__.__name__)
            for child in self.children:
                # reset the children, this ensures old SUCCESS/FAILURE status flags
                # don't break the synchronisation logic below
                if child.status != common.Status.INVALID:
                    child.stop(common.Status.INVALID)
            self.current_child = None
            # subclass (user) handling
            self.initialise()

        # nothing to do
        if not self.children:
            self.current_child = None
            self.stop(common.Status.SUCCESS)
            yield self
            return

        # process them all first
        for child in self.children:                
            for node in child.tick():
                yield node
        
        failed_count = 0
        success_count = 0
        
        for child in self.children:
            if child.status == common.Status.SUCCESS:
                success_count += 1
            elif child.status == common.Status.FAILURE:
                failed_count += 1

        if failed_count >= self.failure_threshold:
           new_status = common.Status.FAILURE
        elif success_count >= self.success_threshold:
            new_status = common.Status.SUCCESS
        else:
            new_status = common.Status.RUNNING
            
        self.current_child = self.children[-1]
        if new_status != common.Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def stop(self, new_status: common.Status = common.Status.INVALID) -> None:
        """
        Ensure that any running children are stopped.

        Args:
            new_status : the composite is transitioning to this new status
        """
        self.logger.debug(
            f"{self.__class__.__name__}.stop()[{self.status}->{new_status}]"
        )

        # clean up dangling (running) children
        for child in self.children:
            if child.status == common.Status.RUNNING:
                # this unfortunately knocks out it's running status for introspection
                # but logically is the correct thing to do, see #132.
                child.stop(common.Status.INVALID)
        Composite.stop(self, new_status)