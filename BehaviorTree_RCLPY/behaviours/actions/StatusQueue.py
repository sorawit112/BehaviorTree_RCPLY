import typing
from py_trees import common
from py_trees.behaviours import StatusQueue as pyStatusQueue

commmon_status_mapping = {
    'success' : common.Status.SUCCESS,
    'running' : common.Status.RUNNING,
    'failure' : common.Status.FAILURE    
}
 

class StatusQueue(pyStatusQueue):
    def __init__(self,
        name: str,
        queue: typing.Union[str, typing.List[common.Status]],
        eventually: typing.Union[str, common.Status, None],
    ):
        if isinstance(queue, str):
            split_queue = queue.split(',')
            queue = []
            
            for status_str in split_queue:
                status_str = status_str.lower().replace(' ', '')
                if status_str in commmon_status_mapping:
                    queue.append(commmon_status_mapping[status_str])
                else:
                    raise ValueError(f"{status_str} not in {list(commmon_status_mapping.keys())}")
        elif isinstance(queue, list):
            if all([isinstance(q, common.Status) for q in queue]):
                raise ValueError(f"some object in queue is not type:{type(common.Status)}")
        else:
            raise TypeError(f"queue type:{type(queue)} not supported")
        
        if eventually:
            if isinstance(eventually, str):    
                try:
                    eventually = commmon_status_mapping[eventually.lower().replace(' ', '')]
                except KeyError as e:
                    raise KeyError(f"{e} can't remapping key:{eventually} to {list(commmon_status_mapping.values())}")
            elif not isinstance(eventually, common.Status):
                raise TypeError(f"eventually type:{type(eventually)} not supported")
        else:
            eventually = None
            
        super().__init__(name, queue, eventually)