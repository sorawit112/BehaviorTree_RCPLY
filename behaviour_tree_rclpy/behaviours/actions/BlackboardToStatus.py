from py_trees.behaviours import BlackboardToStatus as pyBlackboardToStatus

class BlackboardToStatus(pyBlackboardToStatus):
    def __init__(self, **kwargs):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'BlackboardToStatus'
        variable_name: str = kwargs['variable_name']
        super().__init__(name, variable_name)