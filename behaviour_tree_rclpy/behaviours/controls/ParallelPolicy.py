import typing
from py_trees import common
from py_trees.behaviour import Behaviour
from py_trees.composites import Parallel

policy_mapping = {
    'SuccessOnOne' : common.ParallelPolicy.SuccessOnOne,
    'SuccessOnAll' : common.ParallelPolicy.SuccessOnAll,
    'SuccessOnSelected' : common.ParallelPolicy.SuccessOnSelected,
}

class ParallelPolicy(Parallel):
    def __init__(
        self, 
        children: typing.Optional[typing.List[Behaviour]],
        **kwargs,
    ):
        kwargs = kwargs['kwargs']
        name: str = kwargs['name'] if 'name' in kwargs.keys() else 'ParallelPolicy'
        policy: typing.Union[str, common.ParallelPolicy.Base] = kwargs['policy'].replace(' ', '')
        selected_child: typing.Optional[str] = kwargs['selected_child']
        if isinstance(policy, str):
            try:                
                if policy == 'SuccessOnSelected':
                    try:
                        selected_child_ = [int(v.replace(' ', '')) for v in selected_child.split(',')]   
                        selected_child_ = [children[i] for i in range(len(children)) if i in [selected_child_]]
                    except:
                        raise RuntimeError(f"error in decoding selected_child:{selected_child} from string to List[Behaviour]")
                    policy = policy_mapping[policy](children=selected_child_)
                else:
                    policy = policy_mapping[policy]()
            except KeyError as e:
                raise KeyError(f"{e} can't remapping key:{policy} to {list(policy_mapping.values())}")
        
        super().__init__(name, policy, children)