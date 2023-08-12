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
        name: str, 
        policy: typing.Union[str, common.ParallelPolicy.Base],
        selected_child: typing.Optional[str] = None,
        children: typing.Union[str, typing.List[Behaviour], None] = None,
    ):
        if isinstance(policy, str):
            try:
                policy = policy_mapping[policy.replace(' ', '')]
                
                if isinstance(policy, common.ParallelPolicy.SuccessOnSelected):
                    try:
                        selected_child_ = [int(v.replace(' ', '')) for v in selected_child.split(',')]   
                        selected_child_ = [children[i] for i in range(len(children)) if i in [selected_child_]]
                    except:
                        raise RuntimeError(f"error in decoding selected_child:{selected_child} from string to List[Behaviour]")
                    policy.children = selected_child_
            except KeyError as e:
                raise KeyError(f"{e} can't remapping key:{policy} to {list(policy_mapping.values())}")
        
        super().__init__(name, policy, children)