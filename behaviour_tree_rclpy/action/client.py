from rclpy.action import ActionClient as rclpyActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.executors import await_or_execute
from rclpy.qos import QoSProfile, qos_profile_action_status_default, qos_profile_services_default

class ResultMessage:
    def __init__(self, goal_id, result_response):
        self.goal_id = goal_id
        self.status = result_response.status
        self.result = result_response.result
        
    def __repr__(self):
        return 'ResultMesage <id={0}, status={1}, result={2}>'.format(
                self.goal_id,
                self.status,
                self.result)

class ActionClient(rclpyActionClient):
    def __init__(
        self, 
        node, 
        action_type, 
        action_name,  
        callback_group=None, 
        goal_service_qos_profile=qos_profile_services_default,
        result_service_qos_profile=qos_profile_services_default,
        cancel_service_qos_profile=qos_profile_services_default,
        feedback_sub_qos_profile=QoSProfile(depth=10),
        status_sub_qos_profile=qos_profile_action_status_default
    ):
        super().__init__(node, action_type, action_name, callback_group=callback_group, goal_service_qos_profile=goal_service_qos_profile, result_service_qos_profile=result_service_qos_profile, cancel_service_qos_profile=cancel_service_qos_profile, feedback_sub_qos_profile=feedback_sub_qos_profile, status_sub_qos_profile=status_sub_qos_profile)
        
    async def execute(self, taken_data):
        """
        Execute work after data has been taken from a ready wait set.

        This will set results for Future objects for any received service responses and
        call any user-defined callbacks (e.g. feedback).
        """
        if 'goal' in taken_data:
            sequence_number, goal_response = taken_data['goal']
            if sequence_number in self._goal_sequence_number_to_goal_id:
                goal_handle = ClientGoalHandle(
                    self,
                    self._goal_sequence_number_to_goal_id[sequence_number],
                    goal_response)

                if goal_handle.accepted:
                    goal_uuid = bytes(goal_handle.goal_id.uuid)
                    if goal_uuid in self._goal_handles:
                        raise RuntimeError(
                            'Two goals were accepted with the same ID ({})'.format(goal_handle))
                    self._goal_handles[goal_uuid] = goal_handle #weakref.ref(goal_handle)

                self._pending_goal_requests[sequence_number].set_result(goal_handle)
            else:
                self._node.get_logger().warning(
                    'Ignoring unexpected goal response. There may be more than '
                    f"one action server for the action '{self._action_name}'"
                )

        if 'cancel' in taken_data:
            sequence_number, cancel_response = taken_data['cancel']
            if sequence_number in self._pending_cancel_requests:
                self._pending_cancel_requests[sequence_number].set_result(cancel_response)
            else:
                self._node.get_logger().warning(
                    'Ignoring unexpected cancel response. There may be more than '
                    f"one action server for the action '{self._action_name}'"
                )

        if 'result' in taken_data:
            sequence_number, result_response = taken_data['result']
            if sequence_number in self._pending_result_requests:
                result_goal_id = self._result_sequence_number_to_goal_id[sequence_number].uuid
                self._node.get_logger().debug(f'Receive result from goal_id: {result_goal_id}')
                result_msg = ResultMessage(result_goal_id, result_response)
                self._pending_result_requests[sequence_number].set_result(result_msg)
            else:
                self._node.get_logger().warning(
                    'Ignoring unexpected result response. There may be more than '
                    f"one action server for the action '{self._action_name}'"
                )

        if 'feedback' in taken_data:
            feedback_msg = taken_data['feedback']
            goal_uuid = bytes(feedback_msg.goal_id.uuid)
            # Call a registered callback if there is one
            if goal_uuid in self._feedback_callbacks:
                await await_or_execute(self._feedback_callbacks[goal_uuid], feedback_msg)

        if 'status' in taken_data:
            # Update the status of all goal handles maintained by this Action Client
            for status_msg in taken_data['status'].status_list:
                goal_uuid = bytes(status_msg.goal_info.goal_id.uuid)
                status = status_msg.status

                if goal_uuid in self._goal_handles:
                    goal_handle = self._goal_handles[goal_uuid] #self._goal_handles[goal_uuid]()
                    if goal_handle is not None:
                        goal_handle._status = status
                        # Remove "done" goals from the list
                        if (GoalStatus.STATUS_SUCCEEDED == status or
                                GoalStatus.STATUS_CANCELED == status or
                                GoalStatus.STATUS_ABORTED == status):
                            del self._goal_handles[goal_uuid]
                    else:
                        # Weak reference is None
                        del self._goal_handles[goal_uuid]
