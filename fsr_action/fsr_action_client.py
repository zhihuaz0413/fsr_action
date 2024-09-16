import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from fsr_interfaces.action import TriggerFsr

class FsrActionClient(Node):

    def __init__(self):
        super().__init__('fsr_action_client')
        self._action_client = ActionClient(self, TriggerFsr, 'trigger_fsr')

    def send_goal(self, duration):
        goal_msg = TriggerFsr.Goal()
        goal_msg.show_flag = True
        goal_msg.duration = duration
        goal_msg.record_flag = False

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        #self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {}'.format(result.file_name))
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.values))

def main(args=None):
    rclpy.init(args=args)

    action_client = FsrActionClient()

    future = action_client.send_goal(5)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
