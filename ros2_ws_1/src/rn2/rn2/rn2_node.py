import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action_interfaces.action import PrintData

class RN2Server(Node):
    def __init__(self):
        super().__init__('rn2_server')
        self._action_server = ActionServer(
            self,
            PrintData,
            'print_data',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        # Print the received data
        data = goal_handle.request.data
        self.get_logger().info(f"RN2 received data: {data}")

        # Send feedback if desired
        feedback_msg = PrintData.Feedback()
        feedback_msg.feedback = "Processing data..."
        goal_handle.publish_feedback(feedback_msg)

        # Mark the goal as complete and send result
        goal_handle.succeed()
        result = PrintData.Result()
        result.result = "Data processed and printed by RN2"
        return result

def main(args=None):
    rclpy.init(args=args)
    rn2_server = RN2Server()
    rclpy.spin(rn2_server)
    rn2_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
