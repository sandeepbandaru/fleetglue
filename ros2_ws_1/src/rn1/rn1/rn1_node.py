import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_interfaces.action import PrintData
import time
from datetime import datetime
import requests

class RN1(Node):
    def __init__(self):
        super().__init__('rn1_client')
        self._action_client = ActionClient(self, PrintData, 'print_data')
        self.check_interval = 1  # Interval in seconds
        self.api_url = "http://api:8000/messages"


    def fetch_data(self):
        try:
            # Send GET request to the REST API
            response = requests.get(self.api_url)
            response.raise_for_status()
            data = response.json()

            # If data is found, process and send it to RN2
            if data:
                self.get_logger().info(f"Data received from API: {data}")
                for item in data:
                    self.send_goal(item)  # Adjust key based on API response structure
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error fetching data from API: {e}")

    def send_goal(self, data):
        # Create a new message with a timestamp
        goal_msg = PrintData.Goal()
        goal_msg.data = data

        # Send the goal to the Action Server and handle the response
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info(f"Sent message to RN2: {goal_msg.data}")

    def feedback_callback(self, feedback_msg):
        # Optional feedback handling
        self.get_logger().info(f"Feedback from RN2: {feedback_msg.feedback}")

    def run(self):
        while rclpy.ok():
            # self.send_goal("Hello, World!")
            self.fetch_data()
            time.sleep(self.check_interval)

def main(args=None):
    rclpy.init(args=args)
    rn1_client = RN1()
    try:
        rn1_client.run()
    except KeyboardInterrupt:
        pass
    rn1_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
