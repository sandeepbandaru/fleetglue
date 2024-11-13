problem statement: In python, create a REST API with one POST and one GET endpoint. Create ROS2 node 1 (RN1) which checks for data at least every second and hosts an Action Client. When RN1 finds data, it processes the data, and sends a mission to the Action Server, hosted on ROS node 2 (RN2), which simply prints the processed data it receives. So it should function like when you POST to the REST API, RN2 prints.


