import unittest
import launch
import launch_ros
import launch_testing
import launch_testing.actions
import rclpy
from example_interfaces.srv import AddTwoInts

import pytest


@pytest.mark.rostest
def generate_test_description():
    """
    Launch the test description.
    This function spawns the node under test (add_two_ints_server) and the test itself.
    """
    add_two_ints_server = launch_ros.actions.Node(
        package='my_cpp_pkg',
        executable='add_two_ints_server',
        name='add_two_ints_server'
    )

    return launch.LaunchDescription([
        add_two_ints_server,
        # ReadyToTest tells launch_testing that we are ready to start the tests.
        launch_testing.actions.ReadyToTest(),
    ])


class TestAddTwoIntsServer(unittest.TestCase):
    """
    Test suite for the AddTwoInts server.
    This class contains the actual tests that run after the launch description.
    """

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS client library once for the whole class
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS client library after all tests are done
        rclpy.shutdown()

    def setUp(self):
        # Create a transient node for the test client
        self.node = rclpy.create_node('test_add_two_ints_client')

    def tearDown(self):
        # Clean up the node after each test
        self.node.destroy_node()

    def test_add_two_ints(self):
        """
        Test the add_two_ints service.
        Sends a request (5 + 10) and expects a response (15).
        """
        client = self.node.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service
        self.assertTrue(client.wait_for_service(timeout_sec=5.0), "Service not available")

        # Create request
        request = AddTwoInts.Request()
        request.a = 5
        request.b = 10

        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        self.assertTrue(future.done(), "Future not done")
        response = future.result()

        self.assertIsNotNone(response, "Response is None")
        self.assertEqual(response.sum, 15, "Sum is incorrect")
