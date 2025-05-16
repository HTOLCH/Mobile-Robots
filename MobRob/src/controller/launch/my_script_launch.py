import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_publisher'),
        launch_ros.actions.Node(
            package='master',
            executable='master',
            name='master'),
        # launch_ros.actions.Node(
        #     package='ariaNode',
        #     executable='ariaNode',
        #     name='ariaNode',
        #     ),
  ])
