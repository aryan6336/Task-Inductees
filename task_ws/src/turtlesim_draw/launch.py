import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Start the turtlesim node
        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        
        # Start your drawing script
        launch_ros.actions.Node(
            package='turtlesim_draw',
            executable='code.py',
            name='turtle_draw',
            output='screen'
        ),
    ])
