import rospy
from quad_msgs.msg import RobotPlan, RobotState, BodyState  # Replace with the correct message type
import matplotlib.pyplot as plt

# Data storage for plotting
global_z = []
local_z = []
ground_truth_z = []

# Maximum number of points to display
MAX_POINTS = 100

# Callback for global planner
def callback1(msg):
    global global_z
    if msg.states:  # Ensure there are states in the message
        for state in msg.states:
            z_value = state.body.pose.position.z
            global_z.append(z_value)
            if len(global_z) > MAX_POINTS:  # Keep the list size manageable
                global_z.pop(0)

# Callback for local planner
def callback2(msg):
    global local_z
    if msg.states:  # Ensure there are states in the message
        for state in msg.states:
            z_value = state.body.pose.position.z
            local_z.append(z_value)
            if len(local_z) > MAX_POINTS:  # Keep the list size manageable
                local_z.pop(0)

# Callback for ground truth
def callback3(msg):
    global ground_truth_z
    z_value = msg.body.pose.position.z
    ground_truth_z.append(z_value)
    if len(ground_truth_z) > MAX_POINTS:  # Keep the list size manageable
        ground_truth_z.pop(0)

# Plotting function
def plot_z_values():
    plt.ion()  # Enable interactive mode
    fig, ax = plt.subplots()

    while not rospy.is_shutdown():
        ax.clear()
        ax.plot(global_z, label="Global Planner Z")
        ax.plot(local_z, label="Local Planner Z")
        ax.plot(ground_truth_z, label="Ground Truth Z")
        
        ax.set_title("Z Values from Global, Local Planners, and Ground Truth")
        ax.set_xlabel("Samples")
        ax.set_ylabel("Z Value")
        ax.legend()
        ax.grid(True)

        plt.pause(0.1)  # Pause for the plot to update

    plt.ioff()  # Disable interactive mode
    plt.show()

if __name__ == '__main__':
    rospy.init_node('z_value_plotter')
    
    # Subscribe to topics
    rospy.Subscriber('/robot_1/global_plan', RobotPlan, callback1)
    rospy.Subscriber('/robot_1/global_plan_discrete', RobotPlan, callback2)
    rospy.Subscriber('/robot_1/state/ground_truth', RobotState, callback3)

    # Start plotting
    try:
        plot_z_values()
    except rospy.ROSInterruptException:
        pass