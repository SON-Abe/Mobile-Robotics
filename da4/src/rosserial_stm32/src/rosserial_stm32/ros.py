import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
import math

# Function to convert RPY to quaternion
def rpy_to_quaternion(rpy):
    roll, pitch, yaw = rpy.data

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    q0 = cy * cr * cp + sy * sr * sp
    q1 = cy * sr * cp - sy * cr * sp
    q2 = cy * cr * sp + sy * sr * cp
    q3 = sy * cr * cp - cy * sr * sp

    imu_msg = Imu()
    imu_msg.orientation.w = q0
    imu_msg.orientation.x = q1
    imu_msg.orientation.y = q2
    imu_msg.orientation.z = q3

    return imu_msg

# Callback function for /rpy topic subscription
def rpy_callback(rpy_msg):
    imu_msg = rpy_to_quaternion(rpy_msg)
    imu_pub.publish(imu_msg)

if __name__ == '__main__':
    rospy.init_node('rpy_to_imu_converter')
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
    rospy.Subscriber('/rpy', Float32MultiArray, rpy_callback)
    rospy.spin()
