import queue
from mpu6050 import mpu6050
import rospy
from std_msgs.msg import String

# REFERENCE: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers

# TODO: possibly find a way to detect the mpu-6050 dynamically, or just have it plugged in the same way 
#       and roll with it. 

def find_mpu_6050():  # Will return an 0x00 address/register of the IMU
    return None

# TODO: Create a valid publisher and discuss how we might want to implement the publishing
def publish_data():
    pub = rospy.Publisher('imu', String, queue_size=10)  # inits the publisher
    rospy.init_node('imu_pub')  # runs the node, I think the name of this node, as in the publishing node?
    r = rospy.Rate(10)  # sets the rospy publishing rate 10hz, 10 sends per second
    
    sensor = mpu6050(0x68)  # We need a sensor object 
    
    # while loop to keep sending data
    while not rospy.is_shutdown():

        '''
            The publish here implements sending all data, effectively dumping everything
            so we may need different nodes/packages for acceleration/gyroscope/velocity
            etc. and then another node to communicate with those nodes to get precalculated
            data such as the distance travelled like the team requested. In terms of RVIZ 
            we might be able to couple it easily but it depends on what the ros visualizer
            requires, haven't looked into that much.
        '''
        pub.publish(sensor.get_all_data())  # actual publishing to topic 
        r.sleep()
