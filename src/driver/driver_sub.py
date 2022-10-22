import rospy
from std_msgs.msg import String 

def callback(data):
    rospy.loginfo("I heard %s",data.data)
    
def listener():
    rospy.init_node('imu_sub')  # I think the name of the subscriber node, this node?
    rospy.Subscriber("imu", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()