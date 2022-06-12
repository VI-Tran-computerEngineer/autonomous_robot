import rospy
from std_msgs.msg import String


rospy.init_node('trigger__node', anonymous=True)
pub = rospy.Publisher('update_vision', String, queue_size=1000)
_str = String()
_str.data = 'done_first'
pub.publish(_str)