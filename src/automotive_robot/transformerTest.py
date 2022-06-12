import tf 
import rospy
import geometry_msgs

rospy.init_node('robot_global_vision_update', anonymous=True)
while not rospy.is_shutdown():
    k = tf.TransformListener()
    if k.frameExists("/base_link") and k.frameExists("/map"):
            t = k.getLatestCommonTime("/base_link", "/map")
            position, quaternion = self.tf.lookupTransform("/base_link", "/map", t)
            print(position, quaternion)