import os

cmd = "timeout 2s rosrun tf tf_echo /map /base_footprint > ~/test.log"
os.system(cmd)

f = open('logSlamOdom.log', 'r')
f.close()
#open("logSlamOdom.log", "w").close()