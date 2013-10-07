#!/usr/bin/env python
import roslib; roslib.load_manifest('CodyLegoBot') 	# ros
import rospy						# ros
from std_msgs.msg import String				# for ros
import nxt.locator					# lego
from nxt.motor import *					# lego
import nxt, thread, time				# lego

print 'starting...'

def turnmotor(m, power, degrees):
	m.turn(power, degrees)
#b = nxt.locator.find_one_brick()
b = nxt.find_one_brick()
print 'starting - brick connected'
my = nxt.Motor(b, nxt.PORT_A)
mx = nxt.Motor(b, nxt.PORT_B)
vel = 0					# forward speed
turn = 0				# turn speed
degsx = 1080*100
degsy = 1080*100

################################################ Important Code
def callback(data):
    #rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)
    command = str.lower(data.data)
    comsize = len(command)
    print command
    #print "hello again"
    global vel
    global turn
    global degsx
    global degsy

    # switch(command) statement implemented as elseif
    # figure out desire
    if command == 'w':
        #print 'Forward'
	vel = vel + 25
	print vel
	print turn
    elif command == 'a':
        print 'Left'
	turn = turn - 10
	print vel
	print turn
    elif command == 's':
        print 'Backward'
	vel = vel - 25
	print vel
	print turn
    elif command == 'd':
        print 'Right'
	turn = turn + 10
	print vel
	print turn
    else:
        print 'Invalid Command'
    if vel+turn!=0:
	# vel+turn power, lots of degrees
	thread.start_new_thread( turnmotor, (mx , vel+turn , degsx ))
    else:
	# 0 degrees with 1 percent power
	thread.start_new_thread( turnmotor, (mx , 1 , 0 )) 
    if vel-turn!=0:
	thread.start_new_thread( turnmotor, (my , vel-turn , degsy ))
    else:
	thread.start_new_thread( turnmotor, (my , 1 , 0 )) 
    time.sleep(.1)



##################################################

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("commandsend", String, callback)
    rospy.spin()


if __name__ == '__main__':
    print 'Starting'
    listener()
