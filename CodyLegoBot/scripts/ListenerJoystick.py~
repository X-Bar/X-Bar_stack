#!/usr/bin/env python
import roslib; roslib.load_manifest('CodyLegoBot') 	# ros
import rospy						# ros
import sensor_msgs.msg
from bluetooth import *
try:
    from sensor_msgs.msg import Joy
except ImportError:
    from joy.msg import Joy

from std_msgs.msg import String				# for ros
import nxt.locator					# lego
from nxt.motor import *					# lego
import nxt, thread, time				# lego

def turnmotor(m, power, degrees):
	m.turn(power, degrees)
####b = nxt.locator.find_one_brick()
b = nxt.find_one_brick()
print 'starting - brick connected'
my = nxt.Motor(b, nxt.PORT_A)
mx = nxt.Motor(b, nxt.PORT_B)
vel = 0						# forward speed
turn = 0					# turn speed
degsx = 1080*100
degsy = 1080*100

################################################ Important Code
def callback(data, null):
    print 'starting - 2'
    #rospy.loginfo(rospy.get_name() + ": I heard %f" % data.axes[0])
    # first element (+left -right). second element (+forward -backward)
    AxesSize = len(data.axes)
    turn = data.axes[0]*50
    vel = data.axes[1]*100
    print "X axes %f, Y axes %f" %(turn, vel) 
    global degsx
    global degsy
    RightWheel = vel+turn
    LeftWheel = vel-turn
    if(RightWheel > 100):
	RightWheel = 100
    elif (RightWheel < -100):
	RightWheel = -100
    if(LeftWheel > 100):
	LeftWheel = 100
    elif (LeftWheel < -100):
	LeftWheel = -100

   
    if abs(LeftWheel) > 1:
	#a =1
	if abs(RightWheel) > 1:
		a = 1	#for testing without nxt block
		#thread.start_new_thread( turnmotor, (mx , RightWheel , degsx ))		# Move right motor 
		#thread.start_new_thread( turnmotor, (my , LeftWheel , degsy ))		# Move left motor s	
	else: # abs(RightWheel) is < 1
		RightWheel =1
		#thread.start_new_thread( turnmotor, (mx , RightWheel , 1 ))			# stop right motor
		#thread.start_new_thread( turnmotor, (my , LeftWheel , degsy ))		# make left motor go
    else: # LeftWheel is < 1
	if abs(RightWheel) > 1:
		LeftWheel = 1
		#thread.start_new_thread( turnmotor, (mx , RightWheel , degsx ))		# make right motor go
		#thread.start_new_thread( turnmotor, (my , LeftWheel , 1))			# make left motor stop
	else: # abs(RightWheel) is < 1
		RightWheel =1
		LeftWheel = 1
		#thread.start_new_thread( turnmotor, (mx , RightWheel , 1 ))			# make right motor stop
		#thread.start_new_thread( turnmotor, (my , LeftWheel , 1))			# make left motor stop
    print RightWheel
    print LeftWheel


    # switch(command) statement implemented as elseif
    # figure out desire
#    if command == 'w':
#        #print 'Forward'
#	vel = vel + 10
#	print vel
#	print turn
 #   elif command == 'a':
#        print 'Left'
#	turn = turn - 5
#	print vel
#	print turn
#    elif command == 's':
#        print 'Backward'
#	vel = vel - 10
#	print vel
#	print turn
 #   elif command == 'd':
 #       print 'Right'
#	turn = turn + 5
#	print vel
#	print turn
#    else:
 #       print 'Invalid Command'
    #if vel+turn!=0:
	# vel+turn power, lots of degrees
	#thread.start_new_thread( turnmotor, (mx , vel+turn , degsx ))
    #else:
	# 0 degrees with 1 percent power
	#thread.start_new_thread( turnmotor, (mx , 1 , 0 )) 
    #if vel-turn!=0:
	#thread.start_new_thread( turnmotor, (my , vel-turn , degsy ))
    #else:
	#thread.start_new_thread( turnmotor, (my , 1 , 0 )) 
    time.sleep(.1)



##################################################

def listener():
    print 'starting - 1'
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("joy", Joy, callback,[], 1, 1)
    rospy.spin()
    print 'starting - 3'


if __name__ == '__main__':
    listener()
