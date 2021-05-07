###############################################################
#                                                             #
#                     Little command law                      #
#                                                             #
###############################################################

###############################################################
#                          READ ME                            #
#                                                             #
# The code bellow allow the car of our PIE team to drive      #
# autonomously as long as all devices (except the inertial    #
# unit) are connected as indicated in the documentation and   # 
# the initialisation instruction bellow are executed.         #
#                                                             #
#                                                             #
# Here is the frame of the algorithm :                        #
#                                                             #
# Initialisation :                                            #
#  The Raspberry imports useful librairies and connect to     #
#  other components. This part may raise exception, in        #
#  particular if the LIDAR port has been changed (see         #
#  instructions bellow)                                       #
#                                                             #
# Functions definition :                                      #
#  These function are used to communicate with other          #
#  components and choices about the form of data have been    #
#  made in order to facilitate programmation in particular    #
#  with LIDAR data.                                           #
#                                                             #
# Core :                                                      #
#  -Wait until the user press cntrl + C                       #
#  -While the user does not press Crtl + C again :            #
#    -Collect data from the LIDAR, select a range 2 * window  #
#     angles centered on the front position, order them from  #
#     left to right (so the front position became the windowth#
#     one) then replace each value per the average of the     #
#     2 * average_number values around it.                    #
#    -Find the objective which is the farthest point from the #
#     car in data_r_av.                                       #
#    -Calculate the theoretical curve which is 1/R with R     #
#     the maximal radius of the circular trajectory leading   #
#     to the objective.                                       #
#    -Depending on the distance from walls around the car,    #
#     select a "regime" which mean a speed and coefficient we #
#     multiply the theoretical curve by to get the curve      #
#     command. It allows to be more reactive in critical      #
#     situations and faster in controlled ones.               #
#    -Send order to the ESC and servo                         #
#                                                             #
# Disconnection of the lidar and the ESC                      #
#                                                             #
###############################################################

###############################################################
#                 INITIALISATION INSTRUCTION                  #
#                                                             #
# 1- Open the Raspberry terminal and run the following        #
# instruction : " "                                           #
#                                                             #
# 2- Turn the ESC IF IT DOES NOT MAKE NOISE IT DOES NOT WORK  #
#                                                             #
# 3- Run the code initialize_ESC ONLY ONCE. If works only if  #
# it makes noise once again.                                  #
#                                                             #
# IF YOU HAVE ANY DOUBT ABOUT THE SUCCESS OF THIS STEP :      #
# EITHER TURN THE ESC OFF AND RESTAR FROM STEP 2 OR ENSURE    #
# THE CAR PHYSICALLY CAN'T GO AWAY. IF THE ESC IS INITIALIZED #
# TWICE, THE CAR WILL DRIVE AT MAXIMAL SPEED, TURNING LEFT    #
# AND CAN BE DAMAGED                                          #
#                                                             #
###############################################################



###############################################################
#                    Initialisation                           #
###############################################################
import PyLidar3
import time # Time module
import RPi.GPIO as GPIO # Controlling the servo motor
import time
from numpy import sin, mean
from numpy import pi as constante_pi
import pigpio # Controlling the main motor


servoPIN = 17 # This number is the GPIO number which is different from the absolute number
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

# 1. Servo pin configurations
servo = GPIO.PWM(servoPIN, 33) # GPIO 17 for PWM with 33Hz (pulse period of 30ms)
servo.start(2) # Initialization
# -----

#2. Electronic Speed Controller (Motor control) pin configurations
ESC = 4
pi = pigpio.pi()

#2.1. Exit program if there is no connection
if not pi.connected:
    print("ESC is not connected")
    exit()
pi.set_mode(ESC, pigpio.OUTPUT) # Configuring the ESC pin as OUTPUT


# -----

# 3. LIDAR Connection

 ##############################################################
 #                                                            #
 #                       CAUTION                              #
 #                                                            #
 # Every time the Raspberry restarts, the Lidar port may be   #
 # changed even if the LIDAR has not been disconnected.       #
 #                                                            #
 # Please try changing /dev/ttyUSB0 to /dev/ttyUSB1 or        #
 # /dev/ttyUSB2 if this happens.                              # 
 #                                                            #
 ##############################################################

port = "/dev/ttyUSB0"  

Obj = PyLidar3.YdLidarX4(port)    
Obj.Connect()
gen = Obj.StartScanning()


 ##############################################################
 #                  Function definitions                      #
 ##############################################################

def getLidar2():
    """
    Return a list of 360 floats representing the distance detected 
    by the Lidar. The i-th element is the distance in meter for an 
    angle of i degrees from the front direction clockwise.
    In case the LIDAR does not detect anything the function returns
    1 meter.
    """
    data = [0]*360  
    lista = next(gen)
    
    for i in range(360):
    #    data += [[i,lista[i]*0.001]]
        if lista[i]==0:
            lista[i] = 1000
        data[i] = lista[i]*0.001
        

    #return data
    return(data)

def turnServo(i_R):
    """
    Turn the servo in order to obtain a real curve i_R. Positive curve
    correspond do turning to the right.
    """
    servo_angle = (i_R - 0.17528)/0.0737157
    
    duty = servo_angle/18+4
    if duty < 100 and duty > 0:
        servo.ChangeDutyCycle(duty)
    elif duty >=100:
        servo.ChangeDutyCycle(99)
    else :
        servo.ChangeDutyCycle(1)
    return

def set_speed(vitesse, sens = 1):
    pulse = 1500 + sens*vitesse*5 #pulse between 1000 and 2000, 1500 is neutral
    pi.set_servo_pulsewidth(ESC, pulse)



 ##############################################################
 #                          Core                              #
 ##############################################################

# Definition in angle of the half of the range of values of distance
# provided by the LIDAR which will be treated to find the 
# objective.
window = 110 # °

# average_number * 2 is the number of value for which
# the distance is averaged
average_number = 15 # °

in_front_past = 1000
in_front_past_past = 2000

# List done in order to record what the car sees. It must be 
# useful in order to detect the reason of behavior problems

#data = []


##############################################################
#                                                            #
#            Wait until the begining of the race             #
#                                                            #
#             Press Cntrl + C to start the car               #
#                                                            #
##############################################################
try : 
	while True :
		time.sleep(0.001)
except KeyboardInterrupt :
	pass



##############################################################
#                                                            #
#                       START DRIVING                        #
#                                                            #
#              Press Cntrl + C to stop the car               #
#                                                            #
##############################################################

try :
	while True :
    	#----------------------Collect data------------------------
    	
	    #               get data from the LIDAR
	    
	    # data_lidar : lenght : 360 the i-th value correspond to 
	    # the distance for an angle i from the front position clockwise
	    data_lidar = getLidar2() 
	    
	    #                 Storage of the data  
	    # Must be useful in order to find problems in the car behavior
	    
	    # data.append(data_lidar) 
	    
	    #                  Reorder data
	    
	    # data_reordonne : lenght : 2 * window + 2 * average_number
	    # we use there average_number additional data on each side in 
	    # order to be able to make an average for extreme values 
	    
	    data_reordonne = data_lidar[360 - window - average_number: 360 -1] + data_lidar[0 : window + average_number - 1] 
	    
	    #      Replace each value by a spatial average
	    
	    # data_r_av : lenght : 2 * window; here are the data which 
	    # will finally be used
	    data_r_av = [0]*(2* window)
	    for i in range(2 * window) :
	        data_r_av[i] = mean(data_reordonne[i : i + 2 * average_number])	   
	        
	    #------------Determination of the objective and-----------
	    #------------ the theoretical curve to adopt--------------
	    
	    # objective is the angle between -window and + window the car targets
	    # the straight trajectory correspond to the objective 0.
	    # This objective correspond to the maximal value of data_r_av.
	    # val_objective is this distance.
	    # Using data_r_av allow the car to be less sensitive to measure errors 
	    # which does not stretch on a wide range of data (it happends quite often)
	    
	    val_objective = max(data_r_av)
	    objective = data_r_av.index(val_objective) - window
	    
	    # The curve correspond to the curve corresponding to the maximal 
	    # radius R which can be used to reach our objective. curve = 1 / R
	    
	    # In order to find this formula again, you must trace a circle of 
	    # raduis R. The car is tangent to the circle. Trace a diameter 
	    # (of lenght 2R) from the car. Then place the objective on the circle. 
	    # The distance from the car is val_objective. Now trace the square triangle 
	    # with sqare on the objective and the diameter as hypothenus. 
	    # Then calculage the cosinus of pi/2 - objective and you will 
	    # find the formula bellow.

	    curve = 2 * sin(objective * constante_pi / 180) / val_objective # m^-1
    
	    #----------------Determination of the regime---------------
	    
	    # distance right in front of the car
	    in_front = data_r_av[window]
	    
	    
	    if (in_front >= 1.2) and( (data_r_av[window - 90]) <0.8 or (data_r_av[window - 90] <0.8)):
	    # If the car does not see a close wall in front but is close of one laterally. 
	    # The speed remains normal but the curve is exagerated in order to get away from the wall
	        regime = "close to walls"
	        coeff_curve = 2
	        speed =21
	        
	    elif (in_front < 1.2):
	    # If the car sees a close wall just in front of it, it drastically slows down 
	    # in order to manage the situation better and exagerate at maximum curves because
	    # the calculus of the objective does not take obstacle into account so we are almost 
	    # sure we would go into the wall if we don't turn farther
	        regime = "careful"
	        coeff_curve = 40
	        speed = 20
	        
	    elif in_front >3 and curve < 0.1 :
	    # If the car doesn't see any obstacle right in front of it and the theoretical curve 
	    # is very low, the car can go fast and turn less
	        regime = "faster"
	        coeff_curve = 0.8
	        speed = 23
	        
	    elif in_front > 2.5 and curve < 0.2:
	    # If the an obstacle is seen a little closer or the theoretical curve is more important, 
	    # the car goes a little slower and turn a little more in case the car approach a turn
	        regime = "fast"
	        coeff_curve  = 0.9
	        speed = 22
	        
	    else:
	    # If the car is not in any of the previous case in particular it respects the 
	    # theoretical curve and has a moderated speed
	        regime = "moderate"
	        coeff_curve = 1
	        speed = 21
	        
	        
	    # Change the speed consequently to the battery level which influence strongly the relation 
	    # between the speed command and the real speed.
	    speed = speed+1
	    
	    # Modify the curve value depending to the regime
	    curve = curve * coeff_curve
	    
	    
	    #--------------------Order to the servo-------------------
	    turnServo(curve)  
	    
	    #--------------------Order to the engine-------------------
	    set_speed(speed)
	          
	    
	    #----------------------Print information-------------------
	    #print("objective : ", objective, "  val_objective : ", int(100 *val_objective)/10, " regime : ", regime, "curve : ", curve, " in_front : ", in_front)

except KeyboardInterrupt :
	pass	

#############################################################################
#                Disconnection of devices                                   #
#############################################################################

Obj.StopScanning()
Obj.Disconnect()
pi.set_servo_pulsewidth(ESC, 0)
pi.stop()
