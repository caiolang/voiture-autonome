from math import *

class Parameters:
    '''
    This class all basic and initial parameters to run the simulation.
    Attention to update the data according to the information and technical characteristics of the components used
    '''
    
    ##Car Features
    Car_length = 0.4      #meters
    Car_width = 0.19      #meters
    Car_maxspeed = 30/3.6 # meters/s
    Car_radius =sqrt((Car_length/2)**2+(Car_width/2)**2) #Radius of the vehicle. A circle is created with the "car inside" so that nothing goes beyond that circle. 

    #LIDAR Features
    Lidar_steps=360              #Numbers of data that LIDAR receives in one complete turn
    Lidar_delta=1/10             #Time taken by the sensor to read the N points
    Lidar_stepsize = 360/Lidar_steps       #Angled representation of each LIDAR step
    Lidar_maxdistance = 1000     #Maximum distance that the handle can capture an object(meters)

    #Variables that will be used for simulation.
    # radius_margin = 0.05         #Safety margin for vehicle radius (meters)
    radius_margin = 0.01         #Safety margin for vehicle radius (meters)
    Alpha=25                     #Angle range of measures that the algorithm will consider to calculate the trajectory
    Maxdist_relative=10          #Maximum distance I can place a point that moves at relative speed
    
    Max_accelerationlateral = 10
    Max_acceleration = 5
    Min_acceleration = -5
    
    epsilonmax=45 #In degrees
    tsb=0.1  #Taux de surbraquage -> epsilon/lateral acceleration


    #Variables created just to simulate the car in the virtual environment
    car_initial_position =[10,-9]
    car_initial_orientation =180
    car_initial_speed = 0
    
parameters = Parameters()
