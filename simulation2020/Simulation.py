from RouteFunctions import *
from DataFunctions import *
from CarFunctions import *
from Parameters import*

def Simulation(Position,Speed,Orientation, Oldorientation,LidarDataOld, Environment, Car):


    #Reading the LIDAR data
    LidarData    =GetLidarSimulation(Environment,Position,Orientation) #[(step_i,r_i)]

    #We recalculate the distance of the objects according to the size of the car. This way, we create a safety zone where we can trace the path without running the risk of side collisions.
    Data_Safe = SafeZone(LidarData,parameters.Car_radius,parameters.radius_margin)

    if(len(LidarDataOld) == 0): AdvancedData= ImproveData(LidarData,LidarData,Data_Safe,Speed,Orientation,Oldorientation)
    else: AdvancedData= ImproveData(LidarDataOld,LidarData,Data_Safe,Speed,Orientation,Oldorientation) #In reality, Car orientation must be replaced by the accelerometer data
    
    #Defining the optimal route
    Target,Radius,direction,Reverse,Rimpact = FindTarget(AdvancedData,Speed,Car)

    #Calculating the data to be sent to the car (speed and angle of the wheels) and simulating the movement
    [NewPosition,NewOrientation,NewSpeed,AngleWheels]= UpdateCar(Radius,Speed,Position,Target,Orientation,direction,Rimpact,Reverse)

    #print([NewSpeed, AngleWheels])
        
    return([NewPosition,NewOrientation,LidarData,NewSpeed])

def MakeEnv():
    #Make the racetrack
    circuitext=[[0,-3.5],[0.5,-1.5],[1.5,-0.5],[3.5,0],[8,0],[8.5,-0.5],[8.5,-3],[9,-3.5],[12,-3.5],[13,-3.5],[14,-4],[14.5,-5],[14.5,-8.5], [14,-9.5],[12.5,-10],[3,-10],[1.5,-9.5],[0.5,-8.5],[0,-6.5],[0,-3.5]]
    circuitint=[[2.5,-3.5],[3.5,-2.5],[6,-2.5],[6.5,-3],[6.5,-5.5],[7,-6],[11.5,-6],[12,-6.5],[12,-7],[11.5,-7.5], [3.5,-7.5],[2.5,-6.5],[2.5,-3.5]]
    obst1=[[4.5,-8.5],[6,-8.5],[6,-9],[4.5,-9],[4.5,-8.5]]

    envir=[circuitext,circuitint,obst1]
    return(envir)

def MakeEnv2():
    
    circuitext=[[0,-3.5],[0.5,-1.5],[1.5,-0.5],[3.5,0],[8,0],[8.5,-0.5],[8.5,-3],[9,-3.5],[12,-3.5],[13,-3.5],[14,-4],[14.5,-5],[14.5,-8.5], [14,-9.5],[12.5,-10],[3,-10],[1.5,-9.5],[0.5,-8.5],[0,-6.5],[0,-3.5]]
    circuitint=[[2.5,-3.5],[3.5,-2.5],[6,-2.5],[6.5,-3],[6.5,-5.5],[7,-6],[11.5,-6],[12,-6.5],[12,-7],[11.5,-7.5], [3.5,-7.5],[2.5,-6.5],[2.5,-3.5]]
    obst1=[[3.5,-8.5],[4,-8.5],[4,-9],[3.5,-9],[3.5,-8.5]]
    obst2=[[11,-8.5],[11.5,-8.5],[11.5,-9],[11,-9],[11,-8.5]]
    obst3=[[7.5,-4],[8,-4],[8,-4.5],[7.5,-4.5],[7.5,-4]]

    envir=[circuitext,circuitint,obst1]
    return(envir)

def CarRectangle(position,orientation):

    #Exclusive simulation function to print the car chassis
    
    x=position[0]
    y=position[1]
    alphaa=pi+atan(parameters.Car_width/parameters.Car_length)
    alphab=pi-atan(parameters.Car_width/parameters.Car_length)
    alphac=atan(parameters.Car_width/parameters.Car_length)
    alphad=-atan(parameters.Car_width/parameters.Car_length)
    
    xa=x+parameters.Car_radius*(cos(orientation*2*pi/360+alphaa))
    ya=y+parameters.Car_radius*(sin(orientation*2*pi/360+alphaa))
    
    xb=x+parameters.Car_radius*(cos(orientation*2*pi/360+alphab))
    yb=y+parameters.Car_radius*(sin(orientation*2*pi/360+alphab))
    
    xc=x+parameters.Car_radius*(cos(orientation*2*pi/360+alphac))
    yc=y+parameters.Car_radius*(sin(orientation*2*pi/360+alphac))
    
    xd=x+parameters.Car_radius*(cos(orientation*2*pi/360+alphad))
    yd=y+parameters.Car_radius*(sin(orientation*2*pi/360+alphad))
    
    a=[xa,ya]
    b=[xb,yb]
    c=[xc,yc]
    d=[xd,yd]
    
    return([a,b,c,d,a])

