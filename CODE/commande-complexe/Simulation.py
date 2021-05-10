from RouteFunctions import *
from DataFunctions import *
from CarFunctions import *
from Parameters import*

def Simulation(Position,Speed,Orientation, Oldorientation,LidarDataOld, Environment):

    #Reading the LIDAR data
    LidarData    =GetLidarSimulation(Environment,Position,Orientation) #[(step_i,r_i)]

    #We recalculate the distance of the objects according to the size of the car. This way, we create a safety zone where we can trace the path without running the risk of side collisions.
    Data_Safe = SafeZone(LidarData, parameters.Car_radius,parameters.radius_margin)

    if(len(LidarDataOld) == 0): AdvancedData= ImproveData(LidarData,LidarData,Data_Safe,Speed,Orientation,Oldorientation)
    else: AdvancedData= ImproveData(LidarDataOld,LidarData,Data_Safe,Speed,Orientation,Oldorientation)#In reality, Car orientation must be replaced by the accelerometer data

        
    
    #Defining the optimal route
    Target,Radius,direction,Reverse,Rimpact=FindTarget(AdvancedData,Speed)
    

    #Calculating the data to be sent to the car (speed and angle of the wheels) and simulating the movement
    [NewPosition,NewOrientation,NewSpeed,AngleWheels]= UpdateCar(Radius,Speed,Position,Target,Orientation,direction,Rimpact,Reverse)

    #print([NewSpeed, AngleWheels])
        
    return([NewPosition,NewOrientation,LidarData,NewSpeed])


def Simulation_v2(Position,Speed,Orientation, Oldorientation,LidarDataOld, vOld, direction, Environment):

    #Reading the LIDAR data
    LidarData = GetLidarSimulation(Environment,Position,Orientation) #[(step_i,r_i)]
    #We recalculate the distance of the objects according to the size of the car. This way, we create a safety zone where we can trace the path without running the risk of side collisions.
    Data_Safe, new_vOld = SafeZone_v2(LidarData,LidarDataOld, vOld, direction, Speed, parameters.Car_radius,parameters.radius_margin) #new_LidarDataOld,

    # if(len(LidarDataOld) == 0): AdvancedData= ImproveData_v2(LidarData,LidarData,Data_Safe,Speed,Orientation,Oldorientation)
    # else: AdvancedData= ImproveData_v2(LidarDataOld,LidarData,Data_Safe,Speed,Orientation,Oldorientation)#In reality, Car orientation must be replaced by the accelerometer data

    #Defining the optimal route
    Target,Radius,direction,Reverse,Rimpact = FindTarget(Data_Safe,Speed)

    #Calculating the data to be sent to the car (speed and angle of the wheels) and simulating the movement
    [NewPosition,NewOrientation,NewSpeed,AngleWheels]= UpdateCar(Radius,Speed,Position,Target,Orientation,direction,Rimpact,Reverse)

    new_LidarDataOld = LidarData

    #print([NewSpeed, AngleWheels])
        
    return([NewPosition,NewOrientation,new_LidarDataOld,new_vOld,NewSpeed,Target])

def MakeEnv():
    #Make the racetrack
    
    #Circuit d'origine
    circuitext=[[0,-3.5],[0.5,-1.5],[1.5,-0.5],[3.5,0],[8,0],[8.5,-0.5],[8.5,-3],[9,-3.5],[12,-3.5],[13,-3.5],[14,-4],[14.5,-5],[14.5,-8.5], [14,-9.5],[12.5,-10],[3,-10],[1.5,-9.5],[0.5,-8.5],[0,-6.5],[0,-3.5]]
    circuitint=[[2.5,-3.5],[3.5,-2.5],[6,-2.5],[6.5,-3],[6.5,-5.5],[7,-6],[11.5,-6],[12,-6.5],[12,-7],[11.5,-7.5], [3.5,-7.5],[2.5,-6.5],[2.5,-3.5]]
    obst1=[[4.5,-8.5],[6,-8.5],[6,-9],[4.5,-9],[4.5,-8.5]]

    #Circuit type ENS
    # circuitext=[[0.5,0],[13.5,0], #droite
    # [13.83,-0.06],[14.16,-0.166],[14.5,-0.36],[14.83,-0.63],[14.9,-1],[15,-1.5], #1er virage
    # [15,-11.5],[14.98,-11.6],[14.9,-11.7],[14.8,-11.8],[14.6,-11.9],[14.5,-11.95],[14.35,-11.975],[14,-12], #2eme virage
    # [11.5,-11],[11.19802441,-11.48062717],[10.87639421,-11.69002995],[10.48463229,-11.76737959],[10.11264737,-11.69791855],[9.7874304,-11.49581957],[9.60938295,-11.281064933],[9.61461099,-11.27781275],[9.48886894,-11.00297764],[9.45385588,-10.73660319],[9.4992499,-10.43407792],[9.60938295,-10.19214144],[9.67380129,-10.10016538], #demi cercle 1
    # [9.5,-10],[3.5,-6.5],[3.41301802,-6.64528915],[3.28918073,-6.79195566],[2.94889392,-7.02396008],[2.50,-7.11803399],[2.00,-7.00],[1.66534294,-6.74387337],[1.50,-6.50],[1.39446196,-6.16669029],[1.41103461,-5.74671283], #demi cercle 2
    # [1.5,-5.5],[1.36463364,-5.19045199],[0.99602628,-4.96864969],[0.68934898,-4.70286997],[0.33482587,-4.23893548],[0.09229593,-3.67289011], #3eme virage
    # [0,-3.5],[-0.5,-1.5], [-0.56610874,-1.27574763],[-.57772194,-0.97321276], [-0.45746671,-0.57651126],[-0.23794465,-0.29020736],[0.22539893,-0.05002474],  #4eme virage retour 0,0
    # [0.5,0]]
    # circuitint=[[2,-2.5],[10,-2.5],[12,-2.25],[12,-7.5], #blocs droits
    # [11.82259994,-7.48413882],[11.29289322,-7.20710678],[11.05579784,-6.82936648],[11,-6.5], [10.97869706,-6.2946903],[10.83251669,-5.94600004],[10.48463229,-5.62528202],[10.16880526,-5.51435058],
    # [10,-5.5],[8,-5.5],[5,-4],[4.26277232,-3.59676231], [4.07381622,-3.33762155],[4.00880617,-3.11904258],[4,-3],[2,-3],[2,-2.5]]


    envir=[circuitext,circuitint]
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

