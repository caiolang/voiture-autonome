
from functions_commande import *
from Parameters import*

def Simulation_v2(Car,Lidar):

    #Reading the LIDAR data
    Lidar.data = GetLidar()

    #We recalculate the distance of the objects according to the size of the car. This way, we create a safety zone where we can trace the path without running the risk of side collisions.
    Data_safe = SafeZone_v2(Lidar)

    #Defining the optimal route
    FindTarget(Data_Safe,Car)

    #Calculating the data to be sent to the car (speed and angle of the wheels) and simulating the movement
    UpdateCar(Car)

    #print([NewSpeed, AngleWheels])
        
    return([NewPosition,NewOrientation,LidarData,new_vOld,NewSpeed,Target])