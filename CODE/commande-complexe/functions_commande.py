from math import *
import copy
from Parameters import *
from MathFunctions import*

def SafeZone_v2 (Lidar,Car):
    
    """
    This function defines a safety zone within the track layout where the car can pass without 
    running the risk of hitting the sideswe trace another track inside the original track to 
    ensure that the car will not get close to any wall or other obstacle

        Lidar.data    : [step_i, distance_i]                             i(0,N)
        Data_Sorted   : [step_i, distance_i]                             i(-N/4,3N/4) gets the angle interval (-90;270)
        Complete_Data : [step_i, distance_i,x_i,y_i] 
        Safe_Data     : [step_i, distance_i,x_i,y_i, Xsafe_i, Ysafe_i]   Xsafe,Ysafe new calculate points

    Return Data_Safe
    """
    N=parameters.Lidar_steps                                     # #Numbers of data that LIDAR receives in one complete turn
    step = parameters.Lidar_stepsize                             #Angled representation of each LIDAR step
    radius = Car.radius + parameters.radius_margin     #Parameter that will be used in calculations

    #Organizing the received data so that it is in the order [-90º, 270º]. This is necessary because the front of the car is the 0º point
    A=copy.deepcopy(Lidar.data[0:int(N/4)])
    B=copy.deepcopy(Lidar.data[int(3*N/4):N])
    A.reverse()
    B.reverse()
    Data_Sorted=copy.deepcopy(A+B)

    #We are adding Cartesian positions X, Y to the data because the LIDAR only gives polar coordinates.
    Complete_Data=[]
    i=0
    while i<len(Data_Sorted):
        theta=Data_Sorted[i][0]
        r=Data_Sorted[i][1]
        xi=r*cos(theta*step*2*pi/360)
        yi=r*sin(theta*step*2*pi/360)
        Complete_Data.append([theta,r,xi,yi])
        i=i+1
        
    
    Data_Safe=[] 
    i=1
    while i<(len(Complete_Data)-1):

        #Position point i-1
        xim1=Complete_Data[i-1][2] 
        yim1=Complete_Data[i-1][3]
        
        #Position point i
        xi=Complete_Data[i][2]
        yi=Complete_Data[i][3]

        #Position point i+1
        xip1=Complete_Data[i+1][2]
        yip1=Complete_Data[i+1][3]

        #Vitesse relative (dans la direction y) du point i
        vi = (Complete_Data[i][2] - Complete_DataOld[i][2])/(Lidar_delta)

        if (vi > 0): #the obstacle gets closer at "negative" speed relatively to the car
            
            #If the track is shortening. It can be a curve or a taper
            if abs(yip1-yim1)>0.00001:
                #Calculating the coefficient. Tangent of the angle between points
                K=-(xip1-xim1)/(yip1-yim1) 
            
                #moving the point to a value that is safely away from the wall
                xp1=xi+radius/sqrt(1+K**2) 
                yp1=yi+K*radius/sqrt(1+K**2)
            
                #moving the point to a value that is safely away from the wall
                xp2=xi-radius/sqrt(1+K**2)
                yp2=yi-K*radius/sqrt(1+K**2)
            
            #If the track is normal or increasing.If the track is normal or increasing, I don't need to calculate the ideal position. Just make sure the car is far away from the "radius" of the edges "
            if abs(yip1-yim1)<0.00001:
                yp1=yi+radius
                yp2=yi-radius
                xp1=xi
                xp2=xi

            #Choosing the closest point to the car    
            d1=(xp1)**2+(yp1)**2
            d2=(xp2)**2+(yp2)**2
            liste1=Complete_Data[i]+[xp1,yp1]
            liste2=Complete_Data[i]+[xp2,yp2]
            

            if d1<=d2:
                Data_Safe.append(liste1)
            if d1>d2:
                Data_Safe.append(liste2)

        else:
            Rimpact=1
            theta=Data_Sorted[i][0]
            x=Rimpact*cos(theta*step*2*pi/360)
            y=Rimpact*sin(theta*step*2*pi/360)
            d=x**2+y**2
            Data_Safe.append([0,Rimpact,0,0,x,y])
            
        i=i+1
    
    Lidar.prev_data = Complete_Data
    
    return(Data_Safe)



#####################################################################
#######################################################################

def FindTarget (Data_safe, Car):
    """
        Data_safe   :  [step_i, distance_i,x_i,y_i, Xsafe_i or Xvel_i, Ysafe_i or Yvel_i]
    """
    
    R=0
    
    #Filtering the data and creating a list with only the positions of the safe zone and the step number
    Positions=[]  #[step_i, Xsafe_i or Xvel_i, Ysafe_i or Yvel_i]
    i=0 
    while i<len(Data_safe):
        d=Data_safe[i][4]**2+Data_safe[i][5]**2
        liste=Data_safe[i]+[d]
        Positions.append(liste)
        i+=1
    

    Positions.sort(key=lambda x: x[6])
    Positions.reverse()
    
    #Creating a list with obstacle segments
    segments=[] #[step_i, Xsafe_i or Xvel_i, Ysafe_i or Yvel_i,   step_i+1, Xsafe_i+1 or Xvel_i+1, Ysafe_i+1 or Yvel_i+1]
    i=0
    while i<len(Data_safe)-1:
        segments.append([Data_safe[i][4],Data_safe[i][5],Data_safe[i+1][4],Data_safe[i+1][5]])
        i=i+1
    
    
    #Checking all positions in the list until we find the ideal target
    i=0
    while i<len(Data_safe):
        
        brake=False
        xpi=Positions[i][4]
        ypi=Positions[i][5]
        
        #Calculating which way the car will turn
        direction=1 #right
        if ypi<0:
            direction=-1 #lef
            
        #Checking if the target is not intersecting with any segment   
        j=0        
        while j<len(segments) and SimpleIntersection((xpi,ypi),segments[j])!=1:
            j+=1
        
        
        #If we find a good target then we must calculate if there is any arc that connects the car to the target
        if j==len(segments): 
            
            #If the car and the segment are aligned then the arc is a straight line without problems
            if ypi==0:
                R=10000 #A big radius simulates a straight line
                evitement=False
                Car.target = [xpi,ypi]
                Car.direction = direction
                Car.radius = R
                Car.brake = brake

            
            #Checking if the car can make the turn to reach the target without colliding with anything(yes ind =1; no ind =0)
            ind,R,direction=AchievableTarget(segments,[xpi,ypi],Car.speed) 

            #If he succeeds then we have our ideal target
            if  ind==1:
                Car.target = [xpi,ypi]
                Car.direction = direction
                Car.radius = R
                Car.brake = brake
                
            #If the target is not reachable by the car we check if the external trajectory braking and steering fully allows you to join the traj on the right without hitting a wall
          
            Car.brake=True
            ind,R,direction =AchievableTargetAvoid(segments,[xpi,ypi],Car.speed, Car.direction)
            
            #If he succeeds then we have our ideal target
            if ind==1: 
                Car.radius = 10000
                Car.target = [xpi,ypi]
                Car.direction = direction
                Car.brake = brake
            
        i+=1
    
    #The code failed to calculate an ideal target
    print('ERREUR pas de cible optimale')
    Car.radius = 10000
    Car.brake = True
    Car.direction=1
    if Positions[0][5]<0:
        Car.direction=-1
    Car.target = [Positions[0][4],Positions[0][5]

    return(1)


    #######################################################################
    #######################################################################

def UpdateCar(Car):
    
    # def actualise2(R,v,position,obj,cible,orientation,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,vmax,N):
    epsilon=(parameters.tsb*Car.speed**2/R+parameters.Car_length/R)
    
    #If it's a reverse course
    if Car.brake:
        epsilon=parameters.epsilonmax*pi/180

    if Car.speed < Parameters.Max_reverse_speed :
        Car.speed = Parameters.Max_reverse_speed
        

    #If the car turns to left
    if Car.direction<0:
        epsilon=-epsilon
    
    
    #Calculate the real curvature radius
    R_real=abs(parameters.tsb*Car.speed**2/(epsilon)+parameters.Car_length/(epsilon)) 
    
   
    #coordinate of the target in relation to the position of the LIDAR
    xprim=0
    yprim=0
    xprim1=0
    yprim1=0
    psi=0
    
    
    #If speed differs from 0 then the point must be calculated considering the speed
    if Car.speed!=0:
        
        psi=Car.speed*parameters.Lidar_delta/R_real
        yprim= R_real*(1-cos(psi))
        xprim=(R_real-yprim)*sin(psi)
        
        #if the car is backing up
        if Car.speed<0:
            xprim=-(R_real-yprim)*sin(psi)
            
        #If the car turns to left
        if Car.direction<0:
            yprim=-yprim            
            psi=-psi
               
        rprim=sqrt(xprim**2+yprim**2)
        xprim1=rprim*cos(psi+orientation*2*pi/360) #correction en angle
        yprim1=rprim*sin(psi+orientation*2*pi/360)
        
        #if the car is backing up
        if Car.speed<0:
            xprim1=rprim*cos(pi-psi+orientation*2*pi/360)
            yprim1=rprim*sin(pi-psi+orientation*2*pi/360)

        
    #Calculating new angle input 
    Turn_Angle = psi*360/(2*pi)           #It will be used to set the angle of the wheels 
    Car.angle = Turn_Angle     
    
    #Calculating New Speed
    vmaxr=sqrt(parameters.Max_accelerationlateral*R) #Maximum speed we can maintain on a R-size radius curve
    Max_speed_allowed=min(vmaxr,parameters.Car_maxspeed) # Max velocity allowed
    
    a=min(1,(Max_speed_allowed-Car.speed)/(parameters.Lidar_delta*parameters.Max_acceleration))*parameters.Max_acceleration
    
    #If the car is above the recommended speed then it must brake
    if Car.speed >Max_speed_allowed:
        a=min(1,(Max_speed_allowed-Car.speed)/(parameters.Lidar_delta*parameters.Min_acceleration))*parameters.Min_acceleration
    
    #If it's a reverse course
    if Car.brake:
        a=Parameters.Min_acceleration
    
    NewSpeed=min(Parameters.Car_maxspeed,Car.speed+a*Parameters.Lidar_delta,Max_speed_allowed)
    Car.speed = NewSpeed
   
    
    return(1)



    
    