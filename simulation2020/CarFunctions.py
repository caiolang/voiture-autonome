from Parameters import *
from math import*

def UpdateCar(R,Speed,position,target,orientation,direction,Rimpact,reverse):
    
    epsilon=(parameters.tsb*Speed**2/R+parameters.Car_length/R)
    
    #If it's a reverse course
    if reverse:
        epsilon=parameters.epsilonmax*pi/180 # epsilon (rad), epsilonmax (degrees)
        

    #If the car turns to left
    if direction<0:
        epsilon=-epsilon
    
    
    #Calculate the real curvature radius
    R_real=abs(parameters.tsb*Speed**2/(epsilon)+parameters.Car_length/(epsilon)) 
    
   
    #coordinate of the target in relation to the position of the LIDAR
    xprim=0
    yprim=0
    xprim1=0
    yprim1=0
    thetaparc=0
    
    
    #If speed differs from 0 then the point must be calculated considering the speed
    if Speed!=0:
        
        thetaparc=Speed*parameters.Lidar_delta/R_real
        yprim= R_real*(1-cos(thetaparc))
        xprim=(R_real-yprim)*sin(thetaparc)
        
        #if the car is backing up
        if Speed<0:
            xprim=-(R_real-yprim)*sin(thetaparc)
            
        #If the car turns to left
        if direction<0:
            yprim=-yprim            
            thetaparc=-thetaparc
               
        rprim=sqrt(xprim**2+yprim**2)
        xprim1=rprim*cos(thetaparc+orientation*2*pi/360) #correction en angle
        yprim1=rprim*sin(thetaparc+orientation*2*pi/360)
        
        #if the car is backing up
        if Speed<0:
            xprim1=rprim*cos(pi-thetaparc+orientation*2*pi/360)
            yprim1=rprim*sin(pi-thetaparc+orientation*2*pi/360)

        
    
    #Calculation of the target in absolute reference
    xabs=xprim1+position[0]
    yabs=yprim1+position[1]
    
    #Simulating car movement 
    NewPosition=[xabs,yabs]
    Turn_Angle = thetaparc*360/(2*pi)            #It will be used to set the angle of the wheels
    NewOrientation=orientation + Turn_Angle
       
    
    #Calculating New Speed
    
    vmaxr=sqrt(parameters.Max_accelerationlateral*R) #Maximum speed we can maintain on a R-size radius curve
    vmaxant=1.1*sqrt(-2/3*parameters.Min_acceleration*Rimpact) #Maximum speed to give to dodge a wall        
    Max_speed_allowed=min(vmaxr,parameters.Car_maxspeed,vmaxant) # Max velocity allowed
    
    a=min(1,(Max_speed_allowed-Speed)/(parameters.Lidar_delta*parameters.Max_acceleration))*parameters.Max_acceleration
    
    #If the car is above the recommended speed then it must brake
    if Speed >Max_speed_allowed:
        a=min(1,(Max_speed_allowed-Speed)/(parameters.Lidar_delta*parameters.Min_acceleration))*parameters.Min_acceleration
    
    #If it's a reverse course
    if reverse:
        a=parameters.Min_acceleration
   
    NewSpeed=min(parameters.Car_maxspeed,Speed+a*parameters.Lidar_delta)
   
    return(NewPosition,NewOrientation,NewSpeed,Turn_Angle)






