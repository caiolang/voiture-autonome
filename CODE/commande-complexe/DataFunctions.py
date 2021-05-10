"""
In this file are all methods aimed at data acquisition and treatment

Data Types:       Lidar_Data        :  [step_i, distance_i]
                  Complete_Data     :  [step_i, distance_i, x_i, y_i]
                  Data_Safe         :  [step_i, distance_i,x_i,y_i, Xsafe_i, Ysafe_i]
                  Advance_Data      :  [step_i, distance_i,x_i,y_i, Xsafe_i or Xvel_i, Ysafe_i or Yvel_i]
"""

from math import *
import copy
from Parameters import *
from MathFunctions import*


def GetLidarSimulation(environment,position,orientation):

    """
    This function simulates the operation of the LIDAR by returning a list with the polar coordinates of 
    the location of the obstacles

    environment    -> List with all objects that are in the circuit as well as the edges of the circuit
    position      -> Car position [x,y]
    orientation   -> Car orientation (0,360)

    Return

    Lidar_Data    : [step_i, distance_i]      i(0, Lidar_Steps)
    """

    n=len(environment)
    LidarData=[]
    i=0
    #Going through all the sensor steps
    while i<parameters.Lidar_steps:
        j=0
        intersectparliste=[]
        #Going through all objects and edges
        while j<n:
            k=0
            
            intersect=[]
            while k<(len(environment[j])-1):
                [res,inter]=Intersection(position,i*parameters.Lidar_stepsize,environment[j][k]+environment[j][k+1]) 
                
                if res==1:
                    segment=environment[j][k]+environment[j][k+1]
                    d=[(inter[0])**2+(inter[1])**2]
                    contenu=segment+inter+d
                    
                    if (inter[0])*(cos(i*parameters.Lidar_stepsize*2*pi/360))+(inter[1])*(sin(i*parameters.Lidar_stepsize*2*pi/360))>=0:
                            intersect.append(contenu)
                k+=1
                
            if len(intersect)!=0:
                intersect.sort(key=lambda x: x[6])
                intersectparliste.append(intersect[0]) #The closest intersection
            j+=1
        
            
        intersectparliste.sort(key=lambda x: x[6])

        if len(intersectparliste)==0:
            r0 = parameters.Lidar_maxdistance
            alpha0=i
        
        else:
            r0=sqrt(intersectparliste[0][6])
            alpha0=i
        
        #Ensuring that LIDAR only reads what is at its physical limit
        if(r0 > parameters.Lidar_maxdistance): r0 = parameters.Lidar_maxdistance
        
        #Calibration of the orientation of M: the first line of M must correspond to orientation
        LidarData.append([(alpha0-orientation/parameters.Lidar_stepsize)%parameters.Lidar_steps,r0])
        i+=1

    LidarData.sort(key=lambda x: x[0])
    
    return(LidarData)



def SafeZone (Lidar_Data, car_radius,radius_margin):
    
    """
    This function defines a safety zone within the track layout where the car can pass without 
    running the risk of hitting the sideswe trace another track inside the original track to 
    ensure that the car will not get close to any wall or other obstacle

        Lidar_Data    : [step_i, distance_i]                             i(0,N)
        Data_Sorted   : [step_i, distance_i]                             i(-N/4,3N/4) gets the angle interval (-90;270)
        Complete_Data : [step_i, distance_i,x_i,y_i] 
        Safe_Data     : [step_i, distance_i,x_i,y_i, Xsafe_i, Ysafe_i]   Xsafe,Ysafe new calculate points

    Return Data_Safe
    """
    N=parameters.Lidar_steps                                     # #Numbers of data that LIDAR receives in one complete turn
    step = parameters.Lidar_stepsize                             #Angled representation of each LIDAR step
    radius = parameters.Car_radius + parameters.radius_margin     #Parameter that will be used in calculations

    #Organizing the received data so that it is in the order [-90º, 270º]. This is necessary because the front of the car is the 0º point
    A=copy.deepcopy(Lidar_Data[0:int(N/4)])
    B=copy.deepcopy(Lidar_Data[int(3*N/4):N])
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
        
        i=i+1
    
    
    return(Data_Safe)

                                        
                                        
                                        ############# IMPROVED VERSION ###############

def SafeZone_v2 (Lidar_Data,Lidar_DataOld,vOld, direction, Speed, car_radius, radius_margin):
    
    """
    This function defines a safety zone within the track layout where the car can pass without 
    running the risk of hitting the sideswe trace another track inside the original track to 
    ensure that the car will not get close to any wall or other obstacle

        Lidar_Data    : [step_i, distance_i]                             i(0,N)
        Data_Sorted   : [step_i, distance_i]                             i(-N/4,3N/4) gets the angle interval (-90;270)
        Complete_Data : [step_i, distance_i,x_i,y_i] 
        Safe_Data     : [step_i, distance_i,x_i,y_i, Xsafe_i, Ysafe_i]   Xsafe,Ysafe new calculate points

    Return Data_Safe
    """
    N=parameters.Lidar_steps                                     # #Numbers of data that LIDAR receives in one complete turn
    step = parameters.Lidar_stepsize                             #Angled representation of each LIDAR step
    radius = parameters.Car_radius + parameters.radius_margin     #Parameter that will be used in calculations


    #Organizing the received data so that it is in the order [-90º, 270º]. This is necessary because the front of the car is the 0º point
    A=copy.deepcopy(Lidar_Data[0:int(N/4)])
    B=copy.deepcopy(Lidar_Data[int(3*N/4):N])
    A.reverse()
    B.reverse()
    Data_Sorted=copy.deepcopy(A+B)


    #Organizing the received data so that it is in the order [-90º, 270º]. This is necessary because the front of the car is the 0º point
    C=copy.deepcopy(Lidar_DataOld[0:int(N/4)])
    D=copy.deepcopy(Lidar_DataOld[int(3*N/4):N])
    C.reverse()
    D.reverse()
    Data_SortedOld=copy.deepcopy(C+D)


    #We are adding Cartesian positions X, Y to the data because the LIDAR only gives polar coordinates.
    Complete_Data=[]
    Complete_DataOld=[]
    i=0
    while i<len(Data_Sorted):
        theta=Data_Sorted[i][0]
        r=Data_Sorted[i][1]
        xi=r*cos(theta*step*2*pi/360)
        yi=r*sin(theta*step*2*pi/360)
        Complete_Data.append([theta,r,xi,yi])

        thetaOld=Data_SortedOld[i][0]
        rOld=Data_SortedOld[i][1]
        xiOld=rOld*cos(thetaOld*step*2*pi/360)
        yiOld=rOld*sin(thetaOld*step*2*pi/360)
        Complete_DataOld.append([thetaOld,rOld,xiOld,yiOld])
        i=i+1
        
    
    new_vOld=[[(Complete_Data[0][2]-Complete_DataOld[0][2])/parameters.Lidar_delta, (Complete_Data[0][3] - Complete_DataOld[0][3])/(parameters.Lidar_delta)]]
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

        #Vitesse relative

        vix = (Complete_Data[i][2] - Complete_DataOld[i][2])/(parameters.Lidar_delta)
        viy = (Complete_Data[i][3] - Complete_DataOld[i][3])/(parameters.Lidar_delta)
        new_vOld.append([vix,viy])        
        
        accx = (vix-vOld[i][0])/parameters.Lidar_delta #will determine wether we face a real moving object or an artefact (e.g. caused by edges of the racetrack)
        accy = (viy-vOld[i][1])/parameters.Lidar_delta
        
        acc = sqrt(accx**2+accy**2)

            #Parameter that will be used in calculations
        if ((vix < 0.)): # & (abs(acc)<parameters.Max_acceleration) the obstacle gets closer at "negative" speed relatively to the car
            
            if (vix < -parameters.Car_maxspeed):
                vix = -parameters.Car_maxspeed

            d = sqrt((xip1-xi)**2 + (yip1-yi)**2)
            if (d > parameters.edge_gap) & (d < parameters.light_curve_gap):  #we have an artefact ie a turn
                    radius = parameters.Car_radius + abs(vix)/(1/3.6)*parameters.radius_margin
                    
                    if abs(yip1-yi)>0.001:
                        #left turn bc vi<0
                        K = -(xip1-xi)/(yip1-yi)
                    
                        #moving the point to a value that is safely away from the wall
                        xp1=xi+radius/sqrt(1+K**2) 
                        yp1=yi+K*radius/sqrt(1+K**2)
                    
                        #moving the point to a value that is safely away from the wall
                        xp2=xi-radius/sqrt(1+K**2)
                        yp2=yi-K*radius/sqrt(1+K**2)
                    
                    if abs(yip1-yim1)<=0.001:
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

                radiusx = parameters.Car_radius + abs(vix)/(2/3.6)*parameters.radius_margin
                radiusy = parameters.Car_radius + parameters.radius_margin
                
                #If the track is shortening. It can be a curve or a taper
                if abs(yip1-yim1)>0.00001:
                    #Calculating the coefficient. Tangent of the angle between points
                    K=-(xip1-xim1)/(yip1-yim1) 
                
                    #moving the point to a value that is safely away from the wall
                    xp1=xi+radiusx/sqrt(1+K**2) 
                    yp1=yi+K*radiusy/sqrt(1+K**2)
                
                    #moving the point to a value that is safely away from the wall
                    xp2=xi-radiusx/sqrt(1+K**2)
                    yp2=yi-K*radiusy/sqrt(1+K**2)
                
                #If the track is normal or increasing.If the track is normal or increasing, I don't need to calculate the ideal position. Just make sure the car is far away from the "radius" of the edges "
                if abs(yip1-yim1)<0.00001:
                    yp1=yi+radiusy
                    yp2=yi-radiusy
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

            if (vix > parameters.Car_maxspeed):
                    vix = parameters.Car_maxspeed
            
            d = sqrt((xip1-xi)**2 + (yip1-yi)**2)
            if (d > parameters.edge_gap) & (d < parameters.light_curve_gap):                       #we have an artefact ie a turn
                    radius = parameters.Car_radius + 5*abs(vix)*parameters.radius_margin
                    
                    if abs(yip1-yi)>0.001:
                        #right turn bc vi>0
                        K = -(xi-xim1)/(yi-yim1)
                    
                        #moving the point to a value that is safely away from the wall
                        xp1=xi+radius/sqrt(1+K**2) 
                        yp1=yi+K*radius/sqrt(1+K**2)
                    
                        #moving the point to a value that is safely away from the wall
                        xp2=xi-radius/sqrt(1+K**2)
                        yp2=yi-K*radius/sqrt(1+K**2)
                    
                    if abs(yip1-yim1)<=0.001:
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
                Data_Safe.append(Complete_Data[i]+[xi,yi])
        
        i=i+1


    # if (vi > 0): #the obstacle gets closer at "positive" speed relatively to the car

        #     # radiusx = parameters.Car_radius + abs(vi)/(parameters.Car_maxspeed)*parameters.radius_margin
        #     radiusx = parameters.Car_radius + 5/(parameters.Car_maxspeed)*parameters.radius_margin
        #     radiusy  = parameters.Car_radius + parameters.radius_margin

        #     #If the track is shortening. It can be a curve or a taper
        #     if abs(yip1-yim1)>0.00001:
        #         #Calculating the coefficient. Tangent of the angle between points
        #         K=-(xip1-xim1)/(yip1-yim1) 
            
        #         #moving the point to a value that is safely away from the wall
        #         xp1=xi+radiusx/sqrt(1+K**2) 
        #         yp1=yi+K*radiusy/sqrt(1+K**2)
            
        #         #moving the point to a value that is safely away from the wall
        #         xp2=xi-radiusx/sqrt(1+K**2)
        #         yp2=yi-K*radiusy/sqrt(1+K**2)
            
        #     #If the track is normal or increasing.If the track is normal or increasing, I don't need to calculate the ideal position. Just make sure the car is far away from the "radius" of the edges "
        #     if abs(yip1-yim1)<0.00001:
        #         yp1=yi+radiusy
        #         yp2=yi-radiusy
        #         xp1=xi
        #         xp2=xi

        #     #Choosing the closest point to the car    
        #     d1=(xp1)**2+(yp1)**2
        #     d2=(xp2)**2+(yp2)**2
        #     liste1=Complete_Data[i]+[xp1,yp1]
        #     liste2=Complete_Data[i]+[xp2,yp2]
        

        #     if d1<=d2:
        #         Data_Safe.append(liste1)
        #     if d1>d2:
        #         Data_Safe.append(liste2)

        # else:
        #     Rimpact=1000
        #     theta=Data_Sorted[i][0]
        #     x=Rimpact*cos(theta*step*2*pi/360)
        #     y=Rimpact*sin(theta*step*2*pi/360)
        #     d=x**2+y**2
        #     Data_Safe.append([0,Rimpact,0,0,x,y])

    

    # new_Lidar_DataOld=[]
    # for i in range(0,len(Lidar_Data)):
    #     new_Lidar_DataOld.append(Lidar_Data[i])
    
    return(Data_Safe,new_vOld)  #new_Lidar_DataOld




def ImproveData (Lidar_DataOld,Lidar_Data,Data_Safe,Speed,orientation,orientationm1):
    
    """
    The function calculates new positions for obstacles now taking into account the car's relative speed in relation to each point. We need the accelerometer for that.
    
    Return:
    
                    Advanced_Data          :  [step_i, distance_i,x_i,y_i, Xsafe_i or Xvel_i, Ysafe_i or Yvel_i]
    
    """
    
    
    
    """
    
        Filtering the data within the alpha range.

            Data_Safe              -(90,270)
            DataSafeFiltered       (-alpha,+alpha)
    """
    i=0
    DataSafeFiltered = []
    N=parameters.Lidar_steps                                     # #Numbers of data that LIDAR receives in one complete turn
    while i<len(Data_Safe):
        if (Data_Safe[i][0]*360/N)<parameters.Alpha:
            DataSafeFiltered.append(Data_Safe[i])
        i+=1
    i=0
    while i<len(Data_Safe):
        if (Data_Safe[i][0]*360/N)>(360-parameters.Alpha):
            DataSafeFiltered.append(Data_Safe[i])
        i+=1
    i=0



    temp=[]

    while i<len(DataSafeFiltered):

        theta=DataSafeFiltered[i][0]
        theta=int(theta)
        
        #Calculating the relative speed. In reality (orientation-orientationm1) must be replaced by the accelerometer data
        vrel=(Lidar_Data[theta][1]-Lidar_DataOld[(theta+int((orientation-orientationm1)/360*N))%N][1])/parameters.Lidar_delta 
        
        #Polar coordinates of the reference point (angle and radius)
        thetap=atan(DataSafeFiltered[i][5]/DataSafeFiltered[i][4])       
        rp=sqrt(DataSafeFiltered[i][5]**2+DataSafeFiltered[i][4]**2) 
        
        
        #Recalculating the safety points according to the relative speed between the car and the objects. |||Understand better|||
        if vrel<0:
            rpprim=min(rp*Speed*cos(thetap)/(-vrel),parameters.Maxdist_relative) 
                    
        if vrel>=0:
            rp=sqrt(DataSafeFiltered[i][5]**2+DataSafeFiltered[i][4]**2)
            rpprim=parameters.Maxdist_relative+rp
                        
        xpprim=rpprim*cos(thetap)
        ypprim=rpprim*sin(thetap)
                    
        temp.append((DataSafeFiltered[i]+[xpprim,ypprim]))
        i+=1

    i=0
    
    #Updating the data set with the new positions calculated according to the relative speed of the car and the objects
    Advanced_Data=copy.deepcopy(Data_Safe)
    while i<len(temp):
        j=0
        while j<len(Data_Safe):
            if temp[i][0]==Advanced_Data[j][0]:
                Advanced_Data[j][4]=temp[i][6]
                Advanced_Data[j][5]=temp[i][7]
            j+=1
        i+=1
    
    return(Advanced_Data)


                        ########### IMPROVED VERSION #############

def ImproveData_v2 (Lidar_DataOld,Lidar_Data,Data_Safe,Speed,orientation,orientationm1):
    
    """
    The function calculates new positions for obstacles now taking into account the car's relative speed in relation to each point. We need the accelerometer for that.
    
    Return:
    
                    Advanced_Data          :  [step_i, distance_i,x_i,y_i, Xsafe_i or Xvel_i, Ysafe_i or Yvel_i]
    
    """
    
    
    
    """
    
        Filtering the data within the alpha range.

            Data_Safe              -(90,270)
            DataSafeFiltered       (-alpha,+alpha)
    """
    temp=[]
    i=0
    
    #Updating the data set with the new positions calculated according to the relative speed of the car and the objects
    Advanced_Data=copy.deepcopy(Data_Safe)
    while i<len(temp):
        j=0
        while j<len(Data_Safe):
            if temp[i][0]==Advanced_Data[j][0]:
                Advanced_Data[j][4]=temp[i][6]
                Advanced_Data[j][5]=temp[i][7]
            j+=1
        i+=1
    
    return(Advanced_Data)






