

from math import *
from Parameters import *
from MathFunctions import*


def AchievableTarget(segments,target,Speed):
    
    """
    The function checks if the car can make the required curvature to reach the target, taking into account its speed
    
    Return [id, radius, direction}
            id = 1 -> achievable               else id =0
            direction = 1 -> right             direction = -1 -> left
    """
    
    
    Rminamaxlat=Speed**2/parameters.Max_accelerationlateral
    Rminepsilonmax=parameters.tsb*Speed**2/(parameters.epsilonmax*pi/180)+parameters.Car_length/(parameters.epsilonmax*pi/180)
    Rmin=max(Rminamaxlat,Rminepsilonmax)
    Rmax=abs(CurvatureRadius(target))/3
    xp=target[0]
    yp=target[1]
    Ns=len(segments)
    
    #coeficient
    K=0
    if xp!=0:
        K=yp/xp
           
    
    #Calculating which way the car will turn
    direction=1 #right
    if yp<0:
        direction=-1 #lef
    
    #If the radius of curvature is greater than the minimum possible then the objective is not reachable
    if Rmin>Rmax:
        return(0,Rmax,direction)
    
    #Adding possible radius values between the minimum and the maximum in the list R []
    R=[]
    Nr=100
    i=0
    while i<Nr:
        R.append(Rmax-i*(Rmax-Rmin)/(Nr-1))
        i+=1
        
    #Checking all posible radius
    i=0
    while i<Nr:
        
        r=R[i]
        yc=direction*r
        
        #If the car and the segment are aligned then the arc is a straight line without problems
        if yp==0:
            return(1,Rmax,1) 
        
        
        if xp!=0:
            xinter=(-2*K*yc)/(1+K**2)
            yinter=K*xinter
            
            j=0
            while (j<Ns and IntersectionArc([xinter,yinter],segments[j])!=1):
                j+=1
            if j==Ns:
                return(1,r,direction)
            return(0,r,direction)    
            
        xinter=0
        yinter=direction*2*r
        theta=180
        j=0
        while (j<Ns and IntersectionArc([xinter,yinter],segments[j])!=1):
            j+=1
        if j==Ns:
            return(1,r,direction)
        return(0,r,direction)
        i+=1



def AchievableTargetAvoid(segments,target,Speed,direction): 

    Rminamaxlat=Speed**2/parameters.Max_accelerationlateral
    Rminepsilonmax=parameters.tsb*Speed**2/(parameters.epsilonmax*pi/180)+parameters.Car_length/(parameters.epsilonmax*pi/180)
    Rmin=max(Rminamaxlat,Rminepsilonmax)
    Rmax=abs(CurvatureRadius(target))/3
    xp=target[0]
    yp=target[1]
    Ns=len(segments)
    
    
    indtraj,trajectory=ReverseTrajectory(Speed,target,direction)
    Nt=len(trajectory)
    
    if indtraj==0:
        return(0,10,direction) 
        
   
    
    i=0
    while i<Nt:
        j=0
        while j<Ns and intersectionsegments(trajectory[i],segments[j])==0:
            j+=1
            
        if j!=Ns:
            return(0,10000,direction) #la valeur du rayon (1000) est juste la pour respecter la forme de sortie imposée par la fct trouvecible 
        i+=1
            
             
    print('evitement ok')
    return(1,10000,direction) #la valeur du rayon (1000) est juste la pour respecter la forme de sortie imposée par la fct trouvecible 




def ReverseTrajectory(speed,target,direction):
    

    
    Rminamaxlat=speed**2/parameters.Max_accelerationlateral
    Rminepsilonmax=parameters.tsb*speed**2/(parameters.epsilonmax*pi/180)+parameters.Car_length/(parameters.epsilonmax*pi/180)
    Rmin=max(Rminamaxlat,Rminepsilonmax)
    Rmax=abs(CurvatureRadius(target))/4
    xp=target[0]
    yp=target[1]
    

    T=3 #horizont de prévision en secondes
    nb=10 #nb de pts que comporte la trajectoire (plus on en a plus c'est précis) 
    deltaT=T/nb #incréments des temps entre deux pts de la trajectoire
    
    ptstrajectoire=[] #contient les pts de la traj aux instants i*deltaT
    
    #initialiation
    
    vprim=max(speed + parameters.Min_acceleration*deltaT,1) 
    Rminamaxlatprim=vprim**2/parameters.Max_accelerationlateral
    Rminepsilonmaxprim=parameters.tsb*vprim**2/(parameters.epsilonmax*pi/180)+parameters.Car_length/(parameters.epsilonmax*pi/180)
    Rminprim=max(Rminamaxlatprim,Rminepsilonmaxprim)
    theta=0
    thetaparc=(vprim/Rminprim)*deltaT*direction
    thetaprim=theta+thetaparc
    yprimreft=direction*Rminprim*(1-cos(thetaparc)) #ordonnée du pt de traj à t+dt dans le ref de la voiture à t 
    xprimreft=(Rminprim-yprimreft)*sin(thetaparc) #abscisse du pt de traj à t+dt dans le ref de la voiture à t 
    rprim=sqrt(xprimreft**2+yprimreft**2)
    x=0
    y=0
    xprim=x+rprim*cos(thetaprim)
    yprim=x+rprim*sin(thetaprim)
    
    ptstrajectoire.append([x,y])
    ptstrajectoire.append([xprim,yprim])
    
   #hérédité
   
    i=1
    
    while i<nb:
        x,y=xprim,yprim
        speed=vprim
        theta=thetaprim
        
        vprim=max(speed + parameters.Min_acceleration*deltaT,1) 
        Rminamaxlatprim=vprim**2/parameters.Max_accelerationlateral
        Rminepsilonmaxprim=parameters.tsb*vprim**2/(parameters.epsilonmax*pi/180)+parameters.Car_length/(parameters.epsilonmax*pi/180)
        Rminprim=max(Rminamaxlatprim,Rminepsilonmaxprim)
        thetaparc=(vprim/Rminprim)*deltaT*direction
        thetaprim=theta+thetaparc
        yprimreft=direction*Rminprim*(1-cos(thetaparc)) #ordonnée du pt de traj à t+dt dans le ref de la voiture à t 
        xprimreft=(Rminprim-yprimreft)*sin(thetaparc) #abscisse du pt de traj à t+dt dans le ref de la voiture à t 
        rprim=sqrt(xprimreft**2+yprimreft**2)
        xprim=x+rprim*cos(thetaprim)
        yprim=x+rprim*sin(thetaprim)
        ptstrajectoire.append([xprim,yprim])
        
        
        i+=1
    

    
    trajectoire=[]
    i=0
    while i<(len(ptstrajectoire)-1):
        trajectoire.append([ptstrajectoire[i][0],ptstrajectoire[i][1],ptstrajectoire[i+1][0],ptstrajectoire[i+1][1]])
        i=i+1
    
  
    xp=target[0]
    yp=target[1]
    j=0
    while j<len(trajectoire) and SimpleIntersection(target,trajectoire[j])!=1:
        j+=1
    if j==len(trajectoire): #alors aucun segment de la traj ne coupe OP 
        return(0,trajectoire)
    
    return(1,trajectoire[:(j+2)])



def FindTarget (Advance_Data,Speed):
    """
    
    
        Advance_Data      :  [step_i, distance_i,x_i,y_i, Xsafe_i or Xvel_i, Ysafe_i or Yvel_i]
    """
    
    R=0

    
    #Filtering the data and creating a list with only the positions of the safe zone and the step number
    Positions=[]  #[step_i, Xsafe_i or Xvel_i, Ysafe_i or Yvel_i]
    i=0 
    while i<len(Advance_Data):
        d=Advance_Data[i][4]**2+Advance_Data[i][5]**2
        liste=Advance_Data[i]+[d]
        Positions.append(liste)
        i+=1
   
    
    

    Positions.sort(key=lambda x: x[6])
    Positions.reverse()
    
    #Creating a list with obstacle segments
    segments=[] #[step_i, Xsafe_i or Xvel_i, Ysafe_i or Yvel_i,   step_i+1, Xsafe_i+1 or Xvel_i+1, Ysafe_i+1 or Yvel_i+1]
    i=0
    while i<len(Advance_Data)-1:
        segments.append([Advance_Data[i][4],Advance_Data[i][5],Advance_Data[i+1][4],Advance_Data[i+1][5]])
        i=i+1
        
    
    Rimpact=1 #Distance to the wall in the direction straight ahead of the car
    
    #Checking all positions in the list until we find the ideal target
    i=0
    while i<len(Advance_Data):
        
        reverse=False
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
                Rimpact=xpi
                return([xpi,ypi],R,direction,reverse,Rimpact)
            
            #Checking if the car can make the turn to reach the target without colliding with anything(yes ind =1; no ind =0)
            ind,R,direction=AchievableTarget(segments,[xpi,ypi],Speed) 

            #If he succeeds then we have our ideal target
            if  ind==1:
                Rimpact=Positions[int(90/360*parameters.Lidar_steps)][2] 
                return([xpi,ypi],R,direction,reverse,Rimpact)
                
            #If the target is not reachable by the car we check if the external trajectory braking and steering fully allows you to join the traj on the right without hitting a wall
          
            reverse=True
            ind,R,direction =AchievableTargetAvoid(segments,[xpi,ypi],Speed, direction)
            
            #If he succeeds then we have our ideal target
            if ind==1: 
                R=10000
                Rimpact=Advance_Data[int(90/360*parameters.Lidar_steps)][2]
                return([xpi,ypi],R,direction,reverse,Rimpact)
            
        i+=1
    
    #The code failed to calculate an ideal target
    print('ERREUR pas de cible optimale')
    direction=1
    if Positions[0][5]<0:
        direction=-1
    return([Positions[0][4],Positions[0][5]],10000,direction,True,Positions[int(90/360*parameters.Lidar_steps)][2])
    