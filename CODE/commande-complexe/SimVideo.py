
# %matplotlib inline
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
from IPython.display import HTML

from Simulation import *

Steps = 200 #number of simulation steps

#making all the enviroment
RaceTrack = MakeEnv()

#Car1
Car1Position = [10,-8]
Car1Speed = 0
Car1Orientation = 180 # degrees
Car1Orientationold = 0
Car1Orientationold = 0
Lidar1 = 360*[[0,0]]


#Car2
Car2Position = [10,-9] #default 10 -9
Car2Speed =0
Car2Orientation = 180 # degrees
Car2Orientationold = 0
Car2Orientationold = 0
Lidar2 = 360*[[0,0]]
direction = 1
v = 360*[[0,0]] #vitesse relative obstacle

#simulation
Car1x =[]
Car1y =[]
Car1ori =[]
Car2x =[]
Car2y =[]
Car2ori = []
Car2target = []

j=0
while j < Steps:
    
    temp1 = Car1Orientation;
    temp2 = Car2Orientation;
    [Car1Position,Car1Orientation,Lidar1,v,Car1Speed,Target] = Simulation_v2(Car1Position,Car1Speed,Car1Orientation,Car1Orientationold,Lidar1, v, direction, RaceTrack + [CarRectangle(Car2Position,Car2Orientation)])
    [Car2Position,Car2Orientation,Lidar2,v,Car2Speed,Target] = Simulation_v2(Car2Position,Car2Speed,Car2Orientation,Car2Orientationold,Lidar2, v, direction, RaceTrack + [CarRectangle(Car1Position,Car1Orientation)])
    Car1Orientationold = temp1
    Car2Orientationold = temp2
    
    Car1x.append(Car1Position[0])
    Car1y.append(Car1Position[1])
    Car1ori.append(Car1Orientation)
    Car2x.append(Car2Position[0])
    Car2y.append(Car2Position[1])
    Car2ori.append(Car2Orientation)

    # if (len(Target)!=0):
    #     Car2target.append([Car2Position[0]+Target[0], Car2Position[1]+Target[1]])
    # else:
    #     Car2target.append([1000,1000])

    j+=1

# print(v)
 
 
#Plotting
fig = plt.figure(figsize=(20, 18))
plt.axis('equal')

#Racetrack
i =0
n=len(RaceTrack)
while i<n:
    nn=len(RaceTrack[i])
    j=0
    while j<(nn-1):
        plt.plot([RaceTrack[i][j][0],RaceTrack[i][j+1][0]],[RaceTrack[i][j][1],RaceTrack[i][j+1][1]],"c-.")
        j+=1
    i+=1

#
    
    
    
ax = fig.add_subplot(111)
ax.set_xlim(-1,16)
ax.set_ylim(-11,1)

patch1 = patches.Rectangle((0, 0), 0, 0, fc='k')
patch2 = patches.Rectangle((0, 0), 0, 0, fc='r') 
# patch3 = patches.Rectangle((0, 0), 0, 0, fc='g')
point1, = ax.plot([], [],'ko')

def init():
    ax.add_patch(patch1)
    ax.add_patch(patch2)
    # ax.add_patch(patch3)
    point1.set_data([], [])
    return (patch2,point1) #patch1,patch3

def animate(i):
    patch1.set_width(0.4)
    patch1.set_height(0.2)
    patch1.set_xy([Car1x[i], Car1y[i]])
    patch1.angle = Car1ori[i]
    patch2.set_width(0.4)
    patch2.set_height(0.2)
    patch2.set_xy([Car2x[i], Car2y[i]])
    patch2.angle = Car2ori[i]
    # patch3.set_width(0.1)
    # patch3.set_height(0.1)
    # patch3.set_xy(Car2target[i])

    return (patch1, patch2, point1) #patch3

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=len(Car1x),
                               interval=parameters.Lidar_delta*1000,
                               blit=True)


# HTML(anim.to_html5_video()) # Used to make HTML video on a python notebook
plt.show()
HTML.warnings.warn(message, mplDeprecation, stacklevel=1)