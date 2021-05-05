from calculetrajevitementtronquee import *
import matplotlib.pyplot as plt


v=4
p=(0.01,-1)
sgyp=-1
ind,traj=calculetrajevitementtronquee(v,p,sgyp)
Nt=len(traj)
xt=[]
yt=[]
    
i=0
while i<Nt:
    xt.append(traj[i][0])
    yt.append(traj[i][1])
        
    i+=1
plt.plot(xt,yt,'r+')
plt.plot([0,p[0]],[0,p[1]],'bo')
plt.axis('equal')
plt.show()
    
    
    