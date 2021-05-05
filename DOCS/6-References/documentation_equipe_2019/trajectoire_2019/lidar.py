from intersection2 import *



def lidar(environment,position,orientation,N):
    n=len(environment)
    M=[]
    i=0
    alphainc=360/N
    while i<N:
        j=0
        intersectparliste=[]
        while j<n:
            k=0
            
            intersect=[]
            while k<(len(environment[j])-1):
                [res,inter]=intersection2(position,i*alphainc,environment[j][k]+environment[j][k+1]) #inter=coordonnées cartésiennes du pt d'intersection dans le repère du lidar
                if res==1:
                    segment=environment[j][k]+environment[j][k+1]
                    d=[(inter[0])**2+(inter[1])**2]
                    contenu=segment+inter+d #[x1,y1,x2,y2,xinter,yinter,d]
                    if (inter[0])*(cos(i*alphainc*2*pi/360))+(inter[1])*(sin(i*alphainc*2*pi/360))>=0:
                            intersect.append(contenu)
                k+=1
            if len(intersect)!=0:
                intersect.sort(key=lambda x: x[6])
            
                intersectparliste.append(intersect[0]) #contient le pt d'intersection le plus proche de chaque liste
            j+=1
        if len(intersectparliste)==0:
            print('SAAAAAAAAAAANGLIER')
        intersectparliste.sort(key=lambda x: x[6])
        r0=sqrt(intersectparliste[0][6])
        alpha0=i
        #calage de l'orientation de M : il faut que la première ligne de M corresponde à orientation 
        M.append([(alpha0-orientation/alphainc)%N,r0])
        i+=1
    M.sort(key=lambda x: x[0])
    return(M)
        
                    