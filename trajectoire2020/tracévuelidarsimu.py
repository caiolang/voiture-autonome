    #Tracé de ce que voit le lidar
    
    x=position[0]
    y=position[1]
    i=0
    while i<len(MMMR):
        xl=MMMR[i][2] #abscisse du pt lidar dans le ref du lidar 
        yl=MMMR[i][3]
        theta=MMMR[i][0] #nb inc angle de coord polaires dans le ref du lidar
        thetac=(theta*alphainc+orientation)*2*pi/360  #angle de coord polaires dans le ref absolu
        r=MMMR[i][1]
        xlc=r*cos(thetac)+x #rcos(thetac) =abscisse dans le ref du lidar rotationné de orientation xlc=abscisse dans le ref absolu
        ylc=r*sin(thetac)+y

        xlp=MMMR[i][4] #abscisse du pt lidar dans le ref du lidar 
        ylp=MMMR[i][5]
        thetap=atan((ylp)/(xlp))
        if xlp<0:
            thetap=pi+atan(ylp/xlp)
        thetacp=thetap+orientation*2*pi/360
        rp=sqrt(xlp**2+ylp**2)
        xlpc=rp*cos(thetacp)+x
        ylpc=rp*sin(thetacp)+y


        plt.plot(xlc, ylc,"b:o")
        plt.plot(xlpc,ylpc,"r:o")
        i+=1