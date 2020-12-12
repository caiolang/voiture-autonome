from rplidar import RPLidar

#Lidar launching
PORT_NAME ='/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)

def run():
    '''This function returns a table of distances for each angle of a complete rotation.'''

    try:
        for i,scan in enumerate(lidar.iter_scans()):
            if i==0:
                lidar.set_pwm(660)
            data=[]
            for temp in scan:
                if temp[1]>=90 and temp[1]<=270:
                    data.append(([temp[1]-180)%360,temp[2]])
            yield data
            
    except (KeyboardInterrupt, SystemExit):
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

