import lidar_get
import time

old_t=time.time()
l=0

for i,data in enumerate(lidar_get.run()):
    t=time.time()
    if i==150:
        print(data)
