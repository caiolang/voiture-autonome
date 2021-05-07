# Commande Simple - *Voiture Autonome*

The code bellow allow the car of our PIE team to drive autonomously as long as all devices (except the inertial unit) are connected as indicated in the documentation and the initialisation instruction bellow are executed. 

Here is the frame of the algorithm :

## Initialisation :
The Raspberry imports useful librairies and connect to other components. This part may raise exception, in particular if the LIDAR port has been changed (see instructions bellow)

## Functions definition :
These function are used to communicate with other components and choices about the form of data have been made in order to facilitate programmation in particular with LIDAR data. 

## Core :
- Wait until the user press cntrl + C
- While the user does not press Crtl + C again :
    - Collect data from the LIDAR, select a range 2 * window angles centered on the front position, order them from left to right (so the front position became the window one) then replace each value per the average of the 2 * average_number values around it
    - Find the objective which is the farthest point from the car in data_r_av.                                       #
    - Calculate the theoretical curve which is 1/R with  the maximal radius of the circular trajectory leading to the objective.
    -  Depending on the distance from walls around the car, select a "regime" which mean a speed and coefficient we multiply the theoretical curve by to get the curve command. It allows to be more reactive in critical situations and faster in controlled ones.
    -  Send order to the ESC and servo
- Disconnection of the lidar and the ESC

