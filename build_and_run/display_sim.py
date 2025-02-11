# IMPORT THE FILE THAT YOU WANT TO RUN
# Ex. import roundabout to run the roundabout simulation
from roundabout2 import *

#defining the intersection
intersection = Intersection()
#assigning the simulation
sim = intersection.get_sim()
#defining the window that displays the simulation
win = Window(sim)

#starts running the simulation then displays it
win.run()
win.show()