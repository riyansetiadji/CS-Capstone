# CS-Capstone
CS Capstone

TEAM:
Aaron Lutz
Adam Chmurzynski
Andrew Herzberg
Brian Rider
Saul Figueroa
RiyanSetiadji

Create a directory 'build' in this folder. 
Call cmake from the directory.
Make the executable.
Call the executable.

Command list:
mkdir build
cd build
cmake ..
make
./hand_tracking

Chances are your kinect drivers are not right, in which
case it will not be able to find the kinect. It works for
me with libfreenect, follow these instructions:

http://faltastico.tumblr.com/post/90660911262/giving-kinect-libfreenect-a-spin-on-ubuntu-1404

I also installed a bunch of other crap I can't remember which 
may be covertly enabling my build environment. That would be sinister. It might be SensorKinect instead.

I've made a few assumptions about how tracking should go with this architecture that I will try to write some documentation for.
Basically, the program is a linked list of Tracker objects, each of which is assigned an algorithm responsible for finding and remembering one 'thing' within the scene. Everything is controlled by a TrackerManager that interfaces with OpenNI and the Kinect, and renders the visualization. 



