#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						bmt:/home/devel/catkin_ws/src/trajectory_sender/src/
rsync -avzh ./include/trajectory_sender/*.hpp 			bmt:/home/devel/catkin_ws/src/trajectory_sender/include/trajectory_sender/
rsync -avzh CMakeLists.txt 						bmt:/home/devel/catkin_ws/src/trajectory_sender/
rsync -avzh *.xml 								bmt:/home/devel/catkin_ws/src/trajectory_sender/
rsync -avzh ./launch/*.launch					bmt:/home/devel/catkin_ws/src/trajectory_sender/launch/
rsync -avzh *.yaml		 						bmt:/home/devel/catkin_ws/src/trajectory_sender/
rsync -avzh *.md								bmt:/home/devel/catkin_ws/src/trajectory_sender/
rsync -avzh ./urdf/*.urdf						bmt:/home/devel/catkin_ws/src/trajectory_sender/urdf/

echo "All done, Good Success!"