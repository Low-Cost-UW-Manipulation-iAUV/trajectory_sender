#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/trajectory_sender/src/
rsync -avzh ./include/trajectory_sender/*.hpp 	eurathlon_vm:/home/euratlhon/uwesub_msc/src/trajectory_sender/include/trajectory_sender/
rsync -avzh CMakeLists.txt 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/trajectory_sender/
rsync -avzh *.xml 								eurathlon_vm:/home/euratlhon/uwesub_msc/src/trajectory_sender/
rsync -avzh ./launch/*.launch					eurathlon_vm:/home/euratlhon/uwesub_msc/src/trajectory_sender/launch/
rsync -avzh *.yaml		 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/trajectory_sender/
rsync -avzh *.md								eurathlon_vm:/home/euratlhon/uwesub_msc/src/trajectory_sender/
rsync -avzh ./urdf/*.urdf						eurathlon_vm:/home/euratlhon/uwesub_msc/src/trajectory_sender/urdf/

echo "All done, Good Success!"