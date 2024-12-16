# qualisys_mocap

ROS version of Qualisys Motion Capture System forked from : < br / >
https://github.com/KumarRobotics/qualisys.git < br / >
https://github.com/jgoppert/qualisys_ros < br / >

cd Bureau\n
./1_start_network.sh\n

route -n\n
--> 192.168.100.0   0.0.0.0         255.255.255.0   U     0      0        0 eth1\n


cd ~/git_project/mocap_ws/                     \n
source devel/setup.bash                        \n
roslaunch qualisys qualisys.launch             \n



cd ~/git_project/mocap_ws/\n
source devel/setup.bash\n
rostopic list\n

rostopic echo /qualisys/name_of_rigid_body\n


