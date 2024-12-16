# qualisys_mocap

ROS version of Qualisys Motion Capture System forked from : <br/>
https://github.com/KumarRobotics/qualisys.git <br/>
https://github.com/jgoppert/qualisys_ros <br/>

cd Bureau<br/>
./1_start_network.sh<br/>

route -n<br/>
--> 192.168.100.0   0.0.0.0         255.255.255.0   U     0      0        0 eth1<br/>


cd ~/git_project/mocap_ws/                     <br/>
source devel/setup.bash                        <br/>
roslaunch qualisys qualisys.launch             <br/>



cd ~/git_project/mocap_ws/<br/>
source devel/setup.bash<br/>
rostopic list<br/>

rostopic echo /qualisys/name_of_rigid_body<br/>


