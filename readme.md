一个加密狗，manus dashboard，一只一只开连，先开dashboard，连不上的话，关闭手套，重启manus core。windows下运行vs程序
mcp2个自由度，cmc3个，dip pip值相同
ubuntu下执行命令  
```
roslaunch primeii_ros_bridge bringup.launch hostname:=192.168.3.3   
//连接windows下的服务器程序，PrimeIIClient，read 。。time out 原因，无法ping通，windows下防火墙必须全关闭
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300 //机械臂驱动
roslaunch j2n6s300_moveit_config j2n6s300_demo.launch//rviz
rosrun prime_kinova_control prime_kinova_control.py #控制机械臂的demo
```
左右手套一只即可，连接先连左手，食指控制上下，中指控制左右，无名指控制前后
问题：
机械臂usb线连usb2.0接口  
debug:  
rostopic echo /j2n6s300_driver/in/cartesian_velocity  
rostopic echo /GlovesData  
roslaunch j2n6s300_moveit_config j2n6s300_demo.launch  //看一看rviz中能否控制
rqt_graph:
![1](./rqt_graph)