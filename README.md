# BBBCI功能包
## 功能
在进行完手眼标定后，运行该功能包内的节点，对相机获取到的彩色图与深度图进行处理，控制机械臂运动及夹爪抓取
## 依赖说明
* 需要功能包 _robotiq_2f_gripper_control_ 、 _ur_rtde_ 、 _realsense_，这些功能包在后续配置过程中编译完成
* _realsense_ 功能包在CMakeLists文件中的配置方法见 *BBBCI* 功能包下的 _CMakeLists.txt_ 文件
## 需要配置ur_rtde、robotiq、realsense
### 配置ur_rtde
参考网站 <https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html>  
### 配置robotiq
#### 对于ros-melodic  
参考网址 <https://blog.csdn.net/weixin_44951365/article/details/120150137>  
功能包下载地址 <https://gitcode.net/mirrors/ros-industrial/robotiq/-/tree/kinetic-devel/robotiq_2f_gripper_control/include/robotiq_2f_gripper_control>
#### 对于ros-noetic
下载地址 <https://github.com/TAMS-Group/robotiq>  
**注意：** _noetic与melodic下载的功能包不一样，否则robotiq_modbus_tcp依赖不能正确安装！_  
参考<https://blog.csdn.net/m0_74089435/article/details/130068429>
#### 配置realsense
参考网址 <https://blog.csdn.net/qq_44998513/article/details/131517394>  
## 节点说明
### col_align_dep.cpp
控制相机接收图片并且将深度图与彩色图对齐，并利用opencv对图片处理后发布图片及处理动作信号。  
因为涉及到订阅深度图与彩色图的同步，所以需要将根节点定义在类中，为了规范化，将该类由头文件描述  
头文件及对应类的实现文件：_subpicpubsig.cpp_  _subpicpubsig.h_  
在 _subpicpubsig.cpp_ 文件中需要将**相机内参**和**手眼标定矩阵** 进行更新。   
**注意：** 启动前，需要先运行realsense相关launch文件：  
`roslaunch realsense2_camera rs_aligned_depth.launch`  
运行该文件前，可使用 `realsense-viewer` 在可视化界面中检查相机连接及深度类型
### bci_ur_move.cpp
控制机械臂运动和夹爪动作  
依赖于 _robotiq_ 功能包及 _ur_rtde_  
**注意：** 启动前需要先运行 _robotiq_ 功能包内相关节点 
在 *BBBCI* 工作空间下运行：  
`sudo chmod +777 /dev/ttyUSB0`  _给串口赋权限_   
`rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0`  
此时指示灯由红变蓝  
再开一个终端，执行：  
`rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py`
即可在终端利用按键控制夹爪，可利用此步测试夹爪是否能成功控制。  
如果可以顺利控制，则关闭此节点，只留第一个节点保持运行即可。