#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <process_robot_data.h>
#include <config.h>

typedef struct {
  Eigen::Matrix<double,6,1> pose;
  Eigen::Matrix<double,6,1> speed;
  Eigen::Matrix<double,6,1> accel;
  Eigen::Matrix<double,6,1> force;
}Status_Cart;

bool flag =  0; //是否接收到相机开启的信号
Eigen::MatrixXd PosLen;    //用来存储接收到的物块位姿信息
Eigen::MatrixXd Point3D;
int result;

void PosLenCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    int len;
    std::vector<double> pos_len = msg->data;
    ROS_INFO("I heard Position and Length!");
    len = msg->data.size();
    PosLen = Eigen::Map<Eigen::MatrixXd> (pos_len.data(), len / 3, 3);
}

void ResultCallback(const std_msgs::Float64::ConstPtr& msg) {
    ROS_INFO("I heard Result : %f!", msg->data);
    flag = 1;
    result = msg->data;
}

void PointCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    int len;
    std::vector<double> pt_3D = msg->data;
    ROS_INFO("I heard Result Point3D!  %f, %f, %f, %f, %f, %f, %d", pt_3D[0], pt_3D[1], pt_3D[2], pt_3D[3], pt_3D[4], pt_3D[5], msg->data.size());
    len = msg->data.size();
    Point3D = Eigen::Map<Eigen::MatrixXd> (&pt_3D[0], 3, len / 3);
}

//旋转矢量到四元数
Eigen::Quaterniond rotationVector_quaternion(Eigen::Matrix<double,3,1> rotationVector) {
    Eigen::Quaterniond quaternion;
    double theat = rotationVector.norm();
    quaternion.w() = cos(theat / 2);
    if(abs(theat) < 0.0001){
    theat += theat > 0 ? 0.0001 : -0.0001;
    }
    quaternion.x() = rotationVector[0] / theat * sin(theat / 2);
    quaternion.y() = rotationVector[1] / theat * sin(theat / 2);
    quaternion.z() = rotationVector[2] / theat * sin(theat / 2);
    return quaternion;
}

//旋转矩阵转换为欧拉ZYX/RPY
//ZYX顺序，即先绕x轴roll,再绕y轴pitch,最后绕z轴yaw,0表示X轴,1表示Y轴,2表示Z轴
Eigen::Vector3d  rotationMatrix_RPY(Eigen::Matrix3d rotationMatrix) {
    return rotationMatrix.eulerAngles(2, 1, 0);
}

//旋转矩阵转换为旋转矢量
Eigen::Vector3d rotationMatrix_Vec(Eigen::Matrix3d rotation_matrix) {
    Eigen::AngleAxisd rotation_vector(rotation_matrix);
    Eigen::Vector3d rot_vec(rotation_vector.angle() * rotation_vector.axis());
    return rot_vec;
}


int main(int argc, char** argv) {
    //基础类初始化
    Process_Robot_Data URData;
    Robot_Move URMove;
    //ros初始化
    ros::init(argc, argv, "bci_ur_actual");
    ros::NodeHandle n;
    ros::Subscriber URDataSub = n.subscribe("ur_data", 100, &Process_Robot_Data::robotDataCallback, &URData);
    ros::Publisher  urConParPub = n.advertise<hrc::RobotControl>("ur_control", 100);
    ros::Publisher GripSigPub = n.advertise<std_msgs::Bool>("/GripSig", 1);
    std_msgs::Bool GripSigMsg;
    ros::Subscriber PositionLength_Sub = n.subscribe("/PositionLengthNode", 10, &PosLenCallback);
    ros::Subscriber Result_Sub = n.subscribe("/result", 10, &ResultCallback);
    ros::Subscriber Point_Sub = n.subscribe("/Point3D", 1, &PointCallback);
    ros::Rate loop_rate(CTRL_FREQ);
    ros::AsyncSpinner AS(1);
    AS.start();

    //初始化夹爪相关数据类型
    ros::Publisher grip_Pub = n.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 10);
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output GripperOutput;
    GripperOutput.rACT = 1;
    GripperOutput.rGTO = 1;
    GripperOutput.rATR = 0;
    GripperOutput.rPR = 0;      //0时打开，255时完全闭合
    GripperOutput.rSP = 255;
    GripperOutput.rFR = 150;
    grip_Pub.publish(GripperOutput);

    //五个物块序号对应的闪烁频率
    double fre[5] = {8, 10, 12, 14, 16};

    /**************这一步，先运动到物块上方****************/
    //定义机器人初始位姿，速度
    std::vector<double> initPose = INITPOSE;
    Status_Cart status;
    status.pose << Eigen::Map<Eigen::MatrixXd>(&initPose[0], 6, 1);
    status.speed << 0.0,0.0,0.0,0.0,0.0,0.0;
    //等待机器人到达初始位姿
    //在ur_move.cpp中，包含机械臂运动到初始位姿的控制代码，故这里没有控制机械臂运动的代码
    int i = 0;
    // usleep(3000);
    while(!URData.isReachTargetPose(status.pose,0.001) && ros::ok()) {
        usleep(1000);
        if(i++ > 6000) {
        printf("机器人没有到达指定初始位置\n");
        return 0;
        }
    }
    Eigen::Matrix<double,4,4> T_robot_flange;
    T_robot_flange << -1, 0, 0, 0.2, 
                                            0, 1, 0, -0.5, 
                                            0, 0, -1, 0.3,
                                            0, 0, 0, 1;
    while(ros::ok()) {
        //ROS_INFO("flag : %d", flag);
        //当获取到相机发布的位姿信息后，只需要将信息存储，当接收到分析结果时，开始进行轨迹规划并运动
        if(flag) {
            
            //结果是第i个物块
            int i;
            for (i = 0 ; i < 5 ; i++) {
                if(result == fre[i])
                    break;
            }
            ROS_INFO("I = %d", i);
            //目标抓取姿态为绕x轴旋转180度
            Eigen::Matrix3d gripPose;
            Eigen::Vector3d rotVec;
            gripPose << 1, 0, 0,
                                    0, -1, 0,
                                    0, 0, -1;
                                      
            rotVec = rotationMatrix_Vec(gripPose);
            Eigen::Matrix<double, 4, 1>  Pt_TCL, Pt_Base;
            Pt_TCL  << Point3D(0, i), Point3D(1, i), Point3D(2, i), 1.0;
            
            ROS_INFO("Pt_TCL : %f, %f, %f", Pt_TCL(0, 0), Pt_TCL(1, 0), Pt_TCL(2, 0));     
            Pt_Base = T_robot_flange * Pt_TCL;
            status.pose << Pt_Base(0, 0), Pt_Base(1, 0), Pt_Base(2, 0) + 0.4,
                                            rotVec(0), rotVec(1), rotVec(2);
            ROS_INFO("Pt_Base : %f, %f, %f", Pt_Base(0,0), Pt_Base(1, 0), Pt_Base(2, 0));                             
            URMove.sendPoseMsg(urConParPub, status.pose);
            i = 0;
            
            while(!URData.isReachTargetPose(status.pose,0.001) && ros::ok()) {
                usleep(1000);
                if(i++ > 6000) {
                printf("机器人没有到达指定初始位置\n");
                return 0;
                }
            }
            ROS_INFO("已到达目标物体上方！");

            //关闭夹爪
            GripperOutput.rPR = 160;
            grip_Pub.publish(GripperOutput);
            sleep(1);
            ROS_INFO("夹爪已闭合，将回到初始位置！");
            //回到初始点处
            status.pose << Eigen::Map<Eigen::MatrixXd>(&initPose[0], 6, 1);
            URMove.sendPoseMsg(urConParPub, status.pose);
            i = 0;
            while(!URData.isReachTargetPose(status.pose,0.001) && ros::ok()) {
                usleep(1000);
                if(i++ > 6000) {
                printf("机器人没有到达指定初始位置\n");
                return 0;
                }
            }
            //松开夹爪，准备进行下一次抓取
            GripperOutput.rPR = 0;
            grip_Pub.publish(GripperOutput);
            sleep(1);
            ROS_INFO("已回到初始位置，打开夹爪准备下一次抓取！");
            //抓取结束，发布可进行下一次抓取的信号
            GripSigMsg.data = 1;
            GripSigPub.publish(GripSigMsg);
            flag = 0;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}