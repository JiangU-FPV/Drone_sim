/**
 * @file drone_controller.cpp
 * @author c-Lando.chen (3051619248@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-06
 * 
 * 
 */

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Led.hpp>
#include <webots/Joystick.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/GPS.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <iostream>
#include "math_lib.hpp"
#include "PID.hpp"
#include <AttitudeControl.hpp>
#include "tool/json.hpp"
#include <fstream>
#include <vector>
// #include "attitude_control.hpp"
#define RC_THR 3
#define RC_YAW 2
#define RC_PIT 4
#define RC_ROL 5
#define RC_LBTON 1
#define RC_RBTON 0
#define RC_SCALE 32.768f
#define HOLD_THR 537.144
// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace matrix;
using json = nlohmann::json;
// 将 rad/s 转换为 RPM
double radPerSecToRpm(double radPerSec) {
  return radPerSec * 60.0 / (2.0 * M_PI);
}

void delay(Robot *robot, int milliseconds) {
    int timeStep = (int)robot->getBasicTimeStep();
    int steps = milliseconds / timeStep; // 计算需要的时间步数
    for (int i = 0; i < steps; ++i) {
        if (robot->step(timeStep) == -1) {
            break; // 仿真已终止
        }
    }
}
// 定义路径点结构体
struct Point3D {
  double x, y, z;
};
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {

  std::ifstream file("../../map/smoothed_path.json");
  if (!file.is_open()) {
      std::cerr << "无法打开文件！" << std::endl;
      return 1;
  }

  // 解析 JSON
  json j;
  file >> j;

  // 提取 resolution（如果需要）
  double resolution = j["resolution"];

  // 提取路径点
  std::vector<Point3D> path;
  for (const auto& p : j["path"]) {
      if (p.size() == 3) {
          path.push_back({p[0], p[1], p[2]});
      }
  }

  std::cout << "读取路径点数: " << path.size() << std::endl;
  for (const auto& pt : path) {
      std::cout << "(" << pt.x << ", " << pt.y << ", " << pt.z << ")\n";
  }

  // create the Robot instance.
  Robot *robot = new Robot();
  
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  double past_time = robot->getTime();
  
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:

  LED *led = robot->getLED("led");
  

  /**
 * @brief 启用遥控器
 * 
 */
  Joystick rc;
  rc.enable(timeStep);
  delay(robot,1000);
  // 获取手柄模型
  std::string model = rc.getModel();
  if (model.empty()) {
    std::cerr << "No joystick connected!" << std::endl;
  }
  std::cout << "Joystick model: " << model << std::endl;
  delay(robot,200);

  float rc_info[6];

  
  Gyro *gyro        = robot->getGyro("gyro");
  gyro->enable(timeStep);

  InertialUnit *imu = robot->getInertialUnit("imu");
  imu->enable(timeStep);

  GPS *gps  = robot->getGPS("gps");
  gps->enable(timeStep);

  Accelerometer *acc = robot->getAccelerometer("acc");
  acc->enable(timeStep);

  Camera *camera = robot->getCamera("camera");
  camera->enable(4 * timeStep);

  /**
   * @brief 启用电机
   * 
   */
  Motor *motor1 = robot->getMotor("motor1");
  Motor *motor2 = robot->getMotor("motor2");
  Motor *motor3 = robot->getMotor("motor3");
  Motor *motor4 = robot->getMotor("motor4"); 

  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  motor1->setPosition(INFINITY);  // 设置为无限位置模式以使用速度控制
  motor1->setVelocity(0.0);       // 初始速度为 0
  motor2->setPosition(INFINITY);  // 设置为无限位置模式以使用速度控制
  motor2->setVelocity(0.0);       // 初始速度为 0
  motor3->setPosition(INFINITY);  // 设置为无限位置模式以使用速度控制
  motor3->setVelocity(0.0);       // 初始速度为 0
  motor4->setPosition(INFINITY);  // 设置为无限位置模式以使用速度控制
  motor4->setVelocity(0.0);       // 初始速度为 0  
  

    // // 创建姿态控制对象
  AttitudeControl attitude_control;

    // 设置比例增益和偏航权重
  Vector3f proportional_gain = {10.0f, 10.0f, 7.0f};  // 假设是 (roll, pitch, yaw)
  float yaw_weight = 0.5f;  // 偏航权重
  attitude_control.setProportionalGain(proportional_gain, yaw_weight);
  Vector3f rate_limit = {5.0f, 5.0f, 5.0f}; // 设定最大角速度限制（滚转、俯仰、偏航）
  attitude_control.setRateLimit(rate_limit);


  PID rate_pid[3];
    // 设置初始配置
  rate_pid[0].setGains(150.0f, 0.0f, 1.0f);
  rate_pid[0].setOutputLimit(100.0f);
  rate_pid[0].setIntegralLimit(10.0f);
  rate_pid[1].setGains(150.0f, 0.0f, 1.0f);
  rate_pid[1].setOutputLimit(100.0f);
  rate_pid[1].setIntegralLimit(10.0f);  
  rate_pid[2].setGains(500.0f, 0.0f, 5.0f);
  rate_pid[2].setOutputLimit(100.0f);
  rate_pid[2].setIntegralLimit(10.0f);  

  PID ang_pid[3];
    // 设置初始配置
  ang_pid[0].setGains(7.0f, 0.0f, 0.0f);
  ang_pid[0].setOutputLimit(15.0f);
  ang_pid[0].setIntegralLimit(0.0f);
  ang_pid[1].setGains(7.0f, 0.0f, 0.0f);
  ang_pid[1].setOutputLimit(15.0f);
  ang_pid[1].setIntegralLimit(0.0f);
  ang_pid[2].setGains(7.0f, 0.0f, 0.0f);
  ang_pid[2].setOutputLimit(15.0f);
  ang_pid[2].setIntegralLimit(0.0f);  

  PID acc_z_pid;
  acc_z_pid.setGains(50.0f,0.1f,0.0f);
  acc_z_pid.setOutputLimit(10.0f);
  acc_z_pid.setIntegralLimit(100.0f);

  PID vel_z_pid;
  vel_z_pid.setGains(10.0f,0.1f,0.2f);
  acc_z_pid.setIntegralLimit(100.0f);
  vel_z_pid.setOutputLimit(9.8f);

  PID pos_z_pid;
  pos_z_pid.setGains(3.0f,0.0f,0.0f);
  pos_z_pid.setOutputLimit(3.0f);
  pos_z_pid.setIntegralLimit(100.0f);

  PID pos_xy_pid[2];
  pos_xy_pid[0].setGains(1.0f,0.0f,0.0f);
  pos_xy_pid[0].setOutputLimit(10.0f);
  pos_xy_pid[1].setGains(1.0f,0.0f,0.0f);
  pos_xy_pid[1].setOutputLimit(10.0f);

  PID vel_xy_pid[2];
  vel_xy_pid[0].setGains(0.3f,0.0f,0.03f);
  vel_xy_pid[0].setOutputLimit(0.5f);
  vel_xy_pid[1].setGains(0.3f,0.0f,0.03f);
  vel_xy_pid[1].setOutputLimit(0.5f);


  float pos_x = 0;
  float pos_y = 0;
  float pos_z = 0;

  float prev_pos_z = 0;
  float prev_pos_x = 0;
  float prev_pos_y = 0;


  float vel_z = 0;
  float vel_x = 0;
  float vel_y = 0;


  float pos_x_tar = 0.0f;
  float pos_y_tar = 0.0f;

  float vel_x_tar = 0.0f;
  float vel_y_tar = 0.0f;

  float acc_x_tar = 0.0f;
  float acc_y_tar = 0.0f;


  float yaw_tar = 0.0f;
  float height_tar = 3;

  float rpy_ptich;
  float rpy_roll; 
  float rpy_yaw;

  float start_point[3] = {1.0, 1.0, 2.0};
  float end_point[3]   = {19.0, 19.0, 3.0};

  int step = -1;
  double step_time = 0;
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    // Process sensor data here.
    const double dt = robot->getTime() - past_time;
    const double *gyro_info = gyro->getValues();
    const double *rpy_info  = imu->getRollPitchYaw();
    const double *quat_info = imu->getQuaternion();
    const double *gps_info  = gps->getValues();
    const double *acc_info  = acc->getValues();

    rpy_ptich = rpy_info[0];
    rpy_roll  = rpy_info[1];
    rpy_yaw   = rpy_info[2];
    std::cout<<rpy_ptich<<", "<<rpy_roll<<", "<<rpy_yaw<<std::endl;

    pos_x = gps_info[0];
    pos_y = gps_info[1];
    pos_z = gps_info[2];

    std::cout<<"pos_x_tar"<<pos_x_tar<<std::endl;
    std::cout<<"pos_y_tar"<<pos_y_tar<<std::endl;

    vel_x = 0.8*(gps_info[0]-prev_pos_x)/dt+0.2*vel_x;
    vel_y = 0.8*(gps_info[1]-prev_pos_y)/dt+0.2*vel_y;
    vel_z = 0.8*(gps_info[2]-prev_pos_z)/dt+0.2*vel_z;

    prev_pos_x = gps_info[0];
    prev_pos_y = gps_info[1];
    prev_pos_z = gps_info[2];
    // yaw_tar+=0.005f;
    
    float yaw_init_tar = atan2f(path[1].y-path[0].y,path[1].x-path[1].x) - M_PI/2;

    if (m_abs(pos_y_tar-pos_y)>0.2f||
        m_abs(pos_x_tar-pos_x)>0.2f)
    {
      yaw_tar = atan2f(pos_y_tar-pos_y,pos_x_tar-pos_x) - M_PI/2;
    } 

    if (step ==-1)
    {
      pos_x_tar  = path[0].x;
      pos_y_tar  = path[0].y;
      height_tar = path[0].z;
      yaw_tar = yaw_init_tar;
      if ((robot->getTime()-step_time)>5)
      {
        step_time = robot->getTime();
        step = 0;
      }
    }
    
    if (step != -1)
    {
      pos_x_tar  = path[step].x;
      pos_y_tar  = path[step].y;
      height_tar = path[step].z;
      if ((robot->getTime()-step_time)>0.06)
      {
        step_time = robot->getTime();
        step +=1;
      }

      if (step>=(path.size()-1))
      {
        step=path.size()-1;
      }
      
    //   height_tar = 3;
    //   pos_x_tar  = 0;
    //   pos_y_tar  = 0;
    //   //yaw_tar = -M_PI/2;
    //   if ((robot->getTime()-step_time)>3)
    //   {
        
    //     step_time = robot->getTime();
    //     step = 1;
    //     //yaw_tar-= M_PI/2;
    //   }
      
    // }
    // else if (step ==1)
    // {
    //   height_tar = 3;
    //   pos_x_tar  = 15;
    //   pos_y_tar  = 2.5;
    //   // yaw_tar =-M_PI/2;
    //   if ((robot->getTime()-step_time)>6)
    //   {
        
    //     step_time = robot->getTime();
    //     //yaw_tar-= M_PI/2;
    //     step = 2;
    //   }     
    // }
    // else if (step ==2)
    // {
    //   height_tar = 3;
    //   pos_x_tar  = 5;
    //   pos_y_tar  = 7.5;
    //   // yaw_tar = M_PI/2;
    //   if ((robot->getTime()-step_time)>3)
    //   {
    //     //yaw_tar-= M_PI/2;
    //     step_time = robot->getTime();
    //     step = 3;
    //   }
    // }    
    // else if (step ==3)
    // {
    //   height_tar = 3;
    //   pos_x_tar  = 5;
    //   pos_y_tar  = 10;
    //   // yaw_tar = -M_PI;
    //   if ((robot->getTime()-step_time)>3)
    //   {
    //     //yaw_tar-= M_PI/2;
    //     step_time = robot->getTime();
    //     step = 0;
    //   }
    }    
    
    Quatf current_attitude(quat_info[3],quat_info[0], quat_info[1], quat_info[2]);
    
    if (yaw_tar>M_PI)
    {
      yaw_tar-=2*M_PI;
    }
    else if (yaw_tar<-M_PI)
    {
      yaw_tar+=2*M_PI;
    }
    yaw_tar = m_constrain(yaw_tar,-M_PI,M_PI);

    //位置控制器
    pos_xy_pid[0].setSetpoint(pos_x_tar);
    vel_x_tar =  pos_xy_pid[0].update(pos_x,dt);
    
    pos_xy_pid[1].setSetpoint(pos_y_tar);
    vel_y_tar =  pos_xy_pid[1].update(pos_y,dt);

    //速度控制器
    vel_xy_pid[0].setSetpoint(vel_x_tar);
    float acc_x_tar_world = vel_xy_pid[0].update(vel_x,dt);

    vel_xy_pid[1].setSetpoint(vel_y_tar);
    float acc_y_tar_world =  vel_xy_pid[1].update(vel_y,dt); 

    acc_x_tar =  acc_x_tar_world*cos(rpy_yaw) + acc_y_tar_world*sin(rpy_yaw);
    acc_y_tar =  -acc_x_tar_world*sin(rpy_yaw) + acc_y_tar_world*cos(rpy_yaw);
    std::cout<<"x_angle"<<acc_x_tar<<std::endl;
    std::cout<<"y_angle"<<acc_y_tar<<std::endl;

    float stick_attitude[4];
    // euler_to_quaternion(rc_info[RC_PIT]/2000.0f,rc_info[RC_ROL]/2000.0f,yaw_tar,stick_attitude);
    euler_to_quaternion(acc_y_tar,acc_x_tar,yaw_tar,stick_attitude);
    Quatf target_attitude(stick_attitude[0],stick_attitude[1],stick_attitude[2],stick_attitude[3]);

    attitude_control.setAttitudeSetpoint(target_attitude,0);
    // // 更新控制输出（计算角速度设定值）
    Vector3f angular_rate_setpoint = attitude_control.update(current_attitude);

    if (!model.empty())
    {
      for(int i = 0;i<6;i++)
      {
        rc_info[i] = 0.8*rc_info[i]+0.2*(rc.getAxisValue(i)/RC_SCALE);
        rc_info[i] = dead_zone(rc_info[i],8);
      }
    }
    // Enter here functions to send actuator commands, like:
    led->set(1);

    rate_pid[0].setSetpoint(-angular_rate_setpoint(0));
    float pitch_out = rate_pid[0].update(-gyro_info[0],dt);
    
    rate_pid[1].setSetpoint(-angular_rate_setpoint(1));
    float roll_out = rate_pid[1].update(-gyro_info[1],dt);

    rate_pid[2].setSetpoint(-angular_rate_setpoint(2));
    float yaw_out = rate_pid[2].update(-gyro_info[2],dt);
    
    height_tar+=linear_scale(dead_zone(rc_info[RC_THR],400),-1000,1000,-0.015,0.015);
    
    pos_z_pid.setSetpoint(height_tar);
    float pos_z_out = pos_z_pid.update(gps_info[2],dt);

    vel_z_pid.setSetpoint(pos_z_out);
    float vel_z_out = vel_z_pid.update(vel_z,dt);

    acc_z_pid.setSetpoint(vel_z_out);
    float acc_z_out = acc_z_pid.update(acc_info[2]-9.8,dt);    
    //float thrr_out = linear_scale(rc_info[RC_THR],-1000,1000,0,1000);
    float thrr_out = vel_z_out*50+HOLD_THR + pos_z_out*5;
    if (thrr_out<=300)
    {
     thrr_out=300;
    }
    
   // float thrr_out = 550;
    //std::cout << height_tar << std::endl;
    
    //std::cout << "joystick " << rc_info[RC_THR] << ", " << rc_info[RC_PIT] << ", " << rc_info[RC_ROL] <<", " << rc_info[RC_YAW] <<std::endl;
    motor1->setVelocity(-thrr_out+pitch_out+roll_out+yaw_out);
    motor2->setVelocity(thrr_out+pitch_out-roll_out+yaw_out);
    motor3->setVelocity(-thrr_out-pitch_out-roll_out+yaw_out);
    motor4->setVelocity(thrr_out-pitch_out+roll_out+yaw_out);
    past_time =  robot->getTime();
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
