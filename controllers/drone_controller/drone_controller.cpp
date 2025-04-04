// File:          drone_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

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
#include <iostream>
#include "math_lib.hpp"
#include "PID.hpp"
#include <AttitudeControl.hpp>
// #include "attitude_control.hpp"
#define RC_THR 3
#define RC_YAW 2
#define RC_PIT 4
#define RC_ROL 5
#define RC_LBTON 1
#define RC_RBTON 0
#define RC_SCALE 32.768f

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace matrix;
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

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
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
  delay(robot,1000);

  float rc_info[6];
  
  Gyro *gyro        = robot->getGyro("gyro");
  gyro->enable(timeStep);

  InertialUnit *imu = robot->getInertialUnit("imu");
  imu->enable(timeStep);

  GPS *gps  = robot->getGPS("gps");
  gps->enable(timeStep);



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
  rate_pid[0].setGains(100.0f, 0.0f, 1.0f);
  rate_pid[0].setOutputLimit(100.0f);
  rate_pid[0].setIntegralLimit(10.0f);
  rate_pid[1].setGains(100.0f, 0.0f, 1.0f);
  rate_pid[1].setOutputLimit(100.0f);
  rate_pid[1].setIntegralLimit(10.0f);  
  rate_pid[2].setGains(100.0f, 0.0f, 1.0f);
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

    // // 初始姿态四元数 (无旋转)
    Quatf current_attitude(quat_info[3],quat_info[0], quat_info[1], quat_info[2]);

    // // 目标姿态四元数
    Quatf target_attitude(1.0f, 0.0f, 0.0f, 0.0f);

    attitude_control.setAttitudeSetpoint(target_attitude,0);
    // // 更新控制输出（计算角速度设定值）
    Vector3f angular_rate_setpoint = attitude_control.update(current_attitude);
    // 输出结果
    // std::cout << "Angular rate setpoint (roll, pitch, yaw): "
    //           << angular_rate_setpoint(1) << ", "
    //           << angular_rate_setpoint(0) << ", "
    //           << angular_rate_setpoint(2) << std::endl;


    // std::cout << "w: "
    //           << current_attitude.w << ",x: "
    //           << current_attitude.x << ",y: "
    //           << current_attitude.y << ",z: "
    //           << current_attitude.z << std::endl;

    for(int i = 0;i<6;i++)
    {
      rc_info[i] = 0.8*rc_info[i]+0.2*(rc.getAxisValue(i)/RC_SCALE);
      rc_info[i] = dead_zone(rc_info[i],8);
    }

    // Enter here functions to send actuator commands, like:
    led->set(1);
    /*欧拉角数控制*/
    // ang_pid[0].setSetpoint(linear_scale(rc_info[RC_PIT],-1000,1000,-1,1));
    // rate_pid[0].setSetpoint(ang_pid[0].update(-rpy_info[0],dt));
    // float pitch_out = rate_pid[0].update(-gyro_info[0],dt);

    // ang_pid[1].setSetpoint(linear_scale(-rc_info[RC_ROL],-1000,1000,-1,1));
    // rate_pid[1].setSetpoint(ang_pid[1].update(-rpy_info[1],dt));
    // float roll_out = rate_pid[1].update(-gyro_info[1],dt);

    // //ang_pid[2].setSetpoint(linear_scale(-rc_info[RC_YAW],-1000,1000,-1,1));
    // rate_pid[2].setSetpoint(linear_scale(rc_info[RC_YAW],-1000,1000,-5,5));
    // float yaw_out = rate_pid[2].update(-gyro_info[2],dt);

    /*四元数控制*/
    // rate_pid[0].setSetpoint(angular_rate_setpoint[1]);
    // float pitch_out = rate_pid[0].update(-gyro_info[0],dt);
    
    
    // rate_pid[1].setSetpoint(-angular_rate_setpoint[0]);
    // float roll_out = rate_pid[1].update(-gyro_info[1],dt);

    // rate_pid[2].setSetpoint(-angular_rate_setpoint[2]);
    // float yaw_out = rate_pid[2].update(-gyro_info[2],dt);


    rate_pid[0].setSetpoint(-angular_rate_setpoint(0));
    float pitch_out = rate_pid[0].update(-gyro_info[0],dt);
    
    rate_pid[1].setSetpoint(-angular_rate_setpoint(1));
    float roll_out = rate_pid[1].update(-gyro_info[1],dt);

    rate_pid[2].setSetpoint(-angular_rate_setpoint(2));
    float yaw_out = rate_pid[2].update(-gyro_info[2],dt);

    float thrr_out = linear_scale(rc_info[RC_THR],-1000,1000,0,1000);
    
    //std::cout << yaw_out << std::endl;
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
