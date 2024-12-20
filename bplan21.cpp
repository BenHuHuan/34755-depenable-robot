/*
 *
 * Copyright © 2023 DTU,
 * Author:
 * Christian Andersen jcan@dtu.dk
 *
 * The MIT License (MIT)  https://mit-license.org/
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE. */
// 常用指令cd local/svn/robobot/raubase/build
//  ./raubase
#include <string>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include "mpose.h"
#include "steensy.h"
#include "uservice.h"
#include "sencoder.h"
#include "utime.h"
#include "cmotor.h"
#include "cservo.h"
#include "medge.h"
#include "cedge.h"
#include "cmixer.h"
#include "sdist.h"
#include <chrono>
#include <ctime>
#include <array>

#include "bplan21.h"

// #include <opencv2/opencv.hpp>
// #include <iostream>
// #include <vector>
// #include "golf.h"
#include "scam.h"

using std::chrono::seconds;
using std::chrono::system_clock;

// create class object
BPlan21 plan21;

void BPlan21::setup()
{ // ensure there is default values in ini-file
  if (not ini["plan21"].has("log"))
  { // no data yet, so generate some default values
    ini["plan21"]["log"] = "true";
    ini["plan21"]["run"] = "false";
    ini["plan21"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["plan21"]["print"] == "true";
  //
  if (ini["plan21"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_plan21.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission plan21 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlan21::~BPlan21()
{
  terminate();
}

void BPlan21::run()
{
  if (not setupDone)
    setup();
  if (ini["plan21"]["run"] == "false")
    return;
  //
  UTime t;
  bool finished = false;
  bool lost = false;
  // state = -10;
  // state = 11000;
  state = 11701;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  //
  int lost_count = 0;
  toLog("Plan21   started");
  //
  int current_cross = 0;
  int detect_cross_count = 0;
  int axe = 0;
  int cross_for_qiaoqiaoban = 0;
  int detece_line_count = 0;
  float dist_fumen = 0.0;
  float dist_1 = 0.0;
  /**
   * state -10: 初始状态，不用关心
   * state 0: 跟随线
   * state 1 到 19: 编写自己的过岔路逻辑，过了岔路之后，要么进入自己的策略的 state，要么回到 state -1；
   * state 20-29: 跷跷板，结束后回到 state -1
   * state 30-39: 阶梯，结束后回到 state -1
   * state 40-49: 斧门，结束后回到 state -1
   * state 50-59: 封闭赛道，结束后回到 state -1
   * state 6000-6999: 环岛，结束后回到 state -1
   * state 7000-7999: 岔路口逻辑
   * state 100-110: 终点 🏁
   * state 999：丢线，迷路了
   *
   */

  // 获取初始时间点
  auto startTime = system_clock::now();
  auto endTime = system_clock::now();
  auto duration = endTime - startTime;

  // 环岛小车的状态
  // 0 - 未检测到小车
  // 1 - 检测到小车，正在靠近
  int thr_gate_small_car_state = 0;
  // int thr_gate_count_line = 0;
  pose.resetPose();
  ini["edge"]["kp"] = "45";
  ini["edge"]["maxturnrate"] = "8";
  ini["edge"]["lead"] = "0.3 0.2";

  while (not finished and not lost and not service.stop)
  { // run a square with 4 90 deg turns - CCV
    switch (state)
    {
    case -100:
      pose.dist = 0;
      pose.turned = 0;
      mixer.setVelocity(-0.3);
      mixer.setTurnrate(0);
      state = -1001;
      break;
    case -1001:
      if (pose.dist > 100) {
        lost = true;
      }
      break;
    case 9999:
      if (pose.turned <= 3.14)
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(1);
      }
      else
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
      }
      break;
    case -10: // wait for Regbot, then go forward
      // ini["edge"]["rate_ms"] = "4";
      // ini["edge"]["kp"]="35";
      // ini["edge"]["maxturnrate"]="7";
      // ini["edge"]["lead"]="0.3 0.15";
      pose.resetPose();
      toLog("finding line!!!!!!!!");
      mixer.setVelocity(0.2);
      mixer.setTurnrate(0);
      state = -9;
      break;
    case -9: // check if found the line
      if (medge.width > 0.02)
      {
        toLog("found!!!!");
        mixer.setVelocity(0.3);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(true, 0.01);
        state = 0;
        pose.dist = 0;
        pose.turned = 0;
      }
      break;
    case 0: // check if find the crossroad
            // char edge_buffer[100];
            // sprintf(edge_buffer, " line width %f left edge %f right edge %f ", medge.width, medge.leftEdge, medge.rightEdge);
            // toLog(edge_buffer);
      mixer.setVelocity(0.3);
      mixer.setTurnrate(0);
      mixer.setEdgeMode(true, 0);
      // char pose_buffer[100];
      // sprintf(pose_buffer, "pose.dist %f pose.turned %f ", pose.dist, pose.turned);
      // toLog(pose_buffer);

      if (medge.width > 0.02 and medge.width < 0.03)
      {
        detect_cross_count = 0;
      }

      if (medge.width > 0.06)
      {
        detect_cross_count++;
        toLog("found crossroad");

        if (detect_cross_count > 5)
        {
          detect_cross_count = 0;
          pose.dist = 0;
          pose.turned = 0;
          current_cross++;

          if (current_cross == 1)
          {
            // 第一个岔路口，先去环岛
            state = 6000;
            break;
          }
          else if (current_cross == 2)
          {
            // 斧门
            state = 4000;
            break;
          }
          else if (current_cross == 3)
          {
            break;
          }
          else if (current_cross == 4)
          {
            // 封闭隧道
            state = 5000;
          }
          else if (current_cross == 5)
          {
            state = 6000;
          }
          break;
        }
      }

      if (medge.width == 0)
      {
        toLog("lost the line");
        if (lost_count > 10)
        {
          toLog("lost the line for too long");
          mixer.setVelocity(0.1);
          lost_count = 0;
          // lost = true;
        }
      }

      break;
    case 1:
      // toLog("turn crossroad 1: turn left");

      // // mixer.setDesiredHeading(-3.1415926/2);
      // char s1_pose_buffer[100];
      // sprintf(s1_pose_buffer, "pose.dist %f pose.turned %f ", pose.dist, pose.turned);
      // toLog(s1_pose_buffer);

      // true: turn left
      // mixer.setEdgeMode(true, 0);

      if (pose.turned >= 1.52)
      {
        toLog("back to state 0");
        pose.dist = 0;
        pose.turned = 0;
        // state = 0;
      }
      break;
    case 4000:
      pose.dist = 0;
      pose.turned = 0;
      mixer.setVelocity(0.1);
      mixer.setTurnrate(0);
      mixer.setEdgeMode(true, 0);
      state = 4001;
      break;
    case 4001:
      if (pose.dist >= 0.05)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 4010;
      }
      break;
    case 4010:
      // 斧门
      if (dist.dist[0] > 0.01 && dist.dist[0] < 0.4)
      {
        // mixer.setVelocity(0);
        // mixer.setTurnrate(0);
        state = 4015;
      }
      break;
    case 4015:
      dist_1 = dist.dist[0] - 0.2;
      dist_fumen = dist_1 < 0 ? 0 : dist_1;
      pose.dist = 0;
      pose.turned = 0;
      mixer.setVelocity(0.1);
      mixer.setTurnrate(0);
      mixer.setEdgeMode(true, 0);
      state = 4020;
      break;
    case 4020:
      if ((pose.dist >= dist_fumen and axe == 0))
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 4100;
      }
      break;

    case 4100:

      if (dist.dist[0] < 0.3) // 发现斧门
      {
        axe = 1;
      }

      if (axe == 1 and dist.dist[0] > 0.3) // 斧门消失
      {
        axe = 0;
        pose.dist = 0;
        pose.turned = 0;
        state = 5000; // 进入封闭隧道状态
      }

      break;
    /**********************************************************************************
     *
     *
     * 封闭赛道
     *
     *
     ***********************************************************************************/
    case 5000:
      // 进入封闭赛道区域
      pose.dist = 0;
      pose.turned = 0;
      state = 5100;
      break;

    case 5100:
      // 封闭赛道
      if (pose.dist < 0.82) // 3.31 zys从0.7改成了0.4调试   myx 0.35-0.3
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.4);
        mixer.setEdgeMode(true, 0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 5200;
      }
      break; // 向前走x米

    case 5200:
      if (pose.turned < 3.14 / 4) // 转向 45度 准备靠近封闭隧道侧面
      {
        mixer.setTurnrate(3.14 / 4); // 转向45°
        mixer.setVelocity(0);
      }
      else // 然后保持0.1m/s直行并切换到下一个状态
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
        state = 5300;
      }
      break;

    case 5300:
      mixer.setTurnrate(0);
      mixer.setVelocity(0.2);
      if (dist.dist[0] < 0.13) // 发现封闭赛道侧面     myx 0.2改为0.15测试
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(-3.14 / 2);
        mixer.setVelocity(0.1);
        state = 5400;
      }
      break;

    case 5400:
      if (pose.turned < -3.14 / 2) // 转过了90度，准备推门进入隧道
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 5500;
      }
      break;

    case 5410:
      if (dist.dist[0] < 0.2) // 靠近门
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
        state = 5500;
      }
      break;

    case 5500:
      mixer.setVelocity(0.75); // 0.5-2调试
      mixer.setTurnrate(0);
      // state = 5600;
      if (pose.dist > 0.25)
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 5510;
      }
      break;

    case 5510:
      mixer.setVelocity(0.2); // 0.1-0.2调试
      mixer.setTurnrate(0);
      if (pose.dist > 0.05)
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 5610;
      }
      break;

      // 把隧道门推开后，两次左转90度，巡线，进入隧道
    case 5610:
      if (pose.dist < 0.05)
      {
        mixer.setVelocity(0.1);
      }
      else
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(1); // 开始左转
        pose.dist = 0;        // 重置距离计算
        pose.turned = 0;      // 重要：也需要重置转向角度的累计
        state = 5620;
      }
      break;

    case 5620: // 完成左转90度然后直走
      if (pose.turned < 3.14 / 2)
      {
        // 等待转满90度
      }
      else
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.3); // 开始直行
        pose.dist = 0;          // 重置距离计算，为下一步直行做准备
        pose.turned = 0;        // 重要：重置转向角度的累计
        state = 5625;
      }
      break;

    case 5625: // 直走0.4m然后左转

      if (medge.width > 0.02)
      {
        cross_for_qiaoqiaoban++;
      }
      if (cross_for_qiaoqiaoban >= 2)
      {
        toLog("found line for qiaoqiaoban then turn 90 du");
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        cross_for_qiaoqiaoban = 0;
        state = 5627;
      }
      break;

    case 5627:

      if (pose.dist < 0.07)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 5630;
      }
      break;

    case 5630: // 左转90度 然后巡线直行
      if (pose.turned < 3.1 / 2)
      {
        mixer.setTurnrate(1);
        mixer.setVelocity(0);
        // 等待转满90度
      }
      else
      {
        mixer.setTurnrate(0); // 停止转向
        mixer.setVelocity(0); // 停止直行，准备开始巡线
        pose.dist = 0;        // 重置距离计算，为下一步直行做准备
        pose.turned = 0;      // 重要：重置转向角度的累计
        state = 8000;         // 引入新状态以分离转向完成与开始巡线直行的逻辑
      }
      break;

      /**********************************************************************************
       *
       *
       * 赛马场
       *
       *
       ***********************************************************************************/

    case 8000:
      mixer.setTurnrate(0);
      // pose.dist = 0;
      // pose.turned = 0;
      if (pose.dist < 3.7) // 隧道到赛马场起点距离2.5
      {

        mixer.setVelocity(0.3);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(true, 0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 8100;
      }
      break;

    case 8100: // 赛马场门后转向
      if (pose.turned < 3.15)
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(1);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        state = 8120;
      }
      break;

    case 8120: // 寻迹找到方向
      ini["edge"]["kp"] = "45";
      ini["edge"]["maxturnrate"] = "12";
      ini["edge"]["lead"] = "0.3 0.2";
      if (pose.dist < 0.5)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.3);
        mixer.setEdgeMode(true, 0.012);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.3);
        mixer.setEdgeMode(true, 0.012);
        state = 8130;
      }
      break;
    case 8130: // 直线冲刺
      if (pose.dist > 4.7)
      {
        ini["edge"]["kp"] = "45";
        ini["edge"]["maxturnrate"] = "12";
        ini["edge"]["lead"] = "0.3 0.2";
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.3);
        mixer.setEdgeMode(true, 0.01);
        state = 8140;
        break;
      }
      break;

    case 8140: // 第一个转弯
      // char edge_buffer[100];
      // sprintf(edge_buffer, " line width %f pose.turned %f ", medge.width, pose.turned);
      // toLog(edge_buffer);
      if (pose.dist > 1.8)
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 8150;
      }
      break;
    case 8150: // 直线冲刺 2
      ini["edge"]["kp"] = "5";
      ini["edge"]["maxturnrate"] = "0.5";
      ini["edge"]["lead"] = "0.3 0.01";
      if (pose.dist < 1.2)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.7);
        mixer.setEdgeMode(true, 0.012);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.3);
        mixer.setEdgeMode(true, 0.012);
        state = 8160;
      }
      break;

    case 8160: // 第二个转弯
      ini["edge"]["kp"] = "45";
      ini["edge"]["maxturnrate"] = "12";
      ini["edge"]["lead"] = "0.3 0.2";
      // char edge_buffer_1[100];
      // sprintf(edge_buffer_1, " line width %f pose.turned %f pose.dist %f", medge.width, pose.turned, pose.dist);
      // toLog(edge_buffer_1);
      if (pose.dist > 1.1 && pose.turned < -3 / 2)
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 8170;
      }
      break;

    case 8170: // 转弯后继续走，直到穿过门
      pose.dist = 0;
      pose.turned = 0;
      mixer.setVelocity(0.2);
      mixer.setTurnrate(0);
      mixer.setEdgeMode(true, 0.012);
      state = 8175;
      break;
    case 8175:
      // 过了终点线
      if (dist.dist[1] > 0.03 and dist.dist[1] < 0.3)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0);
        state = 11000;
      }
      break;

    // // 原始版本通过终点后
    // case 8180:
    //   if (pose.dist > 0.3)
    //   {
    //     pose.dist = 0;
    //     pose.turned = 0;
    //     mixer.setTurnrate(0);
    //     mixer.setVelocity(0);
    //     state = 8190;
    //   }
    //   break;

    // case 8190:
    //   mixer.setVelocity(0.15);
    //   mixer.setTurnrate(0);
    //   if (medge.width > 0.05)
    //   {
    //     cross_for_qiaoqiaoban++;
    //   }
    //   if (cross_for_qiaoqiaoban >= 2)
    //   {
    //     pose.dist = 0;
    //     pose.turned = 0;
    //     cross_for_qiaoqiaoban = 0;
    //     state = 8200;
    //   }

    //   break;

    // case 8200:
    //   if (pose.dist < 0.07)
    //   {
    //     mixer.setTurnrate(0);
    //     mixer.setVelocity(0.1);
    //   }
    //   else
    //   {
    //     pose.dist = 0;
    //     pose.turned = 0;
    //     state = 8300;
    //   }
    //   break;

    // case 8300:
    //   if (pose.turned < 3.15 / 2)
    //   {
    //     mixer.setTurnrate(3.15 / 2);
    //     mixer.setVelocity(0);
    //   }
    //   else
    //   {
    //     pose.dist = 0;
    //     pose.turned = 0;
    //     state = 9000;
    //   }
      // break;

    /***********************************************************************************
     *
     * 关门
     *
     ***********************************************************************************/
    case 11000:
      // 掉头
      if (pose.turned < 3.15)
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(1);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        mixer.setEdgeMode(false, -0.012); // 掉头之后应该向右巡线
        state = 11100;
      }
      break;
    case 11100:
      // 掉头成功
      pose.dist = 0;
      pose.turned = 0;
      mixer.setTurnrate(0);
      mixer.setVelocity(0.2);
      mixer.setEdgeMode(false, -0.012);
      // state = 11200;
      state = 11150; // 测试时候先注释
      break;
    case 11150:
      if (pose.dist > 3) {
        pose.dist = 0;
        pose.turned = 0;
        state = 11200;
      }
      break;
    // // 测试逻辑
    // case 11151:
    //     pose.dist = 0;
    //     pose.turned = 0;
    //     mixer.setTurnrate(0);
    //     mixer.setVelocity(0.3);
    //     mixer.setEdgeMode(false, -0.012); // 掉头之后应该向右巡线
    //     state = 11200;
    //     break;
    case 11200:
      // 当侧面传感器检测到东西的时候，说明到了隧道
      // 检测到隧道之后，后退出隧道，左转先关前门，再转回去关后门
      if(dist.dist[1] < 0.25 and dist.dist[1] > 0.02)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(-0.2);
        state = 11250;
        break;
      } else {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        mixer.setEdgeMode(false, -0.012); // 掉头之后应该向右巡线
        break;
      }
      break;
    case 11250:
      if (pose.dist < -0.15)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(1);
        mixer.setVelocity(0);
        state = 11300;
      } else {
        mixer.setTurnrate(0);
        mixer.setVelocity(-0.2);
      }
      break;
    case 11300:
      if (pose.turned > 1.5)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        state = 11350;
      } else {
        mixer.setTurnrate(1);
        mixer.setVelocity(0);
      }
      break;
    case 11350:
      if (pose.dist > 0.45)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(-1);
        mixer.setVelocity(0);
        state = 11400;
      } else {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
      }
      break;
    case 11400:
      if (pose.turned < -1.45)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        state = 11500;
      } else {
        mixer.setTurnrate(-1);
        mixer.setVelocity(0);
      }
      break;
      // 左转时正值
    case 11500:
      if (pose.dist > 1.3)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(1);
        mixer.setVelocity(0);
        state = 11550;
      } else {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
      }
      break;
    case 11550:
      // 倒着关门
      if (pose.turned > 1.5)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(-0.2);
        state = 11600;
      } else {
        mixer.setTurnrate(1);
        mixer.setVelocity(0);
      }
      break;
    case 11600:
      if (medge.width > 0.03)
      {
        detece_line_count++;
      }
      if (detece_line_count > 5) {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        detece_line_count = 0;
        state = 11605;
      }else {
        mixer.setTurnrate(0);
        mixer.setVelocity(-0.2);
      }
      break;
    case 11605:
      // 继续往后一点点转向
      if(pose.dist > 0.07) // wait
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(-1);
        mixer.setVelocity(0);
        state = 11650;
      }
      break;
    case 11650:
      // 找到白线转向之后前走巡线修正一下距离
      if (pose.turned < -1.57)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        mixer.setEdgeMode(false, -0.012);
        state = 11655;
      } else {
        mixer.setTurnrate(-1);
        mixer.setVelocity(0);
      }
      break;
    case 11655:
      if(pose.dist > 0.2)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(-0.1);
        state = 11700;
      } else {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        mixer.setEdgeMode(false, -0.012);
      }
      break;
    case 11700:
      // 倒车把门关好
      // 然后往前一点点右转关后门
      if (pose.dist < -0.5)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        mixer.setEdgeMode(false, -0.012);
        state = 11750;
      } else {
        mixer.setTurnrate(0);
        mixer.setVelocity(-0.1);
      }
      break;
    // 测试单独关后门的逻辑
    case 11701:
      pose.dist = 0;
      pose.turned = 0;
      mixer.setTurnrate(0);
      mixer.setVelocity(0.2);
      mixer.setEdgeMode(false, -0.012);
      state = 11750;
      break;
    case 11750:
      // 巡线往前一点点，做修正
      if (pose.dist > 0.2)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(-1);
        state = 11800;
      } else {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        mixer.setEdgeMode(false, -0.012);
      }
      break;
    case 11800:
      // 右转准备关后门
      if (pose.turned < -1.5)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        state = 11850;
      } else {
        mixer.setVelocity(0);
        mixer.setTurnrate(-1);
      }
      break;
    case 11850:
      if (pose.dist > 0.43) {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(-1);
        state = 11900;
      } else {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
      }
      break;
    case 11900:
      if (pose.turned < -1.5)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        state = 11950;
      } else {
        mixer.setVelocity(0);
        mixer.setTurnrate(-1);
      }
      break;
    case 11950:
      if (pose.dist > 1.25)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(1);
        mixer.setVelocity(0);
        state = 12000;
      } else {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
      }
      break;
    case 12000:
      if (pose.turned > 1.57)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(-0.2);
        state = 12050;
      } else {
        mixer.setTurnrate(1);
        mixer.setVelocity(0);
      }
      break;
    case 12050:
      if (medge.width > 0.03)
      {
        detece_line_count++;
      }

      if (detece_line_count > 5) {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        detece_line_count = 0;
        state = 12055;
      } else {
        mixer.setTurnrate(0);
        mixer.setVelocity(-0.2);
      }
      break;
    case 12055:
      if(pose.dist > 0.05)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(-1);
        mixer.setVelocity(0);
        state = 12100;
      }
      break;
    case 12100:
      // 找到白线转向之后前走巡线修正一下距离
      if (pose.turned < -1.57) // wait
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        mixer.setEdgeMode(false, 0);
        state = 12150;
      } else {
        mixer.setTurnrate(-1);
        mixer.setVelocity(0);
      }
      break;
    case 12150:
      // 往前巡线，找到岔路口
      if (medge.width > 0.07)
      {
        detece_line_count++;
      }

      if (detece_line_count > 5) {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(-0.1);
        state = 12200;
      }
      break;
    case 12200:
      // 倒车把门关好
      // 关门结束，巡线到终点
      if (pose.dist < -0.6) {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        mixer.setEdgeMode(true, 0.01);
        state = 12250;
      } else {
        mixer.setTurnrate(0);
        mixer.setVelocity(-0.1);
      }
      break;
    case 12250:
      // 再次过终点线
      if (pose.turned  < 3.0 and dist.dist[1] > 0.03 and dist.dist[1] < 0.2)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
        mixer.setEdgeMode(true, 0.01);
        state = 12300;
      } else {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        mixer.setEdgeMode(true, 0.01);
      }
      break;
    case 12300:
      // 过了终点线
      if (pose.dist > 0.1)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
        state = 12350;
      }
      break;
    case 12350:
      mixer.setVelocity(0.1);
      mixer.setTurnrate(0);
      if (medge.width > 0.05)
      {
        cross_for_qiaoqiaoban++;
      }
      if (cross_for_qiaoqiaoban >= 2)
      {
        pose.dist = 0;
        pose.turned = 0;
        cross_for_qiaoqiaoban = 0;
        state = 12400;
      }
      break;
    case 12400:
      if (pose.dist < 0.07)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 12450;
      }
      break;
    case 12450:
      if (pose.turned < 3.15 / 2)
      {
        mixer.setTurnrate(3.15 / 2);
        mixer.setVelocity(0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 9000;
      }
      break;
    /****************************************
     *
     * 赛道结束
     *
    *****************************************/



    /**********************************************************************************
     *
     *
     * 环岛
     *
     *
     ***********************************************************************************/
    case 6000:
      // 往左侧寻迹前进
      pose.dist = 0;
      pose.turned = 0;
      mixer.setEdgeMode(true, 0);
      state = 6100;
      break;

    case 6100:
      // 直到转过了弯并且小车到达合适位置
      // if (pose.turned > 1.5 && pose.dist > 1) {
      if (pose.turned > 3 / 2)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 6110;
      }
      break;
    case 6110:
      if (pose.dist < 0.58)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        mixer.setEdgeMode(true, 0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 6200;
      }
      break;

    case 6200:
      // 等待小车经过然后继续前进
      if (dist.dist[1] < 0.3 && thr_gate_small_car_state == 0)
      {
        thr_gate_small_car_state = 1;
      }

      // 让小车走过去
      if (thr_gate_small_car_state)
      {
        thr_gate_small_car_state = 0;
        state = 6210;
      }
      break;

    case 6210:
      pose.dist = 0;
      pose.turned = 0;
      mixer.setVelocity(0);
      mixer.setTurnrate(0);
      // mixer.setEdgeMode(true, 0);
      state = 6211;
      break;

    // 继续循迹前进
    case 6211:
      if (pose.dist < 0.98)
      { // 0.98
        mixer.setVelocity(0.2);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(true, 0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 6320;
      }
      break;
    case 6320:
      // 车尾对准环岛切线
      if (pose.turned > -3.14 * 1 / 2)
      {
        pose.dist = 0;
        mixer.setTurnrate(-3.14 * 1 / 3);
        mixer.setVelocity(-0.15);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 6400;
      }
      break;
    case 6400:
      // 倒车冲上去
      pose.dist = 0;
      pose.turned = 0;
      mixer.setTurnrate(0);
      mixer.setVelocity(-0.35);
      state = 6410;
      break;
    case 6410:
      // 登陆环岛  -1.01 -0.73
      if (pose.dist < -0.53)
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 6420;
      }
      break;
    case 6420:
      // 调整小车姿态
      pose.dist = 0;
      pose.turned = 0;
      mixer.setVelocity(0);  // 0.26
      mixer.setTurnrate(-2); // -4.5
      state = 6430;
      break;
    case 6430:
      // 调整完成，准备绕行
      if (pose.turned < -3.14 / 2)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 6500;
      }
      break;
    case 6500:
      // 环岛绕行
      pose.dist = 0;
      pose.turned = 0;
      mixer.setTurnrate(-0.85);
      mixer.setVelocity(0.3); // 0.3
      state = 6600;
      break;

    case 6600:
      // 环岛周长 2.2m
      // 绕行一圈之后，直线前进到环岛边缘
      if (pose.turned < -3.16 * 2)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0);
        pose.resetPose();
        state = 6700;
      }
      break;
    case 6700:
      // 调整姿态，面向外圈
      pose.dist = 0;
      pose.turned = 0;
      mixer.setVelocity(0);
      mixer.setTurnrate(3.14 / 2);
      state = 6710;
      break;
    case 6710:
      // 调整姿态，面向外圈
      // 然后停下来等小车经过
      if (pose.turned > 1.42)
      { // 1.3
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0);
        pose.resetPose();
        state = 6720;
      }
      break;
    case 6720:
      // 检测到小车经过
      if (dist.dist[0] < 0.4 && thr_gate_small_car_state == 0)
      {
        thr_gate_small_car_state = 1;
        startTime = system_clock::now();
      }

      endTime = system_clock::now();
      duration = endTime - startTime;

      // 间隔1秒，让小车过去
      if (thr_gate_small_car_state && duration >= seconds(1))
      {
        thr_gate_small_car_state = 0;
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
        state = 6721;
      }
      break;

    /**********************************************************************************
     *
     *
     * 绕外环岛
     *
     *
     ***********************************************************************************/
    case 6721:
      // 找到白线就右转 90度准备绕外环岛
      mixer.setTurnrate(0);
      mixer.setVelocity(0.2);

      if (medge.width > 0.03)
      {
        cross_for_qiaoqiaoban++;
      }

      if (cross_for_qiaoqiaoban >= 5)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        cross_for_qiaoqiaoban = 0;
        state = 6730;
      }
      break;

    case 6730:
      // 前进一点点转弯
      if (pose.dist < 0.07)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 6740;
      }
      break;

    case 6740:
      if (pose.turned > -2.98 / 2)
      {
        mixer.setTurnrate(-1.5);
        mixer.setVelocity(0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 6750;
      }
      break;

    case 6750:
      mixer.setVelocity(0.25);
      mixer.setTurnrate(0);
      mixer.setEdgeMode(true, 0);
      // mixer.setEdgeMode(false, 0.01);

      if (pose.turned < -3)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 6760;
      }

      if (dist.dist[0] < 0.15)
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
      }

      break;

    case 6760:

      mixer.setVelocity(0.25);
      mixer.setTurnrate(0);
      mixer.setEdgeMode(true, 0);

      if (pose.turned < -3 / 2)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 6800;
      }

      if (dist.dist[0] < 0.15)
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
      }

      break;

    case 6800:

      mixer.setVelocity(0.25);
      mixer.setTurnrate(0);
      mixer.setEdgeMode(true, 0);

      if (dist.dist[0] < 0.15)
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
      }

      if (pose.dist > 0.95)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 6810;
      }
      break;

    case 6810:
      mixer.setVelocity(0);
      mixer.setTurnrate(1);
      if (pose.turned > 3.14 / 4)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 6820;
      }
      break;

    case 6820:
      mixer.setVelocity(0.1);
      mixer.setTurnrate(0);

      if (medge.width > 0.07)
      {
        cross_for_qiaoqiaoban++;
      }

      if (cross_for_qiaoqiaoban >= 3)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        cross_for_qiaoqiaoban = 0;
        state = 6830;
      }

      break;

    case 6830:
      if (pose.dist < 0.06)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 6840;
      }
      break;

    case 6840:

      mixer.setVelocity(0);
      mixer.setTurnrate(1);
      if (pose.turned > 3.14 / 2)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0);
        state = 4000;
      }
      break;
    /**
     * 第一个路口逻辑
     * 从环岛出来后，会再次经过第一个路口，此时方向是反的，要调整回预期的方向（走最外圈）
     * 1. 后退 0.2m 然后左转90度
     * 2. 直接穿过岔路口
     * 3. 用慢速寻迹模式走一点（因为此时的偏移很大，速度太快容易偏）
     * 4. 等寻迹稳定之后加速，回到初始状态
     * 5. 结束
     */
    case 7000:
      // 后退 20cm 直接穿过岔路
      pose.dist = 0;
      pose.turned = 0;
      mixer.setTurnrate(0);
      mixer.setVelocity(-0.3);
      state = 7100;
      break;
    case 7100:
      // 后退了，转向
      if (pose.dist < -0.2)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(3.14 / 2);
        mixer.setVelocity(0.1);
        state = 7200;
      }
      break;
    case 7200:

      // 转向后，直接穿过岔路口
      if (pose.turned > 1.2)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0);
        mixer.setVelocity(0.3);
        state = 7300;
      }
      break;
    case 7300:
      // 找到白线，说明已经穿过岔路口了
      // 这时候启动慢速寻迹模式
      if (medge.width > 0.04)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setEdgeMode(false, 0);
        mixer.setVelocity(0.2);
        state = 7400;
      }
      break;
    case 7400:
      // 慢速走 1.4m 然后回到初始状态
      // 结束
      if (pose.dist > 1.4)
      {
        state = -9;
      }
      break;

      // case 8000:
      // {
      //   UCam cam;
      //   cam.setup();

      //   cv::Mat frame = cam.getFrameRaw(); // 获取未校正的原始图像
      //   if (frame.empty()) {
      //     std::cerr << "Failed to capture a frame." << std::endl;
      //   }
      //   std::vector<cv::Point2f> positions = ImageProcessing(frame);
      //   if (!positions.empty()) {
      //   // 如果检测到至少一个圆形对象，则打印所有检测到的位置
      //     for (const auto& pos : positions) {
      //       std::cout << "Position: x = " << pos.x << ", z = " << pos.y << std::endl;
      //     }
      //   }
      //   else {
      //   std::cout << "No golf balls detected." << std::endl;
      //   }
      //   break;
      // }

    default:
      // lost = true;
      break;

      /**********************************************************************************
       *
       *
       * 上坡 然后下坡 最后转向回来
       *
       *
       ***********************************************************************************/

    case 9000:
      pose.turned = 0;
      pose.dist = 0;
      ini["edge"]["kp"] = "45";
      ini["edge"]["maxturnrate"] = "7";
      ini["edge"]["lead"] = "0.3 0.2";
      mixer.setTurnrate(0);
      mixer.setVelocity(0.2);
      mixer.setEdgeMode(true, 0);
      state = 9100;
      break;

    case 9100: // 经过第一个转弯

      if (pose.turned > -3 / 2)
      {

        mixer.setVelocity(0.2);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(true, 0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 9110;
      }

      break;

    case 9110:
      if (pose.dist < 2.5)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.3);
        mixer.setEdgeMode(true, 0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(true, 0);
        state = 9200;
      }
      break;

    case 9200: // 经过第二个转弯

      if (pose.turned > -2.5 / 2)
      {

        mixer.setVelocity(0.2);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(true, -0.005);
      }

      else
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 9300;
      }
      break;

    case 9300:

      if (pose.dist < 2.5)
      {

        mixer.setVelocity(0.2);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(false, 0);
      }

      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 9400;
        // startTime = system_clock::now();
      }
      break;

    case 9400:

      if (pose.turned < 3.15)
      {

        mixer.setVelocity(0);
        mixer.setTurnrate(1);
      }

      else
      {
        pose.dist = 0;
        pose.turned = 0;
        state = 9405;
        // startTime = system_clock::now();
      }
      break;

    case 9405:
      if (pose.turned < 3.1 / 2)
      {
        mixer.setVelocity(0.3);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(true, 0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 9406;
      }
      break;

    case 9406:
      if (pose.dist < 3)
      {
        mixer.setVelocity(0.3);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(true, 0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 9410;
      }
      break;

    case 9410:
      mixer.setVelocity(0.2);
      mixer.setTurnrate(0);
      mixer.setEdgeMode(true, 0);
      if (medge.width > 0.05)
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 9500;
      }
      break;

    case 9500:
      if (medge.width > 0.05)
      {
        cross_for_qiaoqiaoban++;
      }

      if (cross_for_qiaoqiaoban >= 5)
      {
        toLog("found crossroad for qiaoqiaoban");
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        cross_for_qiaoqiaoban = 0;
        state = 9550;
      }
      break;

    case 9550:
      if (pose.dist < 0.08)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 9600;
      }
      break;

    case 9600:
      if (pose.turned < 3.15 / 2)
      {
        mixer.setTurnrate(1);
        mixer.setVelocity(0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(true, 0.01);
        state = 9610;
      }
      break;

    case 9610:
      if (pose.dist < 0.5)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
        mixer.setEdgeMode(true, 0.01);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 9700;
      }
      break;

    case 9700:
      mixer.setVelocity(0.1);
      mixer.setTurnrate(0);
      mixer.setEdgeMode(true, 0.01);

      if (medge.width < 0.001)
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 9705;
      }
      break;

    case 9705:
      mixer.setVelocity(0.1);
      mixer.setTurnrate(0);
      if (medge.width > 0.05)
      {
        cross_for_qiaoqiaoban++;
      }
      if (cross_for_qiaoqiaoban >= 2)
      {
        toLog("found line for qiaoqiaoban then turn 90 du");
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        cross_for_qiaoqiaoban = 0;
        state = 9710;
      }

      break;

    case 9710:
      if (pose.dist < 0.07)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 9720;
      }
      break;

    case 9720:
      if (pose.turned > -3.15 / 2)
      {
        mixer.setTurnrate(-0.7);
        mixer.setVelocity(0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(true, 0);
        state = 9730;
      }
      break;

    case 9730:
      if (pose.dist < 0.5)
      {
        mixer.setVelocity(0.1);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(true, 0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0.15);
        mixer.setTurnrate(0);
        state = 9735;
      }
      break;

    case 9735:
      if (pose.dist < 0.7)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 9740;
      }
      break;

    case 9740:
      mixer.setVelocity(0.1);
      mixer.setTurnrate(0);
      if (medge.width > 0.05)
      {
        cross_for_qiaoqiaoban++;
      }
      if (cross_for_qiaoqiaoban >= 5)
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        pose.dist = 0;
        pose.turned = 0;
        cross_for_qiaoqiaoban = 0;
        state = 9750;
      }
      break;

    case 9750:
      if (pose.dist < 0.08)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 9760;
      }
      break;

    case 9760:
      if (pose.turned < 3.15 / 2)
      {
        mixer.setTurnrate(1);
        mixer.setVelocity(0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(true, 0);
        state = 9770;
      }
      break;

    // 从跷跷板下来到终点
    case 9770:
      mixer.setVelocity(0.2);
      mixer.setTurnrate(0);
      mixer.setEdgeMode(true, 0);
      if (pose.turned > 3.05 / 2) // 转角90度
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        pose.dist = 0;
        pose.turned = 0;
        state = 9780;
      }
      break;

    case 9780:
      mixer.setTurnrate(0);
      mixer.setVelocity(0.4);
      mixer.setEdgeMode(true, 0);
      break;

    case 3000: // 上斜坡 下楼梯
      mixer.setVelocity(0.2);
      mixer.setTurnrate(0);
      mixer.setEdgeMode(true, 0);
      if (pose.turned < -3.05 / 2) // 转角90度
      {
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        pose.dist = 0;
        pose.turned = 0;
        state = 3100;
      }
      break;

    case 3100:             //
      if (pose.dist < 0.2) // 继续循迹0.2m
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
        mixer.setEdgeMode(true, 0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 3110;
      }
      break;

    case 3110:                     // 检测转弯
      if (pose.turned > -3.14 / 2) // 右拐90°
      {                            // 原地右拐90°
        mixer.setTurnrate(-1);
        mixer.setVelocity(0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 3200;
      }
      break;

    case 3200:

      if (pose.dist < 0.2) // 走0.2m 超过分叉路区域
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 3300;
      }

      break;

    case 3300:
      if (pose.dist < 2.1) // 边下楼梯边循迹
      {
        mixer.setVelocity(0.1);
        mixer.setTurnrate(0);
        mixer.setEdgeMode(false, 0);
      }
      else
      {
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0);
        mixer.setTurnrate(0);
        state = 3400;
      }
      break;

    case 3400:
      if (pose.dist < 0.55)
      { // 下完楼梯，继续慢速前进
        mixer.setTurnrate(0);
        mixer.setVelocity(0.1);
      }
      else
      {

        pose.dist = 0;
        pose.turned = 0;
        state = 3500;
      }
      break;

    case 3500:
      if (pose.turned > -3.14 / 2)
      { // 正在向右转90度
        // 这里不需要再次设置turnrate，因为已经在转向中
        mixer.setVelocity(0); // 停车
        mixer.setTurnrate(-1);
      }
      else
      {
        mixer.setTurnrate(0); // 停止转向
        mixer.setVelocity(0); // 完成右转后开始直行，寻找下一条线
        pose.dist = 0;
        pose.turned = 0;
        state = 3550; // 进入下一个状态，直行寻找下一条线
      }
      break;

    case 3550:
      if (pose.dist < 0.55)
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0.2);
      }
      else
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0);
        pose.dist = 0;
        pose.turned = 0;
        state = 3600;
      }
      break;

    case 3600:
      //
      if (medge.width > 0.02)
      {                       // 假设宽度大于0.02为检测到新的线
        mixer.setVelocity(0); // 停车，准备左转
        state = 3700;         // 转向下一个状态进行左转
      }
      break;

    case 3700:
      if (pose.turned < 3.14 / 2)
      {
        mixer.setTurnrate(1); // 开始原地向左转
      }
      else
      {
        mixer.setTurnrate(0);   // 停止转向
        mixer.setVelocity(0.2); // 完成左转后，沿着新线直行
        pose.dist = 0;
        pose.turned = 0;
        state = 3800;
      }
      break;

    case 3800:
      // 继续沿着线循迹，检测前方是否有物体
      if (pose.dist < 1)
      { //
        mixer.setEdgeMode(true, 0);
        mixer.setVelocity(0.2); // 检测到前方有物体，停下来
                                // 根据需要进行后续操作，例如转换状态或完成任务
      }
      else
      {
        mixer.setTurnrate(0);
        mixer.setVelocity(0);
      }
      break;
    }
    if (state != oldstate)
    { // C-type string print
      snprintf(s, MSL, "State change from %d to %d", oldstate, state);
      toLog(s);
      oldstate = state;
      t.now();
    }
    // wait a bit to offload CPU (4000 = 4ms)
    usleep(4000);
  }
  if (lost)
  { // there may be better options, but for now - stop
    toLog("Plan21 got lost");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("Plan21 finished");
}

void BPlan21::terminate()
{ // just close logfile
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan21::toLog(const char *message)
{
  UTime t("now");
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec() / 100,
            oldstate,
            message);
  }
  if (toConsole)
  {
    printf("%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec() / 100,
           oldstate,
           message);
  }
}
