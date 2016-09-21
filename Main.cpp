#include <iostream>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "RasPiDS3/RasPiDS3.hpp"
#include "RasPiMS/RasPiMS.hpp"
#include <cstdio>

using namespace std;
using namespace RPDS3;
using namespace RPMS;

string pathGet();
ofstream Log;

int main(void){   
  DualShock3 controller;
  MotorSerial ms;
  static int MAX = 200;    //PWMの最大値
  /*-------------GPIOピン割り当て-------------*/
  //動作確認LED
  int RunLED = 13;
  //電磁弁
  int Air_1 = 26;   //回転・上下機構
  //int Air_2 = 17;   //吸引機構
  //リミットスイッチ
  int Lsin_1 = 27;  //回転腕の下端行き過ぎ防止
  int Lsin_2 = 23;  //回転腕の上端行き過ぎ防止
  //int Lsin_3 = 22;  //回転腕と共に移動する
  int Lsin_4 = 17;  //回転腕①開きすぎ防止
  int Lsin_5 = 18;  //回転腕②開きすぎ防止
  //状態確認LED
  /*-------------割り当てここまで-------------*/
  cout << "プログラム開始." << endl;
  //プログラム起動時刻の取得
  time_t progStartTime = time(NULL);
  struct tm *pnow = localtime(&progStartTime);
  //ログファイルを開く
  string path(pathGet().c_str());
  path.erase(path.find_last_of('/'));
  Log.open(path + "/Log.txt");
  Log << pnow->tm_year + 1900<< ":" << pnow->tm_mon + 1 << ":" << pnow->tm_mday << ":" <<  pnow->tm_hour << ":" << pnow->tm_min << ":" << pnow->tm_sec << endl;
  Log << "Path :" + path << endl; 
  //MDD通信セットアップ
  try{
    ms.init();
  }
  catch(const char *str){
    return -1;
  }
  Log << "MotorSerialのセットアップ成功" << endl;
  //コントローラー接続確認
  if(!controller.connectedCheck()){
    cout << "コントローラが接続されていません。" << endl;
    return 0;
  }
  Log << "コントローラの接続成功" << endl;
  cout << "MDD通信準備完了" << endl;
  Log << "Standby MotorDriverDriver Communication." << endl;
  pinMode(RunLED, OUTPUT);
  pinMode(Air_1, OUTPUT);
  //pinMode(Air_2, OUTPUT);
  pinMode(Lsin_1, INPUT);
  pinMode(Lsin_2, INPUT);
  //pinMode(Lsin_3, INPUT);
  pinMode(Lsin_4, INPUT);
  pinMode(Lsin_5, INPUT);
  //リミットスイッチのプルダウン
  pullUpDnControl(Lsin_1, PUD_DOWN);
  pullUpDnControl(Lsin_2, PUD_DOWN);
  //pullUpDnControl(Lsin_3, PUD_DOWN);
  pullUpDnControl(Lsin_4, PUD_DOWN);
  pullUpDnControl(Lsin_5, PUD_DOWN);
  digitalWrite(RunLED, 1);  //動作用LED点灯
  //回転腕
  static bool Flag_1 = false;
  static bool Flag_2 = false;
  static bool Flag_3 = false;

  static bool Flag_O2 = false;    //OpenFlag2
  static bool Flag_O1 = false;    //OpenFlag1

  unsigned int OpenTimer_2;
  unsigned int OpenTimer_1;

  static bool Open_1 = false;
  static bool Open_2 = false;

  static bool timerFlag = false;      //挟む用millis()取得
  static bool timerFlag_2 = false;    //①手前を離す用millis()取得
  static bool timerFlag_3 = false;    //②を離す用millis()取得
  int count_1 = -1;                   //回転腕のモードカウント
  unsigned long close;                //挟む
  unsigned long open_1;               //①前を離す
  unsigned long open_2;               //②を離す
  static bool closeFlag = false;
  unsigned int ATime = false;         //回転腕を戻す用millis()取得
  static bool AirTimer = false;     
  static bool AirFlag_1 = false;
  static bool AirFlag_2 = false;
  unsigned int CloseTime = 1600;             //閉じる時間
  unsigned int OpenTime = 1700;              //開く時間
  static bool Flag_750 = false;
  
  static bool Up_Limit = false;
  unsigned int Up_time;
  static bool Up_Flag = false;

  //自動制御
  static bool AutoClose = false;

  /*static bool CheckFlag = false;
  unsigned int checkTime;
  static int checkcount = 0;
  static int sampling = 0;
  static bool Flag_4 = false;*/

  static int count_sw = 0;

  //STARTボタンとSELECTボタンが押されるまで実行
  UPDATELOOP(controller, !(controller.button(START)&& controller.button(SELECT))){ 	
    //全モーターの非常停止。SELECTを押すと作動、もう一度押すと解除
    /*if(controller.press(SELECT)){
      ms.send(255, 255, 0);
      cout << "全モーター非常停止" << endl;
      UPDATELOOP(controller, !controller.press(SELECT));
      cout << "非常停止解除" << endl;
    }*/

    //回転腕のリミットスイッチ監視
    /*if(!CheckFlag){
      sampling = digitalRead(Lsin_3);
      CheckFlag = true;
      checkTime = millis();
      checkcount++;
    }
    if((millis() - checkTime) > 20)
      CheckFlag = false;
    if(checkcount > 2){
      if(sampling && !Flag_4){
        count_sw++;
        Flag_4 = true;
      }
      else if(!sampling){
        Flag_4 = false;
      }
    }
    if(count_sw == 2){
      cout << "750mm位置" << endl;
      Flag_750 = true;
      count_sw = 0;
    }
    cout << count_sw << endl;*/

    double regulation = 1;    //通常モード（出力は1.0倍）
    //L1を押した時のアクション
    if(controller.button(L1)){
      regulation = 0.2;         //低速モード（出力は通常の0.2倍）
      //自動制御①)750まで上げるだけ)      
      /*if(controller.button(TRIANGLE) && controller.button(L1)){
        ms.send(2, 2, 210);
        Up_time = millis();
        Up_Flag = true;
      }*/
      //自動制御②（灯台完成）
      /*if(controller.press(SQUARE)){
        ms.send(2, 2, -210);
      }*/
      //回転腕で箱を掴む、①を離す、②を離す  count_1 は0→1→12ででloop
      /*if(controller.press(CIRCLE)){
        if(count_1 == 2) Flag_3 = false;
        if(count_1 == 0) Flag_1 = false;
        if(count_1 == 1) Flag_2 = false;
        count_1++;
        if(count_1 > 2) count_1 = 0;
      }*/
      //回転・上下機構の回転 CROSSを押すと回転、もう一度押すと元に戻る
      /*static bool airSpin = false;
      if(controller.press(CROSS))
        airSpin = !airSpin;
      if(!airSpin && controller.press(CROSS)){
        digitalWrite(Air_1, 0);
        AirTimer = false;
        AirFlag_1 = false;
        AirFlag_2 = false;
        cout << "回転腕回転" << endl;
      }
      if(airSpin && controller.press(CROSS)){
        digitalWrite(Air_1, 1);
        ATime = millis();
        AirTimer = true;
      }*/
    }
    bool dual = false;    //trueにするとLeftStickのみでロボット全体を操作できる
    //R1を押した時のアクション
    if(controller.button(R1)){
      //dual = true;
      regulation = 0.1;     //超低速モード（出力が通常のの0.1倍）
    }
    //橋用タイヤ
    if(controller.button(L1) && controller.button(R1)){
      int tire = controller.stick(LEFT_Y) + controller.stick(RIGHT_Y);
      if(tire > MAX)  tire = MAX;
      if(tire < -MAX) tire = -MAX;
      ms.send(1, 2, -tire);
     // cout << -tire << endl;
    }
    if(controller.release(L1) || controller.release(R1))
      ms.send(1, 2, 0);

    //左に平行移動
    bool LeftControl = false;
    if(controller.button(LEFT))
      LeftControl = true;
    //右に平行移動
    bool RightControl = false;
    if(controller.button(RIGHT))
      RightControl = true;

    //足回り（左側前後）
    double left_x = 0;
    double left_y = 0;
    double left_theta = 0;
    int left_w = 0;
    double lb, lf;
    if(LeftControl){
      left_x = -128;
      left_y = 0;
    }
    else if(RightControl){
      left_x = 127;
      left_y = 0;
    }
    else{
      left_y = controller.stick(LEFT_Y);
      left_x = controller.stick(LEFT_X);
    }
    left_w = sqrt(pow(left_x, 2) + pow(left_y, 2)) * 2;
    if(left_w > MAX)  left_w = MAX;
    left_theta = (atan2(-left_y, left_x) + M_PI);
    if(left_theta >= 0 && left_theta <= (M_PI/2)){
      lb = (left_theta * 4 / M_PI) - 1;
      lf = 1;
    }else if(left_theta > (M_PI/2) && left_theta <= M_PI){
      lb = 1;
      lf = -(left_theta * 4 / M_PI) + 3;
    }else if(left_theta > (M_PI) && left_theta <= (M_PI*3/2)){
      lb = -(left_theta * 4 / M_PI) + 5;
      lf = -1;
    }else if(left_theta > (M_PI*3/2) && left_theta <= (M_PI*2)){
      lb = -1;
      lf = (left_theta * 4 / M_PI) - 7;
    }
    ms.send(1, 3, lf * left_w * regulation);  //左前
    ms.send(2, 3, lb * left_w * regulation);  //左後
    //足回り（右側前後）
    double right_x = 0;
    double right_y = 0;
    double right_theta = 0;
    int right_w = 0;
    double rf, rb;
    if(LeftControl){
      right_x = -128;
      right_y = 0;
    }
    else if(RightControl){
      right_x = 127;
      right_y = 0;
    }
    else{
      right_y = controller.stick(RIGHT_Y);
      right_x = controller.stick(RIGHT_X);
    }
    right_w = sqrt(pow(right_x, 2) + pow(right_y, 2)) * 2;
    if(right_w > MAX) right_w = MAX;
    right_theta = (atan2(-right_y, right_x) + M_PI);
    if(right_theta >= 0 && right_theta <= (M_PI/2)){
      rf = -(right_theta * 4 / M_PI) + 1;
      rb = -1;
    }else if(right_theta > (M_PI/2) && right_theta <= M_PI){
      rf = -1;
      rb = (right_theta * 4 / M_PI) - 3;
    }else if(right_theta > (M_PI) && right_theta <= (M_PI*3/2)){
      rf = (right_theta * 4 / M_PI) - 5;
      rb = 1;
    }else if(right_theta > (M_PI*3/2) && right_theta <= (M_PI*2)){
      rf = 1;
      rb = -(right_theta * 4 / M_PI) + 7;
    }
    if(dual){
      ms.send(4, 3, -(lb * left_w * regulation)); //右後
      ms.send(5, 3, -(lf * left_w * regulation)); //右前
    }else{
      ms.send(4, 3, rf * right_w * regulation); //右前
      ms.send(5, 3, rb * right_w * regulation); //右後
    }
    //回転腕機構の上下（手動操作）
    //上
    if(controller.button(UP)){
      Up_Limit = false;
      if(!(digitalRead(Lsin_2)) || digitalRead(Lsin_1))
        ms.send(2, 2, -210);
      else{
        ms.send(2, 2, 0);
        count_sw = 0;
      }
    }
    //下
    if(controller.button(DOWN)){
      Up_Limit = false;
      if(!(digitalRead(Lsin_1)) || digitalRead(Lsin_2))
        ms.send(2, 2, 210);
      else{
        ms.send(2, 2, 0);
        count_sw = 0;
      }
    }
    if(controller.release(UP) || controller.release(DOWN) || Flag_750){
      //箱を挟んでいないとき
      if(digitalRead(Lsin_4) && digitalRead(Lsin_5))
        ms.send(2, 2, -5);
      else
        ms.send(2, 2, -20);
      Flag_750 = false;
    }

    static bool OpenFlag_2 = false;
    if(controller.press(TRIANGLE))
      OpenFlag_2 = !OpenFlag_2;
    if(!OpenFlag_2 && controller.button(TRIANGLE))
      Open_2 = true;
    if(OpenFlag_2 && controller.button(TRIANGLE)){
      ms.send(5, 2, -110);
      OpenTimer_2 = millis();
      Flag_O2 = true;
      Open_2 = false;
    }
    static bool OpenFlag_1 = false;
    if(controller.press(CROSS))
      OpenFlag_1 = !OpenFlag_1;
    if(!OpenFlag_1 && controller.button(CROSS))
      Open_1 = true;
    if(OpenFlag_1 && controller.button(CROSS)){
      ms.send(4, 2, -110);
      OpenTimer_1 = millis();
      Flag_O1 = true;
      Open_1 = false;
    }
    if(Open_1){
      if(!(digitalRead(Lsin_5))){
        ms.send(4, 2, 110);
        cout << "離してる" << endl;
      }
      if(digitalRead(Lsin_5)){
        ms.send(4, 2, 0);
        Open_1 = false;
        cout << "離した" << endl;
      }
    }
    if(Open_2){
      if(!(digitalRead(Lsin_4))){
        ms.send(5, 2, 110);
      }
      if(digitalRead(Lsin_4)){
        ms.send(5, 2, 0);
        Open_2 = false;
      }
    }

    //回転・上下機構の回転 CROSSを押すと回転、もう一度押すと元に戻る
    static bool airSpin = false;
    if(controller.press(SQUARE))
      airSpin = !airSpin;
    if(!airSpin && controller.press(SQUARE)){
      digitalWrite(Air_1, 0);
      AirTimer = false;
      AirFlag_1 = false;
      AirFlag_2 = false;
      cout << "回転腕回転" << endl;
    }
    if(airSpin && controller.press(SQUARE)){
      digitalWrite(Air_1, 1);
      ATime = millis();
      AirTimer = true;
    }
    //回転腕をある程度の位置まで上げる
    if(controller.press(CIRCLE)){
      ms.send(2, 2, 210);
      Up_time = millis();
      Up_Flag = true;
    }
    if(Up_Flag){
      if(millis() - Up_time > 5000 || digitalRead(Lsin_2)){
        ms.send(2, 2, 50);
        cout << "自動制御完了or上端" << endl;
        Up_Flag = false;
      }
    }

    /*if(controller.release(TRIANGLE)|| controller.release(SQUARE))
      ms.send(5, 2, 0);

    if(controller.button(CIRCLE))
      ms.send(4, 2, -100);
    if(controller.button(CROSS)){
      if((!digitalRead(Lsin_5))){
        ms.send(4, 2, 100);
      }
      if(digitalRead(Lsin_5)){
        ms.send(4, 2, 0);
      }
    }
    if(controller.release(CIRCLE) || controller.release(CROSS))
      ms.send(4, 2, 0);*/

    //ロジャーアームの上下
    /*if(controller.button(UP))
      ms.send(4, 2, -MAX * regulation);
    if(controller.button(LEFT))
      ms.send(4, 2, MAX * regulation);
    if(controller.release(UP) || controller.release(LEFT))
      ms.send(4, 2, 0);
    //吸引機構の回転
    if(controller.button(L2))      //左回転
      ms.send(18, 2, (controller.stick(LEFT_T) / 5));
    if(controller.button(R2))      //右回転
      ms.send(18, 2, -(controller.stick(RIGHT_T) / 5));
    if(controller.release(L2) || controller.release(R2))
      ms.send(18, 2, 0);*/
    //電磁石 TRIANGLEを押すとON、もう一度押すとOFF
    /*static bool MagnetFlag = false;
    if(controller.press(TRIANGLE))
      MagnetFlag = !MagnetFlag;
    if(MagnetFlag && controller.press(TRIANGLE) && !(controller.button(L1))){
      ms.send(11, 3, -240);
      //digitalWrite(StatusLED, 1);
      cout << "電磁石ON" << endl;
    }
    if(!MagnetFlag && controller.press(TRIANGLE) && !(controller.button(L1))){
      ms.send(11, 3, 0);
      //digitalWrite(StatusLED, 0);
      cout << "電磁石OFF" << endl;
    }
    //真空ポンプ  CROSS押すとON、もう一度押すとOFF
    static bool PompSpin = false;
    if(controller.press(CROSS))
      PompSpin = !PompSpin;
    if(PompSpin && controller.press(CROSS) && !(controller.button(L1)) ){
      ms.send(11, 2, 240);
      //digitalWrite(StatusLED, 1);
      cout << "真空ポンプ作動中" << endl;
    }
    if(!PompSpin && controller.press(CROSS) && !(controller.button(L1))){
      ms.send(11, 2, 0);
      //digitalWrite(StatusLED, 0);
      cout << "真空ポンプ作動停止" << endl; 
    }
    //吸引機構の電磁弁
    if(controller.press(CIRCLE) && !(controller.press(L1))){
      digitalWrite(Air_2, 1);
      PompOff = millis();
      PTimer_1 = true;
    }*/
    //回転腕の回転が元に戻る
    if(AirTimer){
      if(millis()-ATime > 350 && !AirFlag_1){
        digitalWrite(Air_1, 0);
        //cout << "一瞬だけ1になる" << endl;
        AirFlag_1 = true;
      }
      if(millis()-ATime > 500 && !AirFlag_2){
        digitalWrite(Air_1, 1);
        cout << "回転腕が元に戻る" << endl;
        AirFlag_2 = true;
      }
    }


    if(Flag_O2){
      if(millis()-OpenTimer_2 > 2000){
        ms.send(5, 2, 0);
        cout << "②はさんだ" << endl;
        Flag_O2 = false;
      }
    }

    if(Flag_O1){
      if(millis()-OpenTimer_1 > 2000){
        ms.send(4, 2, 0);
        cout << "①はさんだ" << endl;
        Flag_O1 = false;
      }
    }

    //吸引機構電磁弁動作
    /*if(PTimer_1){
      if((millis() - PompOff) > 5){
        digitalWrite(Air_2, 0);
        cout << "吸引機構電磁弁動作" << endl;
        PTimer_1 = false;
      }
    }*/
    //----------回転腕（手動操作）----------
    //回転腕で箱を挟む
    if(count_1 == 2 || AutoClose){
      if(!Flag_1){
        cout << "はさんでいます" << endl;
        ms.send(5, 2, -100);
        ms.send(23, 2, -100);
        closeFlag = true;
        close = millis();
        timerFlag = true;
        Flag_1 = true;
      }
    }
    if(timerFlag){
      if((millis() - close) > CloseTime){
        ms.send(5, 2, 0);
        ms.send(23, 2, 0);
        cout << "はさんだ" << endl;
        timerFlag = false;
        closeFlag = false;
        AutoClose = false;
      }
    }

    //①を離す
    if(count_1 == 0 && !closeFlag){
      if(!Flag_2){
        cout << "①を離しています" << endl;
        ms.send(5, 2, 100);
        open_1 = millis();
        timerFlag_2 = true;
        Flag_2 = true;
      }
    }
    if(timerFlag_2){
      if((millis() - open_1) > OpenTime){
        ms.send(5, 2, 0);
        cout << "①を離した" << endl;
        timerFlag_2 = false;
      } 
    }
    //②をを離す
    if(count_1 == 1 && !closeFlag){
      if(!Flag_3){
        cout << "②を離しています" << endl;
        ms.send(23, 2, 100);
        open_2 = millis();
        timerFlag_3 = true;
        Flag_3 = true;
      }
      if(timerFlag_3){
        if((millis() - open_2) > OpenTime){
          ms.send(23, 2, 0);
          cout << "②を離した "<< endl;
          timerFlag_3 = false;
        }
      }
    }

    //回転腕のリミットスイッチ監視
    /*if(!CheckFlag){
      sampling = digitalRead(Lsin_3);
      CheckFlag = true;
      checkTime = millis();
      checkcount++;
    }
    if((millis() - checkTime) > 20)
      CheckFlag = false;
    if(checkcount > 2){
      if(sampling && !Flag_4){
        count_sw++;
        Flag_4 = true;
      }
      else if(!sampling){
        Flag_4 = false;
      }
    }
    if(count_sw == 2){
      cout << "750mm位置" << endl;
      Flag_750 = true;
      count_sw = 0;
    }
    cout << count_sw << endl;*/

  }
  Log << "Successful completion" << endl; 
  cout << "プログラム終了" << endl;
  digitalWrite(Air_1, 0);
  digitalWrite(RunLED, 0);    //動作確認LED消灯
  return 0;
}

string pathGet(){
  //現在のパスを取得
  char buf[512] = {};
  readlink("/proc/self/exe", buf, sizeof(buf) - 1);   // 実行ファイルのパスを取得
  string path(buf);
  return path;
}
