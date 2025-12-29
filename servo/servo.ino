#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>

Servo servo1;
Servo servo2;

ros::NodeHandle nh;

int center = 0;

// ステージ設定
int stage1Speed = 2;   // ステージ1（速い）
int stage2Speed = 4;   // ステージ2（遅い）
int stage1Range = 15;  // ステージ1 ±15°
int stage2Range = 30;  // ステージ2 ±30°

String last_cmd = "";

void runStage(int stage);

// ===== ROS からのコマンド受信 =====
void stageCmdCallback(const std_msgs::String& msg) {
  last_cmd = msg.data;

  if (last_cmd == "stage1") {
    runStage(1);
  }
  else if (last_cmd == "stage2") {
    runStage(2);
  }
  else if (last_cmd == "stop") {
    servo1.write(center);
    servo2.write(center);
  }
}

ros::Subscriber<std_msgs::String> sub("smile_status", stageCmdCallback);

// ===== セットアップ =====
void setup() {
  nh.initNode();
  nh.subscribe(sub);

  Serial.begin(9600);
  randomSeed(analogRead(0));

  servo1.attach(48);
  servo2.attach(50);

  servo1.write(center);
  servo2.write(center);
  delay(1000);
}

void loop() {
  nh.spinOnce();
  delay(5);
}

// ===== Stage1 / Stage2 動作 =====
void runStage(int stage) {
  int spd, range;

  if (stage == 1) {
    spd = stage1Speed;
    range = stage1Range;
    Serial.println("Run Stage 1");
  } else {
    spd = stage2Speed;
    range = stage2Range;
    Serial.println("Run Stage 2");
  }

  int left = center + range;
  int right = center - range;

  // パターン（右→左→右→左）
  moveServo(left, right, spd);
  moveServo(right, left, spd);
  moveServo(left, right, spd);
  moveServo(right, left, spd);

  servo1.write(center);
  servo2.write(center);
  delay(100);
}

// ===== サーボゆっくり動作 =====
void moveServo(int from, int to, int spd) {
  if (from < to) {
    for (int i = from; i <= to; i++) {
      servo1.write(i);
      servo2.write(i);
      delay(spd);
    }
  } else {
    for (int i = from; i >= to; i--) {
      servo1.write(i);
      servo2.write(i);
      delay(spd);
    }
  }
}
