#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*=======================
 * Servo
 * ======================*/
Servo servo1;
Servo servo2;

int center = 90;

// ステージ設定
int stage1Speed = 2;   // ステージ1（速い）
int stage2Speed = 4;   // ステージ2（遅い）
int stage1Range = 15;  // ステージ1 ±15°
int stage2Range = 30;  // ステージ2 ±30°

/*=======================
 * OLED(I2C)
 * ======================*/
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool showCircle = true;       // true = ◯, false = ●
unsigned long lastSmilingTime = 0;
const unsigned long inverseDuration = 5000; // 5秒

/*======================
 * ROS
 * =====================*/
ros::NodeHandle nh;

/*======================
 * Publisher
 * =====================*/
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";

/*======================
 * Function Prototypes
 * =====================*/
void runStage(int stage);
void moveServo(int from, int to, int spd);

/*======================
 * Subscriber Callbacks
 * =====================*/

// /movement_status を受信 → Stage1実行
void movementStatusCallback(const std_msgs::String& msg) {
  Serial.println("Received /movement_status");
  runStage(1);  // ステージ1を実行
}

// /smile_status を受信 → Stage2実行 + OLED制御
void smileStatusCallback(const std_msgs::String& msg) {
  Serial.println("Received /smile_status");
  runStage(2);  // ステージ2を実行
  
  // OLED制御: "smiling"を受信したときに●に切り替え
  if (strcmp(msg.data, "smiling") == 0) {
    showCircle = false;
    lastSmilingTime = millis();
  }
}

ros::Subscriber<std_msgs::String> sub_movement("movement_status", movementStatusCallback);
ros::Subscriber<std_msgs::String> sub_smile("smile_status", smileStatusCallback);

/*======================
 * setup
 * =====================*/
void setup() {
  // Serial初期化
  Serial.begin(9600);
  
  // OLED初期化
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 disconnection"));
    for(;;); // OLED初期化失敗
  }
  display.clearDisplay();
  display.display();

  // ROS初期化
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub_movement);  // /movement_status を購読
  nh.subscribe(sub_smile);     // /smile_status を購読

  // Servo初期化
  randomSeed(analogRead(0));
  servo1.attach(48);
  servo2.attach(50);
  servo1.write(center);
  servo2.write(center);
  
  delay(1000);
}

/*======================
 * loop
 * =====================*/
void loop() {
  // ROS hello world publisher
  str_msg.data = hello;
  chatter.publish(&str_msg);

  // ROS通信処理
  nh.spinOnce();

  // OLED描画
  display.clearDisplay();
  if (!showCircle) {
    // ●を描画
    display.fillCircle(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, 25, SSD1306_INVERSE);
    
    // 5秒経過したら元に戻す
    if (millis() - lastSmilingTime > inverseDuration) {
      showCircle = true;
    }
  }
  else {
    // ◯を描画
    display.drawCircle(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, 25, SSD1306_WHITE);
  }
  display.display();

  delay(5);
}

/*======================
 * Stage1 / Stage2 動作
 * =====================*/
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

/*======================
 * サーボゆっくり動作
 * =====================*/
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