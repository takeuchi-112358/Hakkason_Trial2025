#include <ros.h>
#include <std_msgs/String.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


/*=======================
 * OLED(I2C)
 * ======================*/
 
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADRESS 0x3c

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
 * Subscriber(表情状態管理)
 * =====================*/
//std_msgs::Bool rx_msg;
bool showCircle = true;       // true = ◯, false = ●
unsigned long lastSmilingTime = 0;
const unsigned long inverseDuration = 5000; // 5秒

/*======================
 * Subscriber Callback
 * =====================*/
void smileStatusCb(const std_msgs::String &msg)
{
  // "smiling" を受信したときに反応
  if ( strcmp ( msg.data, "smiling" ) == 0)
  {
    showCircle = false;             // ●に切り替え
    lastSmilingTime = millis();     // タイマー開始
  }
}

ros::Subscriber<std_msgs::String> sub( "smile_status", &smileStatusCb );
 
/*
void messageCb(const std_msgs::Bool &msg)
{
  if(msg.data){
    showCircle = false;               // ●に切り替え
    lastInverseTime = millis();       // タイマー開始
  }
}

ros::Subscriber<std_msgs::Bool> sub("XX", &messageCb);
*/

/*======================
 * setup
 * =====================*/
void setup() 
{
  // OLED 初期化
  if ( !display.begin(SSD1306_SWITCHCAPVCC, 0x3C) ) 
  {
    Serial.println(F("SSD1306 disconnection"));
    for(;;); // OLED初期化失敗
  }
  display.clearDisplay();
  display.display();

  // ROS 初期化
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

/*======================
 * loop
 * =====================*/
void loop() 
{
  // ROS hello world publisher
  str_msg.data = hello;
  chatter.publish(&str_msg);

  nh.spinOnce();

  // OLED 描画
  display.clearDisplay();
  if ( !showCircle )
  {
    // ●を描画
    display.fillCircle( SCREEN_WIDTH/2, SCREEN_HEIGHT/2, 25, INVERSE );
    
    // 5秒経過したら元に戻す
    if ( millis() - lastSmilingTime > inverseDuration )
    {
      showCircle = true;
    }
  }
  else
  {
    // ◯を描画
    display.drawCircle( SCREEN_WIDTH/2, SCREEN_HEIGHT/2, 25, SSD1306_WHITE );
  }
  display.display();

  delay(100);
}
