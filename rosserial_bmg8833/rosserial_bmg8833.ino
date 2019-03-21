
// ROS
#include <ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

// Adafruit AMG88XX
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

// Interpolation
#include "interpolation.h"

//ROS
//namespace ros {
//typedef NodeHandle_<ArduinoHardware, 25, 25, 512, 512> NodeHandle;
//}

// AMG88XX
#define AMG88XX_WIDTH  (8)
#define AMG88XX_HEIGHT (AMG88xx_PIXEL_ARRAY_SIZE / AMG88XX_WIDTH)

ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048> nh;
sensor_msgs::Image img_msg;
//sensor_msgs::Image img_msg2;
//sensor_msgs::Image img_msg3;
// sensor_msgs::Image img_imu;
ros::Publisher pub_img("image_raw", &img_msg);
// ros::Publisher pub_img2("image_raw", &img_msg2);
// ros::Publisher pub_img3("image_raw3", &img_msg3);
// ros::Publisher pub_imu("temperature", &img_imu);

Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
// float pixels3[14 * 14];
// uint16_t pixels_u16[AMG88xx_PIXEL_ARRAY_SIZE];

void blink(int num, int msec) {
  for (int i = 0; i < num; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(msec);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(msec);
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite( LED_BUILTIN, HIGH);

  // join i2c bus (address optional for master) 
  // Wire.begin();

  bool status = amg.begin();
  while (!status) {
    status = amg.begin();
  }
#if 1
  img_msg.height = AMG88XX_HEIGHT;
  img_msg.width = AMG88XX_WIDTH;
  // img_msg.encoding = sensor_msgs::image_encodings::MONO8;
  img_msg.encoding = "32FC1";
  img_msg.is_bigendian = 0;
  img_msg.step = 4 * img_msg.width;
  img_msg.data_length = img_msg.height * img_msg.step;
  img_msg.data = reinterpret_cast<uint8_t *>(pixels);
#endif

#if 0
  img_msg2.height = AMG88XX_HEIGHT;
  img_msg2.width = AMG88XX_WIDTH;
  // img_msg.encoding = sensor_msgs::image_encodings::MONO8;
  img_msg2.encoding = "mono16";
  img_msg2.is_bigendian = 1;
  img_msg2.step = 2 * img_msg2.width;
  img_msg2.data_length = img_msg2.height * img_msg2.step;
  img_msg2.data = reinterpret_cast<uint8_t *>(pixels_u16);
#endif

#if 0
  img_msg3.height = 14;
  img_msg3.width = 14;
  // img_msg.encoding = sensor_msgs::image_encodings::MONO8;
  img_msg3.encoding = "32FC1";
  img_msg3.is_bigendian = 0;
  img_msg3.step = 4 * img_msg3.width;
  img_msg3.data_length = img_msg3.height * img_msg3.step;
  img_msg3.data = reinterpret_cast<uint8_t *>(pixels3);
#endif

  blink(5, 500);

  nh.initNode();
  nh.advertise(pub_img);
//  nh.advertise(pub_img2);
//  nh.advertise(pub_img3);
}

long publisher_timer;

void loop() {
  // put your main code here, to run repeatedly:

  amg.readPixels(pixels);
//  interpolate_image(pixels, 8, 8, pixels3, 14, 14);

#if 0
  // value = integer * AMG88xx_PIXEL_TEMP_CONVERSION
  //       signed 12[bit]            0.5
  //       [-2048,2048]
  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    pixels_u16[i] = static_cast<uint16_t>(pixels[i] * 2.0);
  }
#endif

  delay(100);
  pub_img.publish(&img_msg);
//  pub_img2.publish(&img_msg2);
//  pub_img3.publish(&img_msg3);
  
  nh.spinOnce();
}
