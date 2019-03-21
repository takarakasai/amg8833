
// ROS
#include <ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

// Adafruit AMG88XX
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

// Interpolation

// AMG88XX
#define AMG88XX_WIDTH  (8)
#define AMG88XX_HEIGHT (AMG88xx_PIXEL_ARRAY_SIZE / AMG88XX_WIDTH)

ros::NodeHandle_<ArduinoHardware, 5, 5, 512, 2048> nh;
sensor_msgs::Image img_msg;
ros::Publisher pub_img("image_raw", &img_msg);

Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

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

  img_msg.height = AMG88XX_HEIGHT;
  img_msg.width = AMG88XX_WIDTH;
  // img_msg.encoding = sensor_msgs::image_encodings::MONO8;
  img_msg.encoding = "32FC1";
  img_msg.is_bigendian = 0;
  img_msg.step = 4 * img_msg.width;
  img_msg.data_length = img_msg.height * img_msg.step;
  img_msg.data = reinterpret_cast<uint8_t *>(pixels);

  blink(5, 500);

  nh.initNode();
  nh.advertise(pub_img);
}

void loop() {
  amg.readPixels(pixels);

  delay(100);
  pub_img.publish(&img_msg);
  
  nh.spinOnce();
}
