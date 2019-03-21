
// ROS
#include <ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

// Adafruit AMG88XX
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

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

namespace dyne {
  // I2C address
  constexpr uint8_t kLLegAddr = 0x31;
  constexpr uint8_t kRLegAddr = 0x32;

  // I2C command requires 3[bytes]
  // [CMD] [VALUE(upper byte)] [VALUE(lower byte)]
  constexpr uint8_t kVelCmd = 0x10;

  // 1.0[m/sec] ==> 80 (PWM value);
  constexpr float kVel2PWM = 80 / 1.0;

  void SendMotorPwmCommand(uint8_t addr, int16_t vel) {
    Wire.beginTransmission(addr);

    if (vel < -80) {
      vel = -80;
    } else if (vel > +80) {
      vel = +80;
    }

    uint8_t data[3] = {
      kVelCmd,
      (uint8_t)(vel >> 8),
      (uint8_t)(vel)
    };
    Wire.write(data, 3);

    Wire.endTransmission();
  }

  void Stop(void) {
    SendMotorPwmCommand(kLLegAddr, 0);
    SendMotorPwmCommand(kRLegAddr, 0);
  }

  /*
   * in rot [rad/s]
   * out PWM value
   */
  int16_t Linear2Motor(float vel) {
    return kVel2PWM * vel;
  }

  /*
   * in rot [rad/s]
   * out PWM value
   */
  int16_t Rotary2Motor(float rot) {
    // 0.5[m/sec] ==> 80 (PWM value);
    constexpr float kRadius = 0.1; // [m]
    return kVel2PWM * (kRadius * rot);
  }

  constexpr bool IsStop(float vel_x, float rot_z) {
    // FIXME: rot_z condition
    return (-0.01 < vel_x && vel_x < 0.01) &&
           (-0.1 < rot_z && rot_z < +0.1);
  }

  void TwistCallback(const geometry_msgs::Twist& msg) {
    // msg.linear.x/y/z [m/sec]
    // msg.angular.x/y/z [rad/sec]
    float vel_x = msg.linear.x;
    float rot_z = msg.angular.z;

//    if(IsStop(vel_x, rot_z)) {
//      Stop();
//    } else {
      int16_t pwm_lin = Linear2Motor(vel_x);
      int16_t pwm_rot = Rotary2Motor(rot_z);
      // right (0x32)
      //  back : [0xFFDF,0xFFB0] := [-21,-80]
      //  front: [0x0020,0x005A] := [+32,+90]
      //                         -dead    0     +dead
      //   saturate |     back     | stop | stop  |    front     | saturate
      //      -80   | val + offset |  0   |  0    | val + offset |   +80
      int16_t left  = pwm_lin - pwm_rot;
      int16_t right = pwm_lin + pwm_rot;
      if (left > 0) {
        if (left < 4) {
          left = 0;
        } else if (left < 30) {
          left += 30;
        }
      } else {
        if (left > -4) {
        } else if (left > -30) {
          left -= 30;
        }
      }
      if (right > 0) {
        if (right < 4) {
          right = 0;
        } else if (right < 30) {
          right += 30;
        }
      } else {
        if (right > -4) {
          right = 0;
        } else if (right > -35) {
          right -= 35;
        }
      }
      SendMotorPwmCommand(kLLegAddr, -left);
      SendMotorPwmCommand(kRLegAddr, -right);
//    }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub_twist("cmd_vel", dyne::TwistCallback );


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite( LED_BUILTIN, HIGH);

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

  dyne::Stop();

  blink(5, 500);

  nh.initNode();
  nh.advertise(pub_img);
  nh.subscribe(sub_twist);
//  dyne::SendMotorPwmCommand(0x31, -30);
//  dyne::SendMotorPwmCommand(0x32, -35);
}

void loop() {
  amg.readPixels(pixels);

  delay(100);
  pub_img.publish(&img_msg);
  
  nh.spinOnce();
}
