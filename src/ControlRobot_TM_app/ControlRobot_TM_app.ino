/*
 * This node will subscribe to the /cmd_vel topic (Twist).
 * It will publish to /wr and /wl (Float32).
 * /wr and /wl will be computed following the differential drive mobile robot model 
 */
#include <ros.h> //comment
#include <geometry_msgs/Twist.h> //Type of message for the cmd_vel topic
#include <std_msgs/Float32.h>  //Type of message for the /wl and /wr topics 
 #include <std_msgs/Int32.h> //Debug topics of the pwm_left and pwm_right

#define WHEEL_PUB_PERIOD 100 //Period in [ms] in which we will publish wl and wr
#define WHEEL_DIST 0.5 //Distance between wheels [m] 
#define WHEEL_RAD 0.06 //Wheel radius [m]

//Arduino pin definitions
#define MOT1_IN1 8//IN1 of the L298 should be connected to this arduino pin
#define MOT1_IN2 9//IN2 of the L298 should be connected to this arduino pin
#define MOT1_ENA 10 //enable

#define MOT2_IN3 11//IN1 of the L298 should be connected to this arduino pin
#define MOT2_IN4 12//IN2 of the L298 should be connected to this arduino pin
#define MOT2_ENB 7 //enable

#define LED 13 //A led that blinks when receiving

#define adelante_b 6
#define atras_b 2
#define izq_b 3
#define der_b 4
#define manual_s 5

#define PIN_ROJO 44
#define PIN_AZUL 45

int pinaltavoz = 24;
float m = 1.059;         // constante para multiplicar frecuencias

unsigned long time_now = 0;  
volatile float v, w; //Robot linear and angular speeds [m/s] and [rad/s] 
volatile unsigned int detected;


ros::NodeHandle nh; //This structure is necessary to work with ROS (init the node, create publishers, etc)

// Definimos la función song()
void song(int buzzerPin)
{
  tone(buzzerPin, 1661);
  delay(76);
  noTone(buzzerPin);

  tone(buzzerPin, 2637);
  delay(65);
  noTone(buzzerPin);

  tone(buzzerPin, 523);
  delay(22);
  noTone(buzzerPin);

  tone(buzzerPin, 554);
  delay(43);
  noTone(buzzerPin);

  tone(buzzerPin, 2794);
  delay(65);
  noTone(buzzerPin);

  tone(buzzerPin, 1568);
  delay(33);
  noTone(buzzerPin);

  tone(buzzerPin, 523);
  delay(33);
  noTone(buzzerPin);

  tone(buzzerPin, 587);
  delay(22);
  noTone(buzzerPin);

  tone(buzzerPin, 1760);
  delay(65);
  noTone(buzzerPin);

  tone(buzzerPin, 2960);
  delay(76);
  noTone(buzzerPin);

  tone(buzzerPin, 523);
  delay(54);
  noTone(buzzerPin);

  tone(buzzerPin, 587);
  delay(43);
  noTone(buzzerPin);

  tone(buzzerPin, 1661);
  delay(43);
  noTone(buzzerPin);

  tone(buzzerPin, 622);
  delay(87);
  noTone(buzzerPin);
}
/* 
 * Stop the motor
 */
void motor_stop() {
        //Stop if received an wrong direction
                digitalWrite(MOT1_IN1, 0);
                digitalWrite(MOT1_IN2, 0);
                analogWrite(MOT1_ENA, 0);
                
                digitalWrite(MOT2_IN3, 0);
                digitalWrite(MOT2_IN4, 0);
                analogWrite(MOT2_ENB, 0);
}

/* Moves the motor in positive sens
        vel: [0-255] the desired pwm value 255 means full speed
 */
void motor_forward_left(unsigned int vel) {
        digitalWrite(MOT1_IN1, 1);
        digitalWrite(MOT1_IN2, 0);
        analogWrite(MOT1_ENA, vel);
}

void motor_forward_right(unsigned int vel) {
        digitalWrite(MOT2_IN3, 1);
        digitalWrite(MOT2_IN4, 0);
        analogWrite(MOT2_ENB, vel);
}

/* Moves the motor backwards
        vel: [0-255] the desired pwm value 255 means full speed
 */
void motor_backwards_left(unsigned int vel) {
    digitalWrite(MOT1_IN1, 0);
    digitalWrite(MOT1_IN2, 1);
    analogWrite(MOT1_ENA, vel);
}

void motor_backwards_right(unsigned int vel) {
    digitalWrite(MOT2_IN3, 0);
    digitalWrite(MOT2_IN4, 1);
    analogWrite(MOT2_ENB, vel);
}


/***Equations for a differential drive mobile robot.
*      returns a float wl in [rad/s]
***/
float get_wl(float v, float w) {
  float wl;
  wl = (2.0 * v - WHEEL_DIST * w) / (2.0 * WHEEL_RAD); //Left wheel angular speed [rad/s]
  return wl;
}

/*** Equations for a differential drive mobile robot.
*        returns a float wr in [rad/s]
***/
float get_wr(float v, float w) {
  float wr;
  wr = (2.0 * v + WHEEL_DIST * w) / (2.0 * WHEEL_RAD); //Right wheel angular speed [rad/s]
  return wr;
}


void cmd_vel_cb( const geometry_msgs::Twist& vel_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  v=vel_msg.linear.x; 
  w=vel_msg.angular.z; 
}

void detect_cb(const std_msgs::Int32 &detect_msg){
  detected = detect_msg.data;
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_cb );
ros::Subscriber<std_msgs::Int32> detect_sub("detect", &detect_cb );
std_msgs::Float32 wl_msg, wr_msg;
std_msgs::Int32 pwml_msg, pwmr_msg, detected_msg;

ros::Publisher wl_pub("wl", &wl_msg);
ros::Publisher wr_pub("wr", &wr_msg);

ros::Publisher pwml_pub("pwm_left", &pwml_msg);
ros::Publisher pwmr_pub("pwm_right", &pwmr_msg);

void setup()
{
  pinMode(13, OUTPUT);

  
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(detect_sub);
  nh.advertise(wl_pub); //Init the publisher to the /wl topic
  nh.advertise(wr_pub); //Init the publisher to the /wr topic

  nh.advertise(pwml_pub); //Init the publisher to the /wl topic
  nh.advertise(pwmr_pub); //Init the publisher to the /wr topic

  pinMode(adelante_b, INPUT);
  pinMode(atras_b, INPUT);
  pinMode(izq_b, INPUT);
  pinMode(der_b, INPUT);
  pinMode(manual_s, INPUT);

  pinMode(PIN_ROJO, OUTPUT);
  pinMode(PIN_AZUL, OUTPUT);

}



void loop()
{
    float wr, wl; //Wheel speeds wr-> right wheel, wl->left wheel
    wr= get_wr(v,w); //Modify this
    wl= get_wl(v,w); //Modify this
    
    unsigned int pwm_left = map(abs(wl),0,13.89,0,255);
    unsigned int pwm_right = map(abs(wr),0,13.81,0,255);
    int potValue = analogRead(A0); // Read potentiometer value

    int pwmOutput = map(potValue, 0, 1023, 0 , 255); // Map the potentiometer value from 0 to 255
    
       
    if (digitalRead(manual_s) == true){
        
        if (digitalRead(adelante_b) == true) {
            motor_forward_right(pwmOutput);
            motor_forward_left(pwmOutput);
            delay(20);
        }
 
        if (digitalRead(atras_b) == true) {
          motor_backwards_right(pwmOutput);
          motor_backwards_left(pwmOutput);
          delay(20);
        }
      
        if (digitalRead(izq_b) == true) {
           motor_forward_right(pwmOutput);
           motor_backwards_left(pwmOutput);
          delay(20);
        }
      
        if (digitalRead(der_b) == true) {
            motor_forward_left(pwmOutput);
            motor_backwards_right(pwmOutput);
            delay(20);
         }
      
         if (digitalRead(adelante_b) == false && digitalRead(atras_b) == false && digitalRead(izq_b) == false && digitalRead(der_b) == false) {
            motor_stop();
           delay(20);
         }

         
        
     }else{
 
    
        if((unsigned long)(millis() - time_now) > WHEEL_PUB_PERIOD){
          // Enter here every WHEEL_PUB_PERIOD ms to publish data 
          // this is just for debugging 
          time_now = millis(); //reset the time
          wl_msg.data = wl; //Add some value to the wl message
          wr_msg.data = wr; //Add some value to the wl message
    
          pwml_msg.data = pwm_left;
          pwmr_msg.data = pwm_right;


          if(wl>=0 && pwm_left >= 0 && pwm_left <= 255){
            motor_forward_left(pwm_left);
          }else if (wl < 0 && pwm_left >= 0 && pwm_left <= 255){
            motor_backwards_left(pwm_left);
          }else{
            //Stop if received a wrong value or 0
            nh.logerror("Invalid data received; make sure to send a value between -255 and 255");
            nh.logerror("The motor will stop");
            motor_stop();
          }
    
        
          if(wr>=0 && pwm_right >= 0 && pwm_right <= 255){
            motor_forward_right(pwm_right);
          }else if (wr < 0 && pwm_right >= 0 && pwm_right <= 255){
            motor_backwards_right(pwm_right);
          }else{
            //Stop if received a wrong value or 0
            nh.logerror("Invalid data received; make sure to send a value between -255 and 255");
            nh.logerror("The motor will stop");
            motor_stop();
          }
        
        
        
          // Add your code here for example to publish data
          wl_pub.publish( &wl_msg ); //Publish the message to the /wl topic
          wr_pub.publish( &wr_msg ); //Publish the message to the /wr topic
    
          pwml_pub.publish( &pwml_msg ); //Publish the message to the /wl topic
          pwmr_pub.publish( &pwmr_msg ); //Publish the message to the /wr topic
      }
    
    }

    if(detected){
        // Encender las luces rojas
        analogWrite(PIN_ROJO, 255); // Máxima intensidad de rojo
        analogWrite(PIN_AZUL, 0);   // Sin luz azul
        delay(500); // Esperar medio segundo

        // Apagar las luces rojas
        analogWrite(PIN_ROJO, 0); // Sin luz roja
        analogWrite(PIN_AZUL, 255); // Máxima intensidad de azul
        delay(500); // Esperar medio segundo
        // Llamamos a la función song() para reproducir la secuencia de notas
        song(pinaltavoz);
    
        // Ajusta el tiempo de espera según sea necesario para tu aplicación
        //delay(2000); // Espera 2 segundos entre cada reproducción de la canción

    }else{
      analogWrite(PIN_ROJO, 0); // Sin luz roja
      analogWrite(PIN_AZUL, 0);   // Sin luz azul
    }
 
  

  nh.spinOnce(); //Very, very, very important to execute at least once every cycle.

 
}
