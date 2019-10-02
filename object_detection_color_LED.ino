
+++++++++++++++++++++++
// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar.h>

// You need to create an driver instance 
RPLidar lidar;

// Change the pin mapping based on your needs.
/////////////////////////////////////////////////////////////////////////////
#define LED_ENABLE  12 // The GPIO pin for the RGB led's common lead. 
                       // assumes a common positive type LED is used
#define LED_R       9  // The PWM pin for drive the Red LED
#define LED_G       11 // The PWM pin for drive the Green LED
#define LED_B       10 // The PWM pin for drive the Blue LED
    float distance ;
    float angle ;
    bool  startBit
    byte  quality;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal
#define USER_DEFINED_PARAMETER 50  
//////////////////////////////////////////////////////////////////////////////
                        
                        
 
#define NEO_RGBSPACE_MAX (byte)(200L*255/360)
int _r, _g, _b;

//Set current RGB with the hue: HSV(hue, x, x)
void hue_to_rgb( _u8 hue)
{
/*`
    Hue(in Degree):  0 (RED) ----> 60 (Yello) ----> 120 (Green) --...... ----> 360
    Hue'(fit to _u8):0       ----> 60/360*255 ----> 120/260*255 --...... ----> 255
*/
    
    //convert HSV(hue,1,1) color space to RGB space
    if (hue < 120L*255/360)   //R->G
    {
        _g = hue;
        _r = NEO_RGBSPACE_MAX - hue;
        _b = 0;
    }else if (hue < 240L*255/360)  //G->B
    {
        hue -= 120L*255/360;
        _b = hue;
        _g = NEO_RGBSPACE_MAX - hue;
        _r = 0;
    }else //B->R
    {
        hue -= 240L*255/360;
        _r = hue;
        _b = NEO_RGBSPACE_MAX - _r;
        _g = 0;
    }
}

void displayColor(float angle, float distance)
{
    //1. map 0-350 deg to 0-255
    byte hue = angle*255/360;
    hue_to_rgb(hue);
    
    //2. control the light 
    
    int lightFactor = (distance>500.0)?0:(255-distance*255/500);
    _r *=lightFactor;
    _g *=lightFactor;
    _b *=lightFactor;
    
    _r /= 255;
    _g /= 255;
    _b /= 255;    
    
    analogWrite(LED_R, 255-_r);
    analogWrite(LED_G, 255-_g);
    analogWrite(LED_B, 255-_b);   
}
void setColor(int red, int green, int blue)
 {  
   Serial.print("In setColor...");
   red = 255 - red;  
   green = 255 - green;  
   blue = 255 - blue; 

   analogWrite(LED_R, red);  
   analogWrite(LED_G, green);  
   analogWrite(LED_B, blue);
 }

void setup() {
  // bind the RPLIDAR driver to the arduino hardware serial
  lidar.begin(Serial);
  
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  
  pinMode(LED_ENABLE, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  
  //digitalWrite(LED_ENABLE, HIGH);
  //digitalWrite(LED_ENABLE, LOW);
  //analogWrite(LED_R,255);
  //analogWrite(LED_G,255);
  //analogWrite(LED_B,255);
  analogWrite(LED_B,0);
  analogWrite(LED_B,0);
  analogWrite(LED_B,0);
}

float minDistance = 100000;
float angleAtMinDist = 0;

void loop() {
  //delay(1000);
  if (IS_OK(lidar.waitPoint())) {
    //perform data processing here... 
    
     distance = lidar.getCurrentPoint().distance;
     angle = lidar.getCurrentPoint().angle;
      startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
      quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
    
    Serial.print("distance->" );
    Serial.println(distance);
    Serial.print("angle->");
    Serial.println(angle);
    //Serial.println("startBit");
    //Serial.println(startBit);
    //Serial.println("quality");
    //Serial.println(quality);
    
    if (lidar.getCurrentPoint().startBit) {
      // a new scan, display the previous data...
      //digitalWrite(LED_ENABLE, LOW);

      
      
     /* if(angle >= 200)
       {
          //Serial.println("Angle between -30 AND 30");
          //digitalWrite(LED_ENABLE, LOW);
          setColor(0, 255, 255);
       }

       if( (angle >= 0) && (angle <= 199))
       {
          //Serial.println("ANGLE between 90 and 180");
          digitalWrite(LED_ENABLE, HIGH);          
          setColor(255, 0, 255);
        }
       */
       //displayColor(angleAtMinDist, minDistance);
       minDistance = 100000;
       angleAtMinDist = 0;
    } else {
       if ( distance > 0 &&  distance < minDistance) {
          minDistance = distance;
          angleAtMinDist = angle;
       }
    }
  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    Serial.println("LOOSE CONNECTION WITH LIDAR");
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       //detected...
       Serial.println("LIDAR DETECTED..");
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
  if (distance >2000 )
      {
          rgbEnable();         
          setColor(0, 255, 255);
        
      }
      else
      {
        rgbDisable();
         setColor(255, 255, 255);
        
      }
}

void rgbEnable()
{
  digitalWrite(LED_ENABLE, HIGH);
  delay(500);
}
void rgbDisable()
{
   digitalWrite(LED_ENABLE, LOW);
   delay(500);
}
