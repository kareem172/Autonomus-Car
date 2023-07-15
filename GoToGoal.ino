#include <math.h>                                        
#define PI 3.1415926535897932384626433832795            // PI number definition

int N = 20;                                             // number of encoder slots
int counterTicks = 1;                                   // number of ticks for speed calculation (remember that the lower the value, the greater the noise of the measurement)
int tam = 10;                                           // size of the average calculation vector (This value depends on the size of the average vectors vectorL and vectorR)
int k = 10;                                             // sampling time

volatile unsigned Actualsampling = 0;                   // variables for defining the sampling time
volatile unsigned PreviousSample = 0;
volatile unsigned deltasample = 0;

float error = 0;                                        // error variables
float Kp = 40;                                          // Proportional constant control
int PWMr = 0;                                           // Right wheel PWM (right tire control signal)
int PWMl = 0;                                           // Left wheel PWM (Left wheel control signal)

int PWMmax=80;                                          // PWM max 
int PWMmin=50;                                          // PWM min


///------------------------------- Variables Robot position ---------------------------------------------
float Distance = 0;                                     // distance traveled center point
float x = 0;                                            // X axis distance traveled
float y = 0;                                            // Y axis distance traveled
float phi = 0;                                          // position angular

///------------------------------- Variables of Desired position ---------------------------------------------
float Xd = 200;
float Yd = 200;
float Phid= atan2(Yd-y, Xd-x);

///------------------------------- Variables of robot  ---------------------------------------------

float diameter = 6.8;                                    // diameter of wheel in cm
float length_ = 17.3;                                    // robot length between wheels
float V = 0;                                             // Linear speed 
float W = 0;                                             // Angular velocity of the car

///------------------------------- Variables of Right MOTOR ---------------------------------------------

volatile unsigned acual_sampleInterrupcionR = 0;       // variables for defining the interruption time and calculating the speed of the right motor
volatile unsigned prev_sampleInterrupcionR = 0;
double delta_sampleInterrupcionR = 0;

int encoderR = 3;                                        // right encoder connection pin
int motorR = 10;                                         // right wheel connection pin (PWM pin)
double frequencyR = 0;                                   // tire interruption frequency R
double Wr = 0;                                           // Angular velocity R
double Vr = 0;                                           // linear speed
int CR = 0;                                              // tick counter
float vectorR[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};        // data storage vector for average outage time

float Rdistance = 0;                                     // distance traveled right wheel
int Rtick = 0;                                           // right encoder ticks
int RtickAnt = 0;                                        // previous right encoder ticks
int deltaRtick = 0;                                      // right encoder difference

//------------------------------  Variables of Left MOTOR ------------------------------------------------

volatile unsigned acual_sampleInterrupcionL = 0;         // variables for definition of the interruption time and calculation of the left motor speed
volatile unsigned prev_sampleInterrupcionL = 0;
double delta_sampleInterrupcionL = 0;

int encoderL = 2;                                        // Left encoder connection pin
int motorL = 11;                                         // Left wheel connection pin (PWM pin)

double frecuencyL = 0;                                   // Left tire interruption frequency
double Wl = 0;                                           // Angular velocity L
double Vl = 0;                                           // linear speed
int CL = 0;                                              // tick counter
float vectorL[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};        // data storage vector for average outage time

float Ldistance = 0;                                     // distance traveled left tire
int Ltick = 0;                                           // left encoder ticks
int LtickAnt = 0;                                        // previous left encoder ticks
int deltaLtick = 0;                                      // left encoder difference
 
void setup() {
  attachInterrupt(digitalPinToInterrupt(encoderR),REncoder,CHANGE);                // line to add a break to a PIN
  attachInterrupt(digitalPinToInterrupt(encoderL),LEncoder,CHANGE);                // line to add a break to a PIN
  Serial.begin(9600);                                                              // start of serial communication
}

void REncoder() {                                                                  // right wheel encoder interrupt function
      Rtick++;                                                                     // Number of ticks right wheel
    //  Serial.println("rght encoder inturrupt");
      CR++;                                                                        // tick counter increment
      if (CR == counterTicks){                                                     // if the tick counter reaches the tick value determined for the time calculation
          float mean = 0;                                                          // variable created for calculating the average  
//-------------------------------------- -----------------------------    average filter    -----------------------------------------------------------------------------//
          for(int i=0;i < tam-1;i++){                                                                    // filling of the vector for later calculation of the average
              vectorR[i]=vectorR[i+1];                                                                   
          }
          vectorR[tam-1]=delta_sampleInterrupcionR ;                                                     // last data of the vector (current measurement) 

          for(int i=0;i<tam;i++){                                                                        // Sum of vector values
              mean = vectorR[i]+ mean;
          }
          mean = mean/tam;                                                                             //divide by the total of the vector data
          delta_sampleInterrupcionR = mean;                                                            // is replaced by the value of its middle. 
//-------------------------------------- ----------------------------- ---------------------------------------------------------------------------------------------------//           
          frequencyR = (1000)/ delta_sampleInterrupcionR;                                              // interruption frequency     
          prev_sampleInterrupcionR = acual_sampleInterrupcionR;                                   // the previous interruption time is updated
          CR = 0;                                                                                        //tick counter reset
      } 
 } 

void LEncoder() {                                                                                       // left wheel encoder interruption function
      Ltick++;                                                                                           // Number of ticks left wheel
      CL++;                                                                                             // tick counter increment
      if (CL == counterTicks){                                                                         // if the tick counter reaches the tick value determined for the time calculation
          float media = 0;                                                                              // variable created for calculating the average
//-------------------------------------- -----------------------------    average filter    -----------------------------------------------------------------------------//
          for(int i=0;i < tam-1;i++){                                                                    // filling of the vector for subsequent calculation of the average
              vectorL[i]=vectorL[i+1];
          }
          vectorL[tam-1]=delta_sampleInterrupcionL;                                                     // last data of the vector (current measure) 

          for(int i=0;i<tam;i++){                                                                        // Sum of vector values
              media = vectorL[i]+ media;
          }
          media = media/tam;                                                                             //divide by the total of the vector data
          delta_sampleInterrupcionL = media;                                                            // is replaced by the value of its middle 
//-------------------------------------- ----------------------------- ---------------------------------------------------------------------------------------------------//      
          frecuencyL = (1000)/ delta_sampleInterrupcionL;                                              // interruption frequency 
          prev_sampleInterrupcionL = acual_sampleInterrupcionL;                                   // the previous interruption time is updated
          CL = 0;                                                                                        // tick counter reset
       } 
 } 


void loop() { 
    Actualsampling = millis();                                                                           //Current sampling time
    acual_sampleInterrupcionR = millis();                                                              // the execution time is assigned to the sampling actual
    acual_sampleInterrupcionL = millis();                                                              // the execution time is assigned to the sampling actual

    deltasample =(double) Actualsampling - PreviousSample;                                           // sampling delta 
    if ( deltasample >= k)                                                                             // sampling time is ensured
    {   
        float Phid= atan2(Yd-y, Xd-x);                                                                   // Recalculate the desired angle at each iteration, since it changes with each move
         
        delta_sampleInterrupcionR = acual_sampleInterrupcionR -  prev_sampleInterrupcionR;       // difference times of interruptions of ticks of the motor     
        delta_sampleInterrupcionL = acual_sampleInterrupcionL -  prev_sampleInterrupcionL;       // difference times of interruptions of ticks of the motor     

        if(delta_sampleInterrupcionR >= 200*counterTicks){                                              // This is the way to define when the motor is stationary. If deltaSamplingInterruptionR is greater than 40 milliseconds by tick prescaling
          frequencyR=0;                                                                                  // 40 mS is the maximum time a tick takes at the lowest motor speed
        }
        if(delta_sampleInterrupcionL >= 200*counterTicks){                                              // This is the way to define when the motor is stationary. If deltaSamplingInterruptionR is greater than 40 milliseconds by tick prescaling
          frecuencyL=0;                                                                                  // 40 mS is the maximum time a tick takes at the lowest motor speed
        }

        Wr = counterTicks*((2*PI)/N)*frequencyR;                                                        // angular frequency Rad/s
        Vr= Wr*(diameter/2);                                                                              // linear speed cm/s
        Wl = counterTicks*((2*PI)/N)*frecuencyL;                                                        // angular frequency Rad/s
        Vl= Wl*(diameter/2);                                                                              // linear speed cm/s    

//        V = (Vr+Vl)/2;                                                                                    // robot speed calculation
        V = 60;                                                                                           // constant speed to reach the angle
        error = Phid - phi;                                                                               // angular error Desired angle minus robot angle
        W = (Vr-Vl)/length_ + Kp * error;                                                                // Calculation of the angular velocity with the control variables
        PWMr = V + (W*length_)/2;                                                                        // Right wheel PWM control signal
        PWMl = V - (W*length_)/2;                                                                        // Left wheel PWM control signal

//-------------------------------------- conditionals for PWM signal limits ---------------------------------------------------------------------------------------------------//  

        if(PWMr > PWMmax){                                                                               
           PWMr = PWMmax;
        }
        if(PWMr < PWMmin){
          PWMr = PWMmin;
        }
        if(PWMl > PWMmax){
           PWMl = PWMmax;
        }
        if(PWMl < PWMmin){
           PWMl = PWMmin;
        } 

        if( abs(x-Xd) < 10 && abs(y-Yd) < 10){
          analogWrite(motorR,0);
          analogWrite(motorL,0); 
        }
        else {
          analogWrite(motorR,PWMr);
          analogWrite(motorL,PWMl); 
       }
        
                                                                               
        
        odometry();                                                                                      // odometry calculation                 
        
        Serial.print(x);                                                                                  // the time between TIC and TIC is shown
        Serial.print(",");                                                                                 // the time between TIC and TIC is shown
        Serial.println(y);                                                                                // the time between TIC and TIC is shown
       
        PreviousSample = Actualsampling;                                                                 // update of previous sampling
     }
}

void odometry(){ 

   deltaRtick = Rtick - RtickAnt;                                                                         // comparison of the ticks traveled since the last right wheel calculation               
   Rdistance = PI*diameter*(deltaRtick/(double) 20);                                                     // distance traveled by the right wheel since the last calculation

   deltaLtick = Ltick - LtickAnt;                                                                         // comparison of the ticks traveled since the last left rim calculation
   Ldistance = PI*diameter*(deltaLtick/(double) 20);                                                     // distance traveled by the left wheel since the last calculation   

   Distance = (Rdistance + Ldistance)/2;                                                               // distance traveled by the center point since the last calculation
   
   x = x + Distance*cos(phi);                                                                            // current X point position
   y = y + Distance*sin(phi);                                                                            // current Y point position
   
   phi = phi + ((Rdistance - Ldistance)/length_);                                                       // current angle position
   phi = atan2(sin(phi),cos(phi));                                                                         //angular position transformation between -PI and PI
   
   RtickAnt = Rtick;                                                                                       // update the variable RtickAnt with the values ​​of Rtick
   LtickAnt = Ltick;                                                                                       // update the variable LtickAnt with the values ​​of Ltick
 } 
