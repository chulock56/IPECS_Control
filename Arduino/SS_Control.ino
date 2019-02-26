/* 
 * IPECS: Inverted Pendulum to Educate Controls Students
 *  State-Feedback with LQR-Based Observer Control Program
 * 
 * Drexel University MEM SD Team 27:
 *  Chulock A., Givens G., Pruitt W., Insalaco D., Maher C.
 *  
 * Code written 01/25/2019 by Andrew Chulock & Dom Insalaco
 * 
 * This code reads the angles of the rotary arm (theta) and pendulum (alpha), 
 *  applies full state-feedback control using an estimator, 
 *  and applies the control input (voltage) using an L298N motor driver.
 * 
 * This C-Code is to be paired with the MATLAB design code, 
 *  which provides the correct control gains and model parameters.
 * 
 * Useful Info:
 *  Arduino MEGA 2560 digital interrupt pins: 2, 3, 18, 19, 20, 21
 *  
 *  x = [theta, alpha, dtheta, dalpha]^T
 *  u = [Vm]
 *  y = [theta, alpha]^T
 * 
 */

#include <L298N.h>
#include <Encoder.h>
#include <StateSpaceControl.h>

// DEFINE OBJECTS
// Define encoder objects
  // **Ensure these pin numbers are correct!**
Encoder encTheta(2, 3); // rotary arm encoder
Encoder encAlpha(20, 21); // pendulum encoder

// Define motor driver object
  // **Ensure these pin numbers are correct!**
const int ENB = 8;
const int IN3 = 9;
const int IN4 = 7;
L298N motor(ENB, IN3, IN4);

// Define inverted pendulum model: 4 states, 1 input, 2 outputs
  // **Should be copy/pasted from MATLAB code**
InvPendModel model(12.467, -1.689, -0.738, 35.440, -0.778, -2.097, 2.959, 1.363);
 
// Define SS controller: 4 states, 1 input, 2 outputs, state est: true
StateSpaceController<4,1,2,true> controller(model);

// INITIALIZE VARIABLES
// Encoder variables
long countTheta = -999;
long countAlpha = -999;
long newCountTheta;
long newCountAlpha;
float radTheta;
float radAlpha;
const int countPerRev = 3600;
const float radPerCount = 0.00175;
  // 0.00175 rad/count for 900 CPR (3600 count/rev) encoder
long modCountAlpha;
const int offsetCount = -1800; 
  // offset count for modifying pendulum encoder reading

// Motor variables
float uAllow;
const float maxVolt = 10.10; 
  // **Set this to the max motor voltage available**
const float allowVolt = 10.00;
  // **Set this to a safe maximum operating voltage**
int uMapped;
int motorDuty;

// Theta setpoint variables
float thetaSet = 0.0; // radians
unsigned long previousTime = 0;
long interval = 5000; // interval between theta step (ms)

// Controller variables
  // **K and L matrices should be copy/pasted from MATLAB code**
Matrix<1,4> K = {-11.180, 198.833, -8.351, 30.124};

Matrix<4,2> L = {81.299, -1.702,
-1.813, 80.915,
1584.996, -84.351,
-102.423, 1589.121};
                 
Matrix<1,4> stopK = {0.0, 0.0, 0.0, 0.0};
Matrix<2> y;
const float limTheta = 2.3562;
  // theta limit: 135 deg (2.3562 rad) 
const float limAlpha = 0.1745;
  // alpha limit: 10 deg (0.1745 rad)
float uNew;
float u = -999.0;
const float dt = 0.01;

// SETUP
void setup() {
  // Initiate serial monitor
  Serial.begin(115200);

  // Quick delay to allow system to boot up
  delay(100);

  // Set initial motor speed to zero
  motor.setSpeed(0);
  
  controller.K = K; // Initialize controller law matrix
  controller.L = L; // Initialize estimator gain matrix
  controller.initialise(); // Initialise controller, pre-calculate Nbar

  // Initialize reference vector (i.e., setpoint)
  controller.r << thetaSet, 0.0, 0.0, 0.0;
}

// CONTROL LOOP
void loop() {
  /* // Create square wave for theta setpoint
  unsigned long currentTime = millis(); // time since Arduino started
  // compare current time to previous time theta switched, 
    // if greater than set interval, run the if statement:
  if (currentTime - previousTime > interval) 
  {
    previousTime = currentTime; // save current time as last time theta switched
    thetaSet = -thetaSet; // switch theta
    controller.r << thetaSet, 0.0, 0.0, 0.0; // update reference vector
  }
`` */
  // Read encoder measurements, convert measurements to angles
  newCountTheta = encTheta.read();
  newCountAlpha = encAlpha.read();
  if (newCountTheta != countTheta || newCountAlpha != countAlpha) 
  {
    countTheta = -newCountTheta; // negative defines CCW+
    countAlpha = -newCountAlpha;
    
    // Convert theta counts to radians (assuming rotary arm starts 
        // orthogonal to front edge of box)
    radTheta = radPerCount * countTheta;
    
    // Convert alpha counts to radians
    // Assuming pendulum starts from bottom vertical, calculate modulo 
      // to represent position from top vertical
    modCountAlpha = countAlpha - countPerRev*floor(countAlpha/countPerRev) + offsetCount; 
    radAlpha = radPerCount * modCountAlpha; // 0 = top vert., pi = bot. vert.
  }

  // Update output vector w/ current encoder measurements
  y << radTheta,
       radAlpha;
       
  // Check if alpha or theta are greater than model limits. 
    // If so, stop controller completely
  if ( abs(y(0)) >= limTheta || abs(y(1)) >= limAlpha ) 
  {
    controller.K = stopK;
    motor.stop(); 
  }
  else controller.K = K;

  // Update controller, which also updates input (u vector)
  controller.update(y, dt);
  uNew = controller.u(0);

  // Send input to motor driver
    // check if input u changed:
  if (uNew != u) 
  {
    u = uNew; // assign current u to the new u
    uAllow = constrain(u, -allowVolt, allowVolt); // limit u to max output voltage (~9.1V)
    // motor set speed must be a positive int 0-255, so map u to 
      // this range and round to nearest positive integer value:
    uMapped = (int)round(uAllow * 255.00 / maxVolt);
    
    // check if motorDuty is pos, neg, or zero and set motor rotation direction
      // **Ensure these directions correspond w/ the model!**
    if      (uMapped > 0) motor.forward();
    else if (uMapped < 0) motor.backward();
    else                  motor.stop();
    
    // set motor speed
    motorDuty = abs(uMapped);
    motor.setSpeed(motorDuty);
  }
  
  // Print the current system output to serial
  // Serial << "theta = " << y(0) << " alpha = " << y(1) << " voltage = " << u << '\n';
  Serial.print("theta = ");
  Serial.print(y(0));
  Serial.print("\t");
  Serial.print("alpha = ");
  Serial.print(y(1));
  Serial.print("\t");
  Serial.print("voltage = ");
  Serial.println(uAllow);
  
  delay(dt*1000);
}
