/*  OHMni-Stick firmware
    Copyright 2023 oneohm
    http://github.com/oneohm/OHMni-stick
*/

#include <Arduino.h>
#include <Mouse.h>
#include <HX711-multi.h>
#include <RunningMedian.h>

// Debug options
#define ENABLE_MOVES
#define ENABLE_CLICKS
#define ENABLE_PRINTS

// Pin mapping definitions 
#define ONBOARD_LED 13 // XIAO user LED

// Pins to the load cell ADCs
#define CLK 5      // clock pin to the load cell amp
#define DOUT1 4    // data pin to the first ADC
#define DOUT2 3    // data pin to the second ADC
#define DOUT3 2    // data pin to the third ADC

// Pins to the button pressure sensor ADCs
#define CLK_P 6
#define DOUT_P1 1
#define DOUT_P2 0
#define DOUT_P3 10

// Serial message on boot
#define BOOT_MESSAGE "OHMni-Stick"

#define TARE_TIMEOUT_SECONDS 4

// Number of elements in the median filters
#define MEDIAN_FILTER_SIZE 5

byte DOUTS[3] = {DOUT1, DOUT2, DOUT3};
byte DOUTS_P[3] = {DOUT_P1, DOUT_P2, DOUT_P3};

// Pressure sensing button threshold
int pressureThreshold = 1000000;

// Motion activation force in grams
const int TRIGGER_MASS = 25;
// Z-axis "click" force in grams
const int CLICK_FORCE = 80;
// "dead-band" in movement, pixels - this might be better as a force in grams...
float DEAD_BAND = 5;
// Max move size, pixels
float MAX_MOVE = 30;

// Load cell calibration - for converting raw ADC counts to mg
const int CALIBRATION_FACTOR_CELL_1 = 490;
const int CALIBRATION_FACTOR_CELL_2 = 590;
const int CALIBRATION_FACTOR_CELL_3 = 520;

// Conversion factor for translating force to pointer displacement
const float GRAMS_PER_PIXEL = 20;
// Raw force measurements in ADC counts
long int results[3];
// Force measurements in grams
float forceResults[3];
// Raw air pressure measurements from the pneumatic buttons
long int pressureResults[3];

float xyDisplacement[2];
int xyMove[2];

float lastForceMagnitude = 0;


// Timing configuration
unsigned long previousMillis = 0; // Stores the last time the loop was run
const unsigned long updateInterval = 15; // The maximum ADC speed is 80SPS, around 12.5ms
unsigned long restTimer = 0;
const unsigned long driftRestPeriod = 600; // milliseconds of rest to trigger drift compensation

// Drift compensation
int driftDetectionForce = 0;
const int driftDetectionChange = 2;

// Flags
// Flag to skip measuring the pressure-based buttons every other loop
bool skipPressureRead = false;
bool triggered = false;
bool scrollMode = false;

// Setup median filters to smooth force sensor output and disregard outlier samples
RunningMedian medianForce1 = RunningMedian(MEDIAN_FILTER_SIZE);
RunningMedian medianForce2 = RunningMedian(MEDIAN_FILTER_SIZE);
RunningMedian medianForce3 = RunningMedian(MEDIAN_FILTER_SIZE);

// Setup median filters to smooth pressure sensor output and disregard outlier samples
RunningMedian medianPressure1 = RunningMedian(MEDIAN_FILTER_SIZE);
RunningMedian medianPressure2 = RunningMedian(MEDIAN_FILTER_SIZE);
RunningMedian medianPressure3 = RunningMedian(MEDIAN_FILTER_SIZE);

// HX711 ADC interface setup
HX711MULTI forces(3, DOUTS, CLK);
// HX710 ADC interface setup, Using HX711 library... Setting gain to 64 actually sets conversion rate to 40sps
HX711MULTI buttons(3, DOUTS_P, CLK_P, 64); 

void tare() {
  bool tareSuccessful = false;

  unsigned long tareStartTime = millis();
  while (!tareSuccessful && millis()<(tareStartTime+TARE_TIMEOUT_SECONDS*1000)) {
    tareSuccessful = forces.tare(20,10000);  // Reject 'tare' if still ringing
  }
  tareSuccessful = false;
  tareStartTime = millis();
  while (!tareSuccessful && millis()<(tareStartTime+TARE_TIMEOUT_SECONDS*1000)) {
    tareSuccessful = buttons.tare(20,10000);  // Reject 'tare' if still ringing
  }
}

void sendRawData() { 
  // Read the load cell forces
  forces.read(results); // 80 samples per second

  // Filter out bad force measurements...
  // Add the new force measurements to the median filter
  medianForce1.add(results[0]);
  medianForce2.add(results[1]);
  medianForce3.add(results[2]);

  // Stick the median filtered values back into the results array...
  results[0] = medianForce1.getMedian();
  results[1] = medianForce2.getMedian();
  results[2] = medianForce3.getMedian();

  // Convert filtered raw force measurements to grams
  forceResults[0] = (float)results[0]/CALIBRATION_FACTOR_CELL_1;
  forceResults[1] = (float)results[1]/CALIBRATION_FACTOR_CELL_2;
  forceResults[2] = (float)results[2]/CALIBRATION_FACTOR_CELL_3;

  // Print out force debug data in grams
  #ifdef ENABLE_PRINTS
  Serial.print("Forces in grams: ");
  Serial.print(forceResults[0]);
  Serial.print(", ");
  Serial.print(forceResults[1]);
  Serial.print(", ");
  Serial.println(forceResults[2]); 
  #endif

  // Read the air pressure sensors every other pass since only 40 samples per second can be made
  if(skipPressureRead == true){
    skipPressureRead = false;
  }
  else{
    buttons.read(pressureResults); // 40 samples per second
    skipPressureRead = true;

    // Filter out bad pressure measurements...
    // Add the new force measurements to the median filter
    medianPressure1.add(pressureResults[0]);
    medianPressure2.add(pressureResults[1]);
    medianPressure3.add(pressureResults[2]);

    // Stick the median values back into the results array...
    pressureResults[0] = medianPressure1.getMedian();
    pressureResults[1] = medianPressure2.getMedian();
    pressureResults[2] = medianPressure3.getMedian();   

    // Print out pressure debug data
    #ifdef ENABLE_PRINTS
    Serial.println("Pressures:");
    Serial.println(pressureResults[0]);
    Serial.println(pressureResults[1]);
    Serial.println(pressureResults[2]);
    #endif


    // Check for pressure button presses
    // Handle pressure button #1 press
    if(pressureResults[0] > pressureThreshold && !Mouse.isPressed(MOUSE_LEFT)){
      // Capacitive button #1 pressed
      #ifdef ENABLE_CLICKS
      Mouse.press(MOUSE_LEFT);
      #endif
      #ifdef ENABLE_PRINTS
      Serial.println("Button 1 (Left) Pressed");      
      Serial.println(pressureResults[0]);
      #endif
    }
    // Handle pressure button #1 release
    else if(pressureResults[0] <= pressureThreshold && Mouse.isPressed(MOUSE_LEFT)){
      // Capacitive button #1 is no longer pressed
      #ifdef ENABLE_CLICKS
      Mouse.release(MOUSE_LEFT); 
      #endif   
      #ifdef ENABLE_PRINTS
      Serial.println("Button 1 (Left) Released");      
      Serial.println(pressureResults[0]);
      #endif
    }

    // Handle pressure button #2 press
    if(pressureResults[1] > pressureThreshold && !Mouse.isPressed(MOUSE_RIGHT)){
      // Capacitive button #2 pressed
      #ifdef ENABLE_CLICKS
      Mouse.press(MOUSE_RIGHT);
      #endif
      #ifdef ENABLE_PRINTS
      Serial.println("Button 2 (Right) Pressed");      
      Serial.println(pressureResults[1]);
      #endif
    }
    // Handle pressure button #2 release
    else if(pressureResults[1] <= pressureThreshold && Mouse.isPressed(MOUSE_RIGHT)){
      // Capacitive button #2 is no longer pressed
      #ifdef ENABLE_CLICKS
      Mouse.release(MOUSE_RIGHT);   
      #endif 
      #ifdef ENABLE_PRINTS
      Serial.println("Button 2 (Right) Released");      
      Serial.println(pressureResults[1]);
      #endif
    }

    // Handle pressure button #3 press - Used as scroll lock
    if(pressureResults[2] > pressureThreshold && !scrollMode){
      // Capacitive button #3 pressed
      // Used to enable scroll mode instead of a third mouse button
      scrollMode = true;
      //#ifdef ENABLE_CLICKS
      //Mouse.press(MOUSE_MIDDLE);
      //#endif 
      #ifdef ENABLE_PRINTS
      Serial.print("Button 3 (Middle) Pressed");      
      Serial.println(pressureResults[2]);
      #endif
    }
    // Handle pressure button #3 release - Used as scroll lock
    else if(pressureResults[2] <= pressureThreshold && scrollMode){
      // Capacitive button #3 is no longer pressed
      // Used to enable scroll mode instead of a third mouse button
      scrollMode = false;
      //#ifdef ENABLE_CLICKS
      //Mouse.release(MOUSE_MIDDLE); 
      //#endif  
      #ifdef ENABLE_PRINTS
      Serial.println("Button 3 (Middle) Released");      
      Serial.println(pressureResults[2]);
      #endif  
    } 
  }

  // Calculate the total force applied, sum the absolute value of the forces in grams
  float measured_mass = (abs(forceResults[0])+abs(forceResults[1])+abs(forceResults[2]));

  // Force Sensor Drift Compensation
  // Watch for a period of rest with minimal change in overall force 
  if(abs(driftDetectionForce-measured_mass) <= driftDetectionChange){
    restTimer += updateInterval;
    if(restTimer >= driftRestPeriod){
      // No change in force for a while, reset the baseline
      Serial.println("compensating drift...");
      // Re-tare the force sensors with ten samples
      if(forces.tare(10,10000)){
        restTimer = 0; // Reset the rest counter if the tare was successful
      }
    }
  }
  else{
    // Too much change this update, so reset drift detection variables
    driftDetectionForce = measured_mass;
    restTimer = 0;
  }

  #ifdef ENABLE_PRINTS
  Serial.print("Triggered, mass: ");      
  Serial.println(measured_mass);
  Serial.println(forceResults[0]);
  Serial.println(forceResults[1]);
  Serial.println(forceResults[2]);
  #endif

  // Compute XY displacement based on the three vectors spaced 120 degrees
  // Zero out the displacement
  xyDisplacement[0] = 0;
  xyDisplacement[1] = 0;

  // Vector #1: At 0 degrees
  // X component:
  xyDisplacement[1] += forceResults[0];

  // Vector #2: At 120 degrees
  // X component: Vector * Sin(120)
  xyDisplacement[0] -= forceResults[1] * 0.58061118421;
  // Y component: Vector * Cos(120)
  xyDisplacement[1] -= forceResults[1] * 0.81418097052;

  // Vector #3: At 240 degrees
  // X component: Vector * Sin(240)
  xyDisplacement[0] += forceResults[2] * 0.94544515492;
  // Y component: Vector * Cos(240)
  xyDisplacement[1] -= forceResults[2] * 0.32578130553;

  // Scale the X/Y vectors - grams per pixel
  xyDisplacement[0] = xyDisplacement[0]/GRAMS_PER_PIXEL;
  xyDisplacement[1] = xyDisplacement[1]/GRAMS_PER_PIXEL;

  // Find the magnitude of the move
  float moveSize = sqrt((xyDisplacement[0]*xyDisplacement[0])+(xyDisplacement[1]*xyDisplacement[1]));

  // Create a small "dead-band" in movement
  if(moveSize > DEAD_BAND){
    float deadbandScaleFactor = (moveSize - DEAD_BAND) / moveSize;
    xyDisplacement[0] = (xyDisplacement[0] * deadbandScaleFactor);
    xyDisplacement[1] = (xyDisplacement[1] * deadbandScaleFactor);
  }
  else{
    // Move is within the "dead-band", reduce to 0,0
    xyDisplacement[0] = 0;
    xyDisplacement[1] = 0;
  }  

  // Put a cap on max move size
  if(moveSize > MAX_MOVE){
    float maxMoveScaleFactor = MAX_MOVE / moveSize;
    xyDisplacement[0] = (xyDisplacement[0] * maxMoveScaleFactor);
    xyDisplacement[1] = (xyDisplacement[1] * maxMoveScaleFactor);
  }

  // Inertial Modification - Improves precieved responsiveness
  float forceMagnitude = sqrt((xyDisplacement[0]*xyDisplacement[0])+(xyDisplacement[1]*xyDisplacement[1]));
  const float inertiaFactor = 0.15;
  float intertialModifier = inertiaFactor*(forceMagnitude-lastForceMagnitude);
  xyDisplacement[0] = xyDisplacement[0] + (intertialModifier * xyDisplacement[0]);    
  xyDisplacement[1] = xyDisplacement[1] + (intertialModifier * xyDisplacement[1]);

  // More inertial modificaton to improve slow movement
  // Check to make sure the forceMagnitude is not 0 to avoid the dreaded division by 0
  if(forceMagnitude > 0){
    xyDisplacement[0] = xyDisplacement[0] * (1/abs(1+inertiaFactor*(1-lastForceMagnitude/forceMagnitude)));
    xyDisplacement[1] = xyDisplacement[1] * (1/abs(1+inertiaFactor*(1-lastForceMagnitude/forceMagnitude)));
  }
  lastForceMagnitude = forceMagnitude;

  #ifdef ENABLE_PRINTS
  Serial.print("X,Y: ");
  Serial.print(xyDisplacement[0]);
  Serial.print(" , ");
  Serial.println(xyDisplacement[1]);
  #endif

  // Send the mouse position update, if not 0,0
  if(scrollMode == false && (xyDisplacement[0] != 0 || xyDisplacement[1] != 0)){
    #ifdef ENABLE_MOVES
    Mouse.move((int)xyDisplacement[0], (int)xyDisplacement[1], 0);
    #endif
  }
  else if(scrollMode == true && (xyDisplacement[0] != 0 || xyDisplacement[1] != 0)){
    //Mouse.move(xyDisplacement[0], xyDisplacement[1], 0);
    if(xyDisplacement[1] > 0){
      #ifdef ENABLE_MOVES
      Mouse.move(0, 0, 1);
      #endif
      #ifdef ENABLE_PRINTS
      Serial.println("Scroll down!");
      #endif
    }
    else if(xyDisplacement[1] < 0){
      #ifdef ENABLE_MOVES
      Mouse.move(0, 0, -1);
      #endif
      #ifdef ENABLE_PRINTS
      Serial.println("Scroll up!");
      #endif
    }
    
    // Small delay to slow down scrolling
    delay(100);
  }
}

void setup() {
  //#ifdef ENABLE_PRINTS
  Serial.begin(115200);
  Serial.println(BOOT_MESSAGE);
  Serial.flush();
  //#endif

  pinMode(ONBOARD_LED, OUTPUT); 
  digitalWrite(ONBOARD_LED, LOW);
  delay(1000);
  tare();

  Mouse.begin();
}

void loop() {
  // Update the sensor data at a defined rate, rather than just free-run the loop
  unsigned long currentMillis = millis(); 
  if(currentMillis - previousMillis > updateInterval){
//    Serial.println(previousMillis);
//    Serial.println(currentMillis);
    previousMillis = currentMillis;  
    sendRawData();
  }
}