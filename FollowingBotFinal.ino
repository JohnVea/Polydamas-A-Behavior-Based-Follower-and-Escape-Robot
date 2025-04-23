#define MIC_PIN2 35  // Back Microphone input pin
#define MIC_PIN1 34  // Front Microphone input pin
#define LED_PIN LED_BUILTIN  // Built-in LED for alert
#define SAMPLE_WINDOW 500  // 500ms window for sound analysis
#define SCREAM_THRESHOLD 2600//2100//1900 // Threshold for scream detection
#define SCREAM_DURATION 300  // Time (ms) the signal must be loud
#define IN1 14  // Left Motor for forward
#define IN2 27  // Left motor for backwards
#define IN3 26  // Right Motor for forward
#define IN4 25  // Right Motor for backwards
#define TURN_SPEED 255  // PWM Speed 0-255
#define TURN_RATE 380//238  // Estimated turning speed (degrees/sec)
#define TRIG_LEFT 2 // Left ultrasonic
#define ECHO_LEFT 4
#define TRIG_CENTER 5 // Center ultrasonic
#define ECHO_CENTER 18
#define TRIG_RIGHT 22 // Right ultrasonic
#define ECHO_RIGHT 23
#include <math.h>


void setup() {
  Serial.begin(9600);
 
  delay(1000);
  pinMode(LED_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_CENTER, OUTPUT);
  pinMode(ECHO_CENTER, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
}

void stopAllMotors(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW); 
}

// Rotate Left Function 
void rotateRobotLeft(float angle_degrees){
  float turn_time = angle_degrees / TURN_RATE * 1000;  // Convert to milliseconds
  Serial.print("Rotating Left: ");
  Serial.print(angle_degrees);
  Serial.println(" degrees");

  unsigned long start_time = millis();

  

  while ((millis() - start_time) < turn_time) {
    
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(10);
  }

  stopAllMotors();
}


// Rotate Right Function 
void rotateRobotRight(float angle_degrees) {
  float turn_time = angle_degrees / TURN_RATE * 1000;  // Convert to milliseconds
  Serial.print("Rotating Right: ");
  Serial.print(angle_degrees);
  Serial.println(" degrees");

  unsigned long start_time = millis();

  while ((millis() - start_time) < turn_time) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);  // Left motor moves fowards
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);   // Right motor moves backward
    delay(10);
  }
  stopAllMotors();
}


// Function to read the microphone 
int readMicrophone(int pin) {
  int total = 0;
  const int numSamples = 10;  // Takes 5 readings and average them
  for (int i = 0; i < numSamples; i++) {
    total += analogRead(pin);
    delayMicroseconds(10);
  }
  return total / numSamples+0.5;
}


// Function to get distance from the ultrasonic sensor
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
    
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
    
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.0343 / 2;  // Converts to cm

  return (distance > 400) ? 400 : distance;  // Cap max distance at 400cm
}


void moveBackwards(uint32_t duration){
  unsigned long start_time = millis();
  duration = duration + 100;
  while (millis() - start_time < duration) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);  // Left motor moves BACKWARDS
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);   // Right motor moves backward
    delay(10);
  }
  stopAllMotors();
}

// Foward Movement function 
void forward( uint32_t duration ){
    uint32_t startTime = millis();
    duration = duration + 100;
    while (millis() - startTime < duration) {  // Keep moving for the duration
        long leftDist = getDistance(TRIG_LEFT, ECHO_LEFT);
        long centerDist = getDistance(TRIG_CENTER, ECHO_CENTER);
        long rightDist = getDistance(TRIG_RIGHT, ECHO_RIGHT);

        Serial.print("L: "); Serial.print(leftDist);
        Serial.print(" | C: "); Serial.print(centerDist);
        Serial.print(" | R: "); Serial.println(rightDist);
        //delay(500); 

        if (centerDist < 20) {  // If an obstacle is directly ahead
            stopAllMotors();  // Stop before turning
            if (rightDist > leftDist) {
                rotateRobotRight(40);  // Rotate 40 degrees to the right
            } else {
                rotateRobotLeft(40);  // Rotate 40 degrees to the left
            }
        } else if (leftDist < 15) {  // If obstacle is on the left
            stopAllMotors();
            rotateRobotRight(20);  // Slightly adjust right
        } else if (rightDist < 15) {  // If obstacle is on the right
            stopAllMotors();
            rotateRobotLeft(20);  // Slightly adjust left
        } else {
            // if the all front directions has obstacles 
            if(centerDist == leftDist == rightDist){
              stopAllMotors();
              moveBackwards(100);
              rotateRobotLeft(180);
            }
            // Keep moving forward
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        }

        //delay(100);  // Small delay to avoid excessive reads
    }

    stopAllMotors();  // Stop after duration is reached

  // digitalWrite(IN1, HIGH);
  // digitalWrite(IN2, LOW);
  // digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, LOW);
  // delay(duration);
}


void loop() {

  // String mac = WiFi.macAddress();
  // Serial.print("MAC Address: ");
  // Serial.println(mac);
  unsigned long startTime = millis();
  int peakLevel = 0;
  int peakMic = 1;
  int total1 = 0;
  int total2 = 0;
  int sampleCount = 0;
  int avgLevel = 0;

  while (millis() - startTime < SAMPLE_WINDOW) {
    int micValue1 = (readMicrophone(MIC_PIN1));
    int micValue2 = readMicrophone(MIC_PIN2);
    micValue2 += (abs(micValue1 - micValue2) * 0.000299999);
    // Serial.print("front Mic Value ");
    // Serial.println(micValue2);
    total1 += micValue1;
    total2 += micValue2;
    sampleCount++;
      
    // Checks if front mic1 has the highest reading
    if (micValue1 > peakLevel) {
      peakLevel = micValue1;
      peakMic = 1; // Front microphone
      avgLevel = total1 / sampleCount; 
    }
      
    // Check if back mic2 has the highest reading
    if (micValue2 > peakLevel) {
      peakLevel = micValue2;
      peakMic = 2; // Back microphone
      avgLevel = total2 / sampleCount; 
    }
  }

  if (peakMic == 1) {
    Serial.println("Sound is coming from Front!");
    
  } else if (peakMic == 2) {
      Serial.println("Sound is coming from Back!");
  }
  Serial.println(peakLevel);
  Serial.println(avgLevel);
  //rotateRobotLeft(180);
  //moveBackwards(180);
  //forward(5000/*SCREAM_DURATION*/);
  //Detect scream: If peak > 4000 and average exceed thresholds or 99% of threshold, LED comes on during scream detection this is the proper peak level *3900
  if (peakLevel > 3200/*4000*//*5000*/ && avgLevel > (SCREAM_THRESHOLD * 0.995)) {
    Serial.println("Scream Detected!!!!!!!!!!!");
    Serial.print("Peak Level: ");
    Serial.print(peakLevel);
    Serial.print(" | Average Level: ");
    Serial.print(avgLevel);
    Serial.print(" | Mic: ");
    Serial.println(peakMic);
    digitalWrite(LED_PIN, HIGH);
    stopAllMotors();
    if (peakMic == 1) {
      Serial.println(" Rotating and moving away");
      rotateRobotLeft(180);
      forward(10000/*SCREAM_DURATION*/);
    } else {
      Serial.println("Moving Forward");
      forward(10000/*SCREAM_DURATION*/);
    }
    digitalWrite(LED_PIN, LOW);
  } else {
    // Move to follow people cmd  hererere
      if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');  // Reads until newline
        input.trim();  // Removes the  whitespaces

        Serial.print("Received command: ");
        Serial.println(input);

        if (input == "LEFT") {
          rotateRobotLeft(50);
        } else if (input == "RIGHT") {
          rotateRobotRight(50);
        } else if (input == "FORWARD") {
          forward(50);
        } else if (input == "BACK") {
          moveBackwards(50);
        } else if (input == "STOP") {
          stopAllMotors();
      } else {
        Serial.println("not corrct cmd");
      }
  }
    
  }

  delay(300);  // Small delay 
}