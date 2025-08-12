#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal.h>

const int SERVO_COUNT = 4;
const int SERVO_MIN[SERVO_COUNT] = {0, 110, 140, 140};
const int SERVO_MEAN[SERVO_COUNT] = {90, 130, 150, 145};
const int SERVO_MAX[SERVO_COUNT] = {180, 185, 200, 170};


#define I2C_SDA 5
#define I2C_SCL 6

const int SERVO_CHANNELS[SERVO_COUNT] = {3, 4, 5, 6}; // Assuming fourth slot is channel 3

// Create PCA9685 servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo pulse length constants (in microseconds)
#define SERVO_FREQ 50
#define SERVOMIN 150
#define SERVOMAX 600

LiquidCrystal lcd(4, 7, 15, 16, 17, 18);

// WiFi connection settings
const char *ssid = "n16r8-devkits3";
const char *password = "arimakana";

WiFiServer server(80);

int targetAngles[SERVO_COUNT] = {SERVO_MEAN[0], SERVO_MEAN[1], SERVO_MEAN[2], SERVO_MEAN[3]};
int currentAngles[SERVO_COUNT] = {SERVO_MEAN[0], SERVO_MEAN[1], SERVO_MEAN[2], SERVO_MEAN[3]};
const float SPEED_LIMIT = 21.0;   // divisione intera: valori tra 0 e 9 portano a ZERO movimento
const int LOOP_DELAY = 100;
const float MAX_MOVEMENT_PER_ITERATION = SPEED_LIMIT * (LOOP_DELAY / 1000.0);

// Helper function to extract parameter value from URL
String extractParameter(String url, String paramName) {
  int paramStart = url.indexOf(paramName + "=");
  if (paramStart == -1) return "";
  
  paramStart += paramName.length() + 1; // Skip parameter name and '='
  int paramEnd = url.indexOf("&", paramStart);
  if (paramEnd == -1) paramEnd = url.indexOf(" ", paramStart);
  if (paramEnd == -1) paramEnd = url.length();
  
  return url.substring(paramStart, paramEnd);
}

void lcdLog(String msg) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(msg.substring(0, 16)); // prima riga
  if (msg.length() > 16) {
    lcd.setCursor(0, 1);
    lcd.print(msg.substring(16, 32)); // seconda riga
  }
}

// Convert angle to pulse width for PCA9685
int angleToPulse(int angle) {
  // Map angle (0-180) to pulse width (SERVOMIN-SERVOMAX)
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Set servo position using PCA9685
void setServoPosition(int servoNum, int angle) {
  if (servoNum >= 1 && servoNum <= SERVO_COUNT) {
    int servoIndex = servoNum - 1;
    int channel = SERVO_CHANNELS[servoIndex];
    int pulse = angleToPulse(angle);
    pwm.setPWM(channel, 0, pulse);
  }
}

// Set the target angle for a servo with bounds checking
void setTargetAngle(int servoNum, int angle) {
  if (servoNum >= 1 && servoNum <= SERVO_COUNT) {
    int index = servoNum - 1;
    // Apply limits to the requested angle
    angle = constrain(angle, SERVO_MIN[index], SERVO_MAX[index]);
    targetAngles[index] = angle;
  }
}

// Convert percentage to angle
int percentToAngle(int servoNum, int percentage) {
  if (servoNum < 1 || servoNum > SERVO_COUNT) return 0;
  
  // Ensure percentage is between 0 and 100
  percentage = constrain(percentage, 0, 100);
  
  // Convert percentage to angle within the servo's range
  int servoIndex = servoNum - 1;
  int minAngle = SERVO_MIN[servoIndex];
  int maxAngle = SERVO_MAX[servoIndex];
  
  // Calculate angle based on percentage of range
  return minAngle + (percentage * (maxAngle - minAngle) / 100);
}

void setup() {
  delay(1000);

  lcd.begin(16, 2);
  lcdLog("Boot...");
  delay(1000);

  // Initialize I2C with specified pins
  Wire.begin(I2C_SDA, I2C_SCL);
  lcdLog("Wire ok");
  delay(1000);
  
  // Initialize the PCA9685 servo driver
  pwm.begin();
  lcdLog("PWM ok");
  delay(1000);
  
  // Check if PCA9685 is connected
  Wire.beginTransmission(0x40); // Default PCA9685 address
  if (Wire.endTransmission() == 0) {
    pwm.setPWMFreq(SERVO_FREQ);  // Set frequency to 50Hz for servos
    
    // Initialize servos to mean position
    for (int i = 0; i < SERVO_COUNT; i++) {
      int channel = SERVO_CHANNELS[i];
      int pulse = angleToPulse(SERVO_MEAN[i]);
      pwm.setPWM(channel, 0, pulse);
      currentAngles[i] = SERVO_MEAN[i];
      targetAngles[i] = SERVO_MEAN[i];
    }
  } else {
    lcdLog("PWM FAIL");
    delay(2000);
  }

  // Start Access Point mode
  bool apStarted = WiFi.softAP(ssid, password);
  if (apStarted) lcdLog("WiFi AP ok");
  else lcdLog("WiFi AP FAIL");
  server.begin();
  delay(1000);
}

void loop() {
  WiFiClient client = server.available();  // listen for incoming clients

  if (client) {                     // if you get a client,
    String currentLine = "";        // make a String to hold incoming data from the client
    String request = "";            // store the full HTTP request
    
    while (client.connected()) {    // loop while the client's connected
      if (client.available()) {     // if there's bytes to read from the client,
        char c = client.read();     // read a byte, then
        request += c;               // add to the full request
        
        if (c == '\n') {            // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // Handle different request types
            if (request.indexOf("GET /set") >= 0) {
              // Handle servo angle setting
              String servoStr = extractParameter(request, "servo");
              String angleStr = extractParameter(request, "angle");
              
              if (servoStr.length() > 0 && angleStr.length() > 0) {
                int servoNum = servoStr.toInt();
                int angle = angleStr.toInt();
                setTargetAngle(servoNum, angle);
                client.println("<h1>Servo Control</h1>");
                client.println("<p>Servo " + String(servoNum) + " set to " + String(angle) + " degrees</p>");
              } else {
                client.println("<h1>Error</h1><p>Missing servo or angle parameter</p>");
              }
              
            } else if (request.indexOf("GET /setpercent") >= 0) {
              // Handle percentage setting
              String servoStr = extractParameter(request, "servo");
              String percentStr = extractParameter(request, "percentage");
              
              if (servoStr.length() > 0 && percentStr.length() > 0) {
                int servoNum = servoStr.toInt();
                int percentage = percentStr.toInt();
                int angle = percentToAngle(servoNum, percentage);
                setTargetAngle(servoNum, angle);
                client.println("<h1>Servo Control</h1>");
                client.println("<p>Servo " + String(servoNum) + " set to " + String(percentage) + "% (" + String(angle) + " degrees)</p>");
              } else {
                client.println("<h1>Error</h1><p>Missing servo or percentage parameter</p>");
              }
              
            } else if (request.indexOf("GET /status") >= 0) {
              // Handle status request
              client.println("<h1>Servo Status</h1>");
              client.println("<p>Current servo positions:</p>");
              client.println("<ul>");
              for (int i = 0; i < SERVO_COUNT; i++) {
                client.println("<li>Servo " + String(i+1) + ": " + String(currentAngles[i]) + " degrees (Target: " + String(targetAngles[i]) + ")</li>");
              }
              client.println("</ul>");
              
            } else {
              // Default home page
              client.println("<html><head><title>Servo Control</title>");
              client.println("<meta name='viewport' content='width=device-width, initial-scale=1'>");
              client.println("<style>body{font-family:Arial;margin:20px;max-width:600px}");
              client.println(".form-group{margin-bottom:15px}.slider{width:100%}</style>");
              client.println("</head><body>");
              client.println("<h1>Robot Servo Control</h1>");
              
              client.println("<h2>Angle Control</h2>");
              client.println("<form action='/set' method='GET'>");
              client.println("<div class='form-group'>Servo (1-4): <input type='number' name='servo' min='1' max='4' value='1'></div>");
              client.println("<div class='form-group'>Angle: <input type='number' name='angle' value='90'></div>");
              client.println("<input type='submit' value='Set Angle'>");
              client.println("</form>");
              
              client.println("<h2>Percentage Control</h2>");
              client.println("<form action='/setpercent' method='GET'>");
              client.println("<div class='form-group'>Servo (1-4): <input type='number' name='servo' min='1' max='4' value='1'></div>");
              client.println("<div class='form-group'>Percentage (0-100%): <input type='range' class='slider' name='percentage' min='0' max='100' value='50'>");
              client.println("<span id='percentValue'>50%</span></div>");
              client.println("<input type='submit' value='Set Percentage'>");
              client.println("</form>");
              
              client.println("<script>");
              client.println("document.querySelector('.slider').oninput = function() {");
              client.println("  document.getElementById('percentValue').textContent = this.value + '%';");
              client.println("}");
              client.println("</script>");
              
              client.println("<p><a href='/status'>View current status</a></p>");
              client.println("</body></html>");
            }

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {  // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // close the connection:
    client.stop();
  }

  lcd.setCursor(0, 0);
  lcd.print("C:");
  lcd.print(WiFi.softAPgetStationNum());
  lcd.print(" S1:");
  lcd.print((int)round(currentAngles[0]));
  lcd.print(" S2:");
  lcd.print((int)round(currentAngles[1]));

  lcd.setCursor(0, 1);
  lcd.print("S3:");
  lcd.print((int)round(currentAngles[2]));
  lcd.print(" S4:");
  lcd.print((int)round(currentAngles[3]));
  lcd.print("   "); // pulizia
  
  // Main program loop - move servos gradually towards their target positions
  bool anyServoMoving = false;
  
  // Move each servo gradually towards its target position
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (currentAngles[i] != targetAngles[i]) {
      // Calculate how much to move this iteration (respect speed limit)
      float angleDiff = targetAngles[i] - currentAngles[i];
      float movement = constrain(angleDiff, -MAX_MOVEMENT_PER_ITERATION, MAX_MOVEMENT_PER_ITERATION);
      
      // Update current angle
      currentAngles[i] += movement;
      
      // Round to nearest integer for the servo
      int newAngle = round(currentAngles[i]);
      
      // Set the servo to the new position using PCA9685
      setServoPosition(i+1, newAngle);
      
      anyServoMoving = true;
    }
  }
  
  delay(LOOP_DELAY); // Small delay to maintain consistent timing
}