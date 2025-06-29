//file: oculus.ino (new name chosen for github)
//file: esp32_PS4_EyeController.ino (old file name)


// Note: PS4 pair instructions: While pressing and holding the SHARE button, 
//   press and hold the PS Button until the light bar flashes.
// Note: Save yourself some time: board manager select esp32_bluepad32; ESP32-WROOM-DA Module

// 20250515 bav Initially pulled from: menu File->Examples->Bluepad32_ESP32->Controller.
//              Reference: https://bluepad32.readthedocs.io/en/latest/
//              This is where David Kent started from for his inexcpensive wheelchair controller.
// 20250518 bav Added some comments; commented out code that is not needed.
//              Found the pad_l; pad_r; and button_thumb_r and test code.
// 20250520 bav Mushed EyeControl.ino (Cogley_EyeMech) into Controller.ino (Bluetooth PS4)
//              No improovement are made to save time.
// 20250522 bav Changed left to right so eyelids would work in the correct direction.
//              Made the slider control a linear taper.
// 20250525 bav Added testServo to left vertical joystick. Want to test each RC servo  before installation.
//              Too many bad motor commutators. Tested servo sits on channel 15 of the PCA9685
//              Note to self: servo output spline shaft sizes are not standardized. 3d printed servo arms may
//              not fit correctly on servo shafts. There is a 3D printed test block size A and B. Check first!!!
// 20250527 bav Need to clean up demo code (remove it); save to GIT please.
//              Future enhancement: use paddle buttons to replace slide and dial pots. Will have to move the
//              mode selection to another control.
//              Note: PS4 bluetooth frame rate is 250Hz or every 4millsecs. USB connection frame rate is 1000Hz
//              At 250Hz and  an increment/decrement of 1, it would take 4 seconds to traverse 0 to 1024 like
//              an analog pot (slider/dial)
// 20250614 bav Removed slider/dial pots. Can compile with slider/dial (line: 214 in updateInputs() ) Assigned
//              3 settings to the X Y B buttons (xbox speak). The values were determined experimentally. This is
//              a hack.
// 20250626 bav Blink: BUTTON_THUMB_R | BUTTON_SHOULDER_R. Can now blink with the shoulder right button, as per David's request
//              moveScripts() enabled. Not tested yet
// 20250629 bav Tested and delivered for FireFly. Also entered into github https://github.com/framinghammakerspace194/oculus
//


#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "fscale.h"

// This is the historical file header.
//--------------------------------------------------------------------
/* This code seeks to control Will Cogley's 3D printed animatronic eye in a slightly 
 * more advanced way, while also adding simple random automation.
 * The Adafruit PWM Servo Driver library is required, as this code uses a PCA9685 
 * to drive the 6 servos (similar to the original).
 * This design is controlled by a joystick (looking around), a slider (eye wideness),
 * a dial (eyelid bias up/down), and a button (blink) with a momentary two direction 
 * switch (or two buttons) selecting the operating mode.
 * Note that my slider was not linear, so I have included fscale to linearize it. 
 * This can be tweaked pretty easily by setting sliderExp below to 0.
 * This code is free to use (just make sure my name and the names of those 
 * I copied code from stays with it!) for whatever.
 * Written by Nicholas Schwankl (Syber-Space) at Wake Technical Community College - 10/2022
 */

// ++++++++++ Pin Definitions ++++++++++ //

//#define vertJoy       A0 // PS4: ctl->axisRY() -512 to 512
//#define horJoy        A1 // PS4: ctl->axisRX() -512 to 512
#define slider        32 // ADC1-CH4 //A3 //bav 0-4096 0-3.3v
#define dial          33 // ADC1-CH5 //A2 //bav 0-4096 0-3.3v
//#define blinkButton   5 // PS4: ctl->buttons() & BUTTON_THUMB_R
//                                ctl->buttons() & ?

// The following pins are for mode control
//#define selectLeft    3 // PS4: ctl->dpad() == DPAD_LEFT
//#define selectRight   4 // PS4: ctl->dpad() == DPAD_RIGHT
#define buttonLight   2 // GPIO02 Mode output: PS4, light on; Prewritten file, light off.   //6

// #define testJoyServo              // PS4: ctl->axisY() -512 to 512

// ++++++++++ Calibration Values for the PS4 game controller ++++++++++ //

const int vertJoyMAX =  -508;   //0;
const int vertJoyMIN =   512;   //1022;

const int horJoyMAX =    512;   //1000;
const int horJoyMIN =   -508;   //0;

const int sliderMAX =   580;
const int sliderMIN =   0;

const int dialMAX =     0;
const int dialMIN =     1023;

const int testJoyServoMAX = -508;
const int testJoyServoMIN =  512;

const float sliderExp = 0; // I have a linear pot. ------ -8.5; // fscale curve parameter. 0 is linear.
// Used as fscale(sliderMIN, sliderMAX, [low], [high], analogRead(slider), sliderExp)
// fscale() is like map() with the addition of a curve parameter
// fscale moves the input parameter from map() first parameter to the 3rd position. No idea why. Should be fixed.

const int lidScale = 200; // In the range of -1000 - 1000, there is an eyelid center position. 
// From the center position calculate the min and max eyelid opening using lidScale.

const float moveInfluence = 0.4; //As the eye ball moves up and down the eyelid tracks proportionally. 
// Scaled by moveInfluence.
const float offsetInfluence = 0.5; //Move both upper and lower eyelids up and down to center eye pupil
// between the eyelids.


// ++++++++++ PCA9685  servo Controller Settings ++++++++++ //
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();
// 
#define SERVOMIN  95 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  435 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// ++++++++++ RC Servo Names ++++++++++ //
// RC servos are indexed 0 to 15.

#define vertEye     1
#define horEye      0
#define lowerL      3
#define upperL      2
#define lowerR      5
#define upperR      4
#define testServo   15

// ++++++++++ RC Servo Effective Limits ++++++++++ //
// Determined by experiment
const int vertEyeMAX =  320; //
const int vertEyeCEN =  265; //
const int vertEyeMIN =  210; //

const int horEyeMAX =   345; //
const int horEyeCEN =   260; //
const int horEyeMIN =   175; //

const int lowerLMAX =   320; //
const int lowerLCEN =   285; //
const int lowerLMIN =   145; //

const int upperLMAX =   380; //
const int upperLCEN =   255; //
const int upperLMIN =   200; //

const int lowerRMAX =   180; //
const int lowerRCEN =   230; //
const int lowerRMIN =   395; //

const int upperRMAX =   150; //
const int upperRCEN =   275; //
const int upperRMIN =   320; //

const int testServoMAX =  380;
const int testServoCEN =  265;
const int testServoMIN =  200;


// ++++++++++ RC Servo Calculated Constants ++++++++++ //

const int vertDivMAX = max(abs(vertEyeMIN - vertEyeCEN), abs(vertEyeMAX - vertEyeCEN));
const int horDivMAX = max(abs(horEyeMIN - horEyeCEN), abs(horEyeMAX - horEyeCEN));
const int divMAX = max(vertDivMAX, horDivMAX); // Find a scale factor for the largest limit

const int lowerLSign = lowerLCEN > lowerLMIN ? 1 : -1; // Find the sign for the servo direction
const int upperLSign = upperLCEN > upperLMIN ? 1 : -1;
const int lowerRSign = lowerRCEN > lowerRMIN ? 1 : -1;
const int upperRSign = upperRCEN > upperRMIN ? 1 : -1;


// Prerecorde eyelid positions for different emotions.
// Values for variables openMapped and offsetMapped
// Replaces the slider and dial pots.
// Normal: ^ button; Droopy: [] button; 0 button
// Values are normalized open: 0->1000 offset: -1000->1000
// Values are determined experimentally
#define OPEN_EMOTION_NORMAL 410
#define OFFSET_EMOTION_NORMAL 290
#define OPEN_EMOTION_DROOPY 536
#define OFFSET_EMOTION_DROOPY 470
#define OPEN_EMOTION_SUSPICIOUS 275
#define OFFSET_EMOTION_SUSPICIOUS 159


// ++++++++++ Variables ++++++++++ //

// These variables are the dividing point between PS4 inputs and RC servo outputs.
// Values are "normallized"
int horMapped = 0; // joystick horizontal mapping -1000-1000
int vertMapped = 0; // joystick vertical mapping -1000-1000

int openMapped; // slider mapping 0-1000
int offsetMapped; // dial mapping -1000-1000
// Note: Setting openMapped==0/offsetMapped==0 causes a blink.

int testServoMapped = 0; // left joystick vertical mapped to -1000 to 1000

// These variables are the t(n-1) state of moveEyesAdv()
// The variables are used in linearMoveAdv() [programMode == true] for prewritten movements 
// Units are in servo counts via PCA9685 counters (I believe)
// This should be rolled up into a class.
int lastXPos = 0;
int lastYPos = 0;
int lastOpenL = 0;
int lastOpenR = 0;
int lastOffsetL = 0;
int lastOffsetR = 0;
int lastTestServoPos = 0;

volatile bool programMode = false; // false: run input from PS4; true: run input from a prewritten file.


// ++++++++++ Functions ++++++++++ // ++++++++++ Functions ++++++++++ // ++++++++++ Functions ++++++++++ // ++++++++++ Functions ++++++++++ //

void updateInputs(ControllerPtr ctl){ // Update all inputs

  horMapped = map( 
                constrain(
                  ctl->axisRX(),
                  //analogRead(horJoy), 
                  min(horJoyMIN, horJoyMAX), max(horJoyMIN, horJoyMAX)), 
                horJoyMIN, horJoyMAX, -1000, 1000);
  vertMapped = map( 
                constrain(
                  ctl->axisRY(),
                  //analogRead(vertJoy),
                  min(vertJoyMIN, vertJoyMAX), max(vertJoyMIN, vertJoyMAX)),
                vertJoyMIN, vertJoyMAX, -1000, 1000);
  if(0) { // check pots for eyelid positions slider/dial
    openMapped = fscale(sliderMIN, sliderMAX, 0, 1000, 
                    constrain(
                      analogRead(slider),
                      min(sliderMIN, sliderMAX), max(sliderMIN, sliderMAX)),
                  sliderExp);
    offsetMapped = map( 
                    constrain(
                      analogRead(dial),
                      min(dialMIN, dialMAX), max(dialMIN, dialMAX)),
                  dialMIN, dialMAX, -1000, 1000);
  } else { // use the ps4 button presets

    // openMapped: slider mapping 0-1000
    // offsetMapped: dial mapping -1000-1000
    static int openMappedLastButton = OPEN_EMOTION_NORMAL;   // Places to hold the last emotion set by a button.
    static int offsetMappedLastButton = OFFSET_EMOTION_NORMAL; //


    // eyelids emotion neutral
    if (ctl->y()) { // xbox top button / ps4 ^ button
      openMappedLastButton = OPEN_EMOTION_NORMAL;
      offsetMappedLastButton = OFFSET_EMOTION_NORMAL;
    }
    // eylids emotion droopy
    if (ctl->x()) { // xbox left button / ps4 square button
      openMappedLastButton = OPEN_EMOTION_DROOPY;
      offsetMappedLastButton = OFFSET_EMOTION_DROOPY;
    }
    // eylids emotion suspicious
    if (ctl->b()) { // xbox right button / ps4 circle button
      openMappedLastButton = OPEN_EMOTION_SUSPICIOUS;
      offsetMappedLastButton = OFFSET_EMOTION_SUSPICIOUS;
    }
    openMapped = openMappedLastButton;
    offsetMapped = offsetMappedLastButton;
  }

  testServoMapped = map( 
                constrain(
                  ctl->axisY(),
                  min(testJoyServoMIN, testJoyServoMAX), max(testJoyServoMIN, testJoyServoMAX)),
                testJoyServoMIN, testJoyServoMAX, -1000, 1000);

  if(ctl->buttons() & (BUTTON_THUMB_R | BUTTON_SHOULDER_R)) {
  //if(!digitalRead(blinkButton)){
    openMapped = 0;
    offsetMapped = 0;
    Serial.print("blink");
  }
}


// Move eyes to coordinate
void moveEyesAdv(int xPos, int yPos, int lidOpenL, int lidOpenR, int lidOffsetL, int lidOffsetR, int testServoPos){ 
  lastXPos = xPos;
  lastYPos = yPos;
  lastOpenL = lidOpenL;
  lastOpenR = lidOpenR;
  lastOffsetL = lidOffsetL;
  lastOffsetR = lidOffsetR;
  lastTestServoPos = testServoPos;

  // These functions map inputs from +- 1000 to the servo limits, scaled based on the furthest limit
  int vertEyeVal = constrain(map(yPos, -1000, 1000, vertEyeCEN-divMAX, vertEyeCEN+divMAX), vertEyeMIN, vertEyeMAX);
  int horEyeVal = constrain(map(xPos, -1000, 1000, horEyeCEN-divMAX, horEyeCEN+divMAX), horEyeMIN, horEyeMAX);

  lidOffsetL = lidOffsetL + yPos*moveInfluence;
  lidOffsetR = lidOffsetR + yPos*moveInfluence;
  
  int upperLVal = map(constrain((lidOpenL + lidOffsetL*offsetInfluence), -1000, 1000), -1000, 1000, upperLCEN - upperLSign*lidScale, upperLCEN + upperLSign*lidScale);
  int lowerLVal = map(constrain((-lidOpenL + lidOffsetL*offsetInfluence), -1000, 1000), -1000, 1000, lowerLCEN - upperLSign*lidScale, lowerLCEN + lowerLSign*lidScale);
  int upperRVal = map(constrain((lidOpenR + lidOffsetR*offsetInfluence), -1000, 1000), -1000, 1000, upperRCEN - upperRSign*lidScale, upperRCEN + upperRSign*lidScale);
  int lowerRVal = map(constrain((-lidOpenR + lidOffsetR*offsetInfluence), -1000, 1000), -1000, 1000, lowerRCEN - lowerRSign*lidScale, lowerRCEN + lowerRSign*lidScale);

  int upperModifier = (vertEyeVal - vertEyeCEN);
  upperModifier = upperModifier > 0 ? 0 : upperModifier;
  
  upperLVal = constrain(upperLVal, min(upperLMIN, upperLMAX + upperLSign*upperModifier), max(upperLMIN, upperLMAX + upperLSign*upperModifier));
  lowerLVal = constrain(lowerLVal, min(lowerLMIN, lowerLMAX), max(lowerLMIN, lowerLMAX));
  upperRVal = constrain(upperRVal, min(upperRMIN, upperRMAX + upperRSign*upperModifier), max(upperRMIN, upperRMAX + upperRSign*upperModifier));
  lowerRVal = constrain(lowerRVal, min(lowerRMIN, lowerRMAX), max(lowerRMIN, lowerRMAX));
  
  int testServoVal = constrain(map(testServoPos, -1000, 1000, testServoMIN, testServoMAX), testServoMIN, testServoMAX);
 
  servo.setPWM(vertEye, 0, vertEyeVal);
  servo.setPWM(horEye, 0, horEyeVal);
  servo.setPWM(upperL, 0, upperLVal);
  servo.setPWM(lowerL, 0, lowerLVal);
  servo.setPWM(upperR, 0, upperRVal);
  servo.setPWM(lowerR, 0, lowerRVal);
  servo.setPWM(testServo, 0, testServoVal);
  
}



void moveEyes(int xPos, int yPos, int lidOpen, int lidOffset, int testServoPos){
  moveEyesAdv(xPos,yPos,lidOpen,lidOpen,lidOffset,lidOffset, testServoPos);
}



void linearMoveAdv(int xPos, int yPos, int lidOpenL, int lidOpenR, int lidOffsetL, int lidOffsetR, int testServoPos, int moveTime){
  
  moveTime = moveTime / 10;
  
  float xDif = float(xPos - lastXPos) / moveTime;
  float yDif = float(yPos - lastYPos) / moveTime;
  float lidOpenLDif = float(lidOpenL - lastOpenL) / moveTime;
  float lidOpenRDif = float(lidOpenR - lastOpenR) / moveTime;
  float lidOffsetLDif = float(lidOffsetL - lastOffsetL) / moveTime;
  float lidOffsetRDif = float(lidOffsetR - lastOffsetR) / moveTime;
  float testServoDif = float(testServoPos - lastTestServoPos) / moveTime;

  float storeX = lastXPos;
  float storeY = lastYPos;
  float storeOpenL = lastOpenL;
  float storeOpenR = lastOpenR;
  float storeOffsetL = lastOffsetL;
  float storeOffsetR = lastOffsetR;
  float storeTestServo = lastTestServoPos;
  
  for(int i = 0; i < moveTime; i++){
    moveEyesAdv(
      storeX + (xDif * i),
      storeY + (yDif * i),
      storeOpenL + (lidOpenLDif * i),
      storeOpenR + (lidOpenRDif * i),
      storeOffsetL + (lidOffsetLDif * i),
      storeOffsetR + (lidOffsetRDif * i),
      storeTestServo + (testServoDif * i)
      );
    delay(10);
  }
  moveEyesAdv(xPos, yPos, lidOpenL, lidOpenR, lidOffsetL, lidOffsetR, testServoPos);
  
}

void linearMove(int xPos, int yPos, int lidOpen, int lidOffset, int testServoPos, int moveTime){
  linearMoveAdv(xPos, yPos, lidOpen, lidOpen, lidOffset, lidOffset, testServoPos, moveTime);
}





#include "moveScripts.h"

//--------------------------------------------------------------------
// BT and PS4 stuff:
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d, open:%4d, offset:%4d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ(),        // Accelerometer Z

        openMapped,
        offsetMapped
    );
} // dumpGamepad()


void processGamepad(ControllerPtr ctl) {

    // Always look to the gamepad for the dpad controls to set/unset the programMode
    if (ctl->dpad() & DPAD_LEFT) { // manual mode; PS4 joystick active
      Serial.print("dpad l");
      programMode = false;
      digitalWrite(buttonLight, HIGH);
    }
    if (ctl->dpad() & DPAD_RIGHT) { // read movements from a prewritten file
      Serial.print("dpad r");
      programMode = true;
      digitalWrite(buttonLight, LOW);
    }

    // If reading motion for a file, ignore any other PS4 inputs.
    if( !programMode ) {

      // Get and normalize the joystick slide and dial
      updateInputs(ctl);
      moveEyes(horMapped, vertMapped, openMapped, offsetMapped, testServoMapped);

    } // if( !programMode )

    // The rest of the code is PS4 test code.

    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    //  x(), o(), [](), ^()
    //   xbox       ps4
    //     Y         ^
    //   X   B    []   0
    //     A         X

/*
    if (ctl->a()) { // xbox bottom button/ ps4 X button
        static int colorIdx = 0; // off
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) { // rotate colors red/green/blue
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0); // red
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0); // green
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255); // blue
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) { // xbox right button / ps4 O button
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) { // xbox left button / ps4 square button
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0, // delayedStartMs
                            250, // durationMs
                            0x80, // weakMagnitude
                            0x40 ); //strongMagnitude 
    }
*/
    //if (ctl->dpad() & DPAD_LEFT) {Serial.print("dpad l");}
    //if (ctl->dpad() & DPAD_RIGHT) {Serial.print("dpad r");}
    //if (ctl->buttons() & BUTTON_THUMB_R ) {Serial.print("thumb r");}

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);


} // processGamepad()



void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
} // processControllers()

// Arduino setup function. Runs in CPU 1
void setup() {

    // Setup the non PS4 inputs and outputs.
    pinMode(slider, INPUT);
    pinMode(dial,   INPUT);
    pinMode(buttonLight,  OUTPUT);

    // Setup the Bluetooth and PS4 controller
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // Setup RC servos
    //
    servo.begin();
    servo.setOscillatorFrequency(27000000);
    servo.setPWMFreq(SERVO_FREQ);

    delay(10);

    for(int i = 0; i<6; i++){
      servo.setPWM(i, 0, (SERVOMIN + SERVOMAX)/2);
      delay(100);
    }

} // setup()

// Arduino loop function. Runs in CPU 1.
void loop() {
    // Read moves from a file.
    if( programMode ) {
      moveScript[random(moveScriptsNum)]();

      //bav analogWrite(buttonLight, 120); // wink the mode light
      delay(10);
      digitalWrite(buttonLight, LOW);
    } else {
      //bav analogWrite(buttonLight, 80);
      delay(10);
      digitalWrite(buttonLight, HIGH);
    }

    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    // Note: Nothing is going to happen unless the PS4 is paired.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(150);
} // loop()
