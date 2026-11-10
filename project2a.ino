#include "lcdhelperv2.h"
#include "irhelper.h"
#include <math.h>

//simple change to test Git works

//setup pwm
const int pwmFreq = 496;
const int pwmResolution = 8;
const int maxDutyCycle = (int)pow(2, pwmResolution) - 1;

//makes variables for the left motor
const int lencoder = 10;
const int lenenablepwm = 46;
const int lphasedir = 12;
volatile unsigned long leftEncoderCount = 0;
const int lpwmChannel = 1;

//make variables for the right motor
const int rencoder = 8;
const int renablepwm = 14;
const int rphasedir = 13;
volatile unsigned long rightEncoderCount = 0;
const int rpwmChannel = 2;

//step 12 for switches
const int sw1 = 18;
const int sw2 = 17;
const int sw3 = 16;
const int sw4 = 15;

//step 13
const int tiltpin = 35;
const int servopin = 36;

//step14
const int ECHOpin = 37;
const int TRIGpin = 38;

//stuff for option 1
void UpdateDisplay(int Rduty, int LenC, int RenC, int Error);
long DriveError = 0;
const double pivalue = 3.141593;
const double WDIA = 6.5; // wheel diameter(cm)
const double CPR = 20.0; // counts on the encoder per full wheel rotation
const double TG = 100.0; // distance used in the calibration run (cm)
double rdutyStraight = 0.0;    // right dutycycle
const double correctingfactor = 0;


unsigned long lastleftpulse = 0;
unsigned long lastrightpulse = 0;
 

//option 3
//option 3 global variables
float yC = 0;
float xC = 0;
float rInner = 0;
float rOuter = 60;
float radians = 0;
float scaledDistance = 0;
int startAngle = 0;
int endAngle = 0;
float totalDistance = 0;
int measurementCount = 0;
// final object loccation
float theta_f = 0;
float d_f = 0;
float theta_r = 0;
float d_r = 0;
float theta_target = 0;   //final angle target
float d_target = 0;       //final distance of target
int previousAngle = 0;
int previousDistance = 120; //initialize previousDistance to 120cm
float closestDistance = 120;
float closestAngle = 0;
int closestCount = 0;
int closestStartAngle = 0;
int closestEndAngle = 0;
// option 3 disable option #4
bool targetFound = false;


//option 4
bool option1done = false;
bool option3done =  false;
int storedDuty = 0;
float targAng = 0;
float targDist = 0;

const double pivotOffset = 2.0; //dist of from pan tilt assembly to wheel
const double wheelBase = 11.1; //center to center of wheels in cm



//functions
void degreeIncrements();
void servowrite(int pin, int angle, int numpulses);
int calculateDistance(int tPin, int ePin);
void radarFrame();

int xyz = 0;
//
//  Create an instance of the lcdhelper class named oLCD

//
lcdhelper oLCD(ILI9163_4L, 3, 2, 9, 10, 7); 

//  Declare a sprite so we can display a rotated label on the TFT.
//
TFT_eSprite spr = TFT_eSprite(&oLCD); // Sprite object
//
//  Create an instance of irhelper class named oIR.
//
irhelper oIR;
// Get width of TFT display in pixels
int xScreen = 1;
// Get height of TFT display in pixels
int yScreen = 1;
// Define a spacer to represent margins and spacing between elements.
int mScreen = 2;
//
//  Define protypes for functions
//
void ShowDisplay(screen val, char optionstate, char keypressed, bool initdisplay = false);
// *************************************************************************************
// BEGIN: USER FUNCTION
// *************************************************************************************
void clearText(int32_t x, int32_t y, int32_t width, uint16_t color)
{
    int32_t xStart;

    if (x == CENTER)
        xStart = (oLCD.width() - width) / 2;
    else
        xStart = x;

    oLCD.fillRect(xStart, y-6, width, 15, color);
}
//
//  Option1 code
//
void Option1(char optionstate, char keypressed, bool initdisplay)
{
   // holds the words before they are printed
    char text[20];

    // static is important because it will stay what it previously was as we progress
    static String Dcycle = "";
    static bool DriveMode = false;

    // clears the screen and sets the text/background colors
    oLCD.LCDInitialize(LANDSCAPE, initdisplay);
    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);

    // if return is pressed with less than 2 digits it will stay on the screen
    if (optionstate == 'B' && Dcycle.length() != 2)
    {
        // A for entry screen code
        optionstate = 'A';
        oLCD.OPTION_STATE = 'A';
    }
    
    if(DriveMode == true)
    {
        optionstate = 'B'; 
    }

    if (optionstate == 'A')
    {
        // entry screen text
        sprintf(text, "DUTY CYCLE");
        oLCD.print(text, CENTER, 15);
        sprintf(text, "PRESS");
        oLCD.print(text, CENTER, 52);
        sprintf(text, "<RETURN>");
        oLCD.print(text, CENTER, 68);
        sprintf(text, "WHEN");
        oLCD.print(text, CENTER, 84);
        sprintf(text, "READY");
        oLCD.print(text, CENTER, 100);

        // Option 1 was opened the first time
        if (keypressed == ' ')
        {
            Dcycle = "";
            clearText(CENTER, 32, 48, RCB_BLACK);
        }
        // ignores return if less than 2 digits. Repastes what was already typed
        else if (keypressed == 'R' && Dcycle.length() != 2)
        {
            clearText(CENTER, 32, 48, RCB_BLACK);
            if (Dcycle.length() > 0)
            {
                oLCD.setBackColor(RCB_BLACK);
                oLCD.printNumI(Dcycle.toInt(), CENTER, 32, 3, ' ');
            }
        }
        // If key besides RETURN is pressed
        else
        {
            // deletes the last digit
            if (keypressed == 'B')
            {
                if (Dcycle.length() > 0)
                {
                    Dcycle.remove(Dcycle.length() - 1);
                }
            }
            // conditional to check that the number is valid
            else if (Dcycle.length() == 0)
            {
                if (keypressed == '0' || keypressed == '1' || keypressed == '2' || keypressed == '3' || keypressed == '8' || keypressed == '9')
                {
                    Dcycle = "";
                }
                else if (keypressed == '4' || keypressed == '5' || keypressed == '6' || keypressed == '7')
                {
                    Dcycle = Dcycle + keypressed;
                }
                else if (keypressed == 'B')
                {
                    Dcycle = "";
                }
            }
            else if (Dcycle == "7" && keypressed != ' ' && keypressed != 'B') // Default every 2-digit # starting with 7 to 70
            {
                Dcycle = "70";
            }
            else if (Dcycle == "4" && keypressed != ' ' || Dcycle == "5" && keypressed != ' ' || Dcycle == "6" && keypressed != ' ') // Build string with #s
            {
                if (keypressed != 'B')
                {
                    Dcycle = Dcycle + keypressed;
                }
            }
            else if (Dcycle.length() == 2) // #s above 2-digits not allowed
            {
                Dcycle = Dcycle;
            }

            // shows a blankbox after things are deleated
            clearText(CENTER, 32, 20, RCB_BLACK);
            if (Dcycle.length() > 0)
            {

                oLCD.setBackColor(RCB_BLACK);
                oLCD.printNumI(Dcycle.toInt(), CENTER, 32, 3, ' ');
            }
        }
    }
    // second screen of option 1
    else if (optionstate == 'B')
    {
        // circumference (cm)
        double circumference = pivalue * WDIA;
        // equation to store the number of counts needed to reach the target
        const unsigned long counttotarget = (((double)TG * (double)CPR) / (double)circumference);
        
        // resets Encoder Counts
        leftEncoderCount = 0;
        rightEncoderCount = 0;
            // moves both motors forward at the set duty cycle
            digitalWrite(lphasedir, HIGH);
            digitalWrite(rphasedir, HIGH);
            
            // left motor stays at the entered duty for the entire run
            ledcWrite(lpwmChannel, (float)Dcycle.toInt() * maxDutyCycle / 100.0);
            // right motor starts at the entered duty then will be corrected
            rdutyStraight = (float)Dcycle.toInt(); 
            ledcWrite(rpwmChannel, rdutyStraight * maxDutyCycle / 100.0);

            // uses sprintf to copy the words into text in the form of a char array. Then prints the screen text
            sprintf(text, "RIGHT DUTY CYCLE");
            oLCD.print(text, CENTER, 10);
            sprintf(text, "LEFT COUNT");
            oLCD.print(text, CENTER, 38);
            sprintf(text, "RIGHT COUNT");
            oLCD.print(text, CENTER, 66);
            sprintf(text, "ERROR");
            oLCD.print(text, CENTER, 94);
        
        // keep going till the left wheel reaches the counttotarget
        while (leftEncoderCount < counttotarget){


            //stops if the power button is pressed stop the motors and show the current screen
            unsigned long newKey = oIR.GetKeyPressed();
            if(newKey == KEY_POWER){
                 ledcWrite(rpwmChannel, 0);
                 ledcWrite(lpwmChannel, 0);

                 break;
            }
         
            DriveError = (long)leftEncoderCount - (long)rightEncoderCount * 0.77; 
            
            //Adjusting right duty cycle for error 
            rdutyStraight = Dcycle.toInt() + (0.75 * (double)DriveError );

            //Failsafe duty cycle fix 
            if(rdutyStraight > 80.0)
            {
                rdutyStraight = 80.0; 
            } 
            else if (rdutyStraight < 20.0)
            {
                rdutyStraight = 20.0; 
            }

            ledcWrite(rpwmChannel, (float)rdutyStraight * maxDutyCycle / 100.0);
            UpdateDisplay(rdutyStraight, (int)leftEncoderCount, (int)rightEncoderCount, DriveError); 
            delay(20);        
        }
        DriveMode = false; 
        // stops motors when target is reached
        ledcWrite(lpwmChannel, 0);
        ledcWrite(rpwmChannel, 0);
        
        UpdateDisplay(rdutyStraight, (int)leftEncoderCount, (int)rightEncoderCount, DriveError);
        oLCD.OPTION_STATE = 'A';
        // ignores buttons pressed when the robot was in motion
        // oIR.GetKeyPressed();
    }
}
//
//  Option2 code
//
void Option2(char optionstate, char keypressed, bool initdisplay)
{
    oLCD.OPTION_STATE = 'A';

    // holds the words before they are printed
    char text[20];

    // Setting up target distance and target angle variables
    float TargetAngle = 60.0; 
    float TargetDist = 50.0;

    // clears the screen and sets the text/background colors
    oLCD.LCDInitialize(LANDSCAPE, initdisplay);
    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE); 

    // uses sprintf to copy the words into text in the form of a char array then prints the startup screen text
    sprintf(text, "PREPARING");
    oLCD.print(text, CENTER, 38);
    sprintf(text, "TO");
    oLCD.print(text, CENTER, 54);
    sprintf(text, "ACQUIRE");
    oLCD.print(text, CENTER, 70);
    sprintf(text, "TARGET");
    oLCD.print(text, CENTER, 86);
    
    delay(3000); 

    // Initial Ultrasonic sweep 
    // Sweep();
    
    UpdateDisplay2((int)TargetDist, (int)TargetAngle);

    while (TargetDist > 2.0)
    {
        // Stops if the return button is pressed stop the motors and returns to main menu
        unsigned long newKey = oIR.GetKeyPressed();
        if (newKey == KEY_RETURN)
        {
            ledcWrite(rpwmChannel, 0);
            ledcWrite(lpwmChannel, 0);

            ShowDisplay(SC_MAIN, ' ', ' ');
            return; 
        }

        DiffTurn(TargetAngle);
        delay(500); 
        SeekDrive(TargetDist);
        // Scanning stuff 
        UpdateDisplay2((int)TargetDist, (int)TargetAngle);
    }    
}
//
//  Option3 code
//
void Option3(char optionstate, char keypressed, bool initdisplay){
  if(optionstate == 'A'){
    char text[20];
    int angle = 0;
    int distance = 0;

    oLCD.LCDInitialize(LANDSCAPE, initdisplay);
    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
   
    sprintf(text, "ANGLE");
    oLCD.print(text, 10, 10);

    sprintf(text, "DEGREES");
    oLCD.print(text, 92, 10);
     
    sprintf(text, "DISTANCE");
    oLCD.print(text, 24, 30);

    sprintf(text, "CM");
    oLCD.print(text, 116, 30);
   

    //clearText(60, 10, 10, RCB_BLACK);
    oLCD.setBackColor(RCB_BLACK);
    oLCD.printNumI(angle, 60, 10, 3,' ');

    //clearText(88, 30, 10,RCB_BLACK);
    oLCD.setBackColor(RCB_BLACK);
    oLCD.printNumI(distance, 88, 30, 4, ' ');

    rOuter = (xScreen - mScreen)/2.0;
    xC = xScreen / 2;
    yC = yScreen;
    rInner = 30;
    radarFrame();
    degreeIncrements();
    }
    return;
}

//
//  Option4 code
//
void Option4(char optionstate, char keypressed, bool initdisplay){
    
}
// *************************************************************************************
// END: USER FUNCTIONS
// *************************************************************************************
// *************************************************************************************
// BEGIN: CORE FUNCTIONALITY
// *************************************************************************************
//
//  Setup(): Perform initialization
//
void setup()
{
    Serial.begin(115200);
    // Initialize IR Communications
    oIR.IRInitialize();
    // Show main menu when Arduino starts
    // for the 1st time or on reset.
    ShowDisplay(SC_MAIN, ' ', ' ', true);

    ledcSetup(lpwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(lenenablepwm, lpwmChannel);
    ledcWrite(lpwmChannel, 0);
    ledcSetup(rpwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(renablepwm, rpwmChannel);
    ledcWrite(rpwmChannel, 0);
    pinMode(lphasedir, OUTPUT);
    pinMode (rphasedir, OUTPUT);

    pinMode(tiltpin, OUTPUT);
    pinMode(servopin, OUTPUT);
    servowrite(tiltpin, 15, 10);
    servowrite(servopin, 90, 10);
    
    //doing pinmode for steps 11-16
    pinMode(lencoder, INPUT_PULLDOWN);
    pinMode(rencoder, INPUT_PULLDOWN);
    pinMode(sw1, INPUT_PULLUP);
    pinMode(sw2, INPUT_PULLUP);
    pinMode(sw3, INPUT_PULLUP);
    pinMode(sw4, INPUT_PULLUP);
    pinMode(tiltpin, OUTPUT);
    pinMode(servopin, OUTPUT);
    pinMode(ECHOpin, INPUT);
    pinMode(TRIGpin, OUTPUT);
    
    //used to count every pulse from each wheels encoder. Used to check distance traveled
    attachInterrupt(digitalPinToInterrupt(lencoder), LeftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(rencoder), RightEncoderISR, RISING);

    // Get width of TFT display in pixels
    xScreen = oLCD.getViewportWidth();
    // Get height of TFT display in pixels
    yScreen = oLCD.getViewportHeight();
}

//
//  Loops(): Perform looping operations
//
void loop()
{
    Serial.print(leftEncoderCount);
    Serial.print(" ");
    Serial.println(rightEncoderCount);

    unsigned long last_key_processed;
    // Determine which key was pressed.
    last_key_processed = oIR.GetKeyPressed();
    // Return to the main menu any time the
    // menu key is pressed.
    if (last_key_processed == KEY_MENU)
    {
        ShowDisplay(SC_MAIN, ' ', ' ');
    }
    else
    {
        if (oLCD.SCREEN_STATE == SC_MAIN)
        {
            if (last_key_processed == KEY_1)
            {
                ShowDisplay(SC_SUB1, 'A', ' ',false); 
            }
            else if (last_key_processed == KEY_2)
            {
                ShowDisplay(SC_SUB2, 'A', ' ',false); 
            }
            else if (last_key_processed == KEY_3)
            {
                ShowDisplay(SC_SUB3, 'A', ' ',false); 
            }
            else if (last_key_processed == KEY_4)
            {
                ShowDisplay(SC_SUB4, 'A', ' ',false); 
            }
        }
        if (oLCD.SCREEN_STATE == SC_SUB1)
        {
            // If the user presses one of the numeric keys, then display the key in the correct place on the screen.
            if ((last_key_processed == KEY_0  || last_key_processed == KEY_1  || last_key_processed == KEY_2  || last_key_processed == KEY_3  || last_key_processed == KEY_4  ||
                last_key_processed == KEY_5  || last_key_processed == KEY_6  || last_key_processed == KEY_7  || last_key_processed == KEY_8  || last_key_processed == KEY_9 ||
                last_key_processed == KEY_BACK) && oLCD.OPTION_STATE == 'A')  
            {
                ShowDisplay((screen)oLCD.SCREEN_STATE,'A',oIR.TranslateKey(last_key_processed),false);  
            }
            else if (last_key_processed== KEY_RETURN) 
            {
                ShowDisplay((screen)oLCD.SCREEN_STATE,'B',oIR.TranslateKey(last_key_processed), false);  
            }
        }
        else if (oLCD.SCREEN_STATE == SC_SUB2)
        {
            Serial.print("Sub2 trace = ");Serial.println(last_key_processed);
            if (last_key_processed == KEY_RETURN)
            {
                ShowDisplay(SC_MAIN, ' ', ' ');
            }
            else{
                ShowDisplay(SC_SUB2, ' ', oIR.TranslateKey(last_key_processed),false); 
            }
        }
        else if (oLCD.SCREEN_STATE == SC_SUB3)
        {
            Serial.print("Sub3 trace = ");Serial.println(last_key_processed);
            if (last_key_processed == KEY_RETURN)
            {
                ShowDisplay(SC_MAIN, ' ', ' ');
            }
            else{
                ShowDisplay(SC_SUB3, ' ', oIR.TranslateKey(last_key_processed),false); 
            }
        }
        else if (oLCD.SCREEN_STATE == SC_SUB4)
        {
            if (last_key_processed == KEY_RETURN)
            {
                ShowDisplay(SC_MAIN, ' ', ' ');
            }
            else{
                ShowDisplay(SC_SUB4, ' ', oIR.TranslateKey(last_key_processed),false); 
            }
        }
    }
    delay(1000);
}
//
//  Method to display menus on TFT display
//
void ShowDisplay(screen val, char optionstate, char keypressed, bool initdisplay)
{
    // Store screen state.
    oLCD.SCREEN_STATE = val;
    oLCD.OPTION_STATE = optionstate;
    int selectedpid = 0;
   
    if (val == SC_MAIN)
    {
         //
        //  Initialize the display
        //
        oLCD.LCDInitialize(LANDSCAPE, initdisplay);
        oLCD.print("SELECT OPTION", 10, 10);
        oLCD.print("1. CALIBRATE DRIVE", 10, 24);
        oLCD.setTextColor(RCB_RED); 
        oLCD.print("2. SEEK TARGET", 10, 38);
        oLCD.setTextColor(RCB_WHITE); 
        oLCD.print("3. LOCATE TARGET", 10, 52);
        oLCD.setTextColor(RCB_RED);
        oLCD.print("4. NAVIGATE", 10, 66);
    }
    if (val == SC_SUB1)
    {
         Serial.println("option1");
         Option1(optionstate, keypressed, initdisplay);
    }
    else if (val == SC_SUB2)
    {
        //Serial.println("option2");
        //Option2(optionstate, keypressed, initdisplay);
    }
    else if (val == SC_SUB3)
    {
        Serial.println("option3");
        Option3(optionstate, keypressed, initdisplay);
    }
    else if (val == SC_SUB4)
    {
        //Serial.println("option4");
        //Option4(optionstate, keypressed, initdisplay);
    }
}
// *************************************************************************************
// BEGIN: UTILITY FUNCTIONS
// *************************************************************************************
//
//  Function to move the servo to the specified angle in degrees
//
void servowrite(int pin, int angle, int numpulses)
{
    //  Move servo attached to pin to the specified angle.
    //  The numpulses shoul be set to approximately 3 for 
    //  an angular movement of 2 or less degrees.  For a large
    //  angular change numpulses should be set to 10.
    //  Set the pulse height in microseconds associated
    //  with the servo angles of 0 and 180 degrees
    //
    int microzero=500;
    int micro180=2400;
    int lenMicroSecondsOfPeriod = 20 * 1000;
    // pin for servo, angle to move, microseconds for zero degrees, microseconds for 180 degrees
    int lenMicroSecondsOfPulse = map(angle, 0, 180, microzero, micro180); // map angle to microseconds
    // Loop long enough for servo to see enough of waveform to fully 
    // move to the new position.
    for (int i = 0; i<numpulses; i++){
        // Servos work by sending a 20 ms pulse.  
        // 1.0 ms at the start of the pulse will turn the servo to the 0 degree position
        // 1.5 ms at the start of the pulse will turn the servo to the 90 degree position 
        // 2.0 ms at the start of the pulse will turn the servo to the 180 degree position 
        // Turn voltage high to start the period and pulse
        digitalWrite(pin, HIGH);

        // Delay for the length of the pulse
        delayMicroseconds(lenMicroSecondsOfPulse);

        // Turn the voltage low for the remainder of the pulse
        digitalWrite(pin, LOW);

        // Delay this loop for the remainder of the period so we don't
        // send the next signal too soon or too late
        delayMicroseconds(lenMicroSecondsOfPeriod - lenMicroSecondsOfPulse); 
    }
}
//
// Function for calculating the distance measured by the Ultrasonic sensor
//
int calculateDistance(int tPin, int ePin)
{
    // Variables for the duration and the distance
    long duration;
    int distance;
    digitalWrite(tPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(tPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(tPin, LOW);
    duration = pulseIn(ePin, HIGH, 15000); // Reads the echoPin, returns the sound wave travel time in microseconds
    distance = duration * 0.034 / 2;
    return distance;
}
//
//  Code to display y axis label (rotated 90 degrees)
//
void DisplayRotatedText(int x, int y){
    oLCD.setPivot(x/2, y/2);   // Set pivot to middle of TFT screen
    // Create the Sprite
    spr.setColorDepth(16);      // Create an 8bpp Sprite of 60x30 pixels
    spr.createSprite(124, 30);  // 8bpp requires 64 * 30 = 1920 bytes
    //
    // Set pivot relative to top left corner of Sprite (increasing y will push the text
    // closer to the left hand side of the screen.  Increasing x will move the label up and down)
    spr.setPivot(57, 85);      
    spr.fillSprite(RCB_PURPLE); // Fill the Sprite with black

    spr.setTextColor(RCB_WHITE);        // Green text
    spr.setTextDatum(MC_DATUM);         // Middle centre datum
    spr.drawString("speed (rpm)", 60, 15, 2); // Plot text, font 4, in Sprite at 12, 15

    // rotate text on screen by 270 degrees
    spr.pushRotated(270);

    spr.deleteSprite();
}
// *************************************************************************************
// END: UTILITY FUNCTIONS
// *************************************************************************************
// *************************************************************************************
// START: OPTION1 SUPPORT
// *************************************************************************************
//adds 1 to a count specific to the side per pulse
void LeftEncoderISR(){
    unsigned long nowL = micros();
    if(nowL - lastleftpulse >= 3000){
        leftEncoderCount++;
        lastleftpulse = nowL;

    }
  
}
void RightEncoderISR(){
    unsigned long nowR = micros();
    if(nowR - lastrightpulse >= 3000){
        rightEncoderCount++;
        lastrightpulse = nowR;
    }

}

void UpdateDisplay(int Rduty, int LenC, int RenC, int Error)
{
    clearText(CENTER, 24, xScreen - 6, RCB_PURPLE);
    oLCD.printNumI(Rduty, CENTER, 24, 2, ' ');
    clearText(CENTER, 52, xScreen - 6, RCB_PURPLE);
    oLCD.printNumI(LenC, CENTER, 52, 5, '0');
    clearText(CENTER, 80, xScreen - 6, RCB_PURPLE);
    oLCD.printNumI(RenC, CENTER, 80, 5, '0');
    clearText(CENTER, 108, xScreen - 6, RCB_PURPLE);
    oLCD.printNumI(Error, CENTER, 108, 3, '0');
}
// *************************************************************************************
// END: OPTION1 SUPPORT
// *************************************************************************************

// *************************************************************************************
// START: OPTION2 SUPPORT
// *************************************************************************************
void UpdateDisplay2(int dist, int angle)
{   
    char text[20]; 

    // Prints words to screen
    sprintf(text, "TARGET DIST");
    oLCD.print(text, CENTER, 38);
    
    oLCD.printNumF(dist, 1, CENTER, 54); 
    sprintf(text, "CM");
    oLCD.print(text, CENTER, 54);

    sprintf(text, "TARGET ANGLE");
    oLCD.print(text, CENTER, 70);
    
    oLCD.printNumF(angle, 1, CENTER, 86); 
    sprintf(text, "DEGREES");
    oLCD.print(text, CENTER, 86);
}

void DiffTurn(float angle) // turn if angle of robot +/- 10 degrees of centerline 
{ 
    // resets Encoder Counts 
    leftEncoderCount = 0; 
    rightEncoderCount = 0; 

    // Turn counts calculation 
    int counttoturn = (abs(90 - angle))/10; 
    
    if (angle > 90) // Left turn 
    { 
        // Left motor backwards right motor forwards 
        digitalWrite(lphasedir, LOW); 
        digitalWrite(rphasedir, HIGH); 
    } 
    else if (angle < 90) // Right turn 
    { 
        // Left motor forwards right motor backwards 
        digitalWrite(lphasedir, HIGH); 
        digitalWrite(rphasedir, LOW); 
    } 
    
    // Set 30% dutycycle, stop when count to turn is reached
    ledcWrite(lpwmChannel, 30.0 * maxDutyCycle / 100.0); 
    ledcWrite(rpwmChannel, 30.0 * maxDutyCycle / 100.0); 
    if (leftEncoderCount == counttoturn) 
    { 
        ledcWrite(lpwmChannel, 0); 
        ledcWrite(rpwmChannel, 0); 
    } 
} 

void SeekDrive(float dist) 
{
    int counttotarget = 0;
    float Newdist = 0.0;  
    double circumference = pivalue * WDIA;

    // Set both motors forward
    digitalWrite(lphasedir, HIGH); 
    digitalWrite(rphasedir, HIGH);

    // Using incremental distance of 20cm
    if (dist > 22.0)
    {
        Newdist = (dist - 2)/2; 
    } 
    else if (dist <= 22.0)
    {
        // Once in range find true distance within 2cm of object
        Newdist = dist - 2;
    }

    // Define counts until target using Newdist
    counttotarget = (((double)Newdist * (double)CPR) / (double)circumference);

    // Set duty cycle to 60%, move until count target is reached
    ledcWrite(lpwmChannel, 60.0 * maxDutyCycle / 100.0); 
    ledcWrite(rpwmChannel, 60.0 * maxDutyCycle / 100.0);
    
    while (leftEncoderCount < counttotarget)
    { 
        DriveError = (long)leftEncoderCount - (long)rightEncoderCount * 0.77; 
            
        //Adjusting right duty cycle for error 
        rdutyStraight = 60.0 + (0.75 * (double)DriveError );

        //Failsafe duty cycle fix 
        if(rdutyStraight > 70.0)
        {
            rdutyStraight = 70.0; 
        } 
        else if (rdutyStraight < 40.0)
        {
            rdutyStraight = 40.0; 
        }

        // reset dutycycle
        ledcWrite(rpwmChannel, (float)rdutyStraight * maxDutyCycle / 100.0);
        delay(20);        
    }
    
    // Stop motors when rover is 2cm from target
    ledcWrite(lpwmChannel, 0); 
    ledcWrite(rpwmChannel, 0);
}
// *************************************************************************************
// END: OPTION2 SUPPORT
// *************************************************************************************
// *************************************************************************************
// START: OPTION3 SUPPORT
// *************************************************************************************
//Draw radar lines based on measured angle and distance
void drawRay(int angle, int distance)
{
    float rad = angle * pivalue / 180.0;  //convert angle from degrees to radians for trig functions
    float rLimit = rOuter - 2;              // Maximum radius of the radar display
    float rScaled = distance * rLimit / 120.0;  // scale measured distance (0-120cm) to radar radius in pixels

    int xMid = xC + rScaled * cos(rad);    
    int yMid = yC - rScaled * sin(rad);
    int xEnd = xC + rLimit * cos(rad);
    int yEnd = yC - rLimit * sin(rad);
    if (distance >= 120){
        oLCD.drawLine(xC, yC, xEnd, yEnd, RCB_BLUE);
    }
    else{
        oLCD.drawLine(xC, yC, xMid, yMid, RCB_BLUE);  //blue portion
        oLCD.drawLine(xMid, yMid, xEnd, yEnd, RCB_RED); //Red portio
    }
}
// Get and validate distance measurement from ultrasonic sensor
int getDistance()                              
{
    int d = calculateDistance(TRIGpin, ECHOpin);

    // limit invalid readings to maximum radar range
    if (d <= 0 || d > 120)
    {
        d = 120;
    }
    return d;
}
// sweep servo from left to right and back while update radar display
void degreeIncrements()
{
    theta_f = 0;
    theta_r = 0;
    targetFound = false;
    resetSweep();
    //forward sweep
    for (int angle = 15; angle <= 165; angle = angle + 2)
    {
        servowrite(tiltpin, 0, 10);
        servowrite(servopin, angle, 3);
        delay(20);
        int distance = getDistance();
       
        if (distance < 120){
            if(measurementCount == 0 || angle != previousAngle + 2 || abs(distance - (totalDistance / measurementCount)) > 10){
                saveClosestGroup();
                resetGroup(angle);
            }

            Serial.print("Angle: ");
            Serial.print(angle);
            Serial.print(" Distance: ");
            Serial.println(distance);

            endAngle = angle;
            totalDistance = totalDistance + distance;
            measurementCount++;
            previousAngle = angle;
            previousDistance = distance;
        }
        clearText(60, 10, 3, RCB_BLACK);
        oLCD.setBackColor(RCB_BLACK);
        oLCD.printNumI(angle, 60, 10, 3, ' ');
       
        clearText(88, 30, 3, RCB_BLACK);
        oLCD.setBackColor(RCB_BLACK);
        oLCD.printNumI(distance, 88, 30, 3, ' ');
        drawRay(angle, distance);
    }

    saveClosestGroup(); //check the final group
    delay(100);
    if (closestCount >= 3){
        theta_f = closestAngle;  //need to change calc
        d_f = closestDistance;    //need to change calc
    }
    clearRadar();
   
    resetSweep();

    // reverse sweep
    for (int angle = 165; angle >= 15; angle = angle - 2)
    {
        servowrite(tiltpin, 0, 10);
        servowrite(servopin, angle, 3);
        delay(20);
        int distance = getDistance();

        if (distance < 120){
            if(measurementCount == 0 || angle != previousAngle - 2 || abs(distance - (totalDistance / measurementCount)) > 10){
                saveClosestGroup();
                resetGroup(angle);
            }

            Serial.print("Angle: ");
            Serial.print(angle);
            Serial.print(" Distance: ");
            Serial.println(distance);
           
            endAngle = angle;
            totalDistance = totalDistance + distance;
            measurementCount++;
            previousAngle = angle;
            previousDistance = distance;
        }
       
        clearText(60, 10, 3, RCB_BLACK);
        oLCD.setBackColor(RCB_BLACK);
        oLCD.printNumI(angle, 60, 10, 3, ' ');
       
        clearText(88, 30, 3, RCB_BLACK);
        oLCD.setBackColor(RCB_BLACK);
        oLCD.printNumI(distance, 88, 30, 3, ' ');
        drawRay(angle, distance);
    }

    saveClosestGroup();

    if (closestCount >= 3){
        theta_r = closestAngle;
        d_r = closestDistance;
    }

    delay(100);

    calculateTarget();       // calculate and save final target

    servowrite(servopin, 90, 10);
    oLCD.setTextColor(RCB_WHITE, RCB_BLACK);

   
    displayTarget();                  //display final target angle

}

// Clear radar area and redraw radar frame
void clearRadar()
{
    char text[20];
   
    for(int r = 1; r < rOuter; r++){        
    oLCD.drawCircle(xC, yC, r, RCB_PURPLE);
    }    
    radarFrame();

    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);
   
    sprintf(text, "ANGLE");
    oLCD.print(text, 10, 10);

    sprintf(text, "DEGREES");
    oLCD.print(text, 92, 10);
     
    sprintf(text, "DISTANCE");
    oLCD.print(text, 24, 30);

    sprintf(text, "CM");
    oLCD.print(text, 116, 30);
}

// Draw a white reference line at the specified angles
void drawWhiteLine(int angle){
    float rad = angle * pivalue / 180.0;      //scale cm to pixels
    int xEnd = xC + rOuter * cos(rad);        //Object x-locatoin
    int yEnd = yC - rOuter * sin(rad);        //object y-location

    oLCD.drawLine(xC, yC, xEnd, yEnd, RCB_WHITE);
}
//calculate final target angle and distance from both sweeps
void calculateTarget(){
    if (theta_f > 0 && theta_r > 0){
        theta_target = (theta_f + theta_r) / 2.0;
        d_target = (d_f + d_r) / 2.0;
        targetFound = true;
    }
    else{
        targetFound = false;
    }
}
//draw radar circles and white reference lines
void radarFrame(){
    oLCD.drawCircle(xC, yC, rOuter, RCB_WHITE);
    oLCD.drawCircle(xC, yC, rInner, RCB_WHITE);

    drawWhiteLine(30);
    drawWhiteLine(60);
    drawWhiteLine(90);
    drawWhiteLine(120);
    drawWhiteLine(150);
}
//reset measurements when starting a new object group
void resetGroup(int angle){
    startAngle = angle;
    endAngle = angle;
    totalDistance = 0;
    measurementCount = 0;
}
//saves the closest valid object group detected
void saveClosestGroup(){
     if (measurementCount >= 3){
            // calculate the average distance of the group
            float groupDistance = totalDistance / measurementCount;
            //check if this group is closer that the previous closest grou
            if (groupDistance < closestDistance){
                closestDistance = groupDistance;
                closestAngle = (startAngle + endAngle) / 2.0;
                closestCount = measurementCount;
                closestStartAngle = startAngle;
                closestEndAngle = endAngle;
        }
    }
}
//Reset all measurments before starting a new sweep
void resetSweep(){
    startAngle = 0;
    endAngle = 0;
    totalDistance = 0;
    measurementCount = 0;

    //reset previouse measurements
    previousAngle = 0;
    previousDistance = 120;

    closestDistance = 120;
    closestAngle = 0;
    closestCount = 0;
    closestStartAngle = 0;
    closestEndAngle = 0;
}
//display final target angle and distance on LCD
void displayTarget(){

    DrawBox(12, 50, 130, 70, RCB_BLACK);    //black box

    oLCD.setTextColor(RCB_WHITE, RCB_BLACK);

    if (targetFound == true){
        oLCD.print("TARGET", 40, 58);

        oLCD.printNumI((int)theta_target, 25, 78, 3, ' ');
        oLCD.print("DEGREES", 55, 78);

        oLCD.printNumI((int)d_target, 40, 98, 3, ' ');
        oLCD.print("CM", 70, 98);
    }
    else{
        oLCD.print("TARGET ", 35, 68);
        oLCD.print("NOT FOUND", 20, 88);
    }
}
//draws a filled rounded rectangle on LCD
void DrawBox(int x1, int y1, int x2, int y2, uint32_t color)
{
    oLCD.fillRoundRect(x1, y1, x2, y2, 3, color);
}

// *************************************************************************************
// END: OPTION3 SUPPORT
// *************************************************************************************

// *************************************************************************************
// START: OPTION4 SUPPORT
// *************************************************************************************

void ShowTarget(){
    char text[20];

    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);

    sprintf(text, "TARGET DIR");
    oLCD.print(text, CENTER, 15);
   
    oLCD.printNumF(theta_target, 1, CENTER, 31);

    sprintf(text, "DEGREES");
    oLCD.print(text, 90, 31);

    sprintf(text, "TARGET DIST");
    oLCD.print(text, CENTER, 47);

    oLCD.printNumF(d_target, 1, CENTER, 63);

    sprintf(text, "CM");
    oLCD.print(text, 110, 63);
}
void currentDirection(){
    char text[30];

    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);

    sprintf(text, "TARGET DIR");
    oLCD.print(text, CENTER, 15);

    oLCD.printNumF(theta_target, 1, CENTER, 31);

    sprintf(text, "DEGREES");
    oLCD.print(text, 90, 31);

    sprintf(text, "TARGET DIST");
    oLCD.print(text, CENTER, 47);

    oLCD.printNumF(d_target, 1, CENTER, 63);

    sprintf(text, "CM");
    oLCD.print(text, 110, 63);

    sprintf(text, "CURRENT DIR");
    oLCD.print(text, CENTER, 79);

    oLCD.printNumF(theta_current, 1, CENTER, 95);

    sprintf(text, "DEGREES");
    oLCD.print(text, 90, 95);
}
void distanceTravelled(){
    char text[30];

    oLCD.setTextColor(RCB_WHITE, RCB_PURPLE);

    sprintf(text, "TARGET DIR");
    oLCD.print(text, CENTER, 15);

    oLCD.printNumF(theta_target, 1, CENTER, 31);

    sprintf(text, "DEGREES");
    oLCD.print(text, 90, 31);

    sprintf(text, "TARGET DIST");
    oLCD.print(text, CENTER, 47);

    oLCD.printNumF(d_target, 1, CENTER, 63);

    sprintf(text, "CM");
    oLCD.print(text, 110, 63);

    sprintf(text, "CURRENT DIR");
    oLCD.print(text, CENTER, 79);

    oLCD.printNumF(theta_current, 1, CENTER, 95);

    sprintf(text, "DEGREES");
    oLCD.print(text, 90, 95);


    sprintf(text, "CURRENT DIST");
    oLCD.print(text, CENTER, 111);

    oLCD.printNumF(d_current, 1, CENTER, 127);

    sprintf(text, "CM");
    oLCD.print(text, 90, 127);
}

void DrivetoPivot(){
    double circumference = pivalue * WDIA;      //pi * wheel diameter(called in option #1)
    unsigned long counttopivot = (pivotOffset * CPR) / circumference + 0.5;  //calculate encoder counts needed to move the robot the pivot offset distance

    // Reset both wheel encoders
    leftEncoderCount = 0;
    rightEncoderCount = 0;

    // set both motors to forward direction
    digitalWrite(lphasedir, HIGH);
    digitalWrite(rphasedir, HIGH);

    // run both motors with duty cycle of 30%
    ledcWrite(lpwmChannel, 30 * maxDutyCycle / 100);
    ledcWrite(rpwmChannel, 30 * maxDutyCycle / 100);

    //continue moving until left encoder reaches target count
    while(leftEncoderCount < counttopivot){                //if right encoder fixed then add another &&
        delay(1);
    }
    //stop both motors
    ledcWrite(lpwmChannel, 0);
    ledcWrite(rpwmChannel, 0);

    ShowTarget();

    WaitforSwitch(sw1);
}

//wait until the switch is pressed before continuing-only waits for the switch to read LOW does not wait for you to relase the switch afterward
void WaitforSwitch(int pin){
    while(digitalRead(pin) == HIGH){
        delay(10);
    }
}
void TurnDebtoTarget(){
    //Calculate angle robot needs to turn from 90 degrees
    double turnAngle = theta_target - 90.0;
    double circumference = pivalue * WDIA;
    double turnDistance = (pivalue * wheelBase * abs(turnAngle)) / 360.0;
    unsigned long counttoturn = (d_target * CPR) / circumference + 0.5;  
    //Reset wheel encoder counts
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    // check which direction robot needs to turn
    if(turnAngle > 0){                               // 90-165 degrees
        digitalWrite(lphasedir, LOW);
        digitalWrite(rphasedir, HIGH);
    }
    else if(turnAngle < 0){                            // 15-90 degrees
        //Turn clockwise
        digitalWrite(lphasedir, HIGH);
        digitalWrite(rphasedir, LOW);
    }
    else{
        theta_current = 90.0;
        currentDirection();
        WaitforSwitch(sw2);
        return;
    }
    // run both motors with duty cycle of 30%
    ledcWrite(lpwmChannel, 30 * maxDutyCycle / 100);
    ledcWrite(rpwmChannel, 30 * maxDutyCycle / 100);

    while(leftEncoderCount < counttoturn){                //if right encoder fixed then add another &&
        delay(1);
    }

    //stop both motors
    ledcWrite(lpwmChannel, 0);
    ledcWrite(rpwmChannel, 0);

    double actualTurnDistanc = leftEncoderCount * circumference / 360.0;
    double actualTurnAngle = actualTurnDistance * 360.0 / (pivalue * wheelBase);    //converts wheel travel into degrees turned

    // calc actual robot direction
    if(turnAngle < 0){                                   //15-90 degrees
        theta_current = 90.0 - actualTurnAngle;
    }
    else{
        theta_current = 90.0 + actualTurnAngle;             //90-165 degrees
    }

    currentDirection();
    WaitforSwitch(sw2);


}

//driving straight
void MoveDebtoTarget(){
    //calculate encoder counts needed to travel target distance
    double circumference = pivalue * WDIA;                          
    unsigned long counttotarget = (d_target * CPR) / circumference + 0.5;

    //Reset wheel encoder counts
    leftEncoderCount = 0;
    rightEncoderCount = 0;

    //set both motors to forward direction
    digitalWrite(lphasedir, HIGH);  //left motor's direction
    digitalWrite(rphasedir, HIGH);  //right motor's direction

    // start motors using saved duty cycles
    double rduty = rdutyStraight;
    ledcWrite(lpwmChannel, storedDuty * maxDutyCycle / 100.0);
    ledcWrite(rpwmChannel, rduty * maxDutyCycle / 100.0);

    // continue driving until target encoder count is reached
    while(leftEncoderCount < counttotarget){
        delay(1);
    }
    //stop both motors after reaching target
    ledcWrite(lpwmChannel,0);
    ledcWrite(rpwmChannel, 0);
     //calculates average wheel encoder count
    d_current = leftEncoderCount * circumference / CPR;
    distanceTravelled();
    WaitforSwitch(sw3);

}

// *************************************************************************************
// END: OPTION4 SUPPORT
// *************************************************************************************
