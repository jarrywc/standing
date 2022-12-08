#include <Adafruit_PCF8575.h>
#include <LiquidCrystal_I2C.h>
#include "EspMQTTClient.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <Preferences.h>

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

Preferences preferences;

bool startup = false;
// Uses GPIO22 - pin 22 SCL Yellow
// Uses GPIO21 - pin 21 SDA Orange
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x3F, 16 column and 2 rows
const int INT_PIN = 34;

// ****** All of the Pin Assignments ****** //
const int TRIG_PIN = 32; // ESP32 pin GIO connected to Ultrasonic Sensor's TRIG pin
const int ECHO_PIN = 33; // ESP32 pin GIO connected to Ultrasonic Sensor's ECHO pin
// -
const int PWR_RELAY_PIN = 5; // ESP32 pin GPIO connected to Relay's pin
const int RELAY_PIN_A = 16;  // 8
const int RELAY_PIN_B = 17;  // 9
const int RELAY_PIN_C = 18;  // 11
const int RELAY_PIN_D = 19;  // 12
const int INT_PIN = 34;
// const int ADC_PIN_A = 34;
// const int ADC_PIN_B = 35;
// const int ADC_PIN_C = 36;
// const int ADC_PIN_D = 39;

// ****** TIME CONSTANTS ****** //

const int SHORT_PRESS_TIME = 500;
const int LONG_PRESS_TIME = 1500;
const int BACKLIGHT_TIME = 25000;
const int MOTION_TIMEOUT = 30000;

// ****** MOTION CONSTANTS ******
// https://www.officedepot.com/a/products/358370/Realspace-Magellan-Performance-Electric-Height-Adjustable/?utm_source=google&utm_medium=cpc&mediacampaignid=71700000099550481_18179863869&gclid=CjwKCAjw7p6aBhBiEiwA83fGuuWQJbjduPBVZJZcTQxuU4zBx_wC8xQE1wPZpYFUHOfNjAhxn38cuRoCyR8QAvD_BwE&gclsrc=aw.ds#Specs
const int DISTANCE_THRES_LOW = 75;        // LOWEST the desk can go
const int DISTANCE_THRES_HIGH = 116;      // HIGHEST the desk can go
const float DISTANCE_THRES_OFFSET = 8.89; // 8.26// Height of the control box with sensor
const int DISTANCE_PRESET_LOW = 80;
const int DISTANCE_PRESET_HIGH = 110;
const int DISTANCE_RANGE = DISTANCE_THRES_HIGH - DISTANCE_THRES_LOW;

// ****** ACTION VARIABLES ******
// MQTT included
const int STOPPED_STATE = 0;
const int DECREASING_STATE = 1;
const int INCREASING_STATE = 2;
// Non-MQTT -> Nav related
const int RESET = 4;

// ****** BUTTON VARIABLES ******

Adafruit_PCF8575 pcf;

#define DEBOUNCE_TIME 600

bool pcfActive = false;
bool pinsActive = false;
bool pinsNow[4] = {false};
bool pinsLastFlickerState[4] = {false};
bool pinsLastSteadyState[4] = {false};
unsigned long lastDebounceTime[4] = {0};

// ****** PRINT LCD VARIABLES ******

// ****** MOTION VARIABLES ******

float duration_us, distance_cm;
bool backlightActive = false;
unsigned long backlightTimeOn = 0;
bool motionTimeout = false;
unsigned long motionTimeOff = 0;
int currentPosition = 0;
int lastCurrPosition = 0;
int targetPosition = 0;
int lastTargetPosition = 0;
int positionState = STOPPED_STATE;

// ****** WIFI/MQTT VARIABLES ******

EspMQTTClient client(
    "Flores Estate",
    "N0C0unt1ng!",
    "192.168.1.24", // MQTT Broker server ip
    "desk",         // Can be omitted if not needed
    "D3sk1",        // Can be omitted if not needed
    "DeskBase",     // Client name that uniquely identify your device
    1883            // The MQTT port, default to 1883. this line can be omitted
);

// Target position state changes
char *decV = "DECREASING";
char *incV = "INCREASING";
char *stopV = "STOPPED";

// These are Centimeters Measured
// int cmTargetPosition = 50;
// int cmCurrentPositon = 50;
char *positionStateValues = stopV; // "decreasing-value", "increasing-value" are the other

bool positonStateChange = false;

// ***** Queues *****
static QueueHandle_t lcd_print_queue;
static const int lcd_print_queue_len = 3;

static QueueHandle_t target_queue;     // Pecentage values requested
static const int target_queue_len = 5; // Size of delay_queue

static QueueHandle_t serial_queue; // Pos/Neg time int values
static const int serial_queue_len = 5;

void getHeight()
{
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // measure duration of pulse from ECHO pin
    duration_us = pulseIn(ECHO_PIN, HIGH);
    // calculate the distance
    distance_cm = 0.017 * duration_us;
    // record last currentPostion
    lastCurrPosition = currentPosition;
    // return with cm height
    currentPosition = (int)roundTo(distance_cm + DISTANCE_THRES_OFFSET);
    Serial.print('Height is ');
    Serial.println(currentPosition);
}
int roundTo(float num)
{
    int r = ceil(num);
    return (int)r;
}
int percentToCM(int percent)
{
    if (percent <= 100 || percent >= 0)
    {
        float centi = ((percent * DISTANCE_RANGE / 100) + DISTANCE_THRES_LOW);
        ;
        return (int)centi;
    }
    return -1;
}
int cmToPercent(float centimeters)
{
    float perc = (centimeters - DISTANCE_THRES_LOW) / DISTANCE_RANGE;
    return (int)perc;
}

// char * percentToCharArr(int){
//   mqCurrentPositon = cmToPercent(currentPosition);
//     char current[4];
//     itoa(mqCurrentPositon, current, 10);
// }

void onConnectionEstablished()
{

    client.subscribe("desk/set/position/target", [](const String &payload)
                     {
                         Serial.println(payload + " %");
                         const char *str = payload.c_str();
                         int requested = atoi(str);
                         if (xQueueSend(target_queue, (void *)&requested, 10) != pdTRUE)
                         {
                             Serial.print("Couldn't Send to Queue");
                         }
                     });

    // Publish the confirmed changes here - including any manual button pressing
    client.publish("desk/status", "Desk Online"); // You can activate the retain flag by setting the third parameter to true

    // if(currentPosition!=lastCurrPosition||!startup){

    //   mqCurrentPositon = cmToPercent(currentPosition);
    //   char current[4];
    //   itoa(mqCurrentPositon, current, 10);
    //   Serial.print("Current Sent: ");
    //   Serial.println(currentPosition);
    //   client.publish("desk/get/position/current", current);
    //   client.publish("desk/get/position/state", positionStateValues);
    // }
}

void up()
{
    positionStateValues = incV;
    switch (positionState)
    {
    case STOPPED_STATE:
        digitalWrite(RELAY_PIN_A, LOW);
        digitalWrite(RELAY_PIN_B, LOW);
        digitalWrite(RELAY_PIN_C, LOW);
        digitalWrite(RELAY_PIN_D, LOW);
        delay(10);
        digitalWrite(PWR_RELAY_PIN, HIGH);
        positionState = INCREASING_STATE;
        break;
    case DECREASING_STATE:
        digitalWrite(PWR_RELAY_PIN, LOW);
        delay(10);
        digitalWrite(RELAY_PIN_A, LOW);
        digitalWrite(RELAY_PIN_B, LOW);
        digitalWrite(RELAY_PIN_C, LOW);
        digitalWrite(RELAY_PIN_D, LOW);
        delay(10);
        digitalWrite(PWR_RELAY_PIN, HIGH);
        positionState = INCREASING_STATE;
        break;
    }
}
void down()
{
    positionStateValues = incV;
    switch (positionState)
    {
    case STOPPED_STATE:
        digitalWrite(RELAY_PIN_A, HIGH);
        digitalWrite(RELAY_PIN_B, HIGH);
        digitalWrite(RELAY_PIN_C, HIGH);
        digitalWrite(RELAY_PIN_D, HIGH);
        delay(10);
        digitalWrite(PWR_RELAY_PIN, HIGH);
        positionState = DECREASING_STATE;
        break;
    case INCREASING_STATE:
        digitalWrite(PWR_RELAY_PIN, LOW);
        delay(10);
        digitalWrite(RELAY_PIN_A, HIGH);
        digitalWrite(RELAY_PIN_B, HIGH);
        digitalWrite(RELAY_PIN_C, HIGH);
        digitalWrite(RELAY_PIN_D, HIGH);
        delay(10);
        digitalWrite(PWR_RELAY_PIN, HIGH);
        positionState = DECREASING_STATE;
        break;
    }
}
void stop()
{
    positionStateValues = stopV;
    digitalWrite(PWR_RELAY_PIN, LOW);
    delay(100);
    digitalWrite(RELAY_PIN_A, LOW);
    digitalWrite(RELAY_PIN_B, LOW);
    digitalWrite(RELAY_PIN_C, LOW);
    digitalWrite(RELAY_PIN_D, LOW);
    positionState = STOPPED_STATE;
}
void lcdInit()
{
    lcd.init();          // initialize the lcd
    lcd.backlight();     // open the backlight
    lcd.setCursor(0, 0); // move cursor to   (0, 0)
    lcd.print("Starting Desk...");
}
void displayHeight()
{
    // if(currentPosition!=lastCurrPosition||targetPosition!=lastTargetPosition)
    //   lcd.clear();
    // if(currentPosition!=lastCurrPosition){
    lcd.setCursor(0, 0); // start to print at the first row
    lcd.print("Height: ");
    lcd.print(currentPosition);
    lcd.print(" CM  ");
    //}
    // if(targetPosition!=lastTargetPosition){
    lcd.setCursor(0, 1); // start to print at the first row
    lcd.print("Target: ");
    lcd.print(targetPosition);
    lcd.print(" CM  ");
    //}
}
void awakeBacklight()
{
    // Make sure light is on
    if (!backlightActive)
    {
        // If not on, turn on then mark state and time
        lcd.backlight();
        backlightActive = true;
        backlightTimeOn = millis();
    }
}
void btnInit()
{
    if (!pcf.begin(0x20, &Wire))
    {
        Serial.println("Couldn't find PCF8575");
    }
    else
    {
        // Button PINs
        for (uint8_t p = 4; p < 8; p++)
        {
            pcf.pinMode(p, INPUT_PULLUP);
        }
        // Interrupt PIN
        pinMode(INT_PIN, INPUT_PULLUP);
        // Intuerrupt PIN Function
        attachInterrupt(INT_PIN, isr, CHANGE);
        pcfActive = true;
    }
}
void btnRefresh()
{
    if (!pcfActive)
    {
        // ButtonInit called since last attempt to connect failed
        btnInit();
    }
    else
    {
        for (uint8_t p = 4; p < 8; p++)
        {

            pinsNow[p] = (!pcf.digitalRead(p));
            if (pinsNow[p] != pinsLastFlickerState[p])
            {
                lastDebounceTime[p] = millis();
                pinsLastFlickerState[p] = pinsNow[p];
            }
            // Check for a steady press
            if ((millis() - lastDebounceTime[p]) > DEBOUNCE_TIME)
            {
                if (!pinsLastSteadyState[p] && pinsNow[p])
                {
                    Serial.print("The button ");
                    Serial.print(p);
                    Serial.println(" is pressed");
                    btnAction(p);
                }
            }
        }
    }
}
void btnAction(uint8_t btn)
{
    switch (btn)
    {
    case 0:
        Serial.println("Button 0");
        lastTargetPosition = targetPosition;
        targetPosition = currentPosition + 1;
        break;
    case 1:
        Serial.println("Button 1");
        lastTargetPosition = targetPosition;
        targetPosition = currentPosition - 1;
        break;
    case 2:
        awakeBacklight();
        Serial.println("Button 2");
        lastTargetPosition = targetPosition;
        targetPosition = DISTANCE_PRESET_LOW;
        break;
    case 3:
        Serial.println("Button 3");
        lastTargetPosition = targetPosition;
        targetPosition = DISTANCE_PRESET_HIGH;
        break;
    }
}

void deskMotionHeight()
{

    // Have we reached our destination already? If not, start heading there
    if (targetPosition != currentPosition && targetPosition != currentPosition + 1 && targetPosition != currentPosition - 1)
    {
        awakeBacklight();
        // Consider the case of the motion exceeding the limits
        // We should only move if we are inside the boundaries of the desk height ranges
        if (!(DISTANCE_THRES_LOW >= targetPosition || DISTANCE_THRES_HIGH <= targetPosition))
        {
            // Now we are in the safe zone to move per the requestest target position
            // The next consideration is how far can we safely move without exceeding
            // the targetPosition, this can be monitored by either
            // the time in each increment or the height as measured by the
            // ultrasonic sensor so we out to check the sensor height

            // Taking the ultrasonic sensor approach
            getHeight();
            // Update the display to show the height
            displayHeight();
            // Go down
            if (targetPosition < currentPosition)
            {
                motionTimeout = false;
                down();
            }
            // Go up
            else if (targetPosition > currentPosition)
            {
                motionTimeout = false;
                up();
            }
            else
            {
                stop();
                motionTimeOff = millis();
            }
            // IMPORTANT: this is the measure of how long the power is ON -> stop() is the only way out!!!
            delay(100);
        }
        else
        {
            // Reset the position to current since it's out of bounds
            targetPosition = currentPosition;
            stop();
            motionTimeOff = millis();
        }
    }
    else if (targetPosition == currentPosition + 1)
    {
        targetPosition = currentPosition + 1;
        stop();
        motionTimeOff = millis();
    }
    else if (targetPosition == currentPosition - 1)
    {
        targetPosition = currentPosition - 1;
        stop();
        motionTimeOff = millis();
    }
}

void t_deskMotionTime(void *parameters)
{
    int t = 0;
    bool neg = false;
    while (1)
    {
        if (xQueueReceive(serial_queue, (void *)&t, 0) == pdTRUE)
        {
            Serial.print("Received Serial Motion Time: ");
            Serial.println(t);
        }
        if (t < 0)
        {
            neg = true;
            t = (-1 * t);
        }

        if (t > 0)
        {
            if (neg)
            {
                down();
            }
            else
            {
                up();
            }
            vTaskDelay(1 / portTICK_PERIOD_MS);
            t -= 1;
            if (t == 0)
            {
                Serial.println("Ending Timed Motion");
                stop();
                neg = false;
            }
        }
        else
        {
            stop();
            neg = false;
        }
    }
}

void t_serialHandler(void *parameters)
{
    while (1)
    {
        if (Serial.available())
        {

            String s = Serial.readStringUntil('\n');
            const char *time = s.c_str();

            int x;
            x = atoi(time);
            if (xQueueSend(serial_queue, (void *)&x, 10) != pdTRUE)
            {
                Serial.print("Couldn't Send Serial Input to Queue");
            }
        }
    }
}

// void t_print(void *parameters){
//   char * msg;
//   while(1){
//     if(xQueueReceive(print_queue, (void *)&msg, 0)==pdTRUE){
//       Serial.print("Received: ");
//       Serial.println(msg);
//     }
//   }
// }

void backLightTimeout()
{
    if (backlightActive)
    {
        unsigned long duration = millis() - backlightTimeOn;
        if (duration >= BACKLIGHT_TIME)
        {
            // Turn off and Reset backlight state and time or
            lcd.noBacklight();
            backlightActive = false;
            backlightTimeOn = 0;
            setRecentHeight();
        }
    }
}

void setRecentHeight()
{
    // Read height from preferences; if no value - > get as current value
    int tH = preferences.getInt("targetHeight", targetPosition);
    // Compare and save if needed
    if (tH != currentPosition)
    {
        preferences.putInt("targetHeight", targetPosition);
        Serial.println("Saved last target height");
    }
    int cH = preferences.getInt("currentHeight", currentPosition);
    // Compare and save if needed
    if (cH != currentPosition)
    {
        preferences.putInt("currentHeight", currentPosition);
        Serial.println("Saved last current height");
    }
}

void refreshPref()
{
    // Refresh Target Height
    // Refresh Current Height
}

void t_mqtt_loop(void *parameters)
{
    while (1)
    {
        client.loop();
        if (!startup)
        {
            Serial.println("MQTT & Network Task Starting...");
            vTaskDelay(20000 / portTICK_PERIOD_MS);
            startup = true;
        }
    }
}

void t_hardware_loop(void *parameters)
{

    int received = 1;
    char target[4];
    while (1)
    {
        if (!startup)
        {
            Serial.println("Hardware Task Starting...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            startup = true;
        }

        // int newTarget = percentToCM(requested);
        // char tar[4];
        // itoa(newTarget, tar, 10);
        // Serial.print("Target Received: ");
        // Serial.println(str);
        // Serial.print(tar);
        // Serial.println(" CM");

        if (xQueueReceive(target_queue, (void *)&received, 0) == pdTRUE)
        {
            Serial.print("Received: ");
            Serial.println(received);
            itoa(received, target, 10);
            // Reply with confirmation that this target position was received
            client.publish("desk/get/position/target", target);

            // Dummy confirmation that position is reached
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            client.publish("desk/get/position/current", target); // Move to Motion Task
        }
        // // Move desk if needed
        deskMotionHeight();
        // // Save Height if needed
        setRecentHeight();
    }
}

void t_button_loop(void *parameters)
{

    int received = 1;
    while (1)
    {
        if (!startup)
        {
            Serial.println("Button Task Starting...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            startup = true;
        }
        btnRefresh();
    }
}

// Hardware Button interrupt
void IRAM_ATTR isr(void *parameters)
{
    // Change both the read button and desk motion tasks to high priorities
}
// MQTT Interrupt
void mq_isr(void *parameters)
{
    // Change the
}

void setup()
{
    Serial.begin(115200);
    preferences.begin("desk-app", false);

    // ***** Hardware Display Init ***** //
    lcdInit();
    // ***** Hardware Button Init ***** //
    btnInit();

    // ***** Hardware Ultrasonic Sensor Init ***** //
    pinMode(TRIG_PIN, OUTPUT); // config trigger pin to output mode
    pinMode(ECHO_PIN, INPUT);  // config echo pin to input mode

    // ***** Hardware Relay -> Desk Motion Init
    pinMode(PWR_RELAY_PIN, OUTPUT); // Primary power
    pinMode(RELAY_PIN_A, OUTPUT);
    pinMode(RELAY_PIN_B, OUTPUT);
    pinMode(RELAY_PIN_C, OUTPUT);
    pinMode(RELAY_PIN_D, OUTPUT);

    // ***** Mqtt Init & Debug
    client.enableDebuggingMessages(); // Enable debugging messages sent to serial output

    Serial.println("Desk On");

    getHeight();

    // ***** Task Init *****
    // vTaskDelay(2000 / portTICK_PERIOD_MS);

    target_queue = xQueueCreate(target_queue_len, sizeof(int));
    serial_queue = xQueueCreate(serial_queue_len, sizeof(int));
    lcd_print_queue = xQueueCreate(lcd_print_queue_len, sizeof(char[16]));

    xTaskCreatePinnedToCore(t_mqtt_loop, "Task1", 5000, NULL, 1, &Task1, 1);
    xTaskCreatePinnedToCore(t_hardware_loop, "Task2", 5000, NULL, 1, &Task2, 1);
    xTaskCreatePinnedToCore(t_deskMotionTime, "Task3", 5000, NULL, 1, &Task3, 1);
    xTaskCreatePinnedToCore(t_serialHandler, "Task4", 5000, NULL, 1, &Task4, 1);
}

void loop()
{
    // if(targetPosition!=lastTargetPosition||currentPosition!=lastCurrPosition||!startup) {
    //     startup=true;
    //     lcd.clear();
    //     displayHeight();
    // }
    // // Check backlight
    // backLightTimeout();
    // // Read button action
    // btn_refresh();
    // Read homekit action

    // deskMotionTime();
}
