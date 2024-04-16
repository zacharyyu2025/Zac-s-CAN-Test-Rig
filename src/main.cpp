#include <Arduino.h>
#include "virtualTimer.h"
#include "esp_can.h"

#define FLOAT_LED_PIN 16 // LED pin to indicate received correct RX message
#define UINT8_LED_PIN 17
#define BOOL_LED_PIN 18

#define TARGET_VAL_FLOAT 0.0 // set these values depending on what you want to receive
#define TARGET_VAL_UINT8 0
#define TARGET_VAL_BOOL false

#define RESET_TIMER_MS 2000 // time before the LEDs turn off
#define DEAD_TIMER_MS 1000 // time after leds turn off before checking for correct value

VirtualTimerGroup timer_group{};
ESPCAN can_bus{};

MakeSignedCANSignal(float, 0, 16, 0.01, 0) float_tx_signal{};
MakeUnsignedCANSignal(uint8_t, 16, 8, 1, 0) uint8_tx_signal{};
MakeUnsignedCANSignal(bool, 24, 1, 1, 0) bool_tx_signal{};

MakeSignedCANSignal(float, 0, 16, 0.01, 0) float_rx_signal{};
MakeUnsignedCANSignal(uint8_t, 16, 8, 1, 0) uint8_rx_signal{};
MakeUnsignedCANSignal(bool, 24, 1, 1, 0) bool_rx_signal{};

CANTXMessage<2> long_tx_message{
    can_bus, 0x600, 3, 100, timer_group, float_tx_signal, uint8_tx_signal}; // long tx message to send float and int

CANTXMessage<1> short_tx_message{
    can_bus, 0x601, 1, 100, timer_group, bool_tx_signal}; // short tx message to send bool

CANRXMessage<2> long_rx_message{can_bus,
                           0x600,
                           []() { Serial.println("long tx received"); },
                           float_rx_signal,
                           uint8_rx_signal}; // long rx message to receive float and int

CANRXMessage<1> short_rx_message{can_bus,
                           0x601,
                           []() { Serial.println("short tx received"); },
                           bool_rx_signal}; // short rx message to receive bool

int FLOAT_state = 0; // global variable to track LED state
// 0 waiting for correct value
// 1 leds on
// 2 leds dead
long FLOAT_timer = 0; // long for timer 
void evaluate_FLOAT_led_state(){
    if(FLOAT_state == 0){
        digitalWrite(FLOAT_LED_PIN, LOW);
        if(float_rx_signal == TARGET_VAL_FLOAT){ // continuously check for target float value
            FLOAT_state = 1; // set the state
            FLOAT_timer = millis(); // set the start
        }
    }
    else if(FLOAT_state == 1){
        digitalWrite(FLOAT_LED_PIN, HIGH);
        if((millis() - FLOAT_timer) > RESET_TIMER_MS){
            FLOAT_state = 2;
            FLOAT_timer = millis(); // reset the timer for the dead state
        }
    }
    else if(FLOAT_state == 2){
        if((millis() - FLOAT_timer) > DEAD_TIMER_MS){
            FLOAT_state = 0;
            float_rx_signal = 0.0;
        }

    }
}

int UINT8_state = 0; // global variable to track LED state
// 0 waiting for correct value
// 1 leds on
// 2 leds dead
long UINT8_timer = 0; // long for timer 
void evaluate_UINT8_led_state(){
    if(UINT8_state == 0){
        digitalWrite(UINT8_LED_PIN, LOW);
        if(uint8_rx_signal == TARGET_VAL_UINT8){ // continuously check for target uint8 value
            UINT8_state = 1; // set the state
            UINT8_timer = millis(); // set the start
        }
    }
    else if(UINT8_state == 1){
        digitalWrite(UINT8_LED_PIN, HIGH);
        if((millis() - UINT8_timer) > RESET_TIMER_MS){
            UINT8_state = 2;
            UINT8_timer = millis(); // reset the timer for the dead state
        }
    }
    else if(UINT8_state == 2){
        if((millis() - UINT8_timer) > DEAD_TIMER_MS){
            UINT8_state = 0;
            uint8_rx_signal = 0;
        }

    }
}

int BOOL_state = 0; // global variable to track LED state
// 0 waiting for correct value
// 1 leds on
// 2 leds dead
long BOOL_timer = 0; // long for timer 
void evaluate_BOOL_led_state(){
    if(BOOL_state == 0){
        digitalWrite(BOOL_LED_PIN, LOW);
        if(bool_rx_signal == TARGET_VAL_BOOL){ // continuously check for target bool value
            BOOL_state = 1; // set the state
            BOOL_timer = millis(); // set the start
        }
    }
    else if(BOOL_state == 1){
        digitalWrite(BOOL_LED_PIN, HIGH);
        if((millis() - BOOL_timer) > RESET_TIMER_MS){
            BOOL_state = 2;
            BOOL_timer = millis(); // reset the timer for the dead state
        }
    }
    else if(BOOL_state == 2){
        if((millis() - BOOL_timer) > DEAD_TIMER_MS){
            BOOL_state = 0;
            bool_rx_signal = false;
        }
    }
}
void ten_ms_task()
{
    float_tx_signal = 5.195f;
    uint8_tx_signal = 132;
    bool_tx_signal = true;

    float test_float = float_rx_signal;
    uint8_t test_uint8_t = uint8_rx_signal;
    bool test_bool = bool_rx_signal;

    evaluate_FLOAT_led_state();
    evaluate_UINT8_led_state();
    evaluate_BOOL_led_state();

    can_bus.Tick();
}

void setup()
{
    can_bus.Initialize(ICAN::BaudRate::kBaud1M);

    // You can create a new timer in a VirtualTimerGroup using the AddTimer(function, time) function
    timer_group.AddTimer(10, ten_ms_task);

    Serial.begin(9600);
    Serial.println("Started");
    pinMode(FLOAT_LED_PIN, OUTPUT);
    pinMode(UINT8_LED_PIN, OUTPUT);
    pinMode(BOOL_LED_PIN, OUTPUT);
}

void loop() { timer_group.Tick(millis()); }