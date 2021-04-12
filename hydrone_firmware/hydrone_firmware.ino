#include <Servo.h>

#define PRINT_PWM 0

// Calibrated pulse width for FrSky Taranis Q X7 16CH 2.4Ghz transmitter with RX8R receiver
const uint16_t RC_min[4] = {1000, 1000, 1000, 1000};
const uint16_t RC_mid[4] = {1500, 1500, 1500, 1500};
const uint16_t RC_max[4] = {2000, 2000, 2000, 2000};

uint8_t last_channel_1_state = 0, last_channel_2_state = 0, last_channel_3_state = 0, last_channel_4_state = 0;
uint16_t receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
uint32_t timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;

uint32_t last_rc_update = 0;
Servo motor_left, motor_right, toggle_left, toggle_right;

// Interrupt service routine called evey time digital input pin 8, 9, 10, or 11 changes state
// PCINT0_vect is the compiler vector for PCINT0 on PORTB of ATmega328
ISR(PCINT0_vect)
{
    // Channel 1 (pin 8) ==================================
    if (last_channel_1_state == 0 && PINB & B00000001)
    {
        last_channel_1_state = 1;
        timer_channel_1 = micros();
    }
    if (last_channel_1_state == 1 && !(PINB & B00000001))
    {
        last_channel_1_state = 0;
        receiver_input_channel_1 = micros() - timer_channel_1;
    }
    // Channel 2 (pin 9) ==================================
    if (last_channel_2_state == 0 && PINB & B00000010)
    {
        last_channel_2_state = 1;
        timer_channel_2 = micros();
    }
    if (last_channel_2_state == 1 && !(PINB & B00000010))
    {
        last_channel_2_state = 0;
        receiver_input_channel_2 = micros() - timer_channel_2;
    }
    // Channel 3 (pin 10) ==================================
    if (last_channel_3_state == 0 && PINB & B00000100)
    {
        last_channel_3_state = 1;
        timer_channel_3 = micros();
    }
    if (last_channel_3_state == 1 && !(PINB & B00000100))
    {
        last_channel_3_state = 0;
        receiver_input_channel_3 = micros() - timer_channel_3;
    }
    // Channel 4 (pin 11) ==================================
    if (last_channel_4_state == 0 && PINB & B00001000)
    {
        last_channel_4_state = 1;
        timer_channel_4 = micros();
    }
    if (last_channel_4_state == 1 && !(PINB & B00001000))
    {
        last_channel_4_state = 0;
        receiver_input_channel_4 = micros() - timer_channel_4;
    }
}

//Subroutine for displaying the receiver signals
void read_receiver_signals()
{
    Serial.print("Channel 1:");
    if (receiver_input_channel_1 - 1480 < 0)
        Serial.print("<<<");
    else if (receiver_input_channel_1 - 1520 > 0)
        Serial.print(">>>");
    else
        Serial.print("-+-");
    Serial.print(receiver_input_channel_1);

    Serial.print("  Channel 2:");
    if (receiver_input_channel_2 - 1480 < 0)
        Serial.print("^^^");
    else if (receiver_input_channel_2 - 1520 > 0)
        Serial.print("vvv");
    else
        Serial.print("-+-");
    Serial.print(receiver_input_channel_2);

    Serial.print("  Channel 3:");
    if (receiver_input_channel_3 - 1480 < 0)
        Serial.print("vvv");
    else if (receiver_input_channel_3 - 1520 > 0)
        Serial.print("^^^");
    else
        Serial.print("-+-");
    Serial.print(receiver_input_channel_3);

    Serial.print("  Channel 4:");
    if (receiver_input_channel_4 - 1480 < 0)
        Serial.print("<<<");
    else if (receiver_input_channel_4 - 1520 > 0)
        Serial.print(">>>");
    else
        Serial.print("-+-");
    Serial.println(receiver_input_channel_4);
}

void setup()
{
    // ATmega pins default to inputs, therefore there is no need to use the pinMode function
    PCICR |= (1 << PCIE0);   // set PCIE0 bit to to enable PCMSK0 scan
    PCMSK0 |= (1 << PCINT0); // set PCINT0 (digital pin 8) to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT1); // set PCINT0 (digital pin 9) to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT2); // set PCINT0 (digital pin 10) to trigger an interrupt on state change
    PCMSK0 |= (1 << PCINT3); // set PCINT0 (digital pin 11) to trigger an interrupt on state change

    motor_left.attach(9, RC_min[1], RC_max[1]);
    motor_right.attach(10, RC_min[2], RC_max[2]);

    toggle_left.attach(8, RC_min[0], RC_max[0]);
    toggle_right.attach(11, RC_min[3], RC_max[3]);

    Serial.begin(115200);
}

void loop()
{
    if (millis() - last_rc_update > 25)
    {
        motor_left.writeMicroseconds(receiver_input_channel_2);
        motor_right.writeMicroseconds(receiver_input_channel_3);
        last_rc_update = millis();

        if (PRINT_PWM)
        {
            read_receiver_signals();
            delay(250);
        }
    }
}
