uint8_t last_channel_1_state = 0, last_channel_2_state = 0, last_channel_3_state = 0, last_channel_4_state = 0;
uint16_t receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
uint32_t timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4;

// Interrupt service routine called evey time digital input pin 8, 9, 10, or 11 changes state
// PCINT0_vect is the compiler vector for PCINT0 on PORTB of ATmega328
ISR(PCINT0_vect)
{
    // Channel 1==================================
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
    // Channel 2==================================
    if (last_channel_2_state == 0 && PINB & B00000010)
    {
        last_channel_2_state = 1;
        timer_channel_2 = micros();
    }
    if (last_channel_2_state == 1 && !(PINB & B00000010))
    {
        last_channel_2_state = 0;
        receiver_input_channel_2 = micros() - timer_channel_2
    }
    // Channel 3==================================
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
    // Channel 4==================================
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
    Serial.begin(115200);
}

void loop()
{
    delay(250);
    read_receiver_signals();
}