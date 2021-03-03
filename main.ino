PCICR |= (1 << PCIE0);   // set PCIE0 bit to to enable PCMSK0 scan
PCMSK0 |= (1 << PCINT0); // set PCINT0 (digital pin 8) to trigger an interrupt on state change
PCMSK0 |= (1 << PCINT1); // set PCINT0 (digital pin 9) to trigger an interrupt on state change
PCMSK0 |= (1 << PCINT2); // set PCINT0 (digital pin 10) to trigger an interrupt on state change
PCMSK0 |= (1 << PCINT3); // set PCINT0 (digital pin 11) to trigger an interrupt on state change