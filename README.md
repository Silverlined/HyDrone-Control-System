## About the project

This project follows the develepoment of a control system for an unmanned surface vehicle (USV).
It consists of efficient code to read the PWM output of an RC receiver via Pin Change Interrupts and write PWM to the ESCs controlling underwater thrusters.
An IMU sensor is used to keep track of the vehicle's orientation, while Time-of-Flight (ToF) sensors measure the distance to canal walls and control the orientation via PID algorithm.

## Introduction

Inspection of bridges and quay walls, below or above the water line, is the main activity of oQuay.
They conduct inspection in different ways but one of their key instruments is a sonar camera attached to a USV, called the HyDrone.
In order to make recordings of high quality it is important to sail the HyDrone at a constant distance (parallel) to quay walls.
During operation, the position of the vehicle is affected by wind and water currents, making the sonar images blurry.
The lack of a control system to correct the HyDrones orientation is a pitfall in the design and affects the quality of the inspection work.
Moreover, the GoPro camera on the HyDrone has a short transmission range of ~10m.
This becomes a problem when the HyDrone sails further away, losing the video feedback and the control over the camera.
oQuay are looking for something that can guarantee control of the movement of the vehicle and a device that can increase the transmission range of their GoPro camera.

Assembly files are in the relevant repositoty folders.


## MuSCoW

| Must haves                                                                            | Should haves                                                    | Could haves                                                                                           |
|---------------------------------------------------------------------------------------|-----------------------------------------------------------------|-------------------------------------------------------------------------------------------------------|
| Independent adjustment of the orientation of the HyDrone with respect to canal walls | Orientation correction should be smooth with no oscillations   | Adjustment mechanism to adapt for different situations - different heights and shapes of canal walls |
| Sailing at a constant distance 70cm~1m                                               | Stable response in spite of wind and water currents            | Audible feedback from the control commands - start/stop could be indicated with some type of sound   |
| Increased video transmission range to at least 100 meters                            | Validation with experiments in real canal environment          |                                                                                                       |
| Increased GoPro control range to at least 100 meters                                 | Developed system should be portable and detachable             |                                                                                                       |
| Control system must not exceed €100                                                  | Power source of the system should be rechargeable              |                                                                                                       |
| Real-time transmission of the video signal                                           | Video should be shown on one of the already available displays |                                                                                                       |

## Planning

![Plan](res/planning.png)


## Pin Change Interrupts

```c
PCICR |= (1 << PCIE0);   // set PCIE0 bit to to enable PCMSK0 scan
PCMSK0 |= (1 << PCINT0); // set PCINT0 (digital pin 8) to trigger an interrupt on state change

// PCINT0_vect is the compiler vector for PCINT0 on PORTB of ATmega328
ISR(PCINT0_vect)
{
  //    // Channel 1==================================
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
  ...
}
```

## FPV System

![front](res/front.jpeg)

![back](res/back.jpeg)

![inside](res/inside.jpeg)

![real](res/real.jpeg)


## Control System

### Heading determination system (HDS)

- Microcontroller
- IMU, 3-DoF Gyroscope

Angular velocity is:

`ω = Δφ/Δt = (φn − φn−1)/(tn − tn−1)`

Current angle can be calculated from previous angle as:

`φn = φn−1 + ω·Δt`

```c
float angle = 0;  /* or any initial value */
while(1){
    float omega = getAngularVelocity();
    angle += omega * timestep;
}
```

provided that the function `getAngularVelocity` returns a value regularly at timestep intervals. For example, 100 Hz.


### Modelling
Develop a simple MEMS gyro model in Python; Second-order process dead time.
The model can be used to test out the linear control system design in a non-linear simulated environment.
This is especially handy for tweaking control parameters without having to load software onto the embedded device and test it repeatedly physically.
The model can also be applied to check the stability of the designed controller.

### ToF sensors

- I2C
- Arduino Nano (Atmega328) has a single hardware UART. Software UART requires dedicating PCINT (pin change interrupts)

### Raspberry Pi Workflow

1. Install arduino-cli:

```
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
arduino-cli config init
```

2. Add to PATH: (edit .bashrc)

```export PATH=$PATH:/home/pi/bin```

3. Copy Arduino Libraries: (~/Arduino/libraries/)

4. Compile & Upload

```
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli compile -b arduino:avr:nano file_path
arduino-cli upload -p /dev/ttyACM0 -b arduino:avr:nano file_path
```
