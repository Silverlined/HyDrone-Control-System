# PID

## Continuous to Discrete time representation
- Via *Tustin transform*

1. Derivative amplifies HF noise;
> Low Pass filer the signal before/after taking the derivative.
2. Derivative "kick" during setpoint change;
> Derivative-on-measurement, not errro.
3. Integrator can saturate output;
> Integrator anti-windup.
4. Limits on system input amplitude;
> Clamp controll output.
5. Choosing a sample time T
> Sample time of the controller to be 10 times higher than the system's.

## Code Structure:

- Header-file
> #include "pid.h"
- PID controller struct
> contains gains, storage variables, etc.
- Initialisation function
> set gains, sample time, etc.
- Update function
> provide setpoint and measurement, returns contorller output.