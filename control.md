Angular velocity is:

`ω = Δφ/Δt = (φn − φn−1)/(tn − tn−1)`

Current angle can be calculated from previous angle as:

`φn = φn−1 + ω·Δt`

```c
double angle = 0;  /* or any initial value */
while(1){
    double omega = get_angular_velocity();
    angle += omega * timestep;
}
```

provided that the function get_angular_velocity returns a value regularly at timestep intervals.


Develop a simply MEMS gyro model in Python.
The model can be used to test out the linear control system design in a non-linear simulated environment.
This is especially handy for tweaking control parameters without having to load software onto the embedded device and test it repeatedly physically.
The model can also be applied to check the stability of the designed system.


