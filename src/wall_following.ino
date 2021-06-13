#include<math.h>
#include <VL53L1X.h>

#define PI 3.14159265
float sampling_time = 0.01; // 100 [Hz]
float wall_lead = 2; // [m]
float setpoint = 0.7; // [m]

VL53L1X ToF_90, ToF_45;

typedef struct {
    // Orientation in degrees
    float orientation;

    // Rise time constant
    float tau;

    // Damping constant
    float z;

} HyDrone;

HyDrone hydrone;
hydrone->orientation = 0;
hydrone->tau = 0.2;
hydrone->z = 1;

PID_Controller pid;
PIDController_Init(pid);

void updateOrientation(float angular_velocity){
    hydrone->orientation += angular_velocity * sampling_time;
}

float calculateBeta(float orientation) {
    return 45 - orientation;
}

float calculateProjection(float beta, float distance_2) {
    return cos(beta  * (PI / 180.0)) * distance_2;
}

float calculateAlpha(float orientation, float projection) {
    return atan2(projection - setpoint, wall_lead) * (180.0 / PI) - orientation;
}

float getDistance(VL53L1X *sensor){
    return sensor.read();
}

void setup()
{

}

void loop() {
    distance_2 = getDistance(&ToF_45);
    beta = calculateBeta(hydrone->orientation);
    projection = calculateProjection(beta, distance_2);
    error = calculateAlpha(hydrone->orientation, projection);
    pid_output = PIDController_Update(pid, error);

    //PID Output -> PWM
    
}
