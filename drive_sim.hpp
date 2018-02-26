
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

const double simu_time = 20;  // Simulation time [s]
const double dt = 0.001;      // Step time [s]
const int time_vec = simu_time/dt+1;
const std::string u_type = "sin";   // Input signal sin:sine wave, step:step signal

const double StAngle_d = 45.0;  // Steering input angle (-660 ~ 660) [deg]
const double StAngle_r = StAngle_d*M_PI/double(180.0);
const double omega = 0.5;     // Frequency in case of sinusoidal target signal [rad/s]

const double v_km_h = 5.0;      // Velocity [km/h]
const double v_m_s = v_km_h*double(1000.0)/double(3600.0);

// Vehicle parameteres
const double m = 400;         // Vehicle weight [kg]
const double lc = 2.395;      // Full length [m]
const double dc = 1.095;      // Full width [m]
const double hc = 1.505;      // Full height [m]
const double w = 1.530;       // Wheelbase [m]
const double la = 0.896;      // Front axle - distance between centroids [m]
const double lb = 0.634;      // Rear axle - distance between centroids [m]
const double df = 0.93;       // Front wheel tread [m]
const double dr = 0.92;       // Rear wheel tread [m]
const double I = 159;         // Inertia [kgm^2]
const double Cf = 10000;      // Front wheel cornering power [N/rad]
const double Cr = 16000;      // Rear wheel cornering power [N/rad]
const double n = double(36)/double(660);      // Steering gear ratio

// Target signals
double St_In[time_vec] = {};     // Steering angle input [rad]
double v_In = 0.0;               // Velocity input [m/s]

// Outputs of a dynamics model
double beta_[time_vec] = {};     // Sideslip angle [rad]
double gamma_[time_vec] = {};    // Yaw rate [rad/s]
double ay[time_vec] = {};        // lateral acceleration [m/ss]
double px[time_vec] = {};        // Displacement x[m]
double py[time_vec] = {};        // Displacement y[m]
double theta[time_vec] = {};     // Rotational displacement θ[rad]

// Intermediate parameters of a dynamics model
double beta_v[time_vec] = {};    // Used to calculate minute interval integral value (Sideslip angle)
double beta_vInteg = 0;
double gamma_a[time_vec] = {};   // Used to calculate minute interval integral value (yaw angular acceleration)
double gamma_aInteg = 0;
double a11, a12, a21, a22, b11, b21;
double c11, c12, c21, c22, d11, d21;
double yaw_ = 0;
double vx_[time_vec] = {};       // Used to calculate minute interval integral value (Speed ​in x-direction)
double vx_Integ = 0;
double vy_[time_vec] = {};       // Used to calculate minute interval integral value (Speed ​in y-direction)
double vy_Integ = 0;
