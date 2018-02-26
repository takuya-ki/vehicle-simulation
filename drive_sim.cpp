
#include "drive_sim.hpp"

// Generate an input signal
void creatTargetSignal() {

  if(u_type=="sin"){
    for(int sim_time=0; sim_time<time_vec; sim_time++){
      St_In[sim_time] = StAngle_r * sin(omega*(double(sim_time)*dt));
    }
  }else if(u_type=="step"){
    for(int sim_time=0; sim_time<time_vec; sim_time++){
      St_In[sim_time] = StAngle_r;
    }
  }
  v_In = v_m_s;
}

// Calculate the dynamics of a front-wheel steering vehicle (2ws)
void dynamics2ws(const int t, const double St_, const double v_) {

    // ********** Direct yaw moment control mdoel ********** //
    // Calculate matrix of state equation
    a11 = double(-2.0)*(Cf+Cr)/m/v_;
    a12 = double(-1.0)-double(2.0)*(la*Cf-lb*Cr)/m/v_/v_;
    a21 = double(-2.0)*(la*Cf-lb*Cr)/I;
    a22 = double(-2.0)*(la*la*Cf+lb*lb*Cr)/I/v_;

    b11 = double(2.0)*Cf/m/v_;
    b21 = double(2.0)*la*Cf/I;

    // Calculate sideslip angle
    if(v_<=0.0) {
      //double(0.0);
    } else {
      if(v_<=0.05) {
        //double(1.0);
      } else {
        beta_v[t] = (St_*n*b11 + beta_[t-1]*a11 + gamma_[t-1]*a12);

        // The integral value calculated by trapezoidal integration
        beta_vInteg += ((beta_v[t-1]+beta_v[t])*dt/double(2.0));
        beta_[t] = beta_vInteg;
      }
    }

    // Calculate yaw rate
    if(v_<=0.0) {
      //double(0.0);
    } else {
      if(v_<=0.05) {
        //double(1.0);
      } else {
        gamma_a[t] = (St_*n*b21 + beta_[t-1]*a21 + gamma_[t-1]*a22);

        // The integral value calculated by trapezoidal integration
        gamma_aInteg += ((gamma_a[t-1]+gamma_a[t])*dt/double(2.0));
        gamma_[t] = gamma_aInteg;
      }
    }

    // *** lateral motion of equation for a vehicle *** //
    // (Relationship between lateral acceleration and sideslip angle)
    c11 = double(0.0);
    c12 = double(1.0);
    c21 = v_*a11;
    c22 = v_*(a12+double(1.0));

    d11 = double(0.0);
    d21 = v_*b11;

    // lateral acceleration
    ay[t] = (St_*n*d21 + beta_[t-1]*c21 + gamma_[t-1]*c22);

    // ***** Calculate each displacement from the equation of motion of the whole vehicle ***** //
    // (Use trapezoidal integration)
    yaw_ += ((gamma_[t-1]+gamma_[t])*dt/double(2.0)); // yaw angle
    theta[t] = yaw_;

    vx_[t] = (cos( beta_[t] + theta[t] ) * v_);
    vx_Integ += ((vx_[t-1]+vx_[t])*dt/double(2.0));
    px[t] = vx_Integ;

    vy_[t] = (sin( beta_[t] + theta[t]) * v_);
    vy_Integ += ((vy_[t-1]+vy_[t])*dt/double(2.0));
    py[t] = vy_Integ;
}

// Running simulation based on vehicle model and dynamics with MV2 alone
void drivingMV2_1(){

  // Generate a target signal
  creatTargetSignal();

  // Animation Settings (by gnuplot)
  FILE *gp = popen("gnuplot", "w");
  if (gp == NULL) { exit(1); }
  // Enable the mouse function for the current interactive output format
  // In interactive mode this is usually enabled by default
  // but invalid if the command is read from a file
  fputs("set mouse\n", gp);
  fputs("set ticslevel 0\n", gp); // Prevent the 0 point of the z axis from leaving the XY plane
  fputs("set title 'running path'\n", gp);  // Set title
  fputs("set xlabel 'x[m]'\n", gp);
  fputs("set xrange [0:30]\n", gp);
  fputs("set ylabel 'y[m]'\n", gp);
  fputs("set yrange [0:2.5]\n", gp);
  char fname[256] = "running_path.dat";
  std::ofstream fs;


  std::cout << "time vector: " << time_vec << std::endl;
  std::cout << "steering angle[rad]: " << StAngle_r << std::endl;
  fs.open(fname);

  for(int sim_time=1; sim_time<time_vec; sim_time++) {
    dynamics2ws(sim_time, St_In[sim_time], v_In);

    // Output the result of target signal
    //fs << std::to_string(sim_time*dt) << " "
    //   << std::to_string(St_In[sim_time]) << std::endl; // Write position coordinates to file

    // Output the result of dynamics simulation（gnuplot）
    fputs("plot \"-\" w p pt 7 ps 3\n", gp);
    fprintf(gp, "%lf %lf\n", px[sim_time],py[sim_time]);
    fputs("e\n", gp);

    fs << std::to_string(px[sim_time]) << " "
      << std::to_string(py[sim_time]) << std::endl; // Write position coordinates to file
    fflush(gp);	// Spit out data stored in buffer (mandatory)
    // usleep(1000000);    // Unit is [μs]
  }

  fs.close();	// Close the file describing the position
  fputs("plot 'running_path.dat' w p pt 7 ps 1.5 lc rgb \"blue\"\n", gp);
  fflush(gp); // Spit out data stored in buffer (mandatory)
  std::cin.get();
  fputs("exit\n", gp);
  fflush(gp); // Spit out data stored in buffer (mandatory)
  pclose(gp);
}
