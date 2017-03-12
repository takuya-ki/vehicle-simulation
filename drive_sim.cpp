
#include "drive_sim.hpp"

// 入力信号の生成
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

// 前輪操舵車両（2ws）のダイナミクスの計算
void dynamics2ws(const int t, const double St_, const double v_) {

    // ********** 直接ヨーモーメント制御モデル ********** //
    // 状態方程式の行列計算
    a11 = double(-2.0)*(Cf+Cr)/m/v_;
    a12 = double(-1.0)-double(2.0)*(la*Cf-lb*Cr)/m/v_/v_;
    a21 = double(-2.0)*(la*Cf-lb*Cr)/I;
    a22 = double(-2.0)*(la*la*Cf+lb*lb*Cr)/I/v_;

    b11 = double(2.0)*Cf/m/v_;
    b21 = double(2.0)*la*Cf/I;

    // 横滑り角度計算
    if(v_<=0.0) {
      //double(0.0);
    } else {
      if(v_<=0.05) {
        //double(1.0);
      } else {
        beta_v[t] = (St_*n*b11 + beta_[t-1]*a11 + gamma_[t-1]*a12);

        // 台形積分による積分値算出
        beta_vInteg += ((beta_v[t-1]+beta_v[t])*dt/double(2.0));
        beta_[t] = beta_vInteg;
      }
    }

    // ヨーレート計算
    if(v_<=0.0) {
      //double(0.0);
    } else {
      if(v_<=0.05) {
        //double(1.0);
      } else {
        gamma_a[t] = (St_*n*b21 + beta_[t-1]*a21 + gamma_[t-1]*a22);

        // 台形積分による積分値算出
        gamma_aInteg += ((gamma_a[t-1]+gamma_a[t])*dt/double(2.0));
        gamma_[t] = gamma_aInteg;
      }
    }

    // *** 車両の横方向の運動方程式(横加速度と横滑り角の関係) *** //
    c11 = double(0.0);
    c12 = double(1.0);
    c21 = v_*a11;
    c22 = v_*(a12+double(1.0));

    d11 = double(0.0);
    d21 = v_*b11;

    // 横加速度の計算
    ay[t] = (St_*n*d21 + beta_[t-1]*c21 + gamma_[t-1]*c22);

    // ***** 車両全体の運動方程式から各変位の計算(台形積分利用) ***** //
    yaw_ += ((gamma_[t-1]+gamma_[t])*dt/double(2.0)); // ヨー角
    theta[t] = yaw_;

    vx_[t] = (cos( beta_[t] + theta[t] ) * v_);
    vx_Integ += ((vx_[t-1]+vx_[t])*dt/double(2.0));
    px[t] = vx_Integ;

    vy_[t] = (sin( beta_[t] + theta[t]) * v_);
    vy_Integ += ((vy_[t-1]+vy_[t])*dt/double(2.0));
    py[t] = vy_Integ;
}

// MV2単体での車両モデルとダイナミクスに基づく走行シミュレーション
void drivingMV2_1(){

  // 目標信号の生成
  creatTargetSignal();

  // Animation Settings(by gnuplot)
  FILE *gp = popen("gnuplot", "w");  // popen(): 標準入力に書き込む
  if (gp == NULL) { exit(1); }
  // 現在の対話型出力形式に対してマウス機能を有効にする．
  // 対話型モードでは通常デフォルトでこれは有効だが,コマンドがファイルから読み込まれる場合は無効
  fputs("set mouse\n", gp);
  fputs("set ticslevel 0\n", gp); // z軸の0点がXY平面から離れるのを防ぐ
  fputs("set title 'running path'\n", gp);  // タイトルを設定
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

    // 目標信号の結果出力
    //fs << std::to_string(sim_time*dt) << " "
    //   << std::to_string(St_In[sim_time]) << std::endl; // 位置座標をファイルに書き込み

    // ダイナミクスシミュレーション結果出力（gnuplot）
    fputs("plot \"-\" w p pt 7 ps 3\n", gp);
    fprintf(gp, "%lf %lf\n", px[sim_time],py[sim_time]);
    fputs("e\n", gp); // gnuplotの終了

    fs << std::to_string(px[sim_time]) << " "
      << std::to_string(py[sim_time]) << std::endl; // 位置座標をファイルに書き込み
    fflush(gp);	// バッファに格納されているデータを吐き出す（必須）
    // usleep(1000000);    // 単位は[μs]
  }

  fs.close();	// 位置を書いているファイルを閉じる
  fputs("plot 'running_path.dat' w p pt 7 ps 1.5 lc rgb \"blue\"\n", gp);
  fflush(gp); // バッファに格納されているデータを吐き出す（必須）
  std::cin.get();
  fputs("exit\n", gp); // gnuplotの終了
  fflush(gp); // バッファに格納されているデータを吐き出す（必須）
  pclose(gp);
}
