
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>  // ファイルストリームのヘッダ

const double simu_time = 20;  // シミュレーション時間[s]
const double dt = 0.001;      // ステップ時間[s]
const int time_vec = simu_time/dt+1;
const std::string u_type = "sin";   // 入力信号 sin:正弦波, step:ステップ

const double StAngle_d = 45.0;  // ステアリング入力角(-660~660)[deg]
const double StAngle_r = StAngle_d*M_PI/double(180.0);
const double omega = 0.5;     // 正弦波目標信号の場合の周波数[rad/s]

const double v_km_h = 5.0;      // 速度[km/h]
const double v_m_s = v_km_h*double(1000.0)/double(3600.0);

// 車両パラメータ
const double m = 400;         // 車両重量[kg]
const double lc = 2.395;      // 全長[m]
const double dc = 1.095;      // 全幅[m]
const double hc = 1.505;      // 全高[m]
const double w = 1.530;       // ホイールベース[m]
const double la = 0.896;      // 前輪車軸 - 車両重心位置間距離[m]
const double lb = 0.634;      // 後輪車軸 - 車両重心位置間距離[m]
const double df = 0.93;       // 前輪トレッド[m]
const double dr = 0.92;       // 後輪トレッド[m]
const double I = 159;         // イナーシャ[kgm^2]
const double Cf = 10000;      // 前輪コーナリングパワー[N/rad]
const double Cr = 16000;      // 後輪コーナリングパワー[N/rad]
const double n = double(36)/double(660);      // ステアリングギア比

// 目標信号
double St_In[time_vec] = {};     // ステアリング角入力[rad]
double v_In = 0.0;               // 速度入力[m/s]

// ダイナミクスモデルの出力
double beta_[time_vec] = {};     // 横滑り角度[rad]
double gamma_[time_vec] = {};    // ヨーレート[rad/s]
double ay[time_vec] = {};        // 横加速度[m/ss]
double px[time_vec] = {};        // 変位x[m]
double py[time_vec] = {};        // 変位y[m]
double theta[time_vec] = {};     // 回転変位θ[rad]

// ダイナミクスモデルの中間パラメータ
double beta_v[time_vec] = {};    // 微小区間積分値の計算に利用(横滑り角速度)
double beta_vInteg = 0;
double gamma_a[time_vec] = {};   // 微小区間積分値の計算に利用(ヨー角加速度)
double gamma_aInteg = 0;
double a11, a12, a21, a22, b11, b21;
double c11, c12, c21, c22, d11, d21;
double yaw_ = 0;
double vx_[time_vec] = {};       // 微小区間積分値の計算に利用(x方向の速度)
double vx_Integ = 0;
double vy_[time_vec] = {};       // 微小区間積分値の計算に利用(y方向の速度)
double vy_Integ = 0;
