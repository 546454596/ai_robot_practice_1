#include "utils/pid_fuzzy.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>

// 注1：自适应模糊pid最重要的就是论域的选择，要和你应该控制的对象相切合
// 注2：以下各阀值、限幅值、输出值均需要根据具体的使用情况进行更改
// 注3：因为我的控制对象惯性比较大，所以以下各部分取值较小
// 论域e:[-5,5]  ec:[-0.5,0.5]
// 误差的阀值，小于这个数值的时候，不做PID调整，避免误差较小时频繁调节引起震

constexpr double kEMin = 0.0;
constexpr double kEMid = 0.08;
constexpr double kEMax = 0.6;

enum FuzzyRule {
  kNB = 0,
  kNM,
  kNS,
  kZO,
  kPS,
  kPM,
  kPB,
};

int kp[7][7] = {{kPB, kPB, kPM, kPM, kPS, kZO, kZO}, 
                {kPB, kPB, kPM, kPS, kPS, kZO, kZO},
                {kPM, kPM, kPM, kPS, kZO, kNS, kNS}, 
                {kPM, kPM, kPS, kZO, kNS, kNM, kNM},
                {kPS, kPS, kZO, kNS, kNS, kNM, kNM}, 
                {kPS, kZO, kNS, kNM, kNM, kNM, kNB},
                {kZO, kZO, kNM, kNM, kNM, kNB, kNB}};

int kd[7][7] = {{kPS, kNS, kNB, kNB, kNB, kNM, kPS}, 
                {kPS, kNS, kNB, kNM, kNM, kNS, kZO},
                {kZO, kNS, kNM, kNM, kNS, kNS, kZO}, 
                {kZO, kNS, kNS, kNS, kNS, kNS, kZO},
                {kZO, kZO, kZO, kZO, kZO, kZO, kZO}, 
                {kPB, kNS, kPS, kPS, kPS, kPS, kPB},
                {kPB, kPM, kPM, kPM, kPS, kPS, kPB}};

int ki[7][7] = {{kNB, kNB, kNM, kNM, kNS, kZO, kZO}, 
                {kNB, kNB, kNM, kNS, kNS, kZO, kZO},
                {kNB, kNM, kNS, kNS, kZO, kPS, kPS}, 
                {kNM, kNM, kNS, kZO, kPS, kPM, kPM},
                {kNM, kNS, kZO, kPS, kPS, kPM, kPB}, 
                {kZO, kZO, kPS, kPS, kPM, kPB, kPB},
                {kZO, kZO, kPS, kPM, kPM, kPB, kPB}};

/**************求隶属度（三角形）***************/
float FTri(float x, float a, float b, float c) {
  if (x <= a)
    return 0;
  else if ((a < x) && (x <= b))
    return (x - a) / (b - a);
  else if ((b < x) && (x <= c))
    return (c - x) / (c - b);
  else if (x > c)
    return 0;
  else
    return 0;
}

/*****************求隶属度（梯形左）*******************/
float FTraL(float x, float a, float b) {
  if (x <= a)
    return 1;
  else if ((a < x) && (x <= b))
    return (b - x) / (b - a);
  else if (x > b)
    return 0;
  else
    return 0;
}

/*****************求隶属度（梯形右）*******************/
float FTraR(float x, float a, float b) {
  if (x <= a) return 0;

  if ((a < x) && (x < b)) return (x - a) / (b - a);

  if (x >= b)
    return 1;
  else
    return 1;
}

/****************三角形反模糊化处理**********************/
float uFTri(float x, float a, float b, float c) {
  float y, z;
  z = (b - a) * x + a;
  y = c - (c - b) * x;
  return (y + z) / 2;
}

/*******************梯形（左）反模糊化***********************/
float uFTraL(float x, float a, float b) { 
  return b - (b - a) * x; 
}

/*******************梯形（右）反模糊化***********************/
float uFTraR(float x, float a, float b) { 
  return (b - a) * x + a; 
}

/**************************求交集****************************/
float fand(float a, float b) { 
  return (a < b) ? a : b; 
}

/**************************求并集****************************/
float forr(float a, float b) { 
  return (a < b) ? b : a; 
}

FuzzyPid::FuzzyPid(const float& initial_p, 
                   const float& initial_i,
                   const float& initial_d,
                   const float& initial_imax)
    : integrator_(0),
      last_input_(0),
      last_derivative_(0),
      flag_(0),
      d_lpf_alpha_(kPidControllerDTermFilter) {
  this->kp_ = initial_p;
  this->ki_ = initial_i;
  this->kd_ = initial_d;
  this->imax_ = fabs(initial_imax);
  std::cout << "fuzzy PID: kP:" << this->kp_ << " kI:" << this->ki_
            << " kD:" << this->kd_ << " imax:" << this->imax_ << '\n';
  // derivative is invalid on startup
  this->last_derivative_ = NAN;
}

void FuzzyPid::resetIntegrator() {
  this->integrator_ = 0;
  // mark derivative as invalid
  this->last_derivative_ = NAN;
}

void FuzzyPid::loadGains(const char* filePath) {
  std::ifstream file(filePath);
  file >> this->kp_ >> this->ki_ >> this->kd_ >> this->imax_;
  std::cout << " kP:" << this->kp_ << " kI:" << this->ki_ << " kD:" << this->kd_
            << " imax:" << this->imax_ << '\n';
  this->imax_ = fabs(this->imax_);

  file.close();
}

void FuzzyPid::saveGains(const char* filePath) {
  std::ofstream file(filePath);
  file << this->kp_ << " " << this->ki_ << " " << this->kd_ << " " << this->imax_;
  file.close();
}

float FuzzyPid::getP(float error) const { 
  return (float)error * this->kp_; 
}

float FuzzyPid::getI(float error, float dt) {
  if ((this->ki_ != 0) && (dt != 0)) {
    if (fabs(error) < kPidControllerIntegralE) {
      this->integrator_ += ((float)error * this->ki_) * dt;
      if (this->integrator_ < -this->imax_) {
        this->integrator_ = -this->imax_;
      } else if (this->integrator_ > this->imax_) {
        this->integrator_ = this->imax_;
      }
      return this->integrator_;
    } else {
      return 0;
    }
  }
  return 0;
}

float FuzzyPid::getD(float input, float dt) {
  if ((this->kd_ != 0) && (dt != 0)) {
    float derivative;
    if (std::isnan(this->last_derivative_)) {
      // we've just done a reset, suppress the first derivative
      // term as we don't want a sudden change in input to cause
      // a large D output change
      derivative = 0;
      this->last_derivative_ = 0;
    } else {
      // calculate ikNStantaneous derivative
      derivative = (input - this->last_derivative_) / dt;
    }

    // discrete low pass filter, cuts out the
    // high frequency noise that can drive the controller crazy
    derivative = this->last_derivative_ + this->d_lpf_alpha_ * (derivative - this->last_derivative_);

    // update state
    this->last_input_ = input;
    this->last_derivative_ = derivative;
    // add in derivative component
    return this->kd_ * derivative;
  }
  return 0;
}

float FuzzyPid::getPid(float error, float dt) {
  float out_var;  // out调整量

  // 计算隶属度表
  float es[7], ecs[7], e, ec;
  int max_x = 0, max_y = 0;

  // 记录隶属度最大项及相应推理表的p、i、d值
  float lsd;
  int temp_p, temp_d, temp_i;
  float detkp, detkd, detki;  // 推理后的结果
  e = error;
  ec = error - this->last_input_;
  // 当温度差的绝对值小于Emax时，对pid的参数进行调整
  if (fabs(error) <= kEMax) {
    // 计算iError在es与ecs中各项的隶属度
    es[kNB] = FTraL(e * 5, -3, -1);
    es[kNM] = FTri(e * 5, -3, -2, 0);
    es[kNS] = FTri(e * 5, -3, -1, 1);
    es[kZO] = FTri(e * 5, -2, 0, 2);
    es[kPS] = FTri(e * 5, -1, 1, 3);
    es[kPM] = FTri(e * 5, 0, 2, 3);
    es[kPB] = FTraR(e * 5, 1, 3);

    ecs[kNB] = FTraL(ec * 30, -3, -1);
    ecs[kNM] = FTri(ec * 30, -3, -2, 0);
    ecs[kNS] = FTri(ec * 30, -3, -1, 1);
    ecs[kZO] = FTri(ec * 30, -2, 0, 2);
    ecs[kPS] = FTri(ec * 30, -1, 1, 3);
    ecs[kPM] = FTri(ec * 30, 0, 2, 3);
    ecs[kPB] = FTraR(ec * 30, 1, 3);

    // 计算隶属度表，确定e和ec相关联后表格各项隶属度的值
    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < 7; j++) {
        this->form_[i][j] = fand(es[i], ecs[j]);
      }
    }

    // 取出具有最大隶属度的那一项
    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < 7; j++) {
        if (this->form_[max_x][max_y] < this->form_[i][j]) {
          max_x = i;
          max_y = j;
        }
      }
    }

    // 进行模糊推理，并去模糊
    lsd = this->form_[max_x][max_y];
    temp_p = kp[max_x][max_y];
    temp_d = kd[max_x][max_y];
    temp_i = ki[max_x][max_y];

    if (temp_p == kNB) {
      detkp = uFTraL(lsd, -0.3, -0.1);
    } else if (temp_p == kNM) {
      detkp = uFTri(lsd, -0.3, -0.2, 0);
    } else if (temp_p == kNS) {
      detkp = uFTri(lsd, -0.3, -0.1, 0.1);
    } else if (temp_p == kZO) {
      detkp = uFTri(lsd, -0.2, 0, 0.2);
    } else if (temp_p == kPS) {
      detkp = uFTri(lsd, -0.1, 0.1, 0.3);
    } else if (temp_p == kPM) {
      detkp = uFTri(lsd, 0, 0.2, 0.3);
    } else if (temp_p == kPB) {
      detkp = uFTraR(lsd, 0.1, 0.3);
    }

    if (temp_d == kNB) {
      detkd = uFTraL(lsd, -3, -1);
    } else if (temp_d == kNM) {
      detkd = uFTri(lsd, -3, -2, 0);
    } else if (temp_d == kNS) {
      detkd = uFTri(lsd, -3, 1, 1);
    } else if (temp_d == kZO) {
      detkd = uFTri(lsd, -2, 0, 2);
    } else if (temp_d == kPS) {
      detkd = uFTri(lsd, -1, 1, 3);
    } else if (temp_d == kPM) {
      detkd = uFTri(lsd, 0, 2, 3);
    } else if (temp_d == kPB) {
      detkd = uFTraR(lsd, 1, 3);
    }

    if (temp_i == kNB) {
      detki = uFTraL(lsd, -0.06, -0.02);
    } else if (temp_i == kNM) {
      detki = uFTri(lsd, -0.06, -0.04, 0);
    } else if (temp_i == kNS) {
      detki = uFTri(lsd, -0.06, -0.02, 0.02);
    } else if (temp_i == kZO) {
      detki = uFTri(lsd, -0.04, 0, 0.04);
    } else if (temp_i == kPS) {
      detki = uFTri(lsd, -0.02, 0.02, 0.06);
    } else if (temp_i == kPM) {
      detki = uFTri(lsd, 0, 0.04, 0.06);
    } else if (temp_i == kPB) {
      detki = uFTraR(lsd, 0.02, 0.06);
    }
    // pid三项系数的修改
    this->kp_ += detkp;
    if (fabs(this->ki_) < 1e-9) {
      this->ki_ = 0;
    } else {
      this->ki_ += detki;
    }

    if (fabs(this->kd_) < 1e-9) {
      this->kd_ = 0;
    } else {
      this->kd_ += detkd;
    }
    //_kd=0;//取消微分作用

    // 对Kp,Ki进行限幅
    if (this->kp_ < 0) {
      this->kp_ = 0;
    }

    if (this->ki_ < 0) {
      this->ki_ = 0;
    }
  }

  return getP(error) + getI(error, dt) + getD(error, dt);
}

void FuzzyPid::operator()(const float p, const float i, const float d, const int16_t imaxval) {
  this->kp_ = p;
  this->ki_ = i;
  this->kd_ = d;
  this->imax_ = abs(imaxval);
}
