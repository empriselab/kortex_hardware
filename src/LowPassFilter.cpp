#include "LowPassFilter.h"

#include <cmath>
#include <iostream>

LowPassFilter::LowPassFilter(
    double sampling_rate, double cutoff_frequency, int dof)
{
  // Initialize alpha (filter coefficient) based on sampling rate and cutoff
  // frequency and set previous torque vector size to dof
  alpha_ = 1 / (1 + 2 * M_PI * cutoff_frequency * sampling_rate);
  tau_J_prev.resize(dof);
  filtered_tau_J.resize(dof);
  std::cout << "Low pass filter initialized with alpha = " << alpha_
            << std::endl;
}

void LowPassFilter::initLPF(std::vector<double>& init_tau_J)
{
  // Initialize the filter with the first measurement
  tau_J_prev = init_tau_J;
}

std::vector<double> LowPassFilter::getFilteredEffort(
    std::vector<double>& tau_J_raw)
{
  // Filter the torque sensor data
  for (int i = 0; i < filtered_tau_J.size(); i++)
  {
    filtered_tau_J[i] = tau_J_prev[i] * alpha_ + tau_J_raw[i] * (1 - alpha_);
    tau_J_prev[i] = filtered_tau_J[i];
  }
  return filtered_tau_J;
}