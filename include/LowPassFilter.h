#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

#include <vector>

// Low pass filter class for the joint torque sensor
class LowPassFilter
{
public:
  LowPassFilter(double sampling_rate, double cutoff_frequency, int dof);

  void initLPF(std::vector<double>& init_tau_J);

  std::vector<double> getFilteredEffort(std::vector<double>& tau_J_raw);

private:
  double alpha_;
  std::vector<double> tau_J_prev;
  std::vector<double> filtered_tau_J;
};

#endif
