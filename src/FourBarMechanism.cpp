#include "FourBarMechanism.h"

namespace avionics_sim{
FourBarMechanism::FourBarMechanism(const double frame_crank_ratio, const double frame_rocker_ratio){
  _k1 = frame_crank_ratio;
  _k2 = frame_rocker_ratio;
}

double FourBarMechanism::transmit(double normalized_input){

  double crank_angle_rad = crank_angle_from_normalized_input(normalized_input);

  double output_rad = asin(_k1 + _k2 * sin((crank_angle_rad)));

  output_rad = bound_output_rad(output_rad);

  return output_rad;
}


double FourBarMechanism::crank_angle_from_normalized_input(double normalized_input){
  double bounded_input = bound_normalized_input(normalized_input);

  double crank_angle_rad = DEG2RAD(bounded_input * 100);

  return crank_angle_rad;
}

double FourBarMechanism::bound_normalized_input(const double noramlized_input) {
  if (noramlized_input < -1.0) {
    return -1;
  } else if (noramlized_input > 1) {
    return 1;
  }

  return noramlized_input;
}

double FourBarMechanism::bound_output_rad(const double deflection_rad){
  double bounded_deflection_rad = deflection_rad;

  if (deflection_rad > FourBarMechanism::MAX_DEFLECTION_RAD){
    bounded_deflection_rad = FourBarMechanism::MAX_DEFLECTION_RAD;
  }
  else if (deflection_rad < FourBarMechanism::MIN_DEFLECTION_RAD) {
    bounded_deflection_rad = FourBarMechanism::MIN_DEFLECTION_RAD;
  }

  return bounded_deflection_rad;
}

}
