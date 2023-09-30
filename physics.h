#ifndef PHYSICS_H
#define PHYSICS_H

#include <string>

#include "math.h"

// Entities are assumed to have uniform density, therefore the center of mass and geometric center are the same.

class RVector {
 public:
  float x;
  float y;

  RVector(){};
  RVector(float x_in, float y_in);
  ~RVector(){};
};

RVector::RVector(float x_in, float y_in) {
    x = -x_in;
    y = -y_in;
}

class Force {
 public:
  float fx{0};
  float fy{0};

  Force(){};
  Force(float fx_in, float fy_in);
  ~Force(){};

  float magnitude();
  float direction();
};

Force::Force(float fx_in, float fy_in) {
  fx = fx_in;
  fy = fy_in;
}

float Force::magnitude() {
  return sqrt(pow(fx, 2) + pow(fy, 2));
}

float Force::direction() {
  return atan2(fy, fx);
}

class Moment {
 public:
  float value{0};

  Moment(){};
  Moment(Force force, RVector r);
  ~Moment(){};

  void calculate(Force force, RVector r);
};

Moment::Moment(Force force, RVector r) {
    calculate(force, r);
}

void Moment::calculate(Force force, RVector r) {
  value = r.x * force.fy - r.y * force.fx;
}

class MomentOfInertia {
 public:
  float value{0};

  void calculate_disk(float mass, float radius);
  void calculate_rect(float mass, float h, float w);
};

void MomentOfInertia::calculate_disk(float mass, float radius) {
  value = (1.0 / 4.0) * mass * pow(radius, 2);
}

void MomentOfInertia::calculate_rect(float mass, float h, float w) {
  value = (1.0 / 12.0) * mass * (pow(h, 2) + pow(w, 2));
}

#endif