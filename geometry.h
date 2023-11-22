#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "math.h"
#include <iostream>

class Point {
 public:
  float x;
  float y;

  Point(){};
  ~Point(){};
  Point(float x_in, float y_in);

  Point operator+(Point other);
  Point operator-(Point other);
};

Point::Point(float x_in, float y_in) {
  x = x_in;
  y = y_in;
}

Point Point::operator+(Point other) {
  Point p;
  p.x = x + other.x;
  p.y = y + other.y;
  return p;
}

Point Point::operator-(Point other) {
  Point p;
  p.x = x - other.x;
  p.y = y - other.y;
  return p;
}

enum Shape { circle, box };

class Vector2 {
 public:
  float value[2];

  Vector2();
  Vector2(float a, float b);
  Vector2(Point p);  // Convert a point to a vector
  Vector2(Point a, Point b); // Vector from a to b
  ~Vector2(){};

  float operator[](int idx);
  Vector2 operator+(Vector2 other);
  Vector2 operator-(Vector2 other);
  Vector2 operator*(int number);
  Vector2 operator*(float number);
  float operator*(Point pt);
  Vector2 operator=(Point p);

  float norm();
  void convert_to_unit();
  float dot(Vector2 other);
  float cross(Vector2 other);
  Vector2 cross(float number);

  void print();
  void reset();
};

Vector2::Vector2() {
  value[0] = 0;
  value[1] = 0;
}

Vector2::Vector2(float a, float b) {
  value[0] = a;
  value[1] = b;
}

Vector2::Vector2(Point p) {
  value[0] = p.x;
  value[1] = p.y;
}

Vector2::Vector2(Point a, Point b) {
  value[0] = b.x - a.x;
  value[1] = b.y - a.y;
}

float Vector2::operator[](int idx) {
  return value[idx];
}

Vector2 Vector2::operator+(Vector2 other) {
  Vector2 result(value[0] + other[0], value[1] + other[1]);
  return result;
}

Vector2 Vector2::operator-(Vector2 other) {
  Vector2 result(value[0] - other[0], value[1] - other[1]);
  return result;
}

Vector2 Vector2::operator*(int number) {
  Vector2 result(value[0] * number, value[1] * number);
  return result;
}

Vector2 Vector2::operator*(float number) {
  Vector2 result(value[0] * number, value[1] * number);
  return result;
}

float Vector2::operator*(Point pt) {
  return value[0] * pt.x + value[1] * pt.y;
}

Vector2 Vector2::operator=(Point p) {
  Vector2 result(p);
  return result;
}

float Vector2::norm() {
  return sqrt(pow(value[0], 2) + pow(value[1], 2));
}

void Vector2::convert_to_unit() {
  float magnitude = norm();
  value[0] = value[0] / magnitude;
  value[1] = value[1] / magnitude;
}

float Vector2::dot(Vector2 other) {
  return value[0] * other[0] + value[1] * other[1];
}

float Vector2::cross(Vector2 other) {
  return value[0] * other[1] - value[1] * other[0];
}

Vector2 Vector2::cross(float number) {
  Vector2 result(-1 * value[1] * number, value[0] * number);
  return result;
}

void Vector2::print() {
  std::cout << value[0] << ", " << value[1] << "\n";
}

void Vector2::reset() {
  value[0] = 0;
  value[1] = 0;
}

struct Edge {
  Point v1;        // Vertex 1
  Point v2;        // Vertex 2
  float distance;  // Shortest distance from edge to origin
  float index;     // Index of edge in simplex
  Vector2 normal;  // Vector normal to edge
  Point max;       // Farthest projection along a vector
};

#endif