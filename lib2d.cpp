#include "lib2d.h"

Point2D::Point2D (void) { 
  x = 0.0;
  y = 0.0;
}

Point2D::Point2D (double xx, double yy) {
  x = xx;
  y = yy;
}

void Point2D::setXY(double xx, double yy) {
  x = xx;
  y = yy;
}

void Point2D::setZero() {
  x = 0.0;
  y = 0.0;
}

bool Point2D::operator ==(const Point2D& pt) {
  return (x == pt.x) and (y == pt.y);
}

Point2D& Point2D::operator =(const Point2D& pt) {
  x = pt.x;
  y = pt.y;
}

Point2D& Point2D::operator +=(const Point2D& pt) {
  x += pt.x;
  y += pt.y;
}

Point2D& Point2D::operator -=(const Point2D& pt) {
  x -= pt.x;
  y -= pt.y;
}

Point2D Point2D::operator +(const Point2D& pt) {
  Point2D v;
  
  v.x = x + pt.x;
  v.y = y + pt.y;
  return v;
}

Point2D Point2D::operator -(const Point2D& pt) {
  Point2D v;
  
  v.x = x - pt.x;
  v.y = y - pt.y;
  return v; 
}

/***************************** Mètodes de Vector *****************************/

Vect2D::Vect2D () { }

Vect2D::Vect2D (double xx, double yy) {
  x = xx;
  y = yy;
}

Vect2D::Vect2D (Point2D dp, Point2D op) {
  x = dp.x - op.x;
  y = dp.y - op.y;
}

Vect2D::Vect2D (double dx, double dy, double ox, double oy) {
  x = dx - ox;
  y = dy - oy;
}

/* Multiplicacio i divisió per escalar */
Vect2D Vect2D::operator *(const double scalar) {
  Vect2D v;
  
  v.x = x * scalar;
  v.y = y * scalar;
  return v;
}

Vect2D Vect2D::operator /(const double scalar) {
  Vect2D v;
  
  v.x = x / scalar;
  v.y = y / scalar;
  return v;
}

double Vect2D::module() {
  return sqrt(pow(x, 2) + pow(y, 2));
}

Vect2D Vect2D::norm() {
  return *this/module();
}  
  