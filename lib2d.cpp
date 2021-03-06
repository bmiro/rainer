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

bool Point2D::isZero() {
  return (x == 0.0 and y == 0.0);
}

bool Point2D::operator ==(const Point2D& pt) {
  return (x == pt.x) and (y == pt.y);
}

Point2D& Point2D::operator =(const Point2D& pt) {
  x = pt.x;
  y = pt.y;
  return *this;
}

Point2D& Point2D::operator +=(const Point2D& pt) {
  x += pt.x;
  y += pt.y;
  return *this;
}

Point2D& Point2D::operator -=(const Point2D& pt) {
  x -= pt.x;
  y -= pt.y;
  return *this;
}

Point2D& Point2D::operator *=(double s) {
  x *= s;
  y *= s;
  return *this;
}

Point2D& Point2D::operator /=(double s) {
  x /= s;
  y /= s;
  return *this;
}

Point2D Point2D::operator +(const Point2D pt) {
  Point2D v;
  
  v.x = x + pt.x;
  v.y = y + pt.y;
  return v;
}

Point2D Point2D::operator -(const Point2D pt) {
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

Vect2D& Vect2D::operator =(const Vect2D& vec) {
  x = vec.x;
  y = vec.y;
  return *this;
}

Vect2D Vect2D::operator +(const Vect2D vec) {
  Vect2D v;
  
  v.x = x + vec.x;
  v.y = y + vec.y;
  return v;
}

Vect2D Vect2D::operator -(const Vect2D vec) {
  Vect2D v;
  
  v.x = x - vec.x;
  v.y = y - vec.y;
  return v;
}

Vect2D& Vect2D::operator +=(const Vect2D& vec) {
  x += vec.x;
  y += vec.y;
  return *this;
}

Vect2D& Vect2D::operator -=(const Vect2D& vec) {
  x -= vec.x;
  y -= vec.y;
  return *this;
}

Vect2D& Vect2D::operator *=(double s) {
  x *= s;
  y *= s;
  return *this;
}

Vect2D& Vect2D::operator /=(double s) {
  x /= s;
  y /= s;
  return *this;
}

/* Multiplicacio i divisió per escalar generant nou vector */
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
  if (module() != double(0)) {
    return *this/module();
  } else {
    setZero();
    return *this;
  }
}

/* Normalitza a un mòdul concret */
Vect2D Vect2D::norm(const double m) {
  if (module() != double(0)) {
    return (*this/module()) * m;
  } else {
    setZero();
    return *this;
  }
}  