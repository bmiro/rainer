#ifndef LIBP2D_H
#define LIBP2D_H

#include <math.h>

class Point2D {
  private:
  public:
    double x;
    double y;
    
    Point2D ();
    Point2D (double xx, double yy);
    
    void setXY(double xx, double yy);
    void setZero();
    
    bool operator ==(const Point2D& pt);
    Point2D& operator =(const Point2D& pt);
    Point2D& operator +=(const Point2D& pt);
    Point2D& operator -=(const Point2D& pt);
    Point2D operator +(const Point2D& pt);
    Point2D operator -( const Point2D& pt);
    
};

class Vect2D : public Point2D {
  private:
  public:    
    Vect2D ();
    Vect2D (double xx, double yy);
    Vect2D (double dx, double dy, double ox, double oy);
    
    /* Multiplicacio i divisió per escalar */
    Vect2D operator *(const double scalar);
    Vect2D operator /(const double scalar);
    
    Vect2D norm();
    double module();

};  

#endif