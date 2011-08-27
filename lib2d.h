#ifndef LIBP2D_H
#define LIBP2D_H

#include <math.h>

class Point2D {
  private:
  public:
    double x;
    double y;
    
    Point2D (void);
    Point2D (double xx, double yy);
    
    void setXY(double xx, double yy);
    void setZero();
    bool isZero();
    
    bool operator ==(const Point2D& pt);
    Point2D& operator =(const Point2D& pt);
    Point2D& operator +=(const Point2D& pt);
    Point2D& operator -=(const Point2D& pt);
    /* Producte i divisió escalar */
    Point2D& operator *=(double s);
    Point2D& operator /=(double s);
    
    Point2D operator +(const Point2D pt);
    Point2D operator -(const Point2D pt);
    
};

class Vect2D : public Point2D {
  private:
  public:    
    Vect2D ();
    Vect2D (double xx, double yy);
    Vect2D (Point2D dp, Point2D op);
    Vect2D (double dx, double dy, double ox, double oy);

    Vect2D& operator =(const Vect2D& vec);
    Vect2D operator +(const Vect2D vec);
    Vect2D operator -(const Vect2D vec);
    Vect2D& operator +=(const Vect2D& pt);
    Vect2D& operator -=(const Vect2D& pt);
    /* Producte i divisió escalar */
    Vect2D& operator *=(double s);
    Vect2D& operator /=(double s);

    /* Multiplicacio i divisió per escalar */
    Vect2D operator *(const double scalar);
    Vect2D operator /(const double scalar);
    
    Vect2D norm();
    Vect2D norm(const double m);
    double module();

};  

#endif