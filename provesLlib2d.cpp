#include <stdio.h>
#include "lib2d.h"

int main(int argc, char **argv) {

//   Point2D p1(1.0, 2.0);
//   Point2D p2(2.0, 3.0);
//   Point2D p3;
//   Point2D p4;
// 
//   printf("p1 és (%f, %f)\n", p1.x, p1.y);
//   printf("p2 és (%f, %f)\n", p2.x, p2.y);
// 
//   p3 = p1 + p2;
//   printf("p3 és p1 + p2 (%f, %f)\n", p3.x, p3.y);
//   p4 = p1;
//   printf("p4 és p1 (%f, %f)\n", p4.x, p4.y);
//   p4 += p1;
//   printf("p4 += p1 és (%f, %f)\n", p4.x, p4.y);
//   
//   Vect2D v(p2, p1);
//   printf("v es del p2 a p1 (%f, %f)\n", v.x, v.y);
//   v += v;
//   printf("v += v és (%f, %f)\n", v.x, v.y);
//   v = v * 4.0;
//   printf("v = v * 4 és (%f, %f)\n", v.x, v.y);
//   printf("|v| és %f)\n", v.module());
//   v = v.norm();
//   printf("v normalitzat és (%f, %f)\n", v.x, v.y);
//   printf("|v| normalitzat és %f)\n", v.module());
//   
//   Vect2D o(p2, p1);
//   
//   v += o * 0.6;
//   
//   printf(" v += o * 0.6 és (%f, %f)", v.x, v.y);

  Vect2D v(2.0, 4.0);
  v = v.norm(2);
  
  printf(" %f %f %f \n", v.x, v.y, v.module());


}