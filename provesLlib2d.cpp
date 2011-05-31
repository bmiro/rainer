#include <stdio.h>
#include "lib2d.h"

int main(int argc, char **argv) {

  Point2D p1(1.0, 2.0);
  Point2D p2(2.0, 3.0);
  Point2D p3;
  Point2D p4;

  p3 = p1 + p2;
  printf("p3(%f, %f)\n", p3.x, p3.y);
  p4 = p1;
  printf("p4(%f, %f)\n", p4.x, p4.y);
  p4 += p1;
  printf("p4(%f, %f)\n", p4.x, p4.y);

}