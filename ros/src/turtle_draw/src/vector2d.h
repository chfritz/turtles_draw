#ifndef vector2d_h
#define vector2d_h

#include <cmath>

class Vector2D {
  /** a simple, 2d point class */

public:
    float x, y;

    Vector2D(double _x, double _y) : x(_x), y(_y) {}

    /** vector substraction */
    Vector2D operator-(const Vector2D& other) {
      Vector2D rtv(x - other.x, y - other.y);
      return rtv;
    }

    /** vector addition */
    Vector2D operator+(const Vector2D& other) {
      Vector2D rtv(x + other.x, y * other.y);
      return rtv;
    }

    /** length of this vector, L_2 norm */
    double length() {
      return sqrt(x * x + y * y);
    }

    /** the angle of the vector */
    double angle() {
      return atan2(y, x);
    }
};

#endif
