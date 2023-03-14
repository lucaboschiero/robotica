#ifndef CIRCLECHECK_H
#define CIRCLECHECK_H

#include <cmath>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


const double R=0.15;
const Vector2d center(0.0, 0.0); // centro del cerchio

//funzioni
Vector2d checkCircleandTranslate(const Vector2d& point);
bool check(const Vector2d& point);


#endif