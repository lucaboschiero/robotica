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
const double min_z=0.73;
const double max_z=0.4;

//funzioni
Vector2d checkCircleandTranslate(const Vector2d& point);
double check_z(double z);
bool check(const Vector2d& point);
bool check_right_trajectory(Vector3d pos_blocchetto);


#endif