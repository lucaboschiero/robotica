#include "include/circlecheck.h"


bool check(const Vector2d& point){
    double distance = (point - center).norm();
    return distance <= R;
}

Vector2d checkCircleandTranslate(const Vector2d& point) {
    
    if (check(point)) {

        // il punto è interno al cerchio, traslalo appena fuori dal cerchio
        Vector2d normalized = (point - center).normalized();
        Vector2d newPoint = center + normalized * R;

        return newPoint;

    } else {
        // il punto è esterno al cerchio, non fare nulla
        return point;
    }
}

int main() {
    Vector2d point1(0.45, 0.3);
    Vector2d point2(2, 2);

    Vector2d newPoint1 = checkCircleandTranslate(point1);
    Vector2d newPoint2 = checkCircleandTranslate(point2);

    cout << "Punto 1: (" << point1.x() << ", " << point1.y() << ")" << endl;
    cout << "Nuovo punto 1: (" << newPoint1.x() << ", " << newPoint1.y() << ")" << endl;
    cout << "Punto 2: (" << point2.x() << ", " << point2.y() << ")" << endl;
    cout << "Nuovo punto 2: (" << newPoint2.x() << ", " << newPoint2.y() << ")" << endl;

    return 0;
}