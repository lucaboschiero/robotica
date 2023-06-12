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


double check_z(double z){
    if(z>min_z ){
        return min_z;      //punto troppo basso (tocca il tavolo), lo alzo
    }else if(z<max_z){
        return max_z;      //punto troppo alto, lo abbasso
    }else{
        // il punto non è ne troppo alto ne troppo basso, lo lascio così
        return z;
    }
}

bool check_right_trajectory(Vector3d pos_blocchetto){
    if(pos_blocchetto(0)>-0.5 && pos_blocchetto(0)<=0.0 && pos_blocchetto(1)>-0.2 && pos_blocchetto(1)<=0.45){
        return true;
    }
    return false;
}

int main() {
    Vector2d point1(0.45, -0.9);
    Vector2d point2(2, 2);
    Vector3d point3(0.1,0.3,0.85);

    Vector2d newPoint1 = checkCircleandTranslate(point1);
    newPoint1(1)=check_z(point1(1));
    Vector2d newPoint2 = checkCircleandTranslate(point2);

    cout << "Punto 1: (" << point1.x() << ", " << point1.y() << ")" << endl;
    cout << "Nuovo punto 1: (" << newPoint1.x() << ", " << newPoint1.y() << ")" << endl;
    cout << "Punto 2: (" << point2.x() << ", " << point2.y() << ")" << endl;
    cout << "Nuovo punto 2: (" << newPoint2.x() << ", " << newPoint2.y() << ")" << endl;
    cout << check_right_trajectory(point3)<<endl;
    return 0;
}