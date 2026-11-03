// Reads lines on stdin and prints "x y z" for each, from the firmware's hexa_kin:
//   F leg coxa femur tibia   leg_fk foot position
//   N leg                    neutral_foot (the standing stance)
// Used by test_urdf_fk.py.
#include <cstdio>

#include "hexa_kin.h"

int main() {
    const hexa::Geometry& g = hexa::minihexa_geometry();
    char kind;
    int leg;
    while (std::scanf(" %c %d", &kind, &leg) == 2) {
        hexa::Vec3 p;
        if (kind == 'F') {
            float c, f, t;
            if (std::scanf("%f %f %f", &c, &f, &t) != 3) return 1;
            p = hexa::leg_fk(g, leg, {c, f, t});
        } else {
            p = hexa::neutral_foot(g, leg);
        }
        std::printf("%.7f %.7f %.7f\n", p.x, p.y, p.z);
    }
    return 0;
}
