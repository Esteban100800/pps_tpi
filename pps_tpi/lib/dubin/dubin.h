#ifndef DUBINS_H
#define DUBINS_H

#include <cmath>

/* =========================
   Tipos básicos
   ========================= */

struct Pose {
    double x;
    double y;
    double theta;   // radianes
};

struct Vec2 {
    double x;
    double y;
};

/* =========================
   Dubins
   ========================= */

enum SegmentType {
    SEG_LEFT,
    SEG_RIGHT,
    SEG_STRAIGHT
};

struct DubinsSegment {
    SegmentType type;
    double length;   // longitud del tramo
};

struct DubinsPath {
    DubinsSegment seg[3];
    double totalLength;
};

/* =========================
   API pública
   ========================= */

// Calcula una trayectoria Dubins tipo LRL
// Devuelve true si existe, false si no es válida
bool dubinsLRL(
    const Pose& start,
    const Pose& goal,
    double R,
    DubinsPath& path
);

#endif // DUBINS_H
