#ifndef REEDS_SHEPP_H
#define REEDS_SHEPP_H

#include <Arduino.h>
#include <vector>
#include <cmath>
#include "dubin.h"  // Reutiliza struct Pose { double x, y, theta }

// Valores con signo para que reflect() y timeflip() puedan negar directamente
enum RSSegType { LEFT = -1, STRAIGHT = 0, RIGHT = 1 };
enum Gear      { BACKWARD = -1, FORWARD = 1 };

struct RSPathSegment {
    float     length;
    RSSegType type;
    Gear      gear;
};

class ReedsShepp {
public:
    explicit ReedsShepp(float radius) : radius(radius) {}

    // Planificación
    std::vector<std::vector<RSPathSegment>> getAllPaths(const Pose &start, const Pose &end);
    int  getOptimalPathIndex(const std::vector<std::vector<RSPathSegment>> &paths);

    // Normalización: divide x,y por radius (theta ya en radianes, no cambia)
    Pose normalizePose(const Pose &p) const;
    // Desnormalización: multiplica todas las longitudes por radius → metros
    void denormalizePath(std::vector<RSPathSegment> &path) const;

    // Debug
    void printPath(const std::vector<RSPathSegment> &path);
    void printAllPaths(const std::vector<std::vector<RSPathSegment>> &paths);
    void runDemo(const std::vector<Pose> &waypoints);

private:
    float radius;

    // Geometría
    Pose  changeOfBasis(const Pose &p1, const Pose &p2);
    float mod2pi(float theta);
    void  R(float x, float y, float &r, float &theta);
    float M(float theta);   // normaliza ángulo a (-π, π]

    // Valida que una longitud/ángulo de tramo sea >= 0 (con tolerancia).
    // Si es negativo más allá de la tolerancia, el path candidato es
    // geométricamente inválido y debe descartarse (criterio estándar de
    // Reeds-Shepp: t,u,v deben ser no-negativos). Si es un negativo
    // despreciable por error de punto flotante, se recorta a 0.
    static bool nonneg(float &v);

    // Simetría de paths
    RSPathSegment              reverseGear    (const RSPathSegment &seg);
    RSPathSegment              reverseSteering(const RSPathSegment &seg);
    std::vector<RSPathSegment> reflect  (const std::vector<RSPathSegment> &path);
    std::vector<RSPathSegment> timeflip (const std::vector<RSPathSegment> &path);

    // 12 familias de paths Reeds-Shepp
    std::vector<RSPathSegment> path1 (float x, float y, float phi);
    std::vector<RSPathSegment> path2 (float x, float y, float phi);
    std::vector<RSPathSegment> path3 (float x, float y, float phi);
    std::vector<RSPathSegment> path4 (float x, float y, float phi);
    std::vector<RSPathSegment> path5 (float x, float y, float phi);
    std::vector<RSPathSegment> path6 (float x, float y, float phi);
    std::vector<RSPathSegment> path7 (float x, float y, float phi);
    std::vector<RSPathSegment> path8 (float x, float y, float phi);
    std::vector<RSPathSegment> path9 (float x, float y, float phi);
    std::vector<RSPathSegment> path10(float x, float y, float phi);
    std::vector<RSPathSegment> path11(float x, float y, float phi);
    std::vector<RSPathSegment> path12(float x, float y, float phi);
};

#endif
