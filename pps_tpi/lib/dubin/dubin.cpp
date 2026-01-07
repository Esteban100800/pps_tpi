#include "dubin.h"

/* =========================
   Utilidades internas
   ========================= */

static double mod2pi(double a)
{
    while (a < 0)
        a += 2.0 * M_PI;
    while (a >= 2 * M_PI)
        a -= 2.0 * M_PI;
    return a;
}

static Vec2 operator+(const Vec2 &a, const Vec2 &b)
{
    return {a.x + b.x, a.y + b.y};
}

static Vec2 operator-(const Vec2 &a, const Vec2 &b)
{
    return {a.x - b.x, a.y - b.y};
}

static Vec2 operator*(double s, const Vec2 &v)
{
    return {s * v.x, s * v.y};
}

static double norm(const Vec2 &v)
{
    return std::sqrt(v.x * v.x + v.y * v.y);
}

static Vec2 normalize(const Vec2 &v)
{
    double n = norm(v);
    return {v.x / n, v.y / n};
}

// Diferencia angular según sentido
static double angleDiff(double from, double to, bool left)
{
    if (left)
        return mod2pi(to - from);
    else
        return mod2pi(from - to);
}

/* =========================
   LRL
   ========================= */

bool dubinsLRL(const Pose &start, const Pose &goal, double R, DubinsPath &path)
{
    // Centros de los círculos izquierdo-inicial y izquierdo-final
    Vec2 c1 = {
        start.x + R * std::cos(start.theta + M_PI_2),
        start.y + R * std::sin(start.theta + M_PI_2)};

    Vec2 c2 = {
        goal.x + R * std::cos(goal.theta + M_PI_2),
        goal.y + R * std::sin(goal.theta + M_PI_2)};

    // Distancia entre centros
    double D = norm(c2 - c1);

    // Condición de existencia LRL
    if (D > 4.0 * R)
        return false;

    // Cálculo del círculo intermedio
    double alpha = std::atan2(c2.y - c1.y, c2.x - c1.x);
    double beta = std::acos(D / (4.0 * R));

    double theta = alpha + beta;

    Vec2 c3 = {
        c1.x + 2.0 * R * std::cos(theta),
        c1.y + 2.0 * R * std::sin(theta)};

    // Puntos de cambio de círculo (clave)
    Vec2 p1 = c3 + R * normalize(c1 - c3);
    Vec2 p2 = c3 + R * normalize(c2 - c3);

    /* =========================
       Longitudes de arco
       ========================= */

    // Arco 1 (LEFT)
    double a1s = std::atan2(start.y - c1.y, start.x - c1.x);
    double a1e = std::atan2(p1.y - c1.y, p1.x - c1.x);
    double L1 = R * angleDiff(a1s, a1e, true);

    // Arco 2 (RIGHT)
    double a2s = std::atan2(p1.y - c3.y, p1.x - c3.x);
    double a2e = std::atan2(p2.y - c3.y, p2.x - c3.x);
    double L2 = R * angleDiff(a2s, a2e, false);

    // Arco 3 (LEFT)
    double a3s = std::atan2(p2.y - c2.y, p2.x - c2.x);
    double a3e = std::atan2(goal.y - c2.y, goal.x - c2.x);
    double L3 = R * angleDiff(a3s, a3e, true);

    /* =========================
       Construcción del path
       ========================= */

    path.seg[0] = {SEG_LEFT, L1};
    path.seg[1] = {SEG_RIGHT, L2};
    path.seg[2] = {SEG_LEFT, L3};

    path.totalLength = L1 + L2 + L3;

    return true;
}
