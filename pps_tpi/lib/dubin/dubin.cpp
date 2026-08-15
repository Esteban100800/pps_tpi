#include "dubin.h"
#include <algorithm>

DubinsPlanner::DubinsPlanner(float R_) : R(R_) {}

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

double DubinsPlanner::mod2pi(double a)
{
    while (a < 0)
        a += 2.0 * M_PI;
    while (a >= 2.0 * M_PI)
        a -= 2.0 * M_PI;
    return a;
}

double DubinsPlanner::angleDiff(double from, double to, bool left)
{
    return left ? mod2pi(to - from) : mod2pi(from - to);
}

double DubinsPlanner::norm(const Vec2 &v)
{
    return std::sqrt(v.x * v.x + v.y * v.y);
}

bool DubinsPlanner::getTangents(
    double x1, double y1, double r1,
    double x2, double y2, double r2,
    SegmentType type,
    Vec2 &t1, Vec2 &t2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    double d_sq = dx * dx + dy * dy;

    if (d_sq <= (r1 - r2) * (r1 - r2))
        return false;

    double d = std::sqrt(d_sq);
    double vx = dx / d;
    double vy = dy / d;

    double tangents[4][4];
    int i = 0;

    for (int s1 = 0; s1 < 2; ++s1)
    {
        double sign1 = (s1 == 0) ? 1.0 : -1.0;
        double c = (r1 - sign1 * r2) / d;
        if (c * c > 1.0)
            continue;

        double h = std::sqrt(std::max(0.0, 1.0 - c * c));

        for (int s2 = 0; s2 < 2; ++s2)
        {
            double sign2 = (s2 == 0) ? 1.0 : -1.0;

            double nx = vx * c - sign2 * h * vy;
            double ny = vy * c + sign2 * h * vx;

            tangents[i][0] = x1 + r1 * nx;
            tangents[i][1] = y1 + r1 * ny;
            tangents[i][2] = x2 + sign1 * r2 * nx;
            tangents[i][3] = y2 + sign1 * r2 * ny;
            i++;
        }
    }

    if (i < 2)
        return false;

    int idx = -1;
    if (type == SEG_LEFT)
        idx = 1; // LSL
    else if (type == SEG_RIGHT)
        idx = 0; // RSR

    if (idx < 0 || idx >= i)
        return false;

    t1 = {tangents[idx][0], tangents[idx][1]};
    t2 = {tangents[idx][2], tangents[idx][3]};
    return true;
}

/* =========================
   LRL
   ========================= */

bool DubinsPlanner::dubinsLRL(const Pose &start, const Pose &goal, DubinsPath &path)
{
    // Centros de los circulos izquierdo-inicial y izquierdo-final
    Vec2 c1 = {
        start.x + R * std::cos(start.theta + M_PI_2),
        start.y + R * std::sin(start.theta + M_PI_2)};

    Vec2 c2 = {
        goal.x + R * std::cos(goal.theta + M_PI_2),
        goal.y + R * std::sin(goal.theta + M_PI_2)};

    // Distancia entre centros
    double D = norm(c2 - c1);

    // Condicion de existencia LRL
    if (D > 4.0 * R)
        return false;

    // Cálculo del circulo intermedio
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


    path.seg[0] = {SEG_LEFT, L1};
    path.seg[1] = {SEG_RIGHT, L2};
    path.seg[2] = {SEG_LEFT, L3};

    path.totalLength = L1 + L2 + L3;

    return true;
}

bool DubinsPlanner::dubinsLSL(const Pose &s, const Pose &g, DubinsPath &path)
{
    Vec2 c1{
        s.x + R * cos(s.theta + M_PI_2),
        s.y + R * sin(s.theta + M_PI_2)};

    Vec2 c2{
        g.x + R * cos(g.theta + M_PI_2),
        g.y + R * sin(g.theta + M_PI_2)};

    Vec2 p1, p2;
    if (!getTangents(c1.x, c1.y, R, c2.x, c2.y, R, SEG_LEFT, p1, p2))
        return false;

    float D = norm(c2 - c1);
    if (D < 2.0 * R)
        return false;

    double a1s = atan2(s.y - c1.y, s.x - c1.x);
    double a1e = atan2(p1.y - c1.y, p1.x - c1.x);
    double L1 = R * angleDiff(a1s, a1e, true);

    double L2 = norm({p2.x - p1.x, p2.y - p1.y});

    double a3s = atan2(p2.y - c2.y, p2.x - c2.x);
    double a3e = atan2(g.y - c2.y, g.x - c2.x);
    double L3 = R * angleDiff(a3s, a3e, true);

    path.seg[0] = {SEG_LEFT, L1};
    path.seg[1] = {SEG_STRAIGHT, L2};
    path.seg[2] = {SEG_LEFT, L3};
    path.totalLength = L1 + L2 + L3;

    return true;
}

bool DubinsPlanner::dubinsLSR(const Pose &s, const Pose &g, DubinsPath &path)
{
    // Centro izquierdo inicial
    Vec2 c1{
        s.x + R * cos(s.theta + M_PI_2),
        s.y + R * sin(s.theta + M_PI_2)};

    // Centro derecho final
    Vec2 c2{
        g.x + R * cos(g.theta - M_PI_2),
        g.y + R * sin(g.theta - M_PI_2)};

    Vec2 dc = c2 - c1;
    double D = norm(dc);

    // Condicion de existencia
    if (D < 2.0 * R)
        return false;


    double theta = atan2(dc.y, dc.x);
    double phi = acos(2.0 * R / D);

    double ang = theta + phi;

    // Puntos tangentes
    Vec2 p1{
        c1.x + R * cos(ang - M_PI_2),
        c1.y + R * sin(ang - M_PI_2)};

    Vec2 p2{
        c2.x + R * cos(ang + M_PI_2),
        c2.y + R * sin(ang + M_PI_2)};

    // === Arco 1 (LEFT) ===
    double a1s = atan2(s.y - c1.y, s.x - c1.x);
    double a1e = atan2(p1.y - c1.y, p1.x - c1.x);
    double L1 = R * angleDiff(a1s, a1e, true);

    // === Recta ===
    double L2 = norm(p2 - p1);

    // === Arco 3 (RIGHT) ===
    double a3s = atan2(p2.y - c2.y, p2.x - c2.x);
    double a3e = atan2(g.y - c2.y, g.x - c2.x);
    double L3 = R * angleDiff(a3s, a3e, false);

    path.seg[0] = {SEG_LEFT, L1};
    path.seg[1] = {SEG_STRAIGHT, L2};
    path.seg[2] = {SEG_RIGHT, L3};

    path.totalLength = L1 + L2 + L3;
    return true;
}

bool DubinsPlanner::dubinsRLR(const Pose &start, const Pose &goal, DubinsPath &path)
{
    // Centro derecho inicial
    Vec2 c1 = {
        start.x + R * cos(start.theta - M_PI_2),
        start.y + R * sin(start.theta - M_PI_2)};

    // Centro derecho final
    Vec2 c2 = {
        goal.x + R * cos(goal.theta - M_PI_2),
        goal.y + R * sin(goal.theta - M_PI_2)};

    double D = norm(c2 - c1);
    // Condicion de existencia
    if (D > 4.0 * R)
        return false;

    double alpha = atan2(c2.y - c1.y, c2.x - c1.x);
    double beta = acos(D / (4.0 * R));
    double theta = alpha - beta;

    // Centro del circulo intermedio
    Vec2 c3 = {
        c1.x + 2.0 * R * cos(theta),
        c1.y + 2.0 * R * sin(theta)};

    Vec2 p1 = c3 + R * normalize(c1 - c3);
    Vec2 p2 = c3 + R * normalize(c2 - c3);

    // === Arco 1 (RIGHT) ===
    double a1s = atan2(start.y - c1.y, start.x - c1.x);
    double a1e = atan2(p1.y - c1.y, p1.x - c1.x);
    double L1 = R * angleDiff(a1s, a1e, false);

    // === Arco 2 (LEFT) ===
    double a2s = atan2(p1.y - c3.y, p1.x - c3.x);
    double a2e = atan2(p2.y - c3.y, p2.x - c3.x);
    double L2 = R * angleDiff(a2s, a2e, true);

    // === Arco 3 (RIGHT) ===
    double a3s = atan2(p2.y - c2.y, p2.x - c2.x);
    double a3e = atan2(goal.y - c2.y, goal.x - c2.x);
    double L3 = R * angleDiff(a3s, a3e, false);

    path.seg[0] = {SEG_RIGHT, L1};
    path.seg[1] = {SEG_LEFT, L2};
    path.seg[2] = {SEG_RIGHT, L3};

    path.totalLength = L1 + L2 + L3;
    return true;
}

bool DubinsPlanner::dubinsRSL(const Pose &s, const Pose &g, DubinsPath &path)
{
    // Centro derecho inicial
    Vec2 c1{
        s.x + R * cos(s.theta - M_PI_2),
        s.y + R * sin(s.theta - M_PI_2)};

    // Centro izquierdo final
    Vec2 c2{
        g.x + R * cos(g.theta + M_PI_2),
        g.y + R * sin(g.theta + M_PI_2)};

    Vec2 dc = c2 - c1;
    double D = norm(dc);

    if (D < 2.0 * R)
        return false;

    double theta = atan2(dc.y, dc.x);
    double phi = acos(2.0 * R / D);

    double ang = theta - phi;

    Vec2 p1{
        c1.x + R * cos(ang + M_PI_2),
        c1.y + R * sin(ang + M_PI_2)};

    Vec2 p2{
        c2.x + R * cos(ang - M_PI_2),
        c2.y + R * sin(ang - M_PI_2)};

    // === Arco 1 (RIGHT) ===
    double a1s = atan2(s.y - c1.y, s.x - c1.x);
    double a1e = atan2(p1.y - c1.y, p1.x - c1.x);
    double L1 = R * angleDiff(a1s, a1e, false);

    // === Recta ===
    double L2 = norm(p2 - p1);

    // === Arco 3 (LEFT) ===
    double a3s = atan2(p2.y - c2.y, p2.x - c2.x);
    double a3e = atan2(g.y - c2.y, g.x - c2.x);
    double L3 = R * angleDiff(a3s, a3e, true);

    path.seg[0] = {SEG_RIGHT, L1};
    path.seg[1] = {SEG_STRAIGHT, L2};
    path.seg[2] = {SEG_LEFT, L3};

    path.totalLength = L1 + L2 + L3;
    return true;
}

bool DubinsPlanner::dubinsRSR(const Pose &s, const Pose &g, DubinsPath &path)
{
    // Centro derecho inicial
    Vec2 c1{
        s.x + R * cos(s.theta - M_PI_2),
        s.y + R * sin(s.theta - M_PI_2)};

    // Centro derecho final
    Vec2 c2{
        g.x + R * cos(g.theta - M_PI_2),
        g.y + R * sin(g.theta - M_PI_2)};

    // Distancia entre centros
    double D = norm(c2 - c1);

    // Condicion de existencia 
    if (D < 2.0 * R)
    {
        return false;
    }

    // Tangentes externas RSR
    Vec2 p1, p2;
    if (!getTangents(
            c1.x, c1.y, R,
            c2.x, c2.y, R,
            SEG_RIGHT,
            p1, p2))
    {
        return false;
    }

    /* ===== Arco 1 (RIGHT) ===== */
    double a1s = atan2(s.y - c1.y, s.x - c1.x);
    double a1e = atan2(p1.y - c1.y, p1.x - c1.x);
    double L1 = R * angleDiff(a1s, a1e, false);

    /* ===== Tramo recto ===== */
    double L2 = norm(p2 - p1);

    /* ===== Arco 3 (RIGHT) ===== */
    double a3s = atan2(p2.y - c2.y, p2.x - c2.x);
    double a3e = atan2(g.y - c2.y, g.x - c2.x);
    double L3 = R * angleDiff(a3s, a3e, false);

    /* ===== Construcción del path ===== */
    path.seg[0] = {SEG_RIGHT, L1};
    path.seg[1] = {SEG_STRAIGHT, L2};
    path.seg[2] = {SEG_RIGHT, L3};

    path.totalLength = L1 + L2 + L3;

    return true;
}

bool DubinsPlanner::compute(
    const Pose &start,
    const Pose &goal,
    DubinsPath &bestPath)
{
    bool found = false;
    DubinsPath candidate;

    bestPath.totalLength = 1e9; // infinito

    // ===== LSL =====
    if (dubinsLSL(start, goal, candidate))
    {
        if (candidate.totalLength < bestPath.totalLength)
        {
            bestPath = candidate;
            found = true;
        }
    }

    // ===== LRL =====
    if (dubinsLRL(start, goal, candidate))
    {
        if (candidate.totalLength < bestPath.totalLength)
        {
            bestPath = candidate;
            found = true;
        }
    }

    // ===== LSR =====
    if (!dubinsLSL(start, goal, candidate))
    {
        if (dubinsLSR(start, goal, candidate))
        {
            if (candidate.totalLength < bestPath.totalLength)
            {
                bestPath = candidate;
                found = true;
            }
        }

        if (dubinsRSL(start, goal, candidate))
        {
            if (candidate.totalLength < bestPath.totalLength)
            {
                bestPath = candidate;
                found = true;
            }
        }
    }

    // ===== RLR =====
    if (dubinsRLR(start, goal, candidate))
    {
        if (candidate.totalLength < bestPath.totalLength)
        {
            bestPath = candidate;
            found = true;
        }
    }

    // ===== RSR =====
    if (dubinsRSR(start, goal, candidate))
    {
        if (candidate.totalLength < bestPath.totalLength)
        {
            bestPath = candidate;
            found = true;
        }
    }

    return found;
}
float DubinsPlanner::compute_rmin(float delta_max)
{

    const float L = 0.175f;                                  // Longitud del vehiculo en metros
    const float W = 0.18f;                                   // Distancia entre ruedas en metros
    const float delta_max_rad = delta_max * (M_PI / 180.0f); 

    // Cálculo del radio mínimo de giro
    float r_min = (L / tanf(delta_max_rad)) + (W / 2.0f);

    return r_min;
}

Pose DubinsPlanner::getStartPose()
{
    return startPose;
}

Pose DubinsPlanner::getGoalPose()
{
    return goalPose;
}

void DubinsPlanner::setStartPose(const Pose &pose)
{
    startPose = pose;
}

void DubinsPlanner::setGoalPose(const Pose &pose)
{
    goalPose = pose;
}

int DubinsPlanner::getSegmentIndex()
{
    return segmentIndex;
}

void DubinsPlanner::setSegmentIndex(int index)
{
    segmentIndex = index;
}

float DubinsPlanner::getAngleStraight()
{
    return angle_straight;
}

void DubinsPlanner::setAngleStraight(float angle)
{
    angle_straight = angle;
}

bool DubinsPlanner::getNewSegment()
{
    return new_segment;
}

void DubinsPlanner::setNewSegment(bool newSegment)
{
    new_segment = newSegment;
}

