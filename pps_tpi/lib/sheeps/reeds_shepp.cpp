#include "reeds_shepp.h"

// ─────────────────────────────────────────────────────────────────────────────
//  API pública
// ─────────────────────────────────────────────────────────────────────────────

std::vector<std::vector<RSPathSegment>> ReedsShepp::getAllPaths(const Pose &start,
                                                                const Pose &end)
{
    std::vector<std::vector<RSPathSegment>> all_paths;

    Pose local_goal = changeOfBasis(start, end);
    float x     = local_goal.x;
    float y     = local_goal.y;
    float theta = local_goal.theta;

    std::vector<std::vector<RSPathSegment> (ReedsShepp::*)(float, float, float)>
        path_functions = {
            &ReedsShepp::path1,  &ReedsShepp::path2,  &ReedsShepp::path3,
            &ReedsShepp::path4,  &ReedsShepp::path5,  &ReedsShepp::path6,
            &ReedsShepp::path7,  &ReedsShepp::path8,  &ReedsShepp::path9,
            &ReedsShepp::path10, &ReedsShepp::path11, &ReedsShepp::path12};

    for (auto fn : path_functions) {
        std::vector<RSPathSegment> var1 = (this->*fn)(x, y, theta);
        if (!var1.empty()) all_paths.push_back(var1);

        std::vector<RSPathSegment> var2 = timeflip((this->*fn)(-x, y, -theta));
        if (!var2.empty()) all_paths.push_back(var2);

        std::vector<RSPathSegment> var3 = reflect((this->*fn)(x, -y, -theta));
        if (!var3.empty()) all_paths.push_back(var3);

        std::vector<RSPathSegment> var4 = reflect(timeflip((this->*fn)(-x, -y, theta)));
        if (!var4.empty()) all_paths.push_back(var4);
    }

    return all_paths;
}

int ReedsShepp::getOptimalPathIndex(const std::vector<std::vector<RSPathSegment>> &paths)
{
    float min_distance = INFINITY;
    int   min_index    = -1;

    for (size_t i = 0; i < paths.size(); ++i) {
        float total = 0;
        for (const auto &seg : paths[i])
            total += fabs(seg.length);
        if (total < min_distance) {
            min_distance = total;
            min_index    = (int)i;
        }
    }
    return min_index;
}

Pose ReedsShepp::normalizePose(const Pose &p) const
{
    return {p.x / radius, p.y / radius, p.theta};
}

void ReedsShepp::denormalizePath(std::vector<RSPathSegment> &path) const
{
    for (auto &seg : path)
        seg.length *= radius;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Debug
// ─────────────────────────────────────────────────────────────────────────────

void ReedsShepp::printPath(const std::vector<RSPathSegment> &path) {}

void ReedsShepp::printAllPaths(const std::vector<std::vector<RSPathSegment>> &paths) {}

void ReedsShepp::runDemo(const std::vector<Pose> &waypoints) {}

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers geométricos privados
// ─────────────────────────────────────────────────────────────────────────────

Pose ReedsShepp::changeOfBasis(const Pose &p1, const Pose &p2)
{
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return {
        dx * cos(p1.theta) + dy * sin(p1.theta),
       -dx * sin(p1.theta) + dy * cos(p1.theta),
        M(p2.theta - p1.theta)
    };
}

float ReedsShepp::mod2pi(float theta)
{
    float two_pi = 2.0f * M_PI;
    return theta - two_pi * floor(theta / two_pi);
}

void ReedsShepp::R(const float x, const float y, float &r, float &theta)
{
    r     = sqrt(x * x + y * y);
    theta = atan2(y, x);
}

float ReedsShepp::M(float theta)
{
    theta = fmod(theta, 2.0f * M_PI);
    if (theta < -M_PI)       theta += 2.0f * M_PI;
    else if (theta >= M_PI)  theta -= 2.0f * M_PI;
    return theta;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Simetría de paths
// ─────────────────────────────────────────────────────────────────────────────

RSPathSegment ReedsShepp::reverseGear(const RSPathSegment &seg)
{
    RSPathSegment r = seg;
    r.gear = static_cast<Gear>(-seg.gear);
    return r;
}

RSPathSegment ReedsShepp::reverseSteering(const RSPathSegment &seg)
{
    RSPathSegment r = seg;
    r.type = static_cast<RSSegType>(-seg.type);
    return r;
}

std::vector<RSPathSegment> ReedsShepp::reflect(const std::vector<RSPathSegment> &path)
{
    std::vector<RSPathSegment> out;
    for (const auto &seg : path) out.push_back(reverseSteering(seg));
    return out;
}

std::vector<RSPathSegment> ReedsShepp::timeflip(const std::vector<RSPathSegment> &path)
{
    std::vector<RSPathSegment> out;
    for (const auto &seg : path) out.push_back(reverseGear(seg));
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
//  12 familias de paths Reeds-Shepp
// ─────────────────────────────────────────────────────────────────────────────

std::vector<RSPathSegment> ReedsShepp::path1(float x, float y, float phi)
{
    std::vector<RSPathSegment> path;
    float u, t;
    R(x - sin(phi), y - 1.0f + cos(phi), u, t);
    float v = M(phi - t);
    path.push_back({t, LEFT,     FORWARD});
    path.push_back({u, STRAIGHT, FORWARD});
    path.push_back({v, LEFT,     FORWARD});
    return path;
}

std::vector<RSPathSegment> ReedsShepp::path2(float x, float y, float phi)
{
    std::vector<RSPathSegment> path;
    float phi_new = M(phi);
    float rho, t1;
    R(x + sin(phi_new), y - 1.0f - cos(phi_new), rho, t1);
    if (rho * rho >= 4.0f) {
        float u = sqrt(rho * rho - 4.0f);
        float t = M(t1 + atan2(2.0f, u));
        float v = M(t - phi_new);
        path.push_back({t, LEFT,     FORWARD});
        path.push_back({u, STRAIGHT, FORWARD});
        path.push_back({v, RIGHT,    FORWARD});
    }
    return path;
}

std::vector<RSPathSegment> ReedsShepp::path3(float x, float y, float phi)
{
    std::vector<RSPathSegment> path;
    float xi = x - sin(phi), eta = y - 1.0f + cos(phi);
    float rho, theta;
    R(xi, eta, rho, theta);
    if (rho <= 4.0f) {
        float A = acos(constrain(rho / 4.0f, -1.0f, 1.0f));
        float t = M(theta + M_PI / 2.0f + A);
        float u = M(M_PI - 2.0f * A);
        float v = M(phi - t - u);
        path.push_back({t, LEFT,  FORWARD});
        path.push_back({u, RIGHT, BACKWARD});
        path.push_back({v, LEFT,  FORWARD});
    }
    return path;
}

std::vector<RSPathSegment> ReedsShepp::path4(float x, float y, float phi)
{
    std::vector<RSPathSegment> path;
    float xi = x - sin(phi), eta = y - 1.0f + cos(phi);
    float rho, theta;
    R(xi, eta, rho, theta);
    if (rho <= 4.0f) {
        float A = acos(constrain(rho / 4.0f, -1.0f, 1.0f));
        float t = M(theta + M_PI / 2.0f + A);
        float u = M(M_PI - 2.0f * A);
        float v = M(t + u - phi);
        path.push_back({t, LEFT,  FORWARD});
        path.push_back({u, RIGHT, BACKWARD});
        path.push_back({v, LEFT,  BACKWARD});
    }
    return path;
}

std::vector<RSPathSegment> ReedsShepp::path5(float x, float y, float phi)
{
    std::vector<RSPathSegment> path;
    float xi = x - sin(phi), eta = y - 1.0f + cos(phi);
    float rho, theta;
    R(xi, eta, rho, theta);
    if (rho <= 4.0f) {
        float u  = acos(constrain(1.0f - (rho * rho) / 8.0f, -1.0f, 1.0f));
        float A  = asin(constrain(2.0f * sin(u) / rho, -1.0f, 1.0f));
        float t  = M(theta + M_PI / 2.0f - A);
        float v  = M(t - u - phi);
        path.push_back({t, LEFT,  FORWARD});
        path.push_back({u, RIGHT, FORWARD});
        path.push_back({v, LEFT,  BACKWARD});
    }
    return path;
}

std::vector<RSPathSegment> ReedsShepp::path6(float x, float y, float phi)
{
    std::vector<RSPathSegment> path;
    float xi = x + sin(phi), eta = y - 1.0f - cos(phi);
    float rho, theta;
    R(xi, eta, rho, theta);
    if (rho <= 4.0f) {
        float A, t, u, v;
        if (rho <= 2.0f) {
            A = acos(constrain((rho + 2.0f) / 4.0f, -1.0f, 1.0f));
            t = M(theta + M_PI / 2.0f + A);
            u = M(A);
        } else {
            A = acos(constrain((rho - 2.0f) / 4.0f, -1.0f, 1.0f));
            t = M(theta + M_PI / 2.0f - A);
            u = M(M_PI - A);
        }
        v = M(phi - t + 2.0f * u);
        path.push_back({t, LEFT,  FORWARD});
        path.push_back({u, RIGHT, FORWARD});
        path.push_back({u, LEFT,  BACKWARD});
        path.push_back({v, RIGHT, BACKWARD});
    }
    return path;
}

std::vector<RSPathSegment> ReedsShepp::path7(float x, float y, float phi)
{
    std::vector<RSPathSegment> path;
    float xi = x + sin(phi), eta = y - 1.0f - cos(phi);
    float rho, theta;
    R(xi, eta, rho, theta);
    float u1 = (20.0f - rho * rho) / 16.0f;
    if (rho <= 6.0f && u1 >= 0.0f && u1 <= 1.0f) {
        float u = acos(constrain(u1, -1.0f, 1.0f));
        float A = asin(constrain(2.0f * sin(u) / rho, -1.0f, 1.0f));
        float t = M(theta + M_PI / 2.0f + A);
        float v = M(t - phi);
        path.push_back({t, LEFT,  FORWARD});
        path.push_back({u, RIGHT, BACKWARD});
        path.push_back({u, LEFT,  BACKWARD});
        path.push_back({v, RIGHT, FORWARD});
    }
    return path;
}

std::vector<RSPathSegment> ReedsShepp::path8(float x, float y, float phi)
{
    std::vector<RSPathSegment> path;
    float xi = x - sin(phi), eta = y - 1.0f + cos(phi);
    float rho, theta;
    R(xi, eta, rho, theta);
    if (rho >= 2.0f) {
        float u = sqrt(rho * rho - 4.0f) - 2.0f;
        float A = atan2(2.0f, u + 2.0f);
        float t = M(theta + M_PI / 2.0f + A);
        float v = M(t - phi + M_PI / 2.0f);
        path.push_back({t,          LEFT,     FORWARD});
        path.push_back({M_PI/2.0f,  RIGHT,    BACKWARD});
        path.push_back({u,          STRAIGHT, BACKWARD});
        path.push_back({v,          LEFT,     BACKWARD});
    }
    return path;
}

std::vector<RSPathSegment> ReedsShepp::path9(float x, float y, float phi)
{
    std::vector<RSPathSegment> path;
    float xi = x - sin(phi), eta = y - 1.0f + cos(phi);
    float rho, theta;
    R(xi, eta, rho, theta);
    if (rho >= 2.0f) {
        float u = sqrt(rho * rho - 4.0f) - 2.0f;
        float A = atan2(u + 2.0f, 2.0f);
        float t = M(theta + M_PI / 2.0f - A);
        float v = M(t - phi - M_PI / 2.0f);
        path.push_back({t,          LEFT,     FORWARD});
        path.push_back({u,          STRAIGHT, FORWARD});
        path.push_back({M_PI/2.0f,  RIGHT,    FORWARD});
        path.push_back({v,          LEFT,     BACKWARD});
    }
    return path;
}

std::vector<RSPathSegment> ReedsShepp::path10(float x, float y, float phi)
{
    std::vector<RSPathSegment> path;
    float xi = x + sin(phi), eta = y - 1.0f - cos(phi);
    float rho, theta;
    R(xi, eta, rho, theta);
    if (rho >= 2.0f) {
        float t = M(theta + M_PI / 2.0f);
        float u = rho - 2.0f;
        float v = M(phi - t - M_PI / 2.0f);
        path.push_back({t,          LEFT,     FORWARD});
        path.push_back({M_PI/2.0f,  RIGHT,    BACKWARD});
        path.push_back({u,          STRAIGHT, BACKWARD});
        path.push_back({v,          RIGHT,    BACKWARD});
    }
    return path;
}

std::vector<RSPathSegment> ReedsShepp::path11(float x, float y, float phi)
{
    std::vector<RSPathSegment> path;
    float xi = x + sin(phi), eta = y - 1.0f - cos(phi);
    float rho, theta;
    R(xi, eta, rho, theta);
    if (rho >= 2.0f) {
        float t = M(theta);
        float u = rho - 2.0f;
        float v = M(phi - t - M_PI / 2.0f);
        path.push_back({t,          LEFT,     FORWARD});
        path.push_back({u,          STRAIGHT, FORWARD});
        path.push_back({M_PI/2.0f,  LEFT,     FORWARD});
        path.push_back({v,          RIGHT,    BACKWARD});
    }
    return path;
}

std::vector<RSPathSegment> ReedsShepp::path12(float x, float y, float phi)
{
    std::vector<RSPathSegment> path;
    float xi = x + sin(phi), eta = y - 1.0f - cos(phi);
    float rho, theta;
    R(xi, eta, rho, theta);
    if (rho >= 4.0f) {
        float u = std::sqrt(rho * rho - 4.0f) - 4.0f;
        float A = std::atan2(2.0f, u + 4.0f);
        float t = M(theta + M_PI / 2.0f + A);
        float v = M(t - phi);
        path.push_back({t,          LEFT,     FORWARD});
        path.push_back({M_PI/2.0f,  RIGHT,    BACKWARD});
        path.push_back({u,          STRAIGHT, BACKWARD});
        path.push_back({M_PI/2.0f,  LEFT,     BACKWARD});
        path.push_back({v,          RIGHT,    FORWARD});
    }
    return path;
}
