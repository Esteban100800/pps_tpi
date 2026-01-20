#ifndef DUBINS_H
#define DUBINS_H

#include <cmath>
#include <Arduino.h>

struct Pose
{
    double x;
    double y;
    double theta;
};

struct Vec2
{
    double x;
    double y;
};

enum SegmentType
{
    SEG_LEFT,
    SEG_RIGHT,
    SEG_STRAIGHT
};

struct DubinsSegment
{
    SegmentType type;
    double length;
};

struct DubinsPath
{
    DubinsSegment seg[3];
    double totalLength;
};

class DubinsPlanner
{
public:
    explicit DubinsPlanner(float R);

    bool compute(
        const Pose &start,
        const Pose &goal,
        DubinsPath &bestPath);

    float compute_rmin(float rmin);

    Pose getStartPose();
    Pose getGoalPose();
    bool getNewSegment();
    float getAngleStraight();
    int getSegmentIndex();


    void setStartPose(const Pose &pose);
    void setGoalPose(const Pose &pose);
    void setSegmentIndex(int index);
    void setAngleStraight(float angle);
    void setNewSegment(bool newSegment);



private:
    float R;

    // ===== caminos =====
    bool dubinsLSL(const Pose &start, const Pose &goal, DubinsPath &path);
    bool dubinsLRL(const Pose &start, const Pose &goal, DubinsPath &path);
    bool dubinsLSR(const Pose &start, const Pose &goal, DubinsPath &path);
    bool dubinsRLR(const Pose &start, const Pose &goal, DubinsPath &path);
    bool dubinsRSL(const Pose &start, const Pose &goal, DubinsPath &path);
    bool dubinsRSR(const Pose &start, const Pose &goal, DubinsPath &path);

    // ===== tangentes (PORT DIRECTO DE MATLAB) =====
    bool getTangents(
        double x1, double y1, double r1,
        double x2, double y2, double r2,
        SegmentType type,
        Vec2 &t1, Vec2 &t2);

    // ===== helpers =====
    double mod2pi(double a);
    double angleDiff(double from, double to, bool left);
    double norm(const Vec2 &v);

    Pose startPose;
    Pose goalPose;

    bool new_segment = true;

    float angle_straight = 0.0f;

    int segmentIndex = 0;
};

#endif
