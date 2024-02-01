#include "MotionProfile.h"
#include "math.h"

using namespace std;
#define max(a, b) ((a) > (b) ? (a) : (b))

float SCurveProfile::v1(float t)
{
    float output = j1 * pow(t, 2) / 2 + v0;
    return output;
}

float SCurveProfile::v2(float t)
{
    float output = a1 * t - (a1 * a1) / (2 * j1) + v0;
    return output;
}

float SCurveProfile::v3(float t)
{
    float output = -j1 * pow(t - timeMarks[3], 2) / 2 + vmax;
    return output;
}
float SCurveProfile::v4(float t)
{
    float output = vmax;
    return output;
}
float SCurveProfile::v5(float t)
{
    float output = -j2 * pow(t - timeMarks[4], 2) / 2 + vmax;
    return output;
}
float SCurveProfile::v6(float t)
{
    float output = -1 * a2 * (t - timeMarks[5]) - ((a2 * a2) / (2 * j2)) + vmax;
    return output;
}
float SCurveProfile::v7(float t)
{
    float output = j2 * pow(t - timeMarks[7], 2) / 2 + vf;
    return output;
}

float SCurveProfile::p1(float t)
{
    float output = j1 * pow(t, 3) / 6 + v0 * t;
    return output;
}
float SCurveProfile::p2(float t)
{
    float output = distMarks[1] + v1(timeMarks[1]) * (t - timeMarks[1]) + a1 * pow(t - timeMarks[1], 2) / 2;
    return output;
}
float SCurveProfile::p3(float t)
{
    float output = distMarks[2] + v2(timeMarks[2]) * (t - timeMarks[2]) + j1 * pow(t - timeMarks[2], 3) / 6;
    return output;
}
float SCurveProfile::p4(float t)
{
    float output = distStart + vmax * (t - timeMarks[3]);
    return output;
}
float SCurveProfile::p5(float t)
{
    float output = distMarks[4] - j2 * (pow(t - timeMarks[4], 3)) / 6 + vmax * (t - timeMarks[4]);
    return output;
}
float SCurveProfile::p6(float t)
{
    float output = distMarks[5] + v5(timeMarks[5]) * (t - timeMarks[5]) - a2 * pow(t - timeMarks[5], 2) / 2;
    return output;
}
float SCurveProfile::p7(float t)
{
    float a = timeMarks[7] - timeMarks[6];
    float output = j2 * pow(t - timeMarks[7], 3) / 6 + distMarks[6] + v6(timeMarks[6]) * (a) + -1 * a2 * pow(a, 2) / 2 + j2 * pow(a, 3) / 6 + vf * (t - timeMarks[7]);
    return output;
}
float SCurveProfile::tp1(float d)
{
    float temp = sqrt(pow(j1, 3) * (9 * j1 * d * d + 8 * pow(v0, 3))) + 3 * j1 * j1 * d;
    float expression = pow(temp, 2.0 / 3.0) - 2 * j1 * v0;
    float denominator = j1 * (pow(temp, 1.0 / 3.0));

    return expression / denominator;
}
float SCurveProfile::tp2(float d)
{
    float t1 = timeMarks[1];
    double numerator = sqrt((-2 * distMarks[1] * a1) + v1(t1) * v1(t1) + 2 * a1 * d) + v1(t1) - a1 * t1 - 2 * v0;
    double denominator = a1;
    return (numerator / denominator) + timeMarks[1];
}
float SCurveProfile::tp3(float d)
{
    double temp = sqrt(pow(j1, 3) * (9 * j1 * pow(d - distMarks[2], 2) + 8 * pow(v2(timeMarks[2]), 3))) + 3 * pow(j1, 2) * (d - distMarks[2]);
    float expression = pow(temp, 2.0 / 3.0) - (2 * j1 * v2(timeMarks[2]));
    float denominator = j1 * (pow(temp, 1.0 / 3.0));

    return expression / denominator + timeMarks[2];
}

float SCurveProfile::tp4(float d)
{
    float output = 1 / vmax * d - distStart / vmax + timeMarks[3];
    return output;
}
float SCurveProfile::tp5(float d)
{
    lastTime = max(lastTime, timeMarks[4]);
    float output = lastTime;
    while (true)
    {
        output = output - (p5(output) - d) / v5(output);
        if (fabs(p5(output) - d) < 0.001)
        {
            break;
        }
    }
    lastTime = output;
    return output;
}
float SCurveProfile::tp6(float d)
{
    lastTime = max(lastTime, timeMarks[5]);
    float output = lastTime;
    while (true)
    {
        output = output - (p6(output) - d) / v6(output);
        if (fabs(p6(output) - d) < 0.001)
        {
            break;
        }
    }
    lastTime = output;
    return output;
}
float SCurveProfile::tp7(float d)
{
    lastTime = max(lastTime, timeMarks[6]);
    float output = lastTime;
    while (true)
    {
        output = output - (p7(output) - d) / v7(output);
        if (fabs(p7(output) - d) < 0.001)
        {
            break;
        }
    }
    lastTime = output;
    return output;
}
/**
 * @brief Construct a new SCurveProfile::SCurveProfile object
 *
 * @param target
 * @param a1
 * @param a2
 * @param j1
 * @param j2
 * @param v0
 * @param vf
 * @param vmax
 */
SCurveProfile::SCurveProfile(float target, float a1, float a2, float j1, float j2, float v0, float vf, float vmax) : vmax(vmax), a1(a1), a2(a2), j1(j1), j2(j2), v0(v0), vf(vf), target(target)
{
    float temp1 = (vmax - vf) / a2 - (a2 / j2);
    distEnd = 2 * (pow(a2, 3) / (6 * pow(j2, 2))) - (((a2 * (pow(a2, 2) - (3 * j2 * vmax)))) / (3 * pow(j2, 2))) + vf * a2 / j2 + ((-pow(a2, 2) / (2 * j2)) + vmax) * temp1 - ((a2 * pow(temp1, 2)) / 2);
    timeMarks[0] = 0;
    timeMarks[1] = a1 / j1;
    timeMarks[2] = (vmax - v0) / a1;
    timeMarks[3] = timeMarks[2] + timeMarks[1];
        distMarks[1] = p1(timeMarks[1]);
    distMarks[2] = p2(timeMarks[2]);
    distMarks[3] = p3(timeMarks[3]);
    distStart = distMarks[3];
    timeMarks[4] = timeMarks[3] + (target - distMarks[3] - distEnd) / vmax;
    timeMarks[5] = timeMarks[4] + a2 / j2;
    timeMarks[6] = timeMarks[4] + (vmax - vf) / a2;
    timeMarks[7] = timeMarks[6] + a2 / j2;
    endTime = timeMarks[7];

    distMarks[0] = 0;

    distMarks[4] = p4(timeMarks[4]);
    distMarks[5] = p5(timeMarks[5]);
    distMarks[6] = p6(timeMarks[6]);
    distMarks[7] = target;
}

float SCurveProfile::getOutputDist(float d)
{
    float output;
    if (d < distMarks[1])
    {
        output = v1(tp1(d));
    }
    else if (d < distMarks[2])
    {
        output = v2(tp2(d));
    }
    else if (d < distMarks[3])
    {
        output = v3(tp3(d));
    }
    else if (d < distMarks[4])
    {
        output = v4(tp4(d));
    }
    else if (d < distMarks[5])
    {
        output = v5(tp5(d));
    }
    else if (d < distMarks[6])
    {
        output = v6(tp6(d));
        // output =0;
    }
    else if (d < distMarks[7])
    {
        output = v7(tp7(d));
        // output =0;
    }
    else
    {
        output = 0;
    }
    return output;
}

float SCurveProfile::getOutput(float d)
{
    float output;
    if (d < distMarks[1])
    {
        output = (tp1(d));
    }
    else if (d < distMarks[2])
    {
        output = (tp2(d));
    }
    else if (d < distMarks[3])
    {
        output = (tp3(d));
    }
    else if (d < distMarks[4])
    {
        output = (tp4(d));
    }
    else if (d < distMarks[5])
    {
        output = (tp5(d));
    }
    else if (d < distMarks[6])
    {
        output = (tp6(d));
        // output =0;
    }
    else if (d < distMarks[7])
    {
        output = (tp7(d));
        // output =0;
    }
    else
    {
        output = 0;
    }
    return output;
}

float SCurveProfile::getOutputTime(float t)
{
    float output;
    if (t < timeMarks[1])
    {
        output = v1(t);
    }
    else if (t < timeMarks[2])
    {
        output = v2(t);
    }
    else if (t < timeMarks[3])
    {
        output = v3(t);
    }
    else if (t < timeMarks[4])
    {
        output = v4(t);
    }
    else if (t < timeMarks[5])
    {
        output = v5(t);
    }
    else if (t < timeMarks[6])
    {
        output = v6(t);
        // output =0;
    }
    else if (t < timeMarks[7])
    {
        output = v7(t);
        // output =0;
    }
    else
    {
        output = 0;
    }
    return output;
}
