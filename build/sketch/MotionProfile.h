#line 1 "C:\\Users\\nikhi\\Documents\\Arduino\\ScienceResearch2324\\MotionProfile.h"

class SCurveProfile
{
    public:
    float vmax;
    float a1;
    float a2;
    float target;
    float vf;
    float v0;
    float j1;
    float j2;
    float endTime;
    
    float lastTime = 0;

    float distStart;
    float distEnd;

    float timeMarks[8];
    float distMarks[8];

    float p1(float t);
    float p2(float t);
    float p3(float t);
    float p4(float t);
    float p5(float t);
    float p6(float t);
    float p7(float t);

    float v1(float t);
    float v2(float t);
    float v3(float t);
    float v4(float t);
    float v5(float t);
    float v6(float t);
    float v7(float t);

    float tp1(float d);
    float tp2(float d);
    float tp3(float d);
    float tp4(float d);
    float tp5(float d);
    float tp6(float d);
    float tp7(float d);
    
    /**
     * @brief Construct a new SCurveProfile object
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
    SCurveProfile(float target, float a1, float a2, float j1, float j2,float v0, float vf, float vmax = 100);

    float getOutputDist(float d);
    float getOutput(float d);
    float getOutputTime(float t);
};