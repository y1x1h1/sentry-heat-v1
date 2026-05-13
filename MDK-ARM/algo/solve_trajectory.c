#include "solve_trajectory.h"
#include <math.h>

#define GRAVITY 9.78f  
#define ITER_MAX 20    

float Solve_Pitch_Control(float x, float y, float v) {
    float angle = atan2f(y, x); 
    float y_actual, t;
    
    for (int i = 0; i < ITER_MAX; i++) {
        t = x / (v * cosf(angle)); 
        if (t <= 0) break;
        y_actual = v * sinf(angle) * t - 0.5f * GRAVITY * t * t; 
        float error = y - y_actual;
        angle += error / x; 
        if (fabsf(error) < 0.001f) break;
    }
    return angle;
}
