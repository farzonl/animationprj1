#include "equations.h"


void Equations::kinematicEquation(Particle &p,
                                  double timeStep,
                                  double gravity) {
    //vi = initial velocity
    // t = time
    // a = acceleration
    // vf = velocity final
    // displacement =vi*t + 1/2 * a*t^2
    // vf = vi + a*t
    // assume an xyz axis whith y representing the vertical
    // direction.
    double yvi = p.mVelocity[1];
    double d = yvi * timeStep + 0.5 * gravity * std::pow(timeStep,2);
    double vf = yvi + gravity * timeStep;

    p.mPosition[1] += d;  // update position
    p.mVelocity[1] = vf; // update veloicty
}

void Equations::explicitEulerEquation(Particle &p,
                                      double timeStep,
                                      double gravity) {
    //y_i+1 = yi + f(xi,yi)*dX (explicit Euler)
    // let dx = timeStep
    // x'(t) = v(t)
    // v't(t) = a(t)
    // x(h) = x(0) + h*v(0)
    // v(h) = v(0) + h(F/m)
    // F= ma -> a = F/m
    double x_0 = p.mPosition[1];
    double v_0 = p.mVelocity[1];
    double diplus1 = x_0 + v_0 * timeStep;
    double vplus1 = v_0 + gravity * timeStep;
    p.mPosition[1] = diplus1;  // update position
    p.mVelocity[1] = vplus1; // update veloicty
}
