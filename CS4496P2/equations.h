
#ifndef EQUATIONS_H
#define EQUATIONS_H

#include "particle.h"

// class for spline
class Equations {
public:
    static constexpr double EARTH_GRAVITY = -9.81;
    static void kinematicEquation(Particle &p,
                              double timeStep,
                              double gravity);
    static void explicitEulerEquation(Particle &p,
                                  double timeStep,
                                  double gravity);
private:
    Equations() = delete;
       
};

#endif  // EQUATIONS_H
