#ifndef SIMULATOR_H
#define SIMULATOR_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Dense>
#include <vector>
#include <string>

#include "particle.h"

// class containing objects to be simulated
class Simulator {
public:
    Simulator();
    void setInitialVelocity(double v0);
    void simulate();
    
    int getNumParticles();
    
    Particle* getParticle(int);
    
    double getTimeStep();
    
    void reset();
private:
    double initialVelocity;
    double mTimeStep;       // time step
    double mElapsedTime;    // time pased since beginning of simulation
    std::vector<Particle> mParticles;
};

class Equations {
public:
    static double EARTH_GRAVITY;
    static void kinematicEquation(Particle &p,
                              double timeStep,
                              double gravity);

    static void explicitEulerEquation(Particle &p,
                                  double timeStep,
                                  double gravity);
    
    static void midpointEquation(Particle &p,
                                      double timeStep,
                                      double gravity);
    
    static void implicitEulerEquation(Particle &p,
                                      double timeStep,
                                      double gravity);
private:
    Equations(){}
       
};

#endif  // SIMULATOR_H
