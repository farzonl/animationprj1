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
    
    void writeToFile();
    void resetAnalytics();
    void reset();
private:
    double initialVelocity;
    double mTimeStep;       // time step
    double mElapsedTime;    // time pased since beginning of simulation
    int count;
    std::vector<Particle> mParticles;
    std::vector<double> explicitEulerVelocity;
    std::vector<double> implicitEulerVelocity;
    std::vector<double> midpointVelocity;     
    std::vector<double> analyticalVelocity;    
    std::vector<double> analyticalPosition;    
    std::vector<double> explicitEulerPosition;    
    std::vector<double> implicitEulerPosition;  
    std::vector<double> midpointPosition;     
};

#endif  // SIMULATOR_H
