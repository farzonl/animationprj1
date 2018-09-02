#include "simulator.h"
#include <fstream>
#include <iostream>
#include "equations.h"

using namespace std;

Simulator::Simulator() : initialVelocity(0) {
    // initialize the particles
    mParticles.resize(4);
    
    // Init particle positions (default is 0, 0, 0)
    reset();
    
    // Init particle colors (default is red)
    mParticles[1].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    mParticles[2].mColor = Eigen::Vector4d(0.9, 0.9, 0.2, 1.0); // Yellow
    mParticles[3].mColor = Eigen::Vector4d(0.2, 0.9, 0.2, 1.0); // Green
    
    mTimeStep = 0.03;
    mElapsedTime = 0;
}

int Simulator::getNumParticles() {
    return mParticles.size();
}

Particle* Simulator::getParticle(int index) {
    return &mParticles[index];
}

double Simulator::getTimeStep() {
    return mTimeStep;
}

void Simulator::setInitialVelocity(double v0) {
    initialVelocity = v0;
}

void Simulator::reset() {
    mParticles[0].mPosition[0] = -0.2;
    mParticles[0].mPosition[1] = 20.0;
    mParticles[1].mPosition[0] = 0.0;
    mParticles[1].mPosition[1] = 20.0;
    mParticles[2].mPosition[0] = 0.2;
    mParticles[2].mPosition[1] = 20.0;
    mParticles[3].mPosition[0] = 0.4;
    mParticles[3].mPosition[1] = 20.0;
    
    for (int i = 0; i < 4; i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mVelocity[1] = initialVelocity;
        mParticles[i].mAccumulatedForce.setZero();
    }
    
    mElapsedTime = 0;
}

void Simulator::simulate() {
    
    // red partical (kinematicEquation)
    Equations::kinematicEquation(mParticles[0], mTimeStep, Equations::EARTH_GRAVITY);
    
    //explicit Euler blue
    Equations::explicitEulerEquation(mParticles[1], mTimeStep, Equations::EARTH_GRAVITY);
    
    //midpoint yellow
    Equations::midpointEquation(mParticles[2], mTimeStep, Equations::EARTH_GRAVITY);
    
    // implicit Euler green
    Equations::implicitEulerEquation(mParticles[3], mTimeStep, Equations::EARTH_GRAVITY);
    
    mElapsedTime += mTimeStep;
}







