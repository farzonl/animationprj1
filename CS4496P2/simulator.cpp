#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;

double Equations::EARTH_GRAVITY = -9.81;

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
    // y_i+1 = yi + phi(xi,yi,h)*h
    // phi = sigma[n=1,N] an*kn
    // first order a1*k1
    // k1 = f(xi,yi), a1 =1
    //y_i+1 = yi + f(xi,yi)*h (explicit Euler)
    // let h = timeStep.
    // x'(t) = v(t)
    // v't(t) = a(t)
    // x(h) = x(0) + h*v(0)
    // v(h) = v(0) + h(F/m)
    // F= ma -> a = F/m
    double y_0 = p.mPosition[1];
    double v_0 = p.mVelocity[1];
    double diplus1 = y_0 + v_0 * timeStep;
    double viplus1 = v_0 + gravity * timeStep;
    p.mPosition[1] = diplus1;  // update position
    p.mVelocity[1] = viplus1; // update veloicty
}

void Equations::midpointEquation(Particle &p,
                                 double timeStep,
                                 double gravity) {
    //y_i+1 = yi + f(xi+1/2,yi+1/2)*h
    double y_0 = p.mPosition[1];
    double v_0 = p.mVelocity[1];
    double viplus1half = v_0 + ((gravity * timeStep) /2.0);
    double diplus1 = y_0 + viplus1half * timeStep;
    double viplus1 = v_0 + gravity * timeStep;
    
    p.mPosition[1] = diplus1;  // update position
    p.mVelocity[1] = viplus1; // update veloicty
}

void Equations::implicitEulerEquation(Particle &p,
                                  double timeStep,
                                  double gravity) {
    
    // x(t+h) = x(t) + h f(x(t+h))
    // v(t+1) = v(t) + h(F/m)
    // let f(x(t+1)) = v(t+1)
    double y_0 = p.mPosition[1];
    double v_0 = p.mVelocity[1];
    // calcuate velocity at t+1 then displacment.
    double viplus1 = v_0 + gravity * timeStep;
    double diplus1 = y_0 + viplus1 * timeStep;
    
    p.mPosition[1] = diplus1;  // update position
    p.mVelocity[1] = viplus1; // update veloicty

}

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







