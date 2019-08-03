#include "simulator.h"
#include <fstream>
#include <iostream>
#include "equations.h"
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;
using namespace std;

static int limit = 80;

Simulator::Simulator() : initialVelocity(0), count(0) {
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

void Simulator::resetAnalytics() {
    count = 0;
    explicitEulerVelocity.clear();
    implicitEulerVelocity.clear();
    midpointVelocity.clear();   
    analyticalVelocity.clear();
    analyticalPosition.clear();
    explicitEulerPosition.clear();
    implicitEulerPosition.clear();
    midpointPosition.clear();
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

    resetAnalytics();
    
    for (int i = 0; i < 4; i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mVelocity[1] = initialVelocity;
        mParticles[i].mAccumulatedForce.setZero();
    }
    
    mElapsedTime = 0;
}

void Simulator::writeToFile() {
    json j;
    json j_analyticalVelocity(analyticalVelocity);
    json j_analyticalPosition(analyticalPosition);

    j["kinematicEquation"] = { {"position", j_analyticalPosition}, {"velocity", j_analyticalVelocity} };

    json j_explicitEulerVelocity(explicitEulerVelocity);
    json j_explicitEulerPosition(explicitEulerPosition);

    j["explicitEulerEquation"] = { {"position", j_explicitEulerPosition}, {"velocity", j_explicitEulerVelocity} };

    json j_implicitEulerVelocity(implicitEulerVelocity);
    json j_implicitEulerPosition(implicitEulerPosition);

    j["implicitEulerEquation"] = { {"position", j_implicitEulerPosition}, {"velocity", j_implicitEulerVelocity} };

    json j_midpointVelocity(midpointVelocity);
    json j_midpointPosition(midpointPosition);

    j["midpointEquation"] = { {"position", j_midpointPosition}, {"velocity", j_midpointVelocity} };

    std::ofstream o("my_data.json");
    o << std::setw(4) << j << std::endl;
}

void Simulator::simulate() {
    count++;
    if(count >= limit) {
        std::cout << "hit: " << count << std::endl;
        writeToFile();
        resetAnalytics();
    }
    if(count < limit) {
        analyticalVelocity.push_back(mParticles[0].mVelocity[1]);
        analyticalPosition.push_back(mParticles[0].mPosition[1]);   
    }
    // red partical (kinematicEquation)
    Equations::kinematicEquation(mParticles[0], mTimeStep, Equations::EARTH_GRAVITY);
    
    if(count < limit) {
        explicitEulerVelocity.push_back(mParticles[1].mVelocity[1]);
        explicitEulerPosition.push_back(mParticles[1].mPosition[1]);
    }

    //explicit Euler blue
    Equations::explicitEulerEquation(mParticles[1], mTimeStep, Equations::EARTH_GRAVITY);
    
    if(count < limit) {
        midpointVelocity.push_back(mParticles[2].mVelocity[1]);     
        midpointPosition.push_back(mParticles[2].mPosition[1]);  
    }

    //midpoint yellow
    Equations::midpointEquation(mParticles[2], mTimeStep, Equations::EARTH_GRAVITY);
    
    if(count < limit) {
        implicitEulerVelocity.push_back(mParticles[3].mVelocity[1]);
        implicitEulerPosition.push_back(mParticles[3].mPosition[1]);
    }

    // implicit Euler green
    Equations::implicitEulerEquation(mParticles[3], mTimeStep, Equations::EARTH_GRAVITY);
    
    mElapsedTime += mTimeStep;
}







