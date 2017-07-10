/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;


const int numberOfParticles(24);    // also works with 12, just to be sure

ParticleFilter::ParticleFilter()
    : m_isInitialized(false)
{
}

const bool ParticleFilter::isInitialized() const
{
    return m_isInitialized;
}

void ParticleFilter::init(
    const double x, 
    const double y,
    const double theta,
    const double std[])
{
    default_random_engine randomGenerator;

    normal_distribution<double> normDistX(x, std[0]);
    normal_distribution<double> normDistY(y, std[1]);
    normal_distribution<double> normDistTheta(theta, std[2]);
    
    m_particles = std::vector<Particle>(numberOfParticles);
    for (int i(0); i < numberOfParticles; ++i)
    {
        Particle& particle = m_particles[i];
        particle.id = i;
        particle.x = normDistX(randomGenerator);
        particle.y = normDistY(randomGenerator);
        particle.theta = normDistTheta(randomGenerator);
        particle.weight = 1.0;
    }

    m_isInitialized = true;
}

void ParticleFilter::prediction(
    const double delta_t,
    const double std_pos[],
    const double velocity,
    const double yaw_rate)
{
    default_random_engine randomGenerator;

    for (Particle& particle : m_particles)
    {
        double predX;
        double predY;
        double predTheta;

        // move particles by measurement
        if (0 == yaw_rate)
        {
            // vehicle is moving on a straight line
            predX = particle.x + velocity * delta_t * cos(particle.theta);
            predY = particle.y + velocity * delta_t * sin(particle.theta);
            predTheta = particle.theta;
        }
        else
        {
            // vehicle is turning
            predX = particle.x + velocity / yaw_rate * (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta));
            predY = particle.y + velocity / yaw_rate * (cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t));
            predTheta = particle.theta + yaw_rate * delta_t;
        }

        // add Gaussian noise
        normal_distribution<double> normDistX(predX, std_pos[0]);
        normal_distribution<double> normDistY(predY, std_pos[1]);
        normal_distribution<double> normDistTheta(predTheta, std_pos[2]);

        particle.x = normDistX(randomGenerator);
        particle.y = normDistY(randomGenerator);
        particle.theta = normDistTheta(randomGenerator);
    }
}

void ParticleFilter::dataAssociation(
    const std::vector<LandmarkObs>& predicted,
    const std::vector<LandmarkObs>& observations,
    std::vector<LandmarkObs>& nearestNeighbors)
{
    // predicted: 'true' landmarks with ground truth positions in the map
    // observations: noisy measurements of landmarks

    for (const LandmarkObs& observedLandmark : observations)
    {
        const LandmarkObs* nearestNeighbor(nullptr);
        double minDistance = std::numeric_limits<double>::max();
        
        for (const LandmarkObs& predictedLandmark : predicted)
        {
            const double curDistance = dist(
                observedLandmark.x, observedLandmark.y, 
                predictedLandmark.x, predictedLandmark.y);
            
            if (curDistance < minDistance)
            {
                minDistance = curDistance;
                nearestNeighbor = &predictedLandmark;
            }
        }
        
        if (nullptr == nearestNeighbor)
        {
            printf("dataAssociation: could not find nearest neighbor for (%.3f, %.3f)\n",
                observedLandmark.x, observedLandmark.y);
        }
        else
        {
            nearestNeighbors.push_back(*nearestNeighbor);
        }
    }
}

void ParticleFilter::updateWeights(
    const double sensor_range,
    const double std_landmark[],
    const std::vector<LandmarkObs>& observations,
    const Map& map_landmarks)
{
    // Update the weights of each particle using a multi-variate Gaussian distribution.

    const double sigmaX(std_landmark[0]);
    const double sigmaY(std_landmark[1]);

    const double scaleFact(2.0 * M_PI * sigmaX * sigmaY);

    const double twoVarX(2.0 * sigmaX * sigmaX);
    const double twoVarY(2.0 * sigmaY * sigmaY);

    for (Particle& particle : m_particles)
    {
        // transform landmark observations from vehicle coordinates in map coordinates
        std::vector<LandmarkObs> mapCoordObservations;
        for (const LandmarkObs& landmarkObs : observations)
        {
            LandmarkObs mapCoordObs;
            mapCoordObs.id = landmarkObs.id;
            mapCoordObs.x = particle.x + (landmarkObs.x * cos(particle.theta) - landmarkObs.y * sin(particle.theta));
            mapCoordObs.y = particle.y + (landmarkObs.x * sin(particle.theta) + landmarkObs.y * cos(particle.theta));
            mapCoordObservations.push_back(mapCoordObs);
        }

        std::vector<LandmarkObs> landmarksInSensorRange;
        for (const Map::single_landmark_s& mapLandmark : map_landmarks.landmark_list)
        {
            if (    dist(particle.x, particle.y, mapLandmark.x_f, mapLandmark.y_f)
                <   sensor_range)
            {
                landmarksInSensorRange.push_back(
                    LandmarkObs(
                        mapLandmark.id_i,
                        mapLandmark.x_f,
                        mapLandmark.y_f));
            }
        }

        std::vector<LandmarkObs> nearestNeighbors;
        dataAssociation(landmarksInSensorRange, mapCoordObservations, nearestNeighbors);

        double measurementProb(1.0);
        for (int i(0); i < mapCoordObservations.size(); ++i)
        {
            const double dx(mapCoordObservations[i].x - nearestNeighbors[i].x);
            const double dy(mapCoordObservations[i].y - nearestNeighbors[i].y);

            const double prob =     exp(-dx * dx / twoVarX) * exp(-dy * dy / twoVarY)
                                /   scaleFact;

            measurementProb *= prob;
        }

        particle.weight = measurementProb;
    }
}

void ParticleFilter::resample() 
{
    // Resample particles with replacement with probability proportional to their weight. 

    vector<double> weights(m_particles.size());
    for (int i(0); i < m_particles.size(); ++i)
    {
        weights[i] = m_particles[i].weight;
    }

    default_random_engine randGen;
    std::discrete_distribution<int> discrDist(weights.begin(), weights.end());

    std::vector<Particle> resampledParticles;
    for (int i(0); i < weights.size(); ++i)
    {
        int randIdx = discrDist(randGen);
        resampledParticles.push_back(m_particles[randIdx]);
    }

    m_particles = resampledParticles;
}

Particle ParticleFilter::SetAssociations(
    Particle particle, 
    std::vector<int> associations, 
    std::vector<double> sense_x, 
    std::vector<double> sense_y)
{
    // particle: the particle to assign each listed association, and 
    //  association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best)
{
    vector<double> v = best.sense_x;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best)
{
    vector<double> v = best.sense_y;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}
