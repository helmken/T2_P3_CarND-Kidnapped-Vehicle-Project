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
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;


// TODO: find a suitable value for number of particles
// number of particles for the particle filter
// lesson 13 uses 1000 particles
const int numberOfParticles(100);


ParticleFilter::ParticleFilter()
    : m_num_particles(0),
    m_isInitialized(false)
{
}

const bool ParticleFilter::isInitialized() const
{
    return m_isInitialized;
}


void ParticleFilter::init(
    double x, 
    double y, 
    double theta, 
    double std[]) 
{
    // param std: GPS measurement uncertainty: 
    // standard deviation of [x [m], y [m], theta [rad]]

    // TODO: 1) Set the number of particles.
    m_num_particles = numberOfParticles;
    m_particles = std::vector<Particle>(m_num_particles);
    m_weights = std::vector<double>(m_num_particles);

    // TODO: 2) Initialize all particles to first position (based on estimates
    //  of x, y, theta and their uncertainties from GPS)
    // TODO: 3) initialize all weights to 1. 

    // create a normal (Gaussian) distributions for 
    // initial x, y position and yaw
    normal_distribution<double> normDistX(x, std[0]);
    normal_distribution<double> normDistY(y, std[1]);
    normal_distribution<double> normDistTheta(theta, std[2]);

    default_random_engine randomGenerator;

    for (int i(0); i < m_num_particles; ++i)
    {
        Particle& particle = m_particles[i];
        particle.id = i;
        particle.x = normDistX(randomGenerator);
        particle.y = normDistY(randomGenerator);
        particle.theta = normDistTheta(randomGenerator);
        
        // TODO: what's the difference between particle member weight
        // and the vector of weights?
        particle.weight = 1.0;  

        //printf("particle %i: x=%.3f, y=%.3f, theta=%.3f\n",
        //    i, particle.x, particle.y, particle.theta);

        // TODO: what's the difference between particle member weight
        // and the vector of weights?
        m_weights[i] = 1.0;
    }

    // TODO: 4) Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method 
    // (and others in this file).

    // TODO: was the random noise added in the for-loop above or hasor has
    // there still something to be done?

    m_isInitialized = true;
}

void ParticleFilter::prediction(
    double delta_t, 
    double std_pos[], 
    double velocity, 
    double yaw_rate) 
{
    // velocity is the previous velocity
    // yaw rate is the previous yaw rate
    // param std_pos: GPS measurement uncertainty: 
    // standard deviation of [x [m], y [m], theta [rad]]

    if (delta_t < 0.001)
    {
        printf("prediction: delta_t < 0.001 -> no update");
        return;
    }

    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and 
    // std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    default_random_engine randomGenerator;

    for (Particle particle : m_particles)
    {
        // update position and orientation
        const double theta(particle.theta);
        const double v_x = velocity * cos(theta);
        const double v_y = velocity * sin(theta);

        const double pred_x = particle.x + v_x * delta_t;
        const double pred_y = particle.y + v_y * delta_t;
        const double pred_theta = theta * yaw_rate * delta_t;

        // TODO: it's very likely highly inefficent to create
        // for each particle normal distributions
        normal_distribution<double> normDistX(pred_x, std_pos[0]);
        normal_distribution<double> normDistY(pred_y, std_pos[1]);
        normal_distribution<double> normDistTheta(pred_theta, std_pos[2]);

        const double rand_x = normDistX(randomGenerator);
        const double rand_y = normDistY(randomGenerator);
        const double rand_theta = normDistTheta(randomGenerator);

        printf("prediction: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)\n",
            particle.x, particle.y, particle.theta,
            pred_x, pred_y, pred_theta,
            rand_x, rand_y, rand_theta);

        particle.x = rand_x;
        particle.y = rand_y;
        particle.theta = rand_theta;
    }
}

void ParticleFilter::dataAssociation(
    std::vector<LandmarkObs> predicted, 
    std::vector<LandmarkObs>& observations) 
{
    // TODO: Find the predicted measurement that is closest to each observed 
    // measurement and assign the observed measurement to this particular 
    // landmark.
    // NOTE: this method will NOT be called by the grading code. But you will 
    // probably find it useful to implement this method and use it as a helper
    // during the updateWeights phase.

}

void ParticleFilter::updateWeights(
    double sensor_range, 
    double std_landmark[],
    std::vector<LandmarkObs> observations, 
    Map map_landmarks) 
{
    // TODO: Update the weights of each particle using a mult-variate Gaussian
    // distribution. You can read more about this distribution here:
    // https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. 
    //   Your particles are located according to the MAP'S coordinate system.
    //   You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND 
    //   translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to
    //   implement (look at equation 3.33 http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() 
{
    // TODO: Resample particles with replacement with probability proportional
    //  to their weight. 
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
