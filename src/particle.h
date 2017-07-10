#ifndef PARTICLE_H_
#define PARTICLE_H_

struct Particle
{
    int id;
    double x;
    double y;
    double theta;
    double weight;

    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;
};

#endif // PARTICLE_H_