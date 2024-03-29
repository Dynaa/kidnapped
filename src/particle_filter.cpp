/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <random>


#include "helper_functions.h"


using std::normal_distribution;

using std::string;
using std::vector;

#define eps 0.00001

// declare a random engine to be used across multiple and various method calls
static std::default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. -> Ok
   * TODO: Add random Gaussian noise to each particle. -> Ok
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  std::default_random_engine gen; //random engine initialized
  num_particles = 100;  // TODO: Set the number of particles

  // create Gaussian distribution in x, y, theta
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; ++i) 
  {    
    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
    particle.theta = dist_theta(gen);

    particle.weight = 1.0; 
    particles.push_back (particle); 
    weights.push_back(particle.weight);
  }

  is_initialized = true; // particle filter initialized

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  std::default_random_engine gen; //random engine initialized

  // create Gaussian distribution in x, y, theta for zero-mean noise addition to motion
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);

  for (int i = 0; i < num_particles; ++i)
  {
    // avoid dividing by zero
    if (fabs(yaw_rate) < 0.0001)
    {
      // motion model when yaw rate = 0
      particles[i].x += velocity * cos(particles[i].theta) * delta_t;
      particles[i].y += velocity * sin(particles[i].theta) * delta_t;
    }
    else
    {
      // motion model when yaw rate != 0
      particles[i].x +=  (velocity/yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta)-cos(particles[i].theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }    

    // Gaussian noise addition to movement to account for uncertainty
    particles[i].x += dist_x(gen);
    particles[i].y +=  dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }


}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  for (unsigned int i = 0; i < observations.size(); ++i)
  {
    double min_dist = std::numeric_limits<double>::max(); 
    int id_in_map = -1; 
    for (unsigned int j = 0; j < predicted.size(); ++j)
    {
      double diff = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
      if (diff < min_dist)
      {
        min_dist = diff;
        id_in_map = predicted[j].id;
      }
      observations[i].id = id_in_map;
    }
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  double sig_x = std_landmark[0];
  double sig_y = std_landmark[1];
  double gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  for (int i = 0; i < num_particles; ++i)
  {

    Particle& p = particles[i];

    // Step1. TRANSFORM each observation marker from the vehicle's coordinates to the map's coordinates
    vector<LandmarkObs> Transformed_OBS;
    for (unsigned int j = 0; j < observations.size(); ++j)
    {
      double x_map = observations[j].x * cos(p.theta) - observations[j].y * sin(p.theta) + p.x;
      double y_map = observations[j].x * sin(p.theta) + observations[j].y * cos(p.theta) + p.y;
      Transformed_OBS.push_back (LandmarkObs{observations[j].id, x_map, y_map}); 
    }

    // Step2. ENSURE map landmarks are inside sensor range
    vector<LandmarkObs> predicted; 
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j)
    {
      int land_ID = map_landmarks.landmark_list[j].id_i;
      float land_x = map_landmarks.landmark_list[j].x_f;
      float land_y = map_landmarks.landmark_list[j].y_f;
      double pred_distance = dist(p.x, p.y, land_x, land_y);
      if (pred_distance <= sensor_range)
      {
        predicted.push_back(LandmarkObs{land_ID, land_x, land_y});
      }
    }

    // Step3. Nearest Neighbor Data Association
    dataAssociation(predicted, Transformed_OBS);

    // Step4. Compute WEIGHT of particle
    vector<int> association;
    vector<double> sense_x;
      vector<double> sense_y;

    particles[i].weight = 1.0;
    double map_x, map_y, mu_x, mu_y;
    for (unsigned int t = 0; t < Transformed_OBS.size(); ++t)
    {
      map_x =  Transformed_OBS[t].x;
      map_y =  Transformed_OBS[t].y;
      for (unsigned int p = 0; p < predicted.size(); ++p)
      {
        // Associate prediction with transformed observation
        if (predicted[p].id == Transformed_OBS[t].id)
        {
          mu_x = predicted[p].x;
          mu_y = predicted[p].y;
        }
      }


      // Compute exponent
      double exponent = (0.5*pow( (map_x - mu_x)/sig_x, 2.0 )+0.5*pow( (map_y - mu_y)/sig_y, 2.0 ));
      // Compute weight using normalization terms and exponent
      double p_weight = gauss_norm * exp(- exponent);
      particles[i].weight *= p_weight;

      // Append particle associations
      association.push_back(Transformed_OBS[t].id);
      sense_x.push_back(map_x);
      sense_y.push_back(map_y);
    }


    weights[i] = p.weight;

    // For blue lasers (belongs to the best particle )
    SetAssociations(p, association, sense_x, sense_y);

  } 


}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  //Get weights and max weight.
  vector<double> weights;
  double maxWeight = std::numeric_limits<double>::min();
  for(int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
    if(particles[i].weight > maxWeight) {
      maxWeight = particles[i].weight;
    }
  }

  std::uniform_real_distribution<double> distDouble(0.0, maxWeight);
  std::uniform_int_distribution<int> distInt(0, num_particles - 1);
  int index = distInt(gen);
  double beta = 0.0;
  vector<Particle> resampledParticles;
  for(int i = 0; i < num_particles; i++) {
    beta += distDouble(gen) * 2.0;
    while(beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampledParticles.push_back(particles[index]);
  }

  particles = resampledParticles;


}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}