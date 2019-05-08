/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;

// book-keeping
//static long num_iterations = 0;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  cout << "ParticleFilter::init (...)" << endl;

  // number of particles initialized in the constructor
  // can change it here, if required.
  // num_particles = 100;  // TODO: Set the number of particles

  // random number generator
  std::default_random_engine gen;
  // gaussian distributions for x, y, and theta
  std::normal_distribution<double> x_dist(x, std[0]);
  std::normal_distribution<double> y_dist(y, std[1]);
  std::normal_distribution<double> theta_dist(theta, std[2]);

  // for each particle
  for (int i = 0; i < num_particles; ++i){
    // create and initialize the particle
    Particle particle;
    particle.id = i+1;
    particle.x = x_dist(gen);
    particle.y = y_dist(gen);
    particle.theta = theta_dist(gen);
    particle.weight = 1.0;

    // add particle to the particles vector
    particles.push_back(particle);
    // set particle's initial weight to 1.0
    weights.push_back(1.0);
  }

  is_initialized = true;

  // book-keeping
  //num_iterations++;

  cout << "Initialized Particle Filter with " << num_particles << " particles" << endl;
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

  // book-keeping
  // num_iterations++;

  // add measurements with gaussian noise based on the uncertainty (stdev)
  std::default_random_engine generator;
  std::normal_distribution<double> x_dist(0, std_pos[0]);
  std::normal_distribution<double> y_dist(0, std_pos[1]);
  std::normal_distribution<double> theta_dist(0, std_pos[2]);

  // for each particle
  for (unsigned int i = 0; i < particles.size(); ++i){

    double x0 = particles[i].x;
    double y0 = particles[i].y;
    double theta0 = particles[i].theta;

    // new x pos, y pos, theta
    double theta = 0.0;
    double x = 0.0;
    double y = 0.0;

    // here we need to check whether yaw_rate is non-zero, or at least has a significant value
    double epsilon = 1.0E-05;
    if (fabs(yaw_rate) > epsilon) {
      theta = theta0 + yaw_rate*delta_t;
      x = x0 + (velocity/yaw_rate) * (sin(theta) - sin(theta0));
      y = y0 + (velocity/yaw_rate) * (cos(theta0) - cos(theta));
    } else {
      theta = theta0;
      x = x0 + velocity * delta_t * cos(theta0);
      y = y0 + velocity * delta_t * sin(theta0);
    }

    // add gaussian noise
    particles[i].x  = x + x_dist(generator);
    particles[i].y = y + y_dist(generator);
    particles[i].theta = theta + theta_dist(generator);
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

  // for each transformed observation
  for (unsigned int o = 0; o < observations.size(); ++o) {
    double x1 = observations[o].x;
    double y1 = observations[o].y;
    double min_dist = std::numeric_limits<double>::max();  // set it to some high value
    int association_id = 0; // initialize to a known value

    // find the closest predicted landmark for this observation
    for (unsigned int p = 0; p < predicted.size(); ++p) {
      double x2 = predicted[p].x;
      double y2 = predicted[p].y;
      // get the distance from the predicted landmark
      double a_dist = dist(x1, y1, x2, y2);
      // is this a closer landmark?
      if (a_dist < min_dist) {
        min_dist = a_dist;
        association_id = predicted[p].id;
      }
    }
    // assign the id of the closest predicted landmark to this observation
    observations[o].id = association_id;
  } // for each transformed observation
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

  /**
   * STEPS FOR WEIGHT UPDATE OF PARTICLES
   * for each particle:
   * 1. transform ALL landmark observations (sensor measurements) from vehicle coordinate system to map
   *    coordinate system for the particle
   * 2. associate each measurements with a map landmark identifier, by choosing the closest landmark to
   *    each transformed observation/measurement
   * 3. use this information to calculate weight values for the particle using multivariate gaussian pdf
   */

  // for each particle
  for (unsigned int i = 0; i < particles.size(); ++i) {

    double x_particle = particles[i].x;
    double y_particle = particles[i].y;
    double theta_particle = particles[i].theta;

    // 1. for each landmark observation (sensor measurement), get the transformed measurement
    vector<LandmarkObs> transformed_observations;
    for (unsigned int o = 0; o < observations.size(); ++o) {

      LandmarkObs trans_obs;

      double x_obs = observations[o].x;
      double y_obs = observations[o].y;

      // measurement transformation (rotation + translation using homogeneous transformation)
      Eigen::Vector3f transformed_landmark_obs = homogeneous_transformation(x_particle, y_particle,
                                                                            theta_particle, x_obs, y_obs);

      trans_obs.x = transformed_landmark_obs[0];
      trans_obs.y = transformed_landmark_obs[1];
      trans_obs.id = 0;  // just to ensure some known initial value

      // add the observation to the transformed observations list for this particle
      transformed_observations.push_back(trans_obs);
    }

    // get the predicted landmarks from the actual map landmarks
    // these would be the ones in the sensor range from the particle's pose
    vector<LandmarkObs> predicted_landmarks;

    for (unsigned int l = 0; l < map_landmarks.landmark_list.size(); ++l) {

      Map::single_landmark_s map_landmark = map_landmarks.landmark_list.at(l);

      // distance of this landmark from the particle
      double l_dist = dist(x_particle, y_particle, map_landmark.x_f, map_landmark.y_f);

      // the particle can only predict this landmark if the landmark is in the sensor range
      if (l_dist <= sensor_range) {
        // add the landmark to the predicted landmarks list
        predicted_landmarks.push_back(LandmarkObs{map_landmark.id_i, map_landmark.x_f, map_landmark.y_f});
      }
    }

    // 2. associate the observations to predicted landmarks
    // the transformed observations will get the ids of the predicted landmarks
    dataAssociation(predicted_landmarks, transformed_observations);

    // 3. based on the particle's association, calculate and update it's weight
    // this will be the product of the multi-variate gaussian probabilities of all
    // the individual observations for the particle
    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];

    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;

    // new particle weight
    double new_particle_weight = 1.0;

    for (LandmarkObs obs: transformed_observations) {
      // transformed observations and associated map id

      associations.push_back(obs.id);
      sense_x.push_back(obs.x);
      sense_y.push_back(obs.y);

      // map landmark index is 'map landmark id - 1'
      Map::single_landmark_s pred_landmark = map_landmarks.landmark_list.at(
                                             (obs.id-1) % map_landmarks.landmark_list.size());

      double weight = multivariate_gaussian_prob(sigma_x, sigma_y, obs.x, obs.y,
                                                 pred_landmark.x_f, pred_landmark.y_f);

      // particle's new weight as the product of
      // individual observation probabilities
      new_particle_weight *= weight;
    }

    // update particle weight
    particles[i].weight = new_particle_weight;
    weights[i] = new_particle_weight;

    // update the particle's associations, sense_x, sense_y
    // for rendering on the simulator
    SetAssociations(particles[i], associations, sense_x, sense_y);

  }  // for each particle

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  // create a generator
  std::default_random_engine gen;

  // create a discrete distribution with particle weights
  // Note: std::discrete_distribution produces random integers on the interval
  // [0, n), where the probability of each individual integer i is defined as Wi/S,
  // that is the weight of the ith integer divided by the sum of all n weights.
  std::discrete_distribution<int> dist(weights.begin(), weights.end());
  vector<Particle> new_particles;

  // sample the particles based on the weights
  for (int i =0; i < num_particles; ++i) {
    int idx = dist(gen);
    new_particles.push_back(particles.at(idx));
  }

  // cout << "Iteration: " << num_iterations << endl;

  // replace the current set of particles with the new re-sampled ones
  particles = new_particles;
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
