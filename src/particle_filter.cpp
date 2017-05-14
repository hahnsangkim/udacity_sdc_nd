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
#include <map>

#include "particle_filter.h"

using namespace std;

bool debug = false;
bool debug_state = false;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	std::default_random_engine gen;
    std::normal_distribution<double> ndx(x, std[0]);
    std::normal_distribution<double> ndy(y, std[1]);
    std::normal_distribution<double> ndt(theta, std[2]);

	num_particles = 10;
	weights = std::vector<double>(num_particles);
	particles = std::vector<Particle>(num_particles);
	for (int i=0; i < num_particles; i++) {
		particles[i].id = i;
		particles[i].x = ndx(gen);
		particles[i].y = ndy(gen);
		particles[i].theta = ndt(gen);
		particles[i].weight = 1.0;
		weights[i] = 1.0;
		if (0) {
			cout << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << endl;
		}
	}
	if (0) {
		exit(0);
	}
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::default_random_engine gen;
	if (debug_state) {
		cout << "predicted    " << particles[0].x << "  " << particles[0].y << endl;
	}   
	for (int i=0; i < num_particles; i++) {
		double theta = particles[i].theta;
		if (fabs(yaw_rate) >= 1e-6) {
			particles[i].x += velocity/yaw_rate * (sin(theta + yaw_rate*delta_t)-sin(theta));
			particles[i].y += velocity/yaw_rate * (cos(theta) - cos(theta + yaw_rate*delta_t));
			particles[i].theta += yaw_rate*delta_t;
		} else {
			particles[i].x += velocity * delta_t * cos(theta);
			particles[i].y += velocity * delta_t * sin(theta);

		}
		std::normal_distribution<double> ndx(particles[i].x, std_pos[0]);
		std::normal_distribution<double> ndy(particles[i].y, std_pos[1]);
		std::normal_distribution<double> ndt(particles[i].theta, std_pos[2]);
		particles[i].x = ndx(gen);	
		particles[i].y = ndy(gen);	
		particles[i].theta = ndt(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> landmarks, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (auto& obs:observations) {
		double min_dist = std::numeric_limits<double>::max();
		for (const auto lan : landmarks) {
			auto dist_val = dist(obs.x, obs.y, lan.x,lan.y);
			if (dist_val < min_dist) {
				obs.id = lan.id;
				min_dist = dist_val;
			}
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
	double std_x = std_landmark[0];//std_landmark[0]*cos(std_landmark[1]);
	double std_y = std_landmark[1];//std_landmark[0]*sin(std_landmark[1]); 
	/*
	std::vector<LandmarkObs> landmarks;
	for (auto lan : map_landmarks.landmark_list) {
	    landmarks.push_back(LandmarkObs{ lan.id_i,lan.x_f,lan.y_f });
	    if (debug) {
	    	cout << "landmarks [" << lan.id_i-1 << "] x: " << lan.x_f << " y: " << lan.y_f << endl;
	    }
	}
	*/
	for (int i=0; i < num_particles; i++) {
		//cout << "particle["<<i <<"]"<<endl;
		//convert car coordinates into map coordinates
		std::vector<LandmarkObs> tobservations;
		//tobs.resize(observations.size());
		for (const auto obs : observations) {
	    	//cout << "B obs [" << obs.id << "] (" << obs.x << ", " << obs.y << ")" << endl;
	    	//double obsx = obs.x;
	    	//double obsy = obs.y;
	    	double tobsx = cos(particles[i].theta)*obs.x - sin(particles[i].theta)*obs.y + particles[i].x;
	    	double tobsy = sin(particles[i].theta)*obs.x + cos(particles[i].theta)*obs.y + particles[i].y;
	    	tobservations.push_back(LandmarkObs{obs.id, tobsx, tobsy});
	    	//cout << "A obs [" << obs.id << "] (" << obs.x << ", " << obs.y << ")" << endl;
	    }
	    std::vector<LandmarkObs> ilandmarks;
        std::map<int, Map::single_landmark_s> idx2landmark;
        for (const auto& lan : map_landmarks.landmark_list){
            double distance = dist(lan.x_f, lan.y_f, particles[i].x, particles[i].y);
            if (distance <= sensor_range){
                ilandmarks.push_back(LandmarkObs{lan.id_i,lan.x_f,lan.y_f});
                idx2landmark.insert(std::make_pair(lan.id_i, lan)); // log the landmark so that it can be used later
            }
        }

        if (ilandmarks.size() > 0) {

		    dataAssociation(ilandmarks, tobservations);

		    double min_dist =  std::numeric_limits<unsigned int>::max();
		    particles[i].weight = 1;
			for (const auto tobs : tobservations) {
				double lx = idx2landmark[tobs.id].x_f;
		    	double ly = idx2landmark[tobs.id].y_f;
				double tx = tobs.x;
		    	double ty = tobs.y;
		    	//if (dist(par.x, par.y, lx, ly) <= sensor_range) {
				//assert(min_dist < std::numeric_limits<unsigned int>::max());
					double x_diff = pow(tx-lx,2)/(2*pow(std_x,2));
					double y_diff = pow(ty-ly,2)/(2*pow(std_y,2));
					particles[i].weight *= 1/(2*M_PI*std_x*std_y) * exp(-(x_diff + y_diff));
					if (debug) {
						cout << "x_diff: " << x_diff << "y_diff: " << y_diff << endl;
				    	cout << "tobs (" << tx <<"," << ty << ") ";
				    	cout << "landmark (" << lx << ", " << ly <<") weight "<< particles[i].weight << endl;
				    }
			    //}
			}
			if (debug) {
				exit(0);
			}
		    weights[i] = particles[i].weight;
		} else {
			weights[i] = 0.0;
		}
		if (0) {
			cout << particles[i].x << ", " << particles[i].y << ", " << particles[i].weight << endl;
		}
	}
	//exit(0);
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::default_random_engine gen;
    std::vector<Particle> resampled_particles;
    resampled_particles.resize(num_particles);
    //resampled_particles = std::vector<Particle>(num_particles);
    std::discrete_distribution<> d(weights.begin(), weights.end());
    //std::map<int, int> m;
    for(int i=0; i<num_particles; ++i) {
        resampled_particles[i] = particles[d(gen)];
    }
    particles=resampled_particles;
}
/*
Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
*/
void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
