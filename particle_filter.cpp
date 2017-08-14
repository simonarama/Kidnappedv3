/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <vector>
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <sstream>
#include <string>
#include <iterator>
#include "map.h"
#include <map>

//#include "Eigen/Dense"

//David Simon lecture code modified
// accessing structure, resample discrete distribution, normal distribution code modified from forum

using std::vector;

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
    //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    //is_initialized = false;
    num_particles = 5;  //had 150 same results
    
    default_random_engine gen;
    
    //initialize   
    double weight = 1.0;

    // This line creates a normal (Gaussian) distribution for x
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    
    
    for (int i = 0; i < num_particles; i++){
        
     
        
        Particle particle = {};
        particle.id = i;
        particle.x = dist_x(gen);        
        particle.y = dist_y(gen);        
        particle.theta = dist_theta(gen);       
        particle.weight = weight;      
        particles.push_back(particle);
        weights.push_back(weight); //added
        
}
         
    is_initialized = true; 
     
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    default_random_engine gen;
    
    int i;
    
    int num_particles = 5;  //had 150 same result
    

    Particle &particle = particles[i]; 
    for (int i = 0; i < num_particles; ++i) {
        double newx = 0; //initialize
        double newy = 0; //initialize
        double newtheta = 0; //initialize

        
        if(yaw_rate == 0){
            newx = particles[i].x + velocity * delta_t * cos(particles[i].theta);
            newy = particles[i].y + velocity * delta_t * sin(particles[i].theta);
            newtheta = particles[i].theta;
        }
        else{
            newx = particles[i].x + (velocity/yaw_rate)*(sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
            
            newy = particles[i].y + (velocity/yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
                
            newtheta = particles[i].theta + yaw_rate * delta_t;
                        
        }
        
        //add Gaussian noise        
        normal_distribution<double> dist_x(newx, std_pos[0]);
        normal_distribution<double> dist_y(newy, std_pos[1]);
        normal_distribution<double> dist_theta(newtheta, std_pos[2]);
            
        particles[i].x = dist_x(gen); 
       
        particles[i].y = dist_y(gen);
      
        particles[i].theta = dist_theta(gen);
             
}
         
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
    //   implement this method and use it as a helper during the updateWeights phase.



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
    //   3.33
  
    // landmark_list.size
    int i;
    int num_particles = 5; 
    Particle &particle = particles[i];
    
    double sumweight = 1.0; //initialize
    double prob = 1.0;  //initialize
    //initialize transfx, transfy
    for(int i = 0; i < observations.size(); i++){
              
        transfx.push_back(0.0);
        transfy.push_back(0.0);
    }
    
    //reinitialize weights to 1.0 to avoid drift, needed for continuing iterations
    for(int i = 0; i < num_particles; i++){
        weights[i] = 1.0;
        particles[i].weight = 1.0;}
        
    //initialize pred_landmark, error
    for(int m = 0; m < 42; m++){
        pred_landmark.push_back(70.0);}
        
    for(int i = 0; i < num_particles; ++i) {  
    //transform each particle sensor readings
        double weight = 1.0;  
        
        for(int j = 0; j < observations.size(); j++){
           
            LandmarkObs obs;

            //per internet "Planning Algorithms", Steven LaValle, 2006  //not working quite right here
            //transfx[j] = particles[i].x + ((observations[j].x) * (cos(particles[i].theta))) + ((observations[j].y) * (sin(particles[i].theta)));
            //transfy[j] = particles[i].y + (observations[j].x * (-1.0) * sin(particles[i].theta)) + (observations[j].y * cos(particles[i].theta));
            
            //transformation per forum mentor below, working correctly here
            transfx[j] = particles[i].x + ((observations[j].x) * (cos(particles[i].theta))) - ((observations[j].y) * (sin(particles[i].theta)));
            transfy[j] = particles[i].y + (observations[j].x * sin(particles[i].theta)) + (observations[j].y * cos(particles[i].theta));
                      
            double min_error = 80.0; //initialize
            double error = 90.0; //initialize
            for(int k = 0; k < 42; k++){
                            
                pred_landmark[k] = sqrt((map_landmarks.landmark_list[k].x_f - transfx[j])*(map_landmarks.landmark_list[k].x_f - transfx[j])+(map_landmarks.landmark_list[k].y_f - transfy[j])*(map_landmarks.landmark_list[k].y_f - transfy[j]));
                
                if(k > 0){
                    if(pred_landmark[k] < pred_landmark[k-1]){
                        error = pred_landmark[k];}
                                       
                    else if (pred_landmark[k] > pred_landmark[k-1]){
                        error = pred_landmark[k-1];}
                    }
                    if(error < min_error){
                        min_error = error;}
                    else{
                        min_error = min_error;}
                        }
                                  
            prob = (1/(2*M_PI*std_landmark[0]*std_landmark[1]))* (exp((-min_error * min_error)/(2*std_landmark[0]*std_landmark[1]))); 
           
            particles[i].weight *= prob;
            weights[i] *= prob;
           
            }
            
            weight = weights[i];
           
            particle.weight = weight;
            particle.weight = particles[i].weight;
            sumweight = sumweight + weight;
                      
            weights.push_back(weight);
            
            } 
           
		}
            

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight. 
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    double sumweight = 0.0; //initialize

    
    int i;
    Particle &particle = particles[i];
    
    int num_particles = 5;  //had 150 same result
    
    for(i = 0; i < num_particles; i++){
        normprob.push_back(1);}
    
    for(i = 0; i < num_particles; i++){      
        sumweight += particles[i].weight;}

    for(i = 0; i < num_particles; i++){
        normprob[i] = (particles[i].weight/sumweight) * num_particles + 0.5;
    }
        

    
    
    //initialize
    
    //discrete distribution code modified from internet and forum
    
	default_random_engine gen;
	std::discrete_distribution<int>distribution{weights[0],weights[1],weights[2],weights[3],weights[4]};
	//std::discrete_distribution<int>distribution(weights.begin(), weights.end());
	vector<Particle>resample_particles;
	
	for(int i = 0; i < num_particles; i++)
	{
		resample_particles.push_back(particles[distribution(gen)]);
	}
	particles = resample_particles;
	
    
	
	//std::random_device rd;
    //std::mt19937 gen(rd());
    //std::discrete_distribution<>d{particles[0].weight, particles[1].weight,particles[2].weight,particles[3].weight,particles[4].weight};

    //std::discrete_distribution<>d{weights[0],weights[1],weights[2],weights[3],weights[4]};

    //std::discrete_distribution<> d{weights[0],weights[1],weights[2],weights[3],weights[4],weights[5],weights[6],weights[7], 
    //weights[8],weights[9],weights[10],weights[11],weights[12],weights[13],weights[14],weights[15],weights[16],weights[17],
    //weights[18],weights[19],weights[20],weights[21],weights[22],weights[23],weights[24],weights[25],weights[26],weights[27],
    //weights[28],weights[29],weights[30],weights[31],weights[32],weights[33],weights[34],weights[35],weights[36],weights[37],
    //weights[38],weights[39],weights[40],weights[41],weights[42],weights[43],weights[44],weights[45],weights[46],weights[47],
    //weights[48],weights[49],weights[50],weights[51],weights[52],weights[53],weights[54],weights[55],weights[56],weights[57],
    //weights[58],weights[59],weights[60],weights[61],weights[62],weights[63],weights[64],weights[65],weights[66],weights[67],
    //weights[68],weights[69],weights[70],weights[71],weights[72],weights[73],weights[74],weights[75],weights[76],weights[77],
    //weights[78],weights[79],weights[80],weights[81],weights[82],weights[83],weights[84],weights[85],weights[86],weights[87],
    //weights[88],weights[89],weights[90],weights[91],weights[92],weights[93],weights[94],weights[95],weights[96],weights[97],
    //weights[98],weights[99],weights[100],weights[101],weights[102],weights[103],weights[104],weights[105],weights[106],weights[107],
    //weights[108],weights[109],weights[110],weights[111],weights[112],weights[113],weights[114],weights[115],weights[116],weights[117],
    //weights[118],weights[119],weights[120],weights[121],weights[122],weights[123],weights[124],weights[125],weights[126],weights[127],
    //weights[128],weights[129],weights[130],weights[131],weights[132],weights[133],weights[134],weights[135],weights[136],weights[137],
    //weights[138],weights[139],weights[140],weights[141],weights[142],weights[143],weights[144],weights[145],weights[146],weights[147],
    //weights[148],weights[149]};



    //weights.begin(), weights.end()  not working

    //std::map<int, int> m;
   // for(int n = 0; n < num_particles; ++n) {
       // ++m[d(gen)]; was not in loop
        //Particle particle = particles[d(gen)];
       // particles.push_back(particle);}
   
    
}



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