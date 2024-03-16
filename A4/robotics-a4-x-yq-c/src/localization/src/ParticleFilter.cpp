#include "localization/ParticleFilter.h"
#include "localization/Util.h"

#include "tf/tf.h"

#include <ros/ros.h>
#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;

ParticleFilter::ParticleFilter(int numberOfParticles) {
	this->numberOfParticles = numberOfParticles;

	// initialize particles
	for (int i = 0; i < numberOfParticles; i++) {
		this->particleSet.push_back(new Particle());
	}

	// this variable holds the estimated robot pose
	this->bestHypothesis = new Particle();

	// at each correction step of the filter only the laserSkip-th beam of a scan should be integrated
	this->laserSkip = 5;

	// distance map used for computing the likelihood field
	this->distMap = NULL;
}

ParticleFilter::~ParticleFilter() {
	// delete particles
	for (int i = 0; i < numberOfParticles; i++) {
		Particle* p = this->particleSet[i];
		delete p;
	}

	this->particleSet.clear();

	if (this->likelihoodField)
		delete[] this->likelihoodField;

	delete this->bestHypothesis;

	if (this->distMap)
		delete[] this->distMap;
}

int ParticleFilter::getNumberOfParticles() {
	return this->numberOfParticles;
}

std::vector<Particle*>* ParticleFilter::getParticleSet() {
	return &(this->particleSet);
}



void ParticleFilter::initParticlesUniform() {
    int mapWidth, mapHeight;	
    double mapResolution;	
    this->getLikelihoodField(mapWidth, mapHeight, mapResolution);

	for(Particle* p : this->particleSet)
	{
		p->x = Util::uniformRandom(0.0, mapWidth*mapResolution);
		p->y = Util::uniformRandom(0.0, mapHeight*mapResolution);
		p->theta = Util::uniformRandom(0, 2*M_PI);
		p->weight = 1.0/this->numberOfParticles;
	}
}

void ParticleFilter::initParticlesGaussian(double mean_x, double mean_y, double mean_theta,
		 double std_xx, double std_yy, double std_tt) {
	for (Particle* p : this->particleSet) {
		p->x = Util::gaussianRandom(mean_x, std_xx);
		p->y = Util::gaussianRandom(mean_y, std_yy);
		p->theta = Util::gaussianRandom(mean_theta, std_tt);
		p->weight = 1.0/this->numberOfParticles;
	}
}

/**
 *  Initializes the likelihood field as our sensor model.
 */
// @param double zRand 		the minimum likelihood of every cell
// @param double sigmaHit 	the standard deviation of the gaussian likelihood distribution
void ParticleFilter::setMeasurementModelLikelihoodField(
		const nav_msgs::OccupancyGrid& map, double zRand, double sigmaHit) {
	ROS_INFO("Creating likelihood field for laser range finder...");

	// create the likelihood field - with the same discretization as the occupancy grid map
	this->likelihoodField = new double[map.info.height * map.info.width];
	this->likelihoodFieldWidth = map.info.width;
	this->likelihoodFieldHeight = map.info.height;
	this->likelihoodFieldResolution = map.info.resolution;

    // calculates the distance map and stores it in member variable 'distMap'
	// for every map position it contains the distance to the nearest occupied cell.
	calculateDistanceMap(map);

    // Here you have to create your likelihood field
	// HINT0: sigmaHit is given in meters. You have to take into account the resolution of the likelihood field to apply it.
	// HINT1: You will need the distance map computed 3 lines above
	// HINT2: You can visualize it in the map_view when clicking on "show likelihood field" and "publish all".
	// HINT3: Storing probabilities in each cell between 0.0 and 1.0 might lead to round-off errors, therefore it is
	// good practice to convert the probabilities into log-space, i.e. storing log(p(x,y)) in each cell. As a further
	// advantage you can simply add the log-values in your sensor model, when you weigh each particle according the
	// scan, instead of multiplying the probabilities, because: log(a*b) = log(a)+log(b).

	for (int i = 0; i < this->likelihoodFieldWidth; i++)
	{
		for (int j = 0; j < this->likelihoodFieldHeight; j++)
		{
			int ind = computeMapIndex(this->likelihoodFieldWidth,this->likelihoodFieldHeight,i,j);
			double pHit = Util::gaussian(1-zRand, sigmaHit/this->likelihoodFieldResolution, this->distMap[ind]);
			this->likelihoodField[ind] = log((1-zRand)*pHit + zRand);
		}
	}

	ROS_INFO("...DONE creating likelihood field!");
}


void ParticleFilter::calculateDistanceMap(const nav_msgs::OccupancyGrid& map) {
	// calculate distance map = distance to nearest occupied cell
	distMap = new double[likelihoodFieldWidth * likelihoodFieldHeight];
	int occupiedCellProbability = 90;
	// initialize with max distances
	for (int x = 0; x < likelihoodFieldWidth; x++) {
		for (int y = 0; y < likelihoodFieldHeight; y++) {
			distMap[x + y * likelihoodFieldWidth] = 32000.0;
		}
	}
	// set occupied cells next to unoccupied space to zero
	for (int x = 0; x < map.info.width; x++) {
		for (int y = 0; y < map.info.height; y++) {
			if (map.data[x + y * map.info.width] >= occupiedCellProbability) {
				bool border = false;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (!border && x + i >= 0 && y + j >= 0 && x + i
								< likelihoodFieldWidth && y + j
								< likelihoodFieldHeight && (i != 0 || j != 0)) {
							if (map.data[x + i + (y + j) * likelihoodFieldWidth]
									< occupiedCellProbability && map.data[x + i
									+ (y + j) * likelihoodFieldWidth] >= 0)
								border = true;
						}
						if (border)
							distMap[x + i + (y + j) * likelihoodFieldWidth]
									= 0.0;
					}
				}
			}
		}
	}
	// first pass -> SOUTHEAST
	for (int x = 0; x < likelihoodFieldWidth; x++)
		for (int y = 0; y < likelihoodFieldHeight; y++)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}

	// second pass -> NORTHWEST
	for (int x = likelihoodFieldWidth - 1; x >= 0; x--)
		for (int y = likelihoodFieldHeight - 1; y >= 0; y--)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}
}

double* ParticleFilter::getLikelihoodField(int& width, int& height,
		double& resolution) {
	width = this->likelihoodFieldWidth;
	height = this->likelihoodFieldHeight;
	resolution = this->likelihoodFieldResolution;

	return this->likelihoodField;
}

/**
 *  A generic measurement integration method that invokes some specific observation model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::measurementModel(
		const sensor_msgs::LaserScanConstPtr& laserScan) {
	likelihoodFieldRangeFinderModel(laserScan);
}


/**
 *  Method that implements the endpoint model for range finders.
 *  It uses a precomputed likelihood field to weigh the particles according to the scan and the map.
 */
void ParticleFilter::likelihoodFieldRangeFinderModel(
		const sensor_msgs::LaserScanConstPtr & laserScan) {
	this->sumOfParticleWeights = 0.0;
	double res = this->likelihoodFieldResolution;
	for (Particle* p : this->particleSet) {
		double weight_temp = 0.0;
		for (size_t j = 0; j < laserScan->ranges.size(); j+=this->laserSkip) {
            if((laserScan->ranges[j] < laserScan->range_max) && (laserScan->ranges[j] > laserScan->range_min)) {
                double theta_prim = Util::normalizeTheta(p->theta + laserScan->angle_min + laserScan->angle_increment*j);
                int x_prim = (p->x + laserScan->ranges[j]* cos(theta_prim)) / res; //todebug
                int y_prim = (p->y + laserScan->ranges[j]* sin(theta_prim)) / res;
                int ind = computeMapIndex(this->likelihoodFieldWidth,this->likelihoodFieldHeight,x_prim,y_prim);
                if ((ind > likelihoodFieldWidth*likelihoodFieldHeight) || (x_prim < 0) || (x_prim > likelihoodFieldWidth) || (y_prim < 0) || (y_prim > likelihoodFieldHeight)){
                    weight_temp -= 0.4;
                }
                else
                {
                    weight_temp += this->likelihoodField[ind];
                }
            }
			
		}
		p->weight = exp(weight_temp);
		this->sumOfParticleWeights += p->weight;
	}
	for (Particle* p : this->particleSet) {
		p->weight /= this->sumOfParticleWeights;
	}
}

void ParticleFilter::setMotionModelOdometry(double alpha1, double alpha2,
		double alpha3, double alpha4) {
	this->odomAlpha1 = alpha1;
	this->odomAlpha2 = alpha2;
	this->odomAlpha3 = alpha3;
	this->odomAlpha4 = alpha4;
}

/**
 *  A generic motion integration method that invokes some specific motion model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::sampleMotionModel(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	sampleMotionModelOdometry(oldX, oldY, oldTheta, newX, newY, newTheta);
}

/**
 *  Method that implements the odometry-based motion model.
 */
void ParticleFilter::sampleMotionModelOdometry(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
				// ROS_INFO("no, here");
	
	int mapWidth, mapHeight;	
    	double mapResolution;		
    	getLikelihoodField(mapWidth, mapHeight, mapResolution);

	double d_tr = sqrt(pow((newX - oldX),2) + pow((newY - oldY),2));
	double d_1 = Util::normalizeTheta(atan2(newY - oldY, newX - oldX) - oldTheta);
 	double d_2 = Util::normalizeTheta(Util::normalizeTheta(newTheta - oldTheta) - d_1);

	for (Particle* p : this->particleSet) {
		double d1_p = d_1 + Util::gaussianRandom(0, odomAlpha1*abs(d_1)+odomAlpha2*d_tr);
		double d_t_p = d_tr + Util::gaussianRandom(0, odomAlpha3*d_tr+odomAlpha4*(Util::normalizeTheta(abs(d_1)+abs(d_2))));
		double d2_p = d_2 + Util::gaussianRandom(0, odomAlpha1*abs(d_2)+odomAlpha2*d_tr);
		p->x = p->x + d_t_p*cos(Util::normalizeTheta(p->theta+d1_p));
		p->y = p->y + d_t_p*sin(Util::normalizeTheta(p->theta+d1_p));
		p->theta = Util::normalizeTheta(p->theta + Util::normalizeTheta(d1_p+d2_p));
	}
}


/**
 *  The stochastic importance resampling.
 */

void ParticleFilter::resample() {

	std::vector<double> cdf(this->numberOfParticles);
        double u=0.0;
	std::vector<Particle*> S;

	double norm = 1.0/this->numberOfParticles;
	double best_weight = 0.0;
	
    cdf[0] = this->particleSet[0]->weight;
	for(int i = 1; i<this->numberOfParticles; i++)
	{
		cdf[i] = cdf[i-1]+this->particleSet[i]->weight;
	}

    	u = Util::uniformRandom(0.0, norm);
	
	int i = 0;
	for(int j = 0; j < this->numberOfParticles; j++)
	{
		while(u > cdf[i])
		{
			i+=1;
		}

		S.push_back(new Particle(this->particleSet[i]));
		u = u + norm;

		if(best_weight < S[j]->weight)
		{
			best_weight = S[j]->weight;
			this->bestHypothesis = S[j];
		}
	}
    for(int i = 0; i < this->numberOfParticles; i++)
	{
        //delete this->particleSet[i];
        this->particleSet[i] = S[i];
	}
}

Particle* ParticleFilter::getBestHypothesis() {
	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
		int y) {
	return x + y * width;
}
