#include <ctime> 
#include <cstdlib> 
#include <math.h>
#include "golfcart_localization/particle_filter.h" 

void ParticleFilter::Init(double startLat, double startLon, double startHeading, int odomTicks, OdomParam odomModel) {
  this->startLat = startLat;
  this->startLon = startLon;
  this->startHeading = startHeading;
  this->lastTick = odomTicks;
  this->odomModel = odomModel;
    
  // Zero All Particles
  ParticleList.resize(NUM_PARTICLES);
  PropogatedList.resize(NUM_PARTICLES);
  
  for(unsigned int i = 0; i < ParticleList.size(); i++) {
    ParticleList[i].X = startX;
    ParticleList[i].Y = startY;
    ParticleList[i].Heading = startHeading;
  }
  
  currX = startX;
  currY = startY;
  currHeading = startHeading;
  
  srand(time(0));  // Initialize random number generator.
}

void ParticleFilter::RunIteration(int odomTicks, double lat, double lon, double heading, double deltaTime) {
  if(odomTicks == lastTick) return; // Do nothing if we have not moved
  
  PropogateParticles(odomTicks, lat, lon, heading);
  
  ResampleParticles();

  // Calculated Average
  double x_total = 0;
  double y_total = 0;
  double tx_total = 0;
  double ty_total = 0;

  for(unsigned int i = 0; i < ParticleList.size(); i++) {
  	x_total += ParticleList[i].X;
  	y_total += ParticleList[i].Y;
  	tx_total += cos(heading * D2R);
  	ty_total += sin(heading * D2R);
  }
  
  this->currX = x_total;
  this->currY = y_total;
  this->currHeading = R2D * atan2(ty_total, tx_total);
}

void ParticleFilter :: ResampleParticles() {
  if(highestWeight == 0) { ParticleList = PropogatedList; return; }
  
  vector<Particle> sampleSpace (NUM_PARTICLES);
  
	for(unsigned int i = 0; i < PropogatedList.size(); i++) {
	  PropogatedList[i].Weight = PropogatedList[i].Weight / highestWeight;
	  
	  if(PropogatedList[i].Weight > 0.8) {
	    for(int j = 0; j < 5; j++)
	      sampleSpace.push_back(PropogatedList[i]);
	  }
	  else if(PropogatedList[i].Weight > 0.7) {
	    for(int j = 0; j < 4; j++)
	      sampleSpace.push_back(PropogatedList[i]);
	  }
	  else if(PropogatedList[i].Weight > 0.6) {
	    for(int j = 0; j < 3; j++)
	      sampleSpace.push_back(PropogatedList[i]);
	  }
	  else if(PropogatedList[i].Weight > 0.5) {
	    for(int j = 0; j < 2; j++)
	      sampleSpace.push_back(PropogatedList[i]);
	  }
	  else if(PropogatedList[i].Weight > 0.4) {
	    for(int j = 0; j < 1; j++)
	      sampleSpace.push_back(PropogatedList[i]);
	  }
	}
	
	for(unsigned int i = 0; i < ParticleList.size(); i++) {
	  ParticleList[i] = sampleSpace[rand() % sampleSpace.size()];
	}
}

void ParticleFilter::PropogateParticles(int odomTicks, double lat, double lon, double heading) {
  int deltaTick = odomTicks - lastTick;
  
  // Check for encoder rollover
  if( deltaTick < 0) {
    deltaTick += odomModel.maxTicks;
  }
  
  double distTravelled = deltaTick * odomModel.wheelCircum / odomModel.ticksPerRev;
  double angleTraveled;
  
  // Propogate each particle
  highestWeight = 0;  
  for(unsigned int i = 0; i < ParticleList.size(); i++) {
  	angleTraveled = heading - ParticleList[i].Heading;
  	
    // Add randomness
    double trans = distTravelled - sample_normal_dist(0.1 * angleTraveled + 0.5 * distTravelled);
    double rot   = angleTraveled - sample_normal_dist(0.5 * angleTraveled + 0.1 * distTravelled);
    
    PropogatedList[i].X = ParticleList[i].X + trans * cos(D2R * ParticleList[i].Heading + D2R * angleTraveled);
    PropogatedList[i].Y = ParticleList[i].Y + trans * sin(D2R * ParticleList[i].Heading + D2R * angleTraveled);;
    PropogatedList[i].Heading = addAngle(ParticleList[i].Heading, rot);
    
    WeightParticle (lat, lon, PropogatedList[i] );
    
    if(highestWeight < PropogatedList[i].Weight) 
      highestWeight = PropogatedList[i].Weight;
  }
}

void ParticleFilter::WeightParticle(double lat, double lon, Particle &src) {
	src.Weight = 0;
}

/*
* Adds two angles together correcting for rollover in degrees
*/
double ParticleFilter :: addAngle( double ang1, double ang2 ) {
  double sum = ang1 + ang2;
  while (sum > 360) sum -= 360;
  while (sum < 0 )	sum += 360;
  return sum;
}
/*
 * Randomly samples a zero centered normal distribution from a with
 * variance 'var'.
 */
double ParticleFilter :: sample_normal_dist(double var) {
  double sum = 0;
  double stddev = sqrt ( var );
  for ( int i = 0; i < 12; ++i ) {
    sum += (rand() / (static_cast<double>(RAND_MAX) + 1.0)) * (2 * stddev) - stddev;
  }
  return sum;
}
      
