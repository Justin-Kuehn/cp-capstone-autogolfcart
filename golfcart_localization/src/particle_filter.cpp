/*
 * Notes: 
 * right now this particle filter assumes all motion is forward in direction.
 */
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
  
  this->currX = x_total / ParticleList.size();
  this->currY = y_total / ParticleList.size();
  this->currHeading = R2D * atan2(ty_total, tx_total);
}

void ParticleFilter :: ResampleParticles() {
  if(highestWeight == 0) { ParticleList = PropogatedList; return; }
  
  vector<Particle> sampleSpace (NUM_PARTICLES);
  
	for(unsigned int i = 0; i < PropogatedList.size(); i++) {
	  // This normalizes the weights around 1
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
  lastTick = odomTicks;
  
  // Check for encoder rollover
  if( deltaTick < 0) {
    deltaTick += odomModel.maxTicks;
  }
  
  double distTravelled = deltaTick * odomModel.wheelCircum / odomModel.ticksPerRev;
  double angleTraveled = heading - currHeading;
  
  //std::cout << "\t distTravelled: " << distTravelled << std::endl;
  //std::cout << "\t angleTraveled: " << angleTraveled << std::endl;
  
  // Propogate each particle
  highestWeight = 0;  
  for(unsigned int i = 0; i < ParticleList.size(); i++) {  	
    // Add randomness
    double trans = distTravelled - sample_normal_dist(0.1 * distTravelled);
    double rot   = angleTraveled - sample_normal_dist(0.1 * angleTraveled);
    
    
    PropogatedList[i].X = ParticleList[i].X + trans * cos(D2R * ParticleList[i].Heading + D2R * angleTraveled);
    PropogatedList[i].Y = ParticleList[i].Y + trans * sin(D2R * ParticleList[i].Heading + D2R * angleTraveled);;
    PropogatedList[i].Heading = addAngle(ParticleList[i].Heading, rot);
    
    /*
    if( i == 0) {
        std::cout << "\t trans: " << trans << std::endl;
        std::cout << "\t rot: " << rot << std::endl;
        std::cout << "\t PropogatedList[i].X " << PropogatedList[i].X << std::endl;
        std::cout << "\t PropogatedList[i].Y: " << PropogatedList[i].Y << std::endl;
        std::cout << "\t PropogatedList[i].Heading: " << PropogatedList[i].Heading << std::endl;
    }
    */
    
    WeightParticle (lat, lon, PropogatedList[i] );
    
    if(highestWeight < PropogatedList[i].Weight) 
      highestWeight = PropogatedList[i].Weight;
  }
}

void ParticleFilter::WeightParticle(double lat, double lon, Particle &src) {
  double x,y,distFromGPS;
  GPS2Point(lat, lon, x, y);
  distFromGPS = sqrt( pow(src.X - x, 2) + pow(src.Y - y, 2) );
	src.Weight = exp ( -0.5 * (distFromGPS * distFromGPS) / (GPS_ERROR * GPS_ERROR) );
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
  if ( var == 0 ) return 0;
  double sum = 0;
  double stddev = sqrt ( var );
  for ( int i = 0; i < 12; ++i ) {
    sum += (rand() / (static_cast<double>(RAND_MAX) + 1.0)) * (2 * stddev) - stddev;
  }
  return 0.5 * sum;
}

/*
 * Converts a lat long pair to a cartesian point relative to the 
 * startLon, startLat.  North is positive y, East is positive X.
 */
void ParticleFilter :: GPS2Point(double lat, double lon, double& x, double& y) {

  double dist = acos( sin(startLat*D2R) * sin(lat*D2R) +
                      cos(startLat*D2R) * cos(lat*D2R) *
                      cos(lat*D2R - startLat*D2R) * EARTH_RADIUS );
  double x_angle = sin(abs(startLat - lat) * D2R) * cos(lat * D2R);
  double y_angle = cos(startLat * D2R) * sin(lat * D2R) - 
                   sin(startLat * D2R) * cos(lat * D2R) * 
                   cos(abs(startLon - lon) * D2R);
  double angle = atan2(y_angle,x_angle);
  
  x = dist * cos(angle);
  y = dist * sin(angle);
}
      
