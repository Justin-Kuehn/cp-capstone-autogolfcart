#include "golfcart_localization/particle_filter.h" 

void ParticleFilter::Init(double startLat, double startLon, double startHeading, int odomTicks, OdomParam odomModel) {
  this->startLat = startLat;
  this->startLon = startLon;
  this->startHeading = startHeading;
  this->lastTick = odomTicks;
  this->odomModel = odomModel;
  
  // Zero All Particles
  ParticleList.resize(NUM_PARTICLES);
  ReprojList.resize(NUM_PARTICLES);
  
  for(unsigned int i = 0; i < ParticleList.size(); i++) {
    ParticleList[i].X = startX;
    ParticleList[i].Y = startY;
    ParticleList[i].Heading = startHeading;
  }
  
  CurrentState.X = startX;
  CurrentState.Y = startY;
  CurrentState.Heading = startHeading;
}

void ParticleFilter::RunIteration(int odomTicks, double lat, double lon, double heading, double deltaTime) {
  
  // Propogate Particles
  for(unsigned int i = 0; i < ParticleList.size(); i++) {
    PropogateParticle(odomTicks, heading, ParticleList[i]);
  }
  
  // Weight Particles
  
  // Resample Particles
  
  // Calculated Average
  
  
}

void ParticleFilter::PropogateParticle(int odomTicks, double heading, Particle &src) {
  
  int deltaTick = odomTicks - lastTick;
  
  // Check for encoder rollover
  if( deltaTick < 0) {
    deltaTick += odomModel.maxTicks;
  }
  
  
    
}
      
