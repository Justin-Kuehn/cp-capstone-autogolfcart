/* 
 * ParticleFilter Definintion 
 */

#ifndef __PARTICLE_FILTER_H_
#define __PARTICLE_FILTER_H_

#include <vector>

using std::vector;

typedef struct ODOMETRY_t {
  double wheelCircum; // meters
  int ticksPerRev;   
  int maxTicks;       // max encoder ticks before rollover
} OdomParam;

typedef struct PARTICLE_t {
  double X;
  double Y;
  double Heading;
  double Weight;
} Particle;

class ParticleFilter {
  static const int NUM_PARTICLES = 100;
  static const int startX = 0;
  static const int startY = 0;
  
  Particle CurrentState;
  vector<Particle> ParticleList;
  vector<Particle> ReprojList; 
  
  double startLat;
  double startLon;
  double startHeading;
  
  int lastTick;
  OdomParam odomModel;

public:

  ParticleFilter() {};
  
  /*
   * Initializes
   */
  void Init(
      double startLat,     // deg
      double startLon,     // deg
      double startHeading, // deg   
      int odomTicks, 
      OdomParam odomModel // constants that define the odometry model
      );
  /*
   * Runs a single iteration of the particle filter.
   */
  void RunIteration(
      int odomTicks,      // encoder ticks
      double lat,         // deg
      double lon,         // deg
      double heading,     // deg
      double deltaTime    // sec
      );
  /*
   * Propogates the src particle using the odometry model and 
   * stores the result in dst.
   */
  void PropogateParticle(
      int odomTicks,      // encoder ticks
      double heading,     // deg
      Particle &src,      // Particle Source
      Particle &dest      // Particle Destination
      );
  /*
   * Assigns a weight to the particle based on the given gps coordinates.
   */
  void WeightParticle(
      double lat,         // deg
      double lon,         // deg
      Particle &p         // Particle to weigh
      );
  /*
   * Resamples all particles based on each particles weight
   */
  void ResampleParticles();
  
    
};

#endif
