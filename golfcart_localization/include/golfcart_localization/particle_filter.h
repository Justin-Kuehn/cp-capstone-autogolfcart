/* 
 * ParticleFilter Definintion 
 */

#ifndef __PARTICLE_FILTER_H_
#define __PARTICLE_FILTER_H_

typedef struct PARTICLE_t {
  double X;
  double Y;
  double Heading;
} Particle;

class ParticleFilter {
  static const int NUM_PARTICLES = 100;
  /*
  Particle CurrentState;
  Particle[NUM_PARTICLES] ParticleList;
  Particle[NUM_PARTICLES] ReprojList;
  */

public:

  ParticleFilter() {};

  void Init(
      double startLat, 
      double startLon, 
      double startHeading, 
      int max_ticks);
/*   
  void RunIteration(
      int odomTicks, 
      double lat, 
      double lon, 
      double heading, 
      double deltaTime);
*/
    
};

#endif
