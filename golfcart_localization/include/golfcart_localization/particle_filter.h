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
  static const double D2R = 0.0174532925; // Degrees to Rad
  static const double R2D = 57.2957795;   // Radians to Deg
  static const double EARTH_RADIUS = 6378100;     // Meters
  static const double GPS_ERROR = 1.5;            // Meters
  static const int NUM_PARTICLES = 100;

  static const int startX = 0;
  static const int startY = 0;
  
  vector<Particle> ParticleList;
  vector<Particle> PropogatedList; 
  
  double startLat;
  double startLon;
  double startHeading;
  
  double currX;
  double currY;
  double currHeading;
  
  double highestWeight; // Used for normalizing particle weights
  int lastTick;
  OdomParam odomModel;

public:

  ParticleFilter() {};
  
  void getPose(double& x, double &y, double &heading) {
    x = currX; 
    y = currY; 
    heading = currHeading; 
  }
  double getX() { return currX; }
  double getY() { return currY; }
  double getHeading() { return currHeading; }
  /*
   * Initializes the particle filter. Must be called before RunIteration().
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
   
private:

  /*
   * Propogates the src particle using the odometry model and 
   * stores the result in dst.
   */
  void PropogateParticles(
      int odomTicks,      // encoder ticks
      double lat, 
      double lon,
      double heading     // deg
      );
  /*
   * Assigns a weight to the particle based on the distance of each particle 
   * from the given gps coordinates.
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
  /*
   * Converts a lat long pair to a cartesian point relative to the 
   * startLon, startLat.  North is positive y, East is positive X.
   */
  void GPS2Point(double lat, double lon, double& x, double& y);
  /*
   * Randomly samples a zero centered normal distribution from a with
   * variance var.
   */
  double sample_normal_dist(double var);
  /*
   * Adds two angles together correcting for rollover in degrees
   */
  double addAngle(double a1, double a2);
    
};

#endif
