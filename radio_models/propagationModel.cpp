#include <cmath>
#include <iostream>
#include <vector>


// constants ..................................................................
// We mostly use UPM frog 3D.
const double TAG_LOSSES = -4.8;

// 20 * log10 ( c / (4*pi) )
const double LOSS_CONSTANT = 147.55;
// 4*pi/c
const double PHASE_CONSTANT = 4.192e-8;

// This comes from the manufacturer. Azimut
// gain list entries start at -165 degrees to 180 in steps of 15.
const double ANTENNA_LOSSES_LIST [24] = {  -25.2, -25, -20.2, -17.6, -15.6, -14, -11.2, -7.8, -5.2, -2.4, -0.6, 0, -0.6, -2.4, -5.2, -8.8, -12.2, -16.4, -19.2, -20.8, -24.4, -28.2, -24, -22.6};

// M6e RFID reader Specs
// Minimum required power to identify a tag. Also depends on other factors, but
// this value is a guide
 const double SENSITIVITY = -115; // dB

// Max transmitted power may be limited by the Region regulations
// see Power Requirements  in M6e Hardware Guide
 const double MIN_TX_POWER = -25; // dB
 const double MAX_TX_POWER = 0; // dB
 const double STEP_TX_POWER = 0.5; // dB

// These freqs ARE limited depending on the region regulations
// see Regional Frequency Quantization in M6e Hardware Guide
const double STEP_FREQ = 25e3; // Hertzs
const double MIN_FREQ_A= 865e6; //Hertzs
const double MAX_FREQ_A= 869e6; //Hertzs
const double MIN_FREQ_B= 902e6; //Hertzs
const double MAX_FREQ_B= 928e6; //Hertzs

// most likely we will use EU or NA regions....
const double MIN_FREQ_EU= 865.6e6; //Hertzs
const double MAX_FREQ_EU= 867.6e6; //Hertzs
const double STEP_FREQ_EU = 100e3; // Hertzs

const double MIN_FREQ_NA= 902e6; //Hertzs
const double MAX_FREQ_NA= 928e6; //Hertzs
const double STEP_FREQ_NA = 250e3; // Hertzs

//  ..................................................................

/**
 * returns a range vector FROM start TO stop (included) in step increments
 * @param  start min Value in vector
 * @param  stop  max Value in vector
 * @param  step  increment
 * @return       range vector
 */
std::vector<double> range(double start, double stop, double step)
{
 std::vector<double> ans;
 double currVal = start;
   while (currVal <= stop){
     ans.push_back(currVal);
     currVal += step;
   }
 return ans;
}

/**
 * returns MT_242025NRHK antenna losses in a plane
 * @param  angleRad spheric coordinate within the plane (radians)
 * @return          dB losses
 */
double antennaPlaneLoss(double angleRad ){
  double ans = -20.0;
  double angleDeg = angleRad*180.0/M_PI;

  // gain list entries start at -165 degrees to 180 in steps of 15.
  int index = ( ( (int) angleDeg ) + 165 ) / 15;

  if ( (index<24) & (index>=0) )
    ans = ANTENNA_LOSSES_LIST[index];

  return ans;
}

/**
 * my Sign function...
 * @param  x
 * @return   sign of x
 */
float sign(float x){
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return 0.0;
}

/**
 * Returns Spherical coordinates
 * @param x          cartesian coordinate x (m.)
 * @param y          cartesian coordinate y (m.)
 * @param r          spheric coordinate r (m.)
 * @param phi        relative azimut [0,2pi) (angle between XY projection and X) (radians)
 */
void getSphericCoords(double x, double y, double& r, double& phi){
  r = sqrt(x*x+y*y);
  phi = atan2(y,x);

}

/**
 * Get received power from an OMNIDIRECTIONAL tag,
 * given its relative position to antenna.
 * We assume antenna at 0,0,0, facing X coordinate.
 * See http://www.antenna-theory.com/basics/friis.php
 * Sensitivity is -85 dBm / -115 dB
 *
 * @param  tag_x       Tag x coord (m.) with respect to antenna
 * @param  tag_y       Tag y coord (m.) with respect to antenna
 * @param  freq        Transmission frequency (Hertzs)
 * @param  txtPower    Transmitted power (dB)
 * @return             Received power (dB)
 */
 double received_power_friis(double tag_x, double tag_y, double freq, double txtPower) {
     double phi;
     double r;
     double rxPower = txtPower;

     // todo: use a threshold instead of exact value
     if (!((tag_x==0) && (tag_y==0))){
         getSphericCoords(tag_x,tag_y, r, phi);
         /*
          SIMPLIFICATION!!! TAG is OMNIDIRECTIONAL
          (a.k.a. don't have tag radiation pattern and
          Here they say it's ok https://www.hindawi.com/journals/ijap/2013/194145/tab4/
         */
         double antL =  TAG_LOSSES + antennaPlaneLoss(phi);

         // propagation losses
         double propL = LOSS_CONSTANT - (20 * log10  (r * freq)) ;
         // signal goes from antenna to tag and comes back again, so we double the losses
         rxPower +=  2*antL + 2*propL ;
     }
     return rxPower;
 }

/**
 * Received signal estimated phase difference with pi ambiguity
 * @param  tag_x       Tag x coord (m.) with respect to antenna
 * @param  tag_y       Tag y coord (m.) with respect to antenna
 * @param  freq        Transmission frequency (Hertzs)
 * @return      phase difference (radians)
 */
double phaseDifference(double tag_x, double tag_y, double freq) {
  double phi;
  double r;

  getSphericCoords(tag_x,tag_y, r, phi);
  double phase = PHASE_CONSTANT * freq * r;
  phase = fmod(phase, M_PI);
  return phase;

}


/**
 * Get tag detection boundaries for given configuration
 * @param freq        RF signal frequency
 * @param txtPower    Transmitted RF power (dB.)
 * @param sensitivity Minimun power the reader can receive (dB.)
 * @param distStep    distance step for internal power calculations
 * @param minX        minimum X distance where rx power is over sensitivity
 * @param minY        minimum Y distance where rx power is over sensitivity
 * @param maxX        maximum X distance where rx power is over sensitivity
 * @param maxY        maximum Y distance where rx power is over sensitivity
 */
void activeAreaFriis(double freq, double txtPower, double sensitivity, double distStep, double& minX, double& minY, double& maxX, double& maxY) {

  double currX = 0.0;
  double currY = 0.0;
  double currRxPower = txtPower;

  std::cout << "txtPower: " << txtPower << "\n";
  std::cout << "sensitivity: " << sensitivity << "\n";

  // get max X distance within sensitivity
  do{
    currX = currX+distStep;
    currRxPower = received_power_friis(currX, currY, freq, txtPower);
  } while(currRxPower>sensitivity);
  maxX =currX-distStep; // because last iteration should be under sensitivity

  // get min X distance within sensitivity
  currX = 0.0;
  currY = 0.0;
  currRxPower = txtPower;

  do{
    currX = currX-distStep;
    currRxPower = received_power_friis(currX, currY, freq, txtPower);
  } while(currRxPower>sensitivity);
  minX =currX+distStep; // because last iteration should be under sensitivity

  // get max Y distance within sensitivity
  currX = 0.0;
  currY = 0.0;
  currRxPower = txtPower;

  do{
    currY = currY+distStep;
    currRxPower = received_power_friis(currX, currY, freq, txtPower);
  } while(currRxPower>sensitivity);
  maxY =currY-distStep; // because last iteration should be under sensitivity

  // get min Y distance within sensitivity
  currX = 0.0;
  currY = 0.0;
  currRxPower = txtPower;

  do{
    currY = currY-distStep;
    currRxPower = received_power_friis(currX, currY, freq, txtPower);
  } while(currRxPower>sensitivity);
  minY =currY+distStep; // because last iteration should be under sensitivity

}
