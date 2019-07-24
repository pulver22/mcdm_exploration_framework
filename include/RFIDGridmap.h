#ifndef RFIDGRIDMAP_H
#define RFIDGRIDMAP_H


// todo clean imports ...

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/Polygon.hpp"
#include <grid_map_core/grid_map_core.hpp>

#include "grid_map_ros/GridMapRosConverter.hpp"
#include <grid_map_ros/grid_map_ros.hpp>

#include "grid_map_cv/grid_map_cv.hpp"

#include <sensor_msgs/image_encodings.h>

// Eigen
#include <Eigen/Core>

// Limits
#include <cfloat>

// Vector
#include <vector>

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace Eigen;
using namespace grid_map;

class RFIDGridmap {
public:
  // todo
  // RFIDGridmap();

  // todo
  // RFIDGridmap(int num_rows, int num_cols, double gridResolution);

  // todo
  /*
    Auxiliary functions like get numColums, numRows, origin, resolution, etc ...

   */

  /**
   * Constructor.
   * @param fileURI        inital map
   * @param mapResolution  inital map resolution (px/m.)
   * @param gridResolution RFID gridmap resolution (px/m.)
   * @param debug          show debug outputs
   */
  RFIDGridmap(std::string fileURI, double mapResolution, double gridResolution,
              bool debug);

  RFIDGridmap(nav_msgs::OccupancyGrid occGrid, double mapResolution,
              double gridResolution, bool debug);
  /**
   * Constructor without debug information
   */
  RFIDGridmap(std::string fileURI, double mapResolution, double gridResolution);

  /**
   * Store current gridmap at provided location
   * @param fileURI [description]
   */
  void saveAs(std::string fileURI);

  /**
   * Increases by 'likelyhood' the value of an elipsoid area in the gridmap.
   * Points are refered to gridmap coordinates.
   * @param likelihood     increment
   * @param antennaX       'lower' focal point F1, x coordinate (m.)
   * @param antennaY       'lower' focal point F1, y coordinate (m.)
   * @param antennaHeading angle between ellipsoid mayor axis and x axis
   * (radians)
   * @param minX           shortest dist betwen F1 and ellipse (m.)
   * @param maxX           longest dist betwen F1 and ellipse (m.)
   */
  void addEllipse(double likelihood, double antennaX, double antennaY,
                  double antennaHeading, double minX, double maxX);

  /**
   * Increases by 'likelyhood' the value of a circular area in the gridmap
   * Points are refered to gridmap coordinates.
   * @param likelihood increment
   * @param X          Circle center, x coordinate (m.)
   * @param Y          Circle center, y coordinate (m.)
   * @param R          Circle radius, (m.)
   */
  void addCircle(double likelihood, double X, double Y, double R);

  /**
   * Increases by 'likelyhood' the value of a line in the gridmap
   * Points are refered to gridmap coordinates.
   * @param likelihood [description]
   * @param X1         Starting line point, x coordinate (m.)
   * @param Y1         Starting line point, y coordinate (m.)
   * @param X2         Ending line point, x coordinate (m.)
   * @param Y2         Ending line point, y coordinate (m.)
   */
  void addLine(double likelihood, double X1, double Y1, double X2, double Y2);

  /**
   * get value of the corresponding cell of the gridmap
   * @param  i row
   * @param  j column
   * @return   stored value
   */
  double getCell(int i, int j);

  /**
   * Set value of the corresponding cell of the gridmap
   * @param  i row
   * @param  j column
   * @param   stored value
   */
  void setCell(double val, int i, int j);

  /**
   * get value of the corresponding metric Position  at the gridmap
   * @param  x cartesian coordinates (m.)
   * @param  y cartesian coordinates (m.)
   * @return   stored value
   */
  double getPosition(double x, double y);

  /**
   * set value of the corresponding metric Position  at the gridmap
   * @param  x cartesian coordinates (m.)
   * @param  y cartesian coordinates (m.)
   * @oaran   stored value
   */
  void setPosition(double val, double x, double y);

  /**
   * Get number of cols (width) in the grid
   * @return number of cols
   */
  long getNumCols();

  /**
   * Get number of rows (height) in the grid
   * @return number of rows
   */
  long getNumRows();

  // controls debug prints..
  bool debug_;

private:
  grid_map::GridMap RFIDGridmap_;
  std::string layer_name_;
  //! global frame id (for maps)
  std::string global_frame_;
  // ros format matching loaded file
  std::string format_;

  void addEllipse(grid_map::GridMap &map_, std::string layerName,
                  double likelihood, double antennaX, double antennaY,
                  double antennaHeading, double minX, double maxX, bool debug);

  void addLine(grid_map::GridMap &map_, std::string layerName,
               double likelihood, double X1, double Y1, double X2, double Y2,
               bool debug);

  void addCircle(grid_map::GridMap &map_, std::string layerName,
                 double likelihood, double X, double Y, double R, bool debug);

  void saveLayer(grid_map::GridMap map_, std::string layerName,
                 std::string fileURI, bool debug, std::string format);

  void createGrid(grid_map::GridMap &map_, std::string layerName,
                  std::string fileURI, double mapResolution,
                  double gridResolution, bool debug, std::string global_frame,
                  std::string format);

  void setPosition(grid_map::GridMap &map_, std::string layerName, double val,
                   double x, double y, bool debug);

  double getPosition(grid_map::GridMap &map_, std::string layerName, double x,
                     double y, bool debug);

  void setCell(grid_map::GridMap &map_, std::string layerName, double val,
               int i, int j, bool debug);

  double getCell(grid_map::GridMap &map_, std::string layerName, int i, int j,
                 bool debug);

  /**
   * Returns encoding in a string
   * @param  type opencv encoding id number
   * @return human friendly code
   */
  string type2str(int type);
};

#endif
