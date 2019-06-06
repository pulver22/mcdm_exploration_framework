#ifndef MAP_H
#define MAP_H

#include "RFIDGridmap.h"
#include "geometry_msgs/PoseStamped.h"
#include "pose.h"
#include <fstream> // ifstream
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream> // stringstream
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;
namespace dummy {

class Map {

public:
  typedef enum CellValue { FREE = 0, OBST = 1, VIST = 2 } CellValue;

  /**
  * Default constructor
  */
  Map();

  /**
  * Creates Map using provided file
  * @param infile     Map file (binary pgm)
  * @param resolution Map resolution [m./cell]
  */
  Map(std::ifstream &infile, float resolution);

  Map(float plan_resolution, nav_msgs::OccupancyGrid occupancyGrid);
  /**
  *  Creates Map using provided data vector and parameters
  * @param plan_resolution     planning grid resolution [m./cell] (should be
  * greater than nav resolution)
  * @param map_resolution      map, nav and rfid grid resolution [m./cell]
  * @param width          data vector width (== numRows)
  * @param height         data vector height (== numCols)
  * @param data           data vector containing map data
  * @param origin         Where to place 0,0 in map frame_id.
  */
  Map(float plan_resolution, float map_resolution, int width, int height,
      vector<int> data, geometry_msgs::Pose origin);

  /**
  * Set a navigation grid cell value
  * @param  i row index
  * @param  j col index
  * @param  value  navigation grid value
  */
  void setGridValue(Map::CellValue value, long i, long j);

  /**
   * Set a navigation grid cell value
   * @param  ps point in "map" frame_id
   * @param  value  navigation grid value
   */
  void setGridValue(Map::CellValue value, geometry_msgs::PoseStamped ps);

  /**
   * Set a navigation grid cell value
   * @param  i ith element (as vector)
   * @param  value  navigation grid value
   */
  void setGridValue(Map::CellValue value, long i);

  /**
   * Return navigation grid cell value
   * @param  i row index
   * @param  j col index
   * @return   navigation grid value
   */
  Map::CellValue getGridValue(long i, long j) const;

  /**
   * Return navigation grid cell value
   * @param  i ith element (as vector)
   * @return   navigation grid value
   */
  Map::CellValue getGridValue(long i) const;

  Map::CellValue getGridValue(geometry_msgs::PoseStamped ps) const;

  /**
   * Return map grid cell value
   * @param  i row index
   * @param  j col index
   * @return   map grid value
   */
  Map::CellValue getMapValue(long i, long j);

  Map::CellValue getMapValue(long i) const;

  Map::CellValue getMapValue(geometry_msgs::PoseStamped ps) const;

  /**
   * Get number of Rows (height) in navigation grid
   * @return number of rows
   */
  long getNumGridRows() const;

  /**
   * Get number of Cols (width) in navigation grid
   * @return number of cols
   */
  long getNumGridCols() const;

  /**
   * Get number of Rows (height) in map grid
   * @return number of rows
   */
  long getNumRows();

  /**
   * Get number of Cols (width) in map grid
   * @return number of cols
   */
  long getNumCols();

  // TODO: not implemented
  // void findFreeEdges(int cX, int cY);

  /**
  * TODO:Meaningful description here
  * @param x [description]
  * @param y [description]
  */
  void addEdgePoint(int x, int y);

  /**
  * TODO:Meaningful description here
  */
  std::vector<vector<long>> getMap2D();

  /**
   * Returns robot pose
   * @return last stored robot pose
   */
  Pose getRobotPosition();

  /**
   * Stores current robot pose
   * @param p robot pose
   */
  void setCurrentPose(Pose &p);

  /**
   * Number of cells in navigation grid that are not == 1
   * @return number of cells with val != 1
   */
  long getTotalFreeCells();

  /**
   * Stores navigation grid map in temporary binary file at
   * /tmp/result.pgm
   */
  void drawVisitedCells();

  /**
   * Stores navigation grid map in temporary binary file at specified location
   * @param fileURI full path and filename for file
   */
  void storeBinary(std::string fileURI);

  /**
   * Stores navigation grid map in temporary acii file (only 0 indexes) at
   * specified location
   * @param fileURI full path and filename for file
   */
  void storeAscii(std::string fileURI);

  /**
 * Stores visited cells in history at file /tmp/finalResult.txt
 * @param history list of strings describing past robot poses
 */
  void printVisitedCells(vector<string> &history);

  /**
   * Return path planning grid cell value
   * @param  i row index
   * @param  j col index
   * @return   path planning grid value
   */
  Map::CellValue getPathPlanningGridValue(long i, long j) const;

  /**
   * Return path planning grid cell value at given position
   * @param  ps point in "map" frame_id
   * @return   path planning grid value
   */
  Map::CellValue getPathPlanningGridValue(geometry_msgs::PoseStamped ps) const;

  /**
   * Return path planning grid cell value at given position
   * @param  i ith element (as vector)
   * @return   path planning grid value
   */
  Map::CellValue getPathPlanningGridValue(long i) const;

  /**
   * Set a path planning grid cell value
   * @param  i row index
   * @param  j col index
   * @param  value  path planning grid value
   */
  void setPathPlanningGridValue(Map::CellValue value, int i, int j);

  /**
  * Set a path planning grid cell value
  * @param  ps point in "map" frame_id
  * @param  value  path planning grid value
  */
  void setPathPlanningGridValue(Map::CellValue value,
                                geometry_msgs::PoseStamped ps);

  /**
  * Set a path planning grid cell value
  * @param  i ith element (as vector)
  * @param  value  path planning grid value
  */
  void setPathPlanningGridValue(Map::CellValue value, long i);

  /**
   * Get number of Cols (width) in path planning grid
   * @return number of cols
   */
  int getPathPlanningNumCols() const;

  /**
   * Get number of rows (height) in path planning grid
   * @return number of rows
   */
  int getPathPlanningNumRows() const;

  /**
   * Return planning_grid to nav_grid resolution ratio
   * @return ratio
   */
  float getGridToPathGridScale() const;

  /**
  * Iterates over pathplanning-grid in a square and updates according to values
  * in navigation grid
  * @param cellX          Center of the update box, in pathplanning grid cells,
  * x coord
  * @param cellY          Center of the update box, in pathplanning grid cells,
  * y coord
  * @param rangeInCells   Half length of the square (radius?) in pathplanning
  * grid cells
  * @param power          Unused...
  */
  void updatePathPlanningGrid(int cellX, int cellY, int rangeInCells,
                              double power);

  /**
  * Iterates over pathplanning-grid in a square and updates according to values
  * in navigation grid
  * @param posX          Center of the update box, x coord
  * @param posY          Center of the update box, y coord
  * @param rangeInMeters Half length of the square (radius?) in meters
  */
  void updatePathPlanningGrid(float posX, float posY, float rangeInMeters);

  /**
  * @brief Map::getRelativeTagCoord calculate the relative position of the RFID
  * tag wrt the antenna
  * @param absTagX: absolute x-position of the RFID tag
  * @param absTagY: absolute x-position of the RFID tag
  * @param antennaX: absolute x-position of the antenna
  * @param antennaY: absolute y-position of the antenna
  * @return a pair containing the relative position of the tag to the antenna
  */
  std::pair<int, int> getRelativeTagCoord(int absTagX, int absTagY,
                                          int antennaX, int antennaY);

  /**
  * update the grid cell summing the current value with a new reading
  * @param power: the sensed power by the antenna
  * @param i: the x-position in the grid
  * @param j: the y-position in the grid
  */
  void setRFIDGridValue(float power, int i, int j);

  /**
  * update the grid cell summing the current value with a new reading
  * @param  ps point in "map" frame_id
  * @param  value  the sensed power by the antenna
  */
  void setRFIDGridValue(float value, geometry_msgs::PoseStamped ps);

  /**
  * update the grid cell summing the current value with a new reading
  * @param  i ith element (as vector)
  * @param  value  the sensed power by the antenna
  */
  void setRFIDGridValue(float value, long i);
  /**
   * Stores rfid grid map in temporary binary file at /tmp/rfid_result.pgm
   * also saturates values to be between 0 and 255.
   * Like drawRFIDGridScan(RFIDGridmap grid) but using internal rfid structure
   */
  void drawRFIDScan();

  /**
  * Save on disk an image representing the scanned environment.
  * Lighter zone represents higher probability for the presence of the RFID tag
  * file is save at /tmp/rfid_result_gridmap.pgm
  *
  * @param grid RFIDGridmap containing the data
  */
  void drawRFIDGridScan(RFIDGridmap grid);

  /**
   * Return RFID grid cell value
   * @param  i row index
   * @param  j col index
   * @return   RFID grid value
   */
  float getRFIDGridValue(long i, long j) const;

  /**
   * Return RFID grid cell value
   * @param  i ith element (as vector)
   * @return   RFID grid value
   */
  float getRFIDGridValue(long i) const;

  /**
   * Return RFID grid cell value
   * @param  ps point in "map" frame_id
   * @return   RFID grid value
   */
  float getRFIDGridValue(geometry_msgs::PoseStamped ps) const;

  /**
   * Get number of Cols (width) in  RFID grid cell
   * @return number of cols
   */
  long getRFIDGridNumCols();

  /**
  * Get number of rows (height) in  RFID grid cell
  * @return number of rows
  */
  long getRFIDGridNumRows();

  // TODO: not implemented...
  // void updateRFIDGrid(double power, double phase, int antennaX, int
  // antennaY);

  /**
   * Returns tag pose in rfid grid cell indexs
   * Like drawRFIDGridScan(RFIDGridmap grid) but using internal rfid structure
   * @return [row,col] in rfid grid coordinates
   */
  std::pair<int, int> findTag();

  /**
   * Returns tag pose in rfid grid cell indexs
   * @param grid RFIDGridmap containing the data
   */
  std::pair<int, int> findTagfromGridMap(RFIDGridmap grid);

  /**
  * Returns the i,j index from pathplanning grid of the given metric position
  * (in "map" frame id)
  * @param  x  metric position x coordinate (in "map" frame id) in m.
  * @param  y  metric position y coordinate (in "map" frame id) in m.
  * @param  i  corresponding cell row index
  * @param  j  corresponding cell col index
  * @return    True if index was retrieved
  */
  bool getPathPlanningIndex(double x, double y, long &i, long &j);

  /**
  * Returns the metric position (in "map" frame id) from pathplanning  grid of
  * the given i,j index
  * @param  x  corresponding metric position x coordinate (in "map" frame id) in
  * m.
  * @param  y  corresponding metric position y coordinate (in "map" frame id) in
  * m.
  * @param  i  cell row index
  * @param  j  cell col index
  * @return    True if position was retrieved
  */
  bool getPathPlanningPosition(double &x, double &y, long i, long j);

  /**
  * Returns the metric position (in "map" frame id) from any grid of the given
  * i,j index
  * @param  x  corresponding metric position x coordinate (in "map" frame id) in
  * m.
  * @param  y  corresponding metric position y coordinate (in "map" frame id) in
  * m.
  * @param  i  cell row index
  * @param  j  cell col index
  * @return    True if position was retrieved
  */
  bool getPosition(double &x, double &y, long i, long j, grid_map::GridMap *gm);

  /**
   * Returns the i,j index from navigation grid of the given metric position (in
   * "map" frame id)
   * @param  x  metric position x coordinate (in "map" frame id) in m.
   * @param  y  metric position y coordinate (in "map" frame id) in m.
   * @param  i  corresponding cell row index
   * @param  j  corresponding cell col index
   * @return    True if index was retrieved
   */
  bool getGridIndex(double x, double y, long &i, long &j);

  /**
   * Returns the metric position (in "map" frame id) from navigation grid of the
   * given i,j index
   * @param  x  corresponding metric position x coordinate (in "map" frame id)
   * in m.
   * @param  y  corresponding metric position y coordinate (in "map" frame id)
   * in m.
   * @param  i  cell row index
   * @param  j  cell col index
   * @return    True if position was retrieved
   */
  bool getGridPosition(double &x, double &y, long i, long j);

  /**
   * Returns the i,j index from map grid of the given metric position (in "map"
   * frame id)
   * @param  x  metric position x coordinate (in "map" frame id) in m.
   * @param  y  metric position y coordinate (in "map" frame id) in m.
   * @param  i  corresponding cell row index
   * @param  j  corresponding cell col index
   * @return    True if index was retrieved
   */
  bool getMapIndex(double x, double y, long &i, long &j);

  /**
   * Returns the metric position (in "map" frame id) from map grid of the given
   * i,j index
   * @param  x  corresponding metric position x coordinate (in "map" frame id)
   * in m.
   * @param  y  corresponding metric position y coordinate (in "map" frame id)
   * in m.
   * @param  i  cell row index
   * @param  j  cell col index
   * @return    True if position was retrieved
   */
  bool getMapPosition(double &x, double &y, long i, long j);

  /**
   * Returns the metric position (in "map" frame id) from RFID grid of the given
   * i,j index
   * @param  x  corresponding metric position x coordinate (in "map" frame id)
   * in m.
   * @param  y  corresponding metric position y coordinate (in "map" frame id)
   * in m.
   * @param  i  cell row index
   * @param  j  cell col index
   * @return    True if position was retrieved
   */
  bool getRFIDPosition(double &x, double &y, long i, long j);

  /**
   * Returns the i,j index from RFID grid of the given metric position (in "map"
   * frame id)
   * @param  x  metric position x coordinate (in "map" frame id) in m.
   * @param  y  metric position y coordinate (in "map" frame id) in m.
   * @param  i  corresponding cell row index
   * @param  j  corresponding cell col index
   * @return    True if index was retrieved
   */
  bool getRFIDIndex(double x, double y, long &i, long &j);

  void plotPathPlanningGridColor(std::string fileURI);

  void plotGridColor(std::string fileURI);

  // ...........................................................................
  // following methods were at newray.cpp ......................................
  // ...........................................................................

  /**
   * Check if a path planning cell is candidate: is next to an empty cell
   * @param  i   cell row index in pathplanning grid
   * @param  j   cell col index in pathplanning grid
   * @return     1 == cell is adjacent to at least one free cell, 0 otherwise
   */
  int isCandidate(long i, long j);

  int planning_iterate_func(grid_map::SubmapIterator iterator);

  /**
   * Like isCandidate, but inside each of the surrounding path planning cells,
   *       checks if any of the corresponding nav cells are empty
   * @param  i   cell row index in pathplanning grid
   * @param  j   cell col index in pathplanning grid
   * @return     1 == cell is adjacent to at least one free nav cell, 0
   * otherwise
   */
  int isCandidate2(long i, long j);

  int nav_iterate_func(grid_map::SubmapIterator iterator);

  int isCandidate_inner(long i, long j, int mode);

  void findCandidatePositions_inner(int mode, double pos_X_m, double pos_Y_m,
                                    double heading_rad, double FOV_rad,
                                    double range_m);

  void calculateInfoGainSensingTime(double pos_X_m, double pos_Y_m,
                                    double heading_rad, double FOV_rad,
                                    double range_m);

  /**
   * Given a starting point and heading, updates candidatePositions.
   * Candidate positions  are:
   *        -  closer to starter point than range
   *        -  within a given FOV from starting heading.
   *        -  free from obstacles between them and starting point
   *        -  "Candidate" as in function isCandidate.
   * @param  pos_X_m      metric position x coordinate (in "map" frame id) in m.
   * @param  pos_Y_m      metric position y coordinate (in "map" frame id) in m.
   * @param  heading_rad  metric orientation coordinate (in "map" frame id) in
   * radians
   * @param  FOV_rad      Field of View from current heading (Arc width in
   * radians)
   * @param  range_m      Max. distance to consider in m.
   */
  void findCandidatePositions(double pos_X_m, double pos_Y_m,
                              double heading_rad, double FOV_rad,
                              double range_m);

  /**
   * This function is exactly as findCandidatePositions but using Candidate2
   * Given a starting point and heading, updates candidatePositions.
   * Candidate positions  are:
   *        -  closer to starter point than range
   *        -  within a given FOV from starting heading.
   *        -  free from obstacles between them and starting point
   *        -  "Candidate2" as in function isCandidate.
   * @param  pos_X_m      metric position x coordinate (in "map" frame id) in m.
   * @param  pos_Y_m      metric position y coordinate (in "map" frame id) in m.
   * @param  heading_rad  metric orientation coordinate (in "map" frame id) in
   * radians
   * @param  FOV_rad      Field of View from current heading (Arc width in
   * radians)
   * @param  range_m      Max. distance to consider in m.
   */
  void findCandidatePositions2(double pos_X_m, double pos_Y_m,
                               double heading_rad, double FOV_rad,
                               double range_m);

  /**
   * Returns candidate positions: path planning cell indexes list built with
   * findCandidatePositions or  findCandidatePositions2
   */
  vector<std::pair<float, float>> getCandidatePositions();

  /**
   * Clears candidate positions (built with findCandidatePositions or
   * findCandidatePositions2)
   */
  void emptyCandidatePositions();

  /**
   * Calculate the sensing time of a possible scanning operation, at nav grid
   * level.
   * @param  pos_X_m      metric position x coordinate (in "map" frame id) in m.
   * @param  pos_Y_m      metric position y coordinate (in "map" frame id) in m.
   * @param  heading_rad  metric orientation coordinate (in "map" frame id) in
   * radians
   * @param  FOV_rad      Field of View from current heading (Arc width in
   * radians)
   * @param  range_m      Max. distance to consider in m.
   * @return pair         Minimum FOV required to scan all the free cells from
   * the considered pose
   */
  std::pair<double, double> getSensingTime(double pos_X_m, double pos_Y_m,
                                           double heading_rad, double FOV_rad,
                                           double range_m);

  /**
   * Perform the sensing operation by setting the value of the free cell scanned
   * to 2
   * @param  pos_X_m      metric position x coordinate (in "map" frame id) in m.
   * @param  pos_Y_m      metric position y coordinate (in "map" frame id) in m.
   * @param  heading_rad  metric orientation coordinate (in "map" frame id) in
   * radians
   * @param  FOV_rad      Field of View from current heading (Arc width in
   * radians) [UNUSED]
   * @param  range_m      Max. distance to consider in m.
   * @param  minAngle_rad Min angle from current heading (in radians) to
   * consider in scan
   * @param  maxAngle_rad Max angle from current heading (in radians) to
   * consider in scan
   * @return int          Number of modified cells at nav grid.
   */
  int performSensingOperation(double pos_X_m, double pos_Y_m,
                              double heading_rad, double FOV_rad,
                              double range_m, double minAngle_rad,
                              double maxAngle_rad);

  /**
   * Returns number of free cells in scanning area at nav grid
   * @param  pos_X_m      metric position x coordinate (in "map" frame id) in m.
   * @param  pos_Y_m      metric position y coordinate (in "map" frame id) in m.
   * @param  heading_rad  metric orientation coordinate (in "map" frame id) in
   * radians
   * @param  FOV_rad      Field of View from current heading (Arc width in
   * radians)
   * @param  range_m      Max. distance to consider in m.
   * @return int          Number of free cells at nav grid area.
   */
  int getInformationGain(double pos_X_m, double pos_Y_m, double heading_rad,
                         double FOV_rad, double range_m);
  // ...........................................................................
  // End of methos previously at newray.cpp ....................................
  // ...........................................................................

  bool isGridValueObst(grid_map::Index ind);

  bool isGridValueVist(grid_map::Index ind);

  bool isGridValueFree(grid_map::Index ind);

  bool isPathPlanningGridValueFree(grid_map::Index ind);

  bool isPathPlanningGridValueObst(grid_map::Index ind);

  grid_map_msgs::GridMap toMessagePathPlanning();

  grid_map_msgs::GridMap toMessageGrid();

  void countPathPlanningCells(long *n_obsts, long *n_free, long *n_vist,
                              long *n_others);
  void countGridCells(long *n_obsts, long *n_free, long *n_vist,
                      long *n_others);

protected:
  void createMap(nav_msgs::OccupancyGrid occupancyGrid);

  /**
  * creates map grid from input stream
  * @param infile input stream
  */
  void createMap(std::ifstream &infile);

  /**
  * creates map grid from opencv matrix
  * @param imageCV        opencv mat
  * @param origin         position for grid center
  * @param map_frame_id   map frame id (usually "map")
  * @param map_resolution map resolution in m./cell (usually 0.1)
  */
  void createMap(cv::Mat imageCV, geometry_msgs::Pose origin,
                 std::string map_frame_id, double map_resolution);

  /**
  * creates map grid from data vector
  * @param width          data vector width (== numRows)
  * @param height         data vector height (== numCols)
  * @param resolution     data vector height data resolution
  * @param data           data vector containing map data
  * @param origin         position for grid center*
  */
  void createMap(int width, int height, double resolution, vector<int> data,
                 geometry_msgs::Pose origin);

  /**
  * Creates an  opencv matrix from data vector
  * @param width          data vector width (== numRows)
  * @param height         data vector height (== numCols)
  * @param data           data vector containing map data
  * @return              opencv matrix with data vector
  */
  cv::Mat MreadImage(int width, int height, vector<int> data);

  /**
  * Creates an  opencv matrix from  input stream
  * @param infile input stream
  */
  cv::Mat MreadImage(std::ifstream &input);

  /**
   * creates navigation grid with given resolution (a rescalling of map grid)
   * @param resolution nav_grid resolution (m./cell)
   */
  void createGrid(float resolution);

  /**
   * Creates path planning grid with given resolution (a rescalling of map grid)
   * This should be bigger than nav_grid resolution
   * @param resolution planning_grid resolution (m./cell)
   */
  void createPathPlanningGrid(float resolution);

  /**
   * Creates RFID grid with given resolution (a rescalling of map grid)
   * @param resolution RFID grid resolution (m./cell)
   */
  void createRFIDGrid(float resolution);
  /**
  * Stores navigation grid map in temporary (ascii/binary) files at
  * /tmp/test.pgm
  * /tmp/freeCell.txt
  */
  void createNewMap();

  /**
   * Reduces free cells count in navigation grid by 1
   * Free means val!=1
   */
  void decreaseFreeCells();

  // number of rows of the map grid
  long numRows;

  // number of cols of the map grid
  long numCols;

  // number of rows of the path planning grid
  int numPathPlanningGridRows;

  // number of cols of the  path planning grid
  int numPathPlanningGridCols;

  // Number of cells in navigation grid that are not == 1
  long totalFreeCells;

  // last stored robot pose
  Pose currentPose;

  // GRIDS! Each one has just 1 layer, named "layer"
  // "map"      original input map
  grid_map::GridMap map_grid_;
  // "nav"	    reescaled input map for navigation
  grid_map::GridMap nav_grid_;
  // "planning" reescaled input map for path planning
  grid_map::GridMap planning_grid_;
  // "RFID"     reescaled input map for sensors
  grid_map::GridMap rfid_grid_;

  // planning_grid to nav_grid resolution ratio
  float gridToPathGridScale;

  // number of rows of the navigation grid
  long numGridRows;

  // number of cols of the navigation grid
  long numGridCols;

private:
  // TODO: meaningful description here
  std::vector<std::pair<float, float>> edgePoints;

  /**
   * Returns the i,j index from grid of the given metric position (in "map"
   * frame id)
   * @param  x  metric position x coordinate (in "map" frame id) in m.
   * @param  y  metric position y coordinate (in "map" frame id) in m.
   * @param  i  corresponding cell row index
   * @param  j  corresponding cell col index
   * @param  gm grid map where we operate
   * @return    True if index was retrieved
   */
  bool getIndex(double x, double y, long &i, long &j, grid_map::GridMap *gm);

  /**
  * Get a grid cell value
  * @param  i row index
  * @param  j col index
  * @param  gm  grid_map to retrieve
  * @return     grid value at indexes
  */
  float getValue(long i, long j, const grid_map::GridMap *gm) const;

  /**
  * Get a grid cell value
  * @param  ps  Pose stamped in "map" frame_id
  * @param  gm  grid_map to retrieve
  * @return     grid value at pose stamped
  */
  float getValue(geometry_msgs::PoseStamped ps,
                 const grid_map::GridMap *gm) const;

  /**
  * Get a grid cell value
  * @param  i ith element (as vector)
  * @return   navigation grid value
  */
  float getValue(long i, const grid_map::GridMap *gm) const;

  /**
  * Set a grid cell value
  * @param  value value to store
  * @param  i row index
  * @param  j col index
  * @param  gm  grid_map to retrieve
  */
  void setValue(float value, long i, long j, grid_map::GridMap *gm);

  /**
  * Set a grid cell value
  * @param  value value to store
  * @param  ps  Pose stamped in "map" frame_id
  * @param  gm  grid_map to retrieve
  */
  void setValue(float value, geometry_msgs::PoseStamped ps,
                grid_map::GridMap *gm);

  /**
  * Set a grid cell value
  * @param  value value to store
  * @param  i ith element (as vector)
  */
  void setValue(float value, long i, grid_map::GridMap *gm);

  /**
   * Get number of Cols (width) in grid_map
   * @param  gm  grid_map to retrieve
   * @return number of cols
   */
  int getGridNumCols(const grid_map::GridMap *gm) const;

  /**
  * Get number of rows (height) in grid_map
  * @param  gm  grid_map to retrieve
  * @return number of rows
  */
  int getGridNumRows(const grid_map::GridMap *gm) const;

  // aux function ...
  string type2str(int type);

  /**
   * Prints grid boundaries and point pose
   * @param point point to print
   * @param gm    grid_map
   */
  void printErrorReason(grid_map::Position point,
                        const grid_map::GridMap *gm) const;

  void printScannedCells();

  void printGridData(std::string grid_name, const grid_map::GridMap *gm);

  cv::Mat binarizeImage(cv::Mat imageCV);

  void plotMyGrid(std::string fileURI, const grid_map::GridMap *gm);

  void plotMyGridColor(std::string fileURI, const grid_map::GridMap *gm);

  grid_map_msgs::GridMap toMessage(grid_map::GridMap *gm, bool full);

  /**
   * constraints angle into -pi,pi range
   * @param  x angle in radians
   * @return   -pi,pi wrapped angle
   */
  double constrainAnglePI(double x);

  Map::CellValue toCellValue(float floatVal, std::string who) const;
  float toFloat(Map::CellValue value) const;
  void encodeGrid(grid_map::GridMap *gm, int obstValue, int freeValue);
  void countCells(long *n_obsts, long *n_free, long *n_vist, long *n_others,
                  const grid_map::GridMap *gm);
  void printSubmapBoundaries(grid_map::Index startIndex,
                             grid_map::Index bufferSize,
                             const grid_map::GridMap *gm) const;

  // percentage of nav cells inside a planning cell that need to be visited
  // before marking the planning cell as visited.
  float PLANNING_VISIT_RATIO = 0.6;
};
}

#endif
