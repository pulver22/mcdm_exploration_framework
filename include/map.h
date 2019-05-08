#ifndef MAP_H
#define MAP_H

#include <fstream> // ifstream
#include <sstream> // stringstream
#include <vector>
#include <string>
#include <iostream>
#include "pose.h"
#include <unordered_map>
#include <nav_msgs/OccupancyGrid.h>
#include "geometry_msgs/PoseStamped.h"
#include "RFIDGridmap.h"


using namespace std;
namespace dummy{

class Map
{

public:


   /**
   * Default constructor
   */
  Map();

   /**
   * Creates Map using provided file
   * @param infile     Map file (binary pgm)
   * @param resolution Map resolution [m./cell]
   */
  Map(std::ifstream& infile, float resolution);

   /**
   *  Creates Map using provided data vector and parameters
   * @param resolution     navigation grid resolution [m./cell]
   * @param costresolution path planning grid resolution [m./cell] (should be greater than nav resolution)
   * @param width          data vector width (== numRows)
   * @param height         data vector height (== numCols)
   * @param data           data vector containing map data
   * @param origin         Where to place 0,0 in map frame_id.
   */
  Map(float resolution, float costresolution, int width, int height, vector< int > data, geometry_msgs::Pose origin);

   /**
   * Set a navigation grid cell value
   * @param  i row index
   * @param  j col index
   * @param  value  navigation grid value
   */
  void setGridValue(int value, long i, long j);

  /**
   * Set a navigation grid cell value
   * @param  ps point in "map" frame_id
   * @param  value  navigation grid value
   */
  void setGridValue(int value, geometry_msgs::PoseStamped ps);

  /**
   * Set a navigation grid cell value
   * @param  i ith element (as vector)
   * @param  value  navigation grid value
   */
  void setGridValue(int value, long i);

  /**
   * Return navigation grid cell value
   * @param  i row index
   * @param  j col index
   * @return   navigation grid value
   */
  int getGridValue(long i, long j) const;

  /**
   * Return navigation grid cell value
   * @param  i ith element (as vector)
   * @return   navigation grid value
   */
  int getGridValue(long i) const;

  int getGridValue(geometry_msgs::PoseStamped ps) const;

  /**
   * Return map grid cell value
   * @param  i row index
   * @param  j col index
   * @return   map grid value
   */
  int getMapValue(long i, long j);

  int getMapValue(long i) const;

  int getMapValue(geometry_msgs::PoseStamped ps) const;

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

  //TODO: not implemented
  //void findFreeEdges(int cX, int cY);

 /**
 * TODO:Meaningful description here
 * @param x [description]
 * @param y [description]
 */
  void addEdgePoint(int x, int y);

  /**
  * TODO:Meaningful description here
  */
  std::vector<vector<long> > getMap2D();

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
   * Stores navigation grid map in temporary acii file (only 0 indexes) at specified location
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
  int getPathPlanningGridValue(long i,long j) const;

  /**
   * Return path planning grid cell value at given position
   * @param  ps point in "map" frame_id
   * @return   path planning grid value
   */
  int getPathPlanningGridValue(geometry_msgs::PoseStamped ps) const;

  /**
   * Return path planning grid cell value at given position
   * @param  i ith element (as vector)
   * @return   path planning grid value
   */
  int getPathPlanningGridValue(long i) const;

  /**
   * Set a path planning grid cell value
   * @param  i row index
   * @param  j col index
   * @param  value  path planning grid value
   */
  void setPathPlanningGridValue(int value, int i, int j);

  /**
  * Set a path planning grid cell value
  * @param  ps point in "map" frame_id
  * @param  value  path planning grid value
  */
  void setPathPlanningGridValue(int value, geometry_msgs::PoseStamped ps);

  /**
  * Set a path planning grid cell value
  * @param  i ith element (as vector)
  * @param  value  path planning grid value
  */
  void setPathPlanningGridValue(int value, long i);

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
  int getGridToPathGridScale() const;

  /**
  * Iterates over pathplanning-grid in a square and updates according to values in navigation grid
  * @param cellX          Center of the update box, in pathplanning grid cells, x coord
  * @param cellY          Center of the update box, in pathplanning grid cells, y coord
  * @param rangeInCells   Half length of the square (radius?) in pathplanning grid cells
  * @param power          Unused...
  */
  void updatePathPlanningGrid(int cellX, int cellY, int rangeInCells, double power);

  /**
  * @brief Map::getRelativeTagCoord calculate the relative position of the RFID tag wrt the antenna
  * @param absTagX: absolute x-position of the RFID tag
  * @param absTagY: absolute x-position of the RFID tag
  * @param antennaX: absolute x-position of the antenna
  * @param antennaY: absolute y-position of the antenna
  * @return a pair containing the relative position of the tag to the antenna
  */
  std::pair<int, int> getRelativeTagCoord(int absTagX, int absTagY, int antennaX, int antennaY);

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
  int getRFIDGridValue(long i,long j) const;


  /**
   * Return RFID grid cell value
   * @param  i ith element (as vector)
   * @return   RFID grid value
   */
  int getRFIDGridValue(long i) const;

  /**
   * Return RFID grid cell value
   * @param  ps point in "map" frame_id
   * @return   RFID grid value
   */
  int getRFIDGridValue(geometry_msgs::PoseStamped ps) const;

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

  //TODO: not implemented...
  //void updateRFIDGrid(double power, double phase, int antennaX, int antennaY);

  /**
   * Returns tag pose in rfid grid cell indexs
   * Like drawRFIDGridScan(RFIDGridmap grid) but using internal rfid structure
   * @return [row,col] in rfid grid coordinates
   */
  std::pair<int,int> findTag();

  /**
   * Returns tag pose in rfid grid cell indexs
   * @param grid RFIDGridmap containing the data
   */
  std::pair<int,int> findTagfromGridMap(RFIDGridmap grid);

protected:

  /**
  * creates map grid from input stream
  * @param infile input stream
  */
  void createMap(std::ifstream& infile);

  /**
  * creates map grid from opencv matrix
  * @param imageCV        opencv mat
  * @param origin         position for grid center
  * @param map_frame_id   map frame id (usually "map")
  * @param map_resolution map resolution in m./cell (usually 0.1)
  */
  void createMap(cv::Mat imageCV,geometry_msgs::Pose origin, std::string map_frame_id, double map_resolution);

  /**
  * creates map grid from data vector
  * @param width          data vector width (== numRows)
  * @param height         data vector height (== numCols)
  * @param resolution     data vector height data resolution
  * @param data           data vector containing map data
  * @param origin         position for grid center*
  */
  void createMap(int width,int height, double resolution, vector<int> data, geometry_msgs::Pose origin);

  /**
  * Creates an  opencv matrix from data vector
  * @param width          data vector width (== numRows)
  * @param height         data vector height (== numCols)
  * @param data           data vector containing map data
  * @return              opencv matrix with data vector
  */
  cv::Mat MreadImage(int width,int height,vector<int> data);

  /**
  * Creates an  opencv matrix from  input stream
  * @param infile input stream
  */
  cv::Mat MreadImage(std::ifstream& input);

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

  // TODO: meaningful description here
  std::vector<std::pair<int, int> > edgePoints;

  // Number of cells in navigation grid that are not == 1
  long totalFreeCells;

  // last stored robot pose
  Pose currentPose;

  // GRIDS! Each one has just 1 layer, named "layer"
  // "map"      original input map
  grid_map::GridMap  map_grid_;
  // "nav"	    reescaled input map for navigation
  grid_map::GridMap  nav_grid_;
  // "planning" reescaled input map for path planning
  grid_map::GridMap  planning_grid_;
  // "RFID"     reescaled input map for sensors
  grid_map::GridMap  rfid_grid_;

  // planning_grid to nav_grid resolution ratio
  int gridToPathGridScale;

  // number of rows of the navigation grid
  long numGridRows;

  // number of cols of the navigation grid
  long numGridCols;

private:
  /**
  * Get a grid cell value
  * @param  i row index
  * @param  j col index
  * @param  gm  grid_map to retrieve
  * @return     grid value at indexes
  */
  int  getValue(long i,long j, grid_map::GridMap gm) const;

  /**
  * Get a grid cell value
  * @param  ps  Pose stamped in "map" frame_id
  * @param  gm  grid_map to retrieve
  * @return     grid value at pose stamped
  */
  int  getValue(geometry_msgs::PoseStamped ps, grid_map::GridMap gm) const;

  /**
  * Get a grid cell value
  * @param  i ith element (as vector)
  * @return   navigation grid value
  */
  int getValue(long i, grid_map::GridMap gm) const;

  /**
  * Set a grid cell value
  * @param  value value to store
  * @param  i row index
  * @param  j col index
  * @param  gm  grid_map to retrieve
  */
  void setValue(int value, long i,long j, grid_map::GridMap gm) const;

  /**
  * Set a grid cell value
  * @param  value value to store
  * @param  ps  Pose stamped in "map" frame_id
  * @param  gm  grid_map to retrieve
  */
  void setValue(int value, geometry_msgs::PoseStamped ps, grid_map::GridMap gm) const;

  /**
  * Set a grid cell value
  * @param  value value to store
  * @param  i ith element (as vector)
  */
  void setValue(int value, long i, grid_map::GridMap gm) const;

  /**
   * Get number of Cols (width) in grid_map
   * @param  gm  grid_map to retrieve
   * @return number of cols
   */
  int getGridNumCols(grid_map::GridMap gm) const;

  /**
  * Get number of rows (height) in grid_map
  * @param  gm  grid_map to retrieve
  * @return number of rows
  */
  int getGridNumRows(grid_map::GridMap gm) const;

  // aux function ...
  string type2str(int type) ;

  /**
   * Prints grid boundaries and point pose
   * @param point point to print
   * @param gm    grid_map
   */
  void printErrorReason(grid_map::Position point, grid_map::GridMap gm) const;

};
}

#endif
