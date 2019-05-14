#include <algorithm>
#include <iostream>
#include <iterator>
#include "map.h"
#include "newray.h"
#include "mcdmfunction.h"
#include "PathFinding/astar.h"
#include "Criteria/traveldistancecriterion.h"
#include "radio_models/propagationModel.cpp"
# define PI           3.14159265358979323846  /* pi */
#include <unistd.h>
#include <time.h>
#include <ctime>
// #include "RFIDGridmap.h"
#include <ros/ros.h>
#include "movebasegoal.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <costmap_2d/costmap_2d_ros.h>
//mfc ...
#include <ros/console.h>
//mfc ...



using namespace std;
using namespace dummy;
bool contains ( std::list< Pose >& list, Pose& p );
void cleanPossibleDestination2 ( std::list< Pose > &possibleDestinations, Pose& p );
void pushInitialPositions ( dummy::Map map, int x, int y, int orientation,  int range, int FOV, double threshold,
                            string actualPose, vector< pair< string, list< Pose > > > *graph2 );
double calculateScanTime ( double scanAngle );
void calculateDistance(list<Pose> list, dummy::Map& map, Astar* astar);
Pose createFromInitialPose ( Pose pose, int variation, int range, int FOV );
void updatePathMetrics(int* count, Pose* target, Pose* previous, string actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                       dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int encodedKeyValue, Astar* astar , long* numConfiguration,
                       double* totalAngle, double * travelledDistance, int* numOfTurning , double scanAngle);
list<Pose> cleanHistory(vector<string>* history, EvaluationRecords* record_history);
void printResult(long newSensedCells, long totalFreeCells, double precision, long numConfiguration, double travelledDistance,
                 int numOfTurning, double totalAngle, double totalScanTime);

// ROS varies
void move(float x, float y, float orW, float orZ, int resolution, int costresolution, float time_travel);
void update_callback(const map_msgs::OccupancyGridUpdateConstPtr &msg);
void grid_callback(const nav_msgs::OccupancyGridConstPtr &msg);
Pose getCurrentPose(float resolution, float costresolution, dummy::Map* map, double initFov, int initRange);

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
vector<int> occdata;
int costmapReceived = 0;
float costresolution;
int costwidth;
int costheight;
geometry_msgs::Pose costorigin;
nav_msgs::OccupancyGrid costmap_grid;
double min_pan_angle, max_pan_angle, min_tilt_angle, max_tilt_angle, sample_delay, tilt_angle;
int num_pan_sweeps, num_tilt_sweeps;
double sensing_range, offsetY_base_rmld, FoV;
int statusPTU, prevStatusPTU;
double timeOfScanning = 0;
bool btMode = false;
double min_robot_speed = 0.1;


// Input : ./mcdm_online_exploration_ros ./../Maps/map_RiccardoFreiburg_1m2.pgm 100 75 5 0 15 180 0.95 0.12
// resolution x y orientation range centralAngle precision threshold
int main ( int argc, char **argv )
{

  //mfc ...........................
  // some param control ...
  if (argc<6)
  {
    ROS_FATAL("Missing input arguments: Got (%d) and should be (%d) [Field of View, Sensing Range, Precision, Threshold,Resolution]",argc-1,6-1);
    return 1;
  }
  else
  {
    ROS_INFO("Parameters:\n- Field of View (%3.3f)\n- Sensing Range (%d)\n- Precision (%3.3f)\n- Threshold (%3.3f)\n- Resolution: (%3.3f)",
             atof(argv[1]),atoi(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]));
  }
  // sets console output to debug mode...
//  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//  {
//   ros::console::notifyLoggerLevelsChanged();
//  }
  //mfc ...........................

  auto startMCDM = chrono::high_resolution_clock::now();
  ros::init(argc, argv, "mcdm_exploration_framework_node");
  ros::NodeHandle nh;
  ros::ServiceClient map_service_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");
  nav_msgs::GetMap srv_map;
  //ros::Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("mcdm_grid", 1000);
  ros::Publisher moveBasePub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
  MoveBaseClient ac("move_base", true);
  ros::Subscriber costmap_sub;
  ros::Subscriber costmap_update_sub;
  ros::Rate r(1);



  ROS_INFO("Waiting for move_base action server to come up");
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("... waiting ...");
  }

  bool disConnected = true;
  while (disConnected)
  {
    ROS_INFO("Waiting for static_map service to respond...");
    if (map_service_client_.call(srv_map))
    {
      costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("move_base/global_costmap/costmap", 100, grid_callback);
      costmap_update_sub = nh.subscribe<map_msgs::OccupancyGridUpdate>("move_base/global_costmap/costmap_updates", 10, update_callback);
      disConnected=false;
    }
    else
    {
      r.sleep();
    }

  }



  while (ros::ok())
  {

    if (costmapReceived == 0)
    {
      ROS_INFO_STREAM_THROTTLE(60,"waiting for costmap" << std::endl);
      //cout << "Waiting for costmap" << endl;
    }
    if (costmapReceived == 1)
    {
      double initFov = atoi(argv[1]);
      initFov = initFov * PI / 180;
      FoV = initFov;
      int initRange = atoi(argv[2]);
      sensing_range = initRange;
      double precision = atof(argv[3]);
      double threshold = atof(argv[4]);

      /* resolution = 0 -> full resolution
       * resolution = 1 -> 1mx1m
       * resolution = X -> X%(full resolution)
       *NOTE: LOWER RES VALUE, HIGHER REAL RESOLUTION*/
      double resolution = atof(argv[5]);
      cout << "Config: " << endl;
      cout << "   InitFov: " << initFov << endl;
      cout << "   InitRange: " << initRange << endl;
      cout << "   precision: " << precision << endl;
      cout << "   threshold: " << threshold << endl;


      cout << "   Resolution: " << resolution << "\n   Costresolution: " << costresolution << endl;

      dummy::Map map = dummy::Map(resolution, costresolution, costwidth, costheight, occdata, costorigin);
      ROS_DEBUG("Map created correctly");
//        RFIDGridmap myGrid(argv[1], resolution, costresolution, false);
//        cout << "RFIDgrid created correctly" << endl;
      ros::Publisher marker_pub = nh.advertise<geometry_msgs::PointStamped>("goal_pt", 10);
      ROS_DEBUG("[pure_navigation.cpp@main] publisher created ...");
      int gridToPathGridScale = map.getGridToPathGridScale();
      cout << "gridToPathGridScale: " << gridToPathGridScale << endl;
      ROS_DEBUG("[pure_navigation.cpp@main] grid To Path Grid Scale obtained");

      /*NOTE: Transform between map and image, to be enabled if not present in the launch file
      //tf::Transform tranMapToImage;
      //tranMapToImage.setOrigin(tf::Vector3(0, 30, 0.0));
      //tf::Vector3 vecImageToMap = tf::Vector3(0, 30,0.0);
      */

      // Get the initial pose in map frame
      Pose start_pose = getCurrentPose( resolution, costresolution, &map, initFov, initRange);
      Pose target = start_pose;
      Pose previous = target;
      cout << "[AFTER]Initial position in the image frame: " << target.getY() << "," << target.getX()
           << " m. "<< endl;
      Pose invertedInitial = createFromInitialPose(start_pose, 180, initRange, initFov);
      Pose eastInitial = createFromInitialPose(start_pose, 90, initRange, initFov);
      Pose westInitial = createFromInitialPose(start_pose, 270, initRange, initFov);

      long numConfiguration = 1;
      vector<pair<string, list<Pose>>> graph2;
      NewRay ray;
      ray.setGridToPathGridScale(gridToPathGridScale);
      MCDMFunction function;
      long sensedCells = 0;
      long newSensedCells = 0;
      long totalFreeCells = map.getTotalFreeCells();
      int count = 0;
      int countBT;
      double travelledDistance = 0;
      int numOfTurning = 0;
      double totalAngle = 0;
      unordered_map<string, int> visitedCell;
      vector<string> history;
      history.push_back(function.getEncodedKey(target, 1));
      //amount of time the robot should do nothing for scanning the environment ( final value expressed in second)
      unsigned int microseconds = 5 * 1000 * 1000;
      //cout << "total free cells in the main: " << totalFreeCells << endl;
      list<Pose> unexploredFrontiers;
      list<Pose> tabuList;
      list<Pose> nearCandidates;
      EvaluationRecords record;
      Astar astar;
      bool scan = true;
      double totalScanTime = 0;
      int encodedKeyValue = 0;

      // RFID
      double absTagX = 0; //std::stod(argv[12]); // m.
      double absTagY = 0; //std::stod(argv[11]); // m.
      double freq = 0; //std::stod(argv[13]); // Hertzs
      double txtPower = 0; // std::stod(argv[14]); // dBs
      double rxPower = 0;
      std::pair<int, int> relTagCoord;
      long i, j;
      long cell_i, cell_j;
      do
      {
        // If we are doing "forward" navigation towards cells never visited before
        if ( btMode == false )
        {
          // At every iteration, the current pose of the robot is taken from the TF-tree
          Pose target = getCurrentPose(resolution, costresolution, &map, initFov, initRange);
          cout << "[Current Pose]: " << target.getY() << ", " << target.getX()<<" m. " << ", " << target.getOrientation() << ", " << target.getFOV() <<", " << target.getRange() << endl;
//            cout << "[Tmp]: " << tmp_target.getY() << ", " << tmp_target.getX() << ", " << (tmp_target.getOrientation() + 360 ) % 360 << ", " << tmp_target.getFOV() <<", " << tmp_target.getRange() << endl;
          map.getPathPlanningIndex(target.getX(), target.getY(), i, j);
          cout << "[Current CELL INDEX in PathPlanningGrid]: " << i << ", " << j << endl;
          map.getGridIndex(target.getX(), target.getY(), i, j);
          cout << "[Current CELL INDEX in NavigationGrid]: " << i << ", " << j << endl;


          long x = target.getX();
          long y = target.getY();
          int orientation = (target.getOrientation() + 360) % 360;  // cast orientation in [0, 360]
          int range = target.getRange();
          double FOV = target.getFOV();
          string actualPose = function.getEncodedKey ( target,0 );
          map.setCurrentPose ( target );
          string encoding = to_string ( target.getX() ) + to_string ( target.getY() );
          visitedCell.emplace ( encoding,0 );
          // Get the sensing time required for scanning
          target.setScanAngles ( ray.getSensingTime ( &map,x,y,orientation,FOV,range ) );
          // Perform a scanning operation
//          map.getGridIndex(x, y, cell_i, cell_j);
          newSensedCells = sensedCells + ray.performSensingOperation ( &map, x, y, orientation, FOV, range, target.getScanAngles().first, target.getScanAngles().second );
          // Calculate the scanning angle
          double scanAngle = target.getScanAngles().second - target.getScanAngles().first;
          // Update the overall scanning time
          totalScanTime += calculateScanTime ( scanAngle*180/PI );
          // Calculare the relative RFID tag position to the robot position
//            relTagCoord = map.getRelativeTagCoord(absTagX, absTagY, target.getX(), target.getY());
          // Calculate the received power and phase
//            double rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, freq, txtPower);
//            double phase = phaseDifference(relTagCoord.first, relTagCoord.second, freq);
          // Update the path planning and RFID map
//          cout << endl << "[pure_navigation.cpp]" << map.getGridValue(target.getX() + 1, target.getY() + 1) << endl;

          map.getPathPlanningIndex(x, y, cell_i, cell_j);
//          range = range / costresolution;
          map.updatePathPlanningGrid ( cell_i, cell_j, range, rxPower - SENSITIVITY);
//            myGrid.addEllipse(rxPower - SENSITIVITY, map.getNumGridCols() - target.getX(),  target.getY(), target.getOrientation(), -0.5, 7.0);
          // Search for new candidate position
//          map.getGridIndex(x, y, cell_i, cell_j);

          ray.findCandidatePositions ( &map, x, y, orientation, FOV, range );
          vector<pair<long,long> >candidatePosition = ray.getCandidatePositions();
          cout << "Size of initial candidate postions: " << candidatePosition.size() << endl;
          ray.emptyCandidatePositions();
          cout << " Candidate cleaned!" << endl;

          if (scan)
          {
            //NOTE: perform gas sensing------------
//              offsetY_base_rmld = 0.904;
//              tilt_angle = (atan(sensing_range / offsetY_base_rmld) * (180 / PI)) - 90;
//              //tilt_angle = -10;
//              num_pan_sweeps = 1;
//              num_tilt_sweeps = 1;
//              sample_delay = 0.1;
//              //cout << "angolo finale:" << endl;
//              max_pan_angle = getPtuAngle(target.getScanAngles().second, target.getOrientation());
//              //cout << "angolo iniziale:" << endl;
//              min_pan_angle = getPtuAngle(target.getScanAngles().first, target.getOrientation());
//
//              boost::thread mythread(scanning);
//              mythread.join();
//              min_pan_angle = 0;
//              max_pan_angle = 0;
            //-------------------------------------
          }
          // If the exploration just started
          if ( count == 0 )
          {
            // Calculate other three pose given the starting one
            string invertedPose = function.getEncodedKey ( invertedInitial,0 );
            string eastPose = function.getEncodedKey ( eastInitial,0 );
            string westPose = function.getEncodedKey ( westInitial,0 );
            list<Pose> empty ;
            std::pair<string,list<Pose>> pair1 = make_pair ( invertedPose,empty );
            std::pair<string,list<Pose>> pair2 = make_pair ( eastPose,empty );
            std::pair<string,list<Pose>> pair3 = make_pair ( westPose,empty );
            // And add them (with empty candidates) to the graph structure
            graph2.push_back ( pair1 );
            graph2.push_back ( pair2 );
            graph2.push_back ( pair3 );
          }

          // If it's not the first step but we are in one of the initial position (we come back here with backtracking)
          if ( count != 0 && ( target.isEqual ( invertedInitial ) || target.isEqual ( eastInitial ) || target.isEqual ( westInitial ) ) )
          {
            // If there are no more destination in the graph, terminates the navigation
            if ( graph2.size() == 0 ) break;
            graph2.pop_back();
            actualPose = function.getEncodedKey ( target,0 );
            // Add to the graph the initial positions and the candidates from there (calculated inside the function)
            pushInitialPositions ( map, x, y, orientation, range, FOV, threshold, actualPose, &graph2 );
          }


          // If there are no new candidate positions from the current pose of the robot
          if ( candidatePosition.size() == 0 )
          {
            // Find candidates
            ray.findCandidatePositions2 ( &map,x,y,orientation,FOV,range );
            candidatePosition = ray.getCandidatePositions();
            ray.emptyCandidatePositions();

            cout << "No other candidate position" << endl;
            cout << "----- BACKTRACKING -----" << endl;
            // If the graph contains cells that can be explored
            if ( graph2.size() > 1 )
            {
              // Get the last position in the graph and then remove it
              string targetString = graph2.at ( graph2.size()-1 ).first;
              graph2.pop_back();
              //          EvaluationRecords record;
              target = record.getPoseFromEncoding ( targetString );
              // Add it to the history as cell visited more than once
              history.push_back ( function.getEncodedKey ( target,2 ) );
              cout << "[BT]No significative position reachable. Come back to previous position" << endl;
              count = count + 1;
              scan = false;
            }
              //...otherwise, if the graph does not contain cells that can be explored
              // The navigation is finished!
            else
            {
              cout << "Num configuration: " << numConfiguration << endl;
              cout << "Travelled distance calculated during the algorithm: " << travelledDistance << endl;
              cout << "------------------ HISTORY -----------------" << endl;
              // Retrieve the cell visited only the first time
              list<Pose> tmp_history = cleanHistory(&history, &record);
              calculateDistance(tmp_history, map, &astar );

              cout << "------------------ TABULIST -----------------" << endl;
              // Calculate the path connecting the cells in the tabulist, namely the cells that are visited one time and couldn't be visite again
              calculateDistance(tabuList, map, &astar );

              // Normalise the travel distance in meter
              // NOTE: assuming that the robot is moving at 0.5m/s and the resolution of the map is 0.5m per cell)
              if ( resolution == 1.0 )
              {
                travelledDistance = travelledDistance/2;
              }
              printResult(newSensedCells, totalFreeCells, precision, numConfiguration, travelledDistance, numOfTurning,
                          totalAngle, totalScanTime);
              auto endMCDM = chrono::high_resolution_clock::now();
              double totalTimeMCDM = chrono::duration<double, milli>(endMCDM - startMCDM).count();
              cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "ms, "
                   << totalTimeMCDM / 1000 << " s, " <<
                   totalTimeMCDM / 60000 << " m " << endl;
              cout << "Total time in empirical way : " << travelledDistance / 0.25 + timeOfScanning / 1000
                   << endl;
              exit ( 0 );
            }

            sensedCells = newSensedCells;

          }
            //... otherwise, if there are further candidate new position from the current pose of the robot
          else
          {
            // need to convert from a <int,int pair> to a Pose with also orientation,laser range and angle
            list<Pose> frontiers;
            // For every candidate positio, create 8 pose with a different orientation each and consider them as frontiers
            vector<pair<long,long> >::iterator it =candidatePosition.begin();
            for ( it; it != candidatePosition.end(); it++ )
            {
              Pose p1 = Pose ( ( *it ).first, ( *it ).second,0 ,range,FOV );
              Pose p2 = Pose ( ( *it ).first, ( *it ).second,45,range,FOV );
              Pose p3 = Pose ( ( *it ).first, ( *it ).second,90,range,FOV );
              Pose p4 = Pose ( ( *it ).first, ( *it ).second,135,range,FOV );
              Pose p5 = Pose ( ( *it ).first, ( *it ).second,180,range,FOV );
              Pose p6 = Pose ( ( *it ).first, ( *it ).second,225,range,FOV );
              Pose p7 = Pose ( ( *it ).first, ( *it ).second,270,range,FOV );
              Pose p8 = Pose ( ( *it ).first, ( *it ).second,315,range,FOV );
              frontiers.push_back ( p1 );
              frontiers.push_back(p2);
              frontiers.push_back ( p3 );
              frontiers.push_back(p4);
              frontiers.push_back ( p5 );
              frontiers.push_back(p6);
              frontiers.push_back ( p7 );
              frontiers.push_back(p8);

            }

            unexploredFrontiers = frontiers;

            // Evaluate the frontiers and return a list of <frontier, evaluation> pairs
            EvaluationRecords *record = function.evaluateFrontiers ( frontiers, &map, threshold );
            nearCandidates = record->getFrontiers();

            // If there are candidate positions
            if ( record->size() != 0 )
            {
              // Set the previous pose equal to the current one (represented by target)
              previous = target;
              // Select the new robot destination from the list of candidates
              std::pair<Pose,double> result = function.selectNewPose ( record );
              target = result.first;
              // If the selected destination does not appear among the cells already visited
              if ( ! contains ( tabuList,target ))
              {
                // Add it to the list of visited cells as first-view
                encodedKeyValue = 1;
                updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                                  &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle);
                //---------------------------PRINT GOAL POSITION
                geometry_msgs::PointStamped p;
                p.header.frame_id = "map";
                p.header.stamp = ros::Time::now();
                //NOTE: as before, Y in map are X in image
//                if (resolution != costresolution){// && resolution != costresolution) {
//                  //NOTE: full resolution
//                  cout << " [Marker] Full resolution" << endl;
//                  p.point.y = - float(target.getX() + costorigin.position.y / costresolution) * costresolution;
//                  p.point.x = - float(map.getPathPlanningNumRows() - target.getY() + costorigin.position.x / costresolution) * costresolution;
//
//                } else {
                  //NOTE: 1mx1m
                  cout << " [Marker] 1mx1m" << endl;
                  p.point.x = (target.getX()) + costorigin.position.x;//* costresolution;
                  p.point.y = (target.getY()) + costorigin.position.y;// * costresolution;
//                }
                cout << "   (" << p.point.x << "," << p.point.y << ")" << endl;

//                if (resolution == costresolution)
//                {
//                  cout << "   resolution = costresolution" << endl;
//                  tf::Vector3 vec = tf::Vector3(p.point.x, p.point.y, 0.0);
//                  //vec = transform.operator*(vec);
//                  p.point.x = vec.getY() + costorigin.position.x;
//                  p.point.y = vec.getX() + costorigin.position.y;
//                }
                cout << "   New goal in map: X = " << p.point.x << ", Y = " << p.point.y << " m. "<<endl;
                //NOTE: not requested for testing purpose
                //usleep(microseconds);
                marker_pub.publish(p);
                //----------------------------------------------
                move_base_msgs::MoveBaseGoal goal;
                double orientZ = (double) (target.getOrientation() * PI / (2 * 180));
                double orientW = (double) (target.getOrientation() * PI / (2 * 180));

                string path = astar.pathFind ( target.getX(),target.getY(),previous.getX(),previous.getY(), &map );
                float travel_distance = astar.lenghtPath(path);
                cout << "Target is at " << astar.lenghtPath(path) << " cells from the robot" << endl;
                cout << "Resolution : " << resolution << endl;
                float time_travel = travel_distance / min_robot_speed;
                if (resolution == 0)
                {
                  travel_distance = travel_distance * costresolution;
                  time_travel = travel_distance / min_robot_speed;
                }
                cout << "Target is at " << travel_distance << " cells from the robot" << endl;



                if (resolution != 0)
                {
                  cout << "[map] 1mx1m or similar clustered map " << endl;
                  move(p.point.x, p.point.y , sin(orientZ), cos(orientW), resolution, costresolution, time_travel);
                } else
                {
                  cout << "[map] Full resolution" << endl;
                  move(p.point.x, p.point.y, sin(orientZ), cos(orientW), resolution, costresolution, time_travel);  // full resolution
                }

                scan = true;
              }
                // ...otherwise, if the selected cell has already been visited
              else
              {
                // If the graph is empty, stop the navigation
                if ( graph2.size() == 0 ) break;
                // If there still are more candidates to explore from the last pose in the graph
                if ( graph2.at ( graph2.size()-1 ).second.size() != 0 )
                {
                  cout << "[BT1 - Tabulist]There are visible cells but the selected one is already explored!Come back to second best position from the previous position"<< endl;
                  // Remove the current position from possible candidates
                  cleanPossibleDestination2 ( nearCandidates,target );
                  // Get the list of new candidate position with associated evaluation
                  record = function.evaluateFrontiers ( nearCandidates, &map,threshold );
                  // If there are candidate positions
                  if ( record->size() != 0 )
                  {
                    // Select the new pose of the robot
                    std::pair<Pose,double> result = function.selectNewPose ( record );
                    target = result.first;
                    encodedKeyValue = 1;
                    updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                                      &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle);
                    scan = false;
                    // Set that we are now in backtracking
                    btMode = true;
                  }
                    // If there are no more candidate position from the last position in the graph
                  else
                  {
                    // if the graph is now empty, stop the navigation
                    if ( graph2.size() == 0 ) break;
                    // Otherwise, select as new position the last cell in the graph and then remove it from there
                    string targetString = graph2.at ( graph2.size()-1 ).first;
                    graph2.pop_back();
                    target = record->getPoseFromEncoding ( targetString );
                    scan = false;
                  }
                }
                  // ... if the graph still does not present anymore candidate positions for its last pose
                else
                {
                  // Remove the last element (cell and associated candidate from there) from the graph
                  graph2.pop_back();
                  // Select as new target, the new last element of the graph
                  string targetString = graph2.at ( graph2.size()-1 ).first;
                  target = record->getPoseFromEncoding ( targetString );
                  // Save it history as cell visited more than once
                  history.push_back ( function.getEncodedKey ( target,2 ) );
                  cout << "[BT2 - Tabulist]There are visible cells but the selected one is already explored!Come back to two position ago"<< endl;
                  count = count + 1;
                  scan = false;
                }

              }
            }
              // ... otherwise, if there are no candidate positions
            else
            {
              // If the graph is empty, stop the navigation
              if ( graph2.size() == 0 ) break;
              // Select as new target the last one in the graph structure
              string targetString = graph2.at ( graph2.size()-1 ).first;
              // Remove it from the graph
              graph2.pop_back();
              target = record->getPoseFromEncoding ( targetString );
              // Check if the selected cell in the graph is the previous robot position
              if ( !target.isEqual ( previous ) )
              {
                // if it's not, set the old position as the current one
                previous = target;  //TODO: WHY?
                cout << "[BT3]There are no visible cells so come back to previous position in the graph structure" << endl;
                // Save the new target in the history as cell visited more than once
                history.push_back ( function.getEncodedKey ( target,2 ) );
                count = count + 1;
              }
                // If the selected cell is the old robot position
              else
              {
                // If there are no more cells in the graph, just finish the navigation
                if ( graph2.size() == 0 )
                {
                  cout << "[BT4]No other possibilities to do backtracking on previous positions" << endl;
                  break;
                }
                // Select the last position in the graph
                string targetString = graph2.at ( graph2.size()-1 ).first;
                // and remove it from the graph
                graph2.pop_back();
                target = record->getPoseFromEncoding ( targetString );
                // Set the previous pose as the current one
                previous = target;
                cout << "[BT5]There are no visible cells so come back to previous position" << endl;
                cout << "[BT5]Cell already explored!Come back to previous position"<< endl;
                // Add it in history as cell visited more than once
                history.push_back ( function.getEncodedKey ( target,2 ) );
                count = count + 1;
              }

            }



            //NOTE: not requested for testing purpose
            //usleep(microseconds);
            sensedCells = newSensedCells;
            frontiers.clear();
            candidatePosition.clear();
            delete record;
          }

        }
          // ... otherwise, if we are doing backtracking
        else
        {
          long x = target.getX();
          long y = target.getY();
          int orientation = target.getOrientation();
          int range = target.getRange();
          double FOV = target.getFOV();
          string actualPose = function.getEncodedKey ( target,0 );
          map.setCurrentPose ( target );
          //NOTE; calculate path and turnings between actual position and goal
          cout<< function.getEncodedKey ( target,1 ) << endl;
          // Calculate the distance between the previous robot pose and the next one (target)
          string path = astar.pathFind ( target.getX(),target.getY(),previous.getX(),previous.getY(), &map );
          // Update the overall covered distance
          travelledDistance = travelledDistance + astar.lenghtPath( path );
          cout << "BT: " << astar.lenghtPath ( path ) << endl;
          // Update the overall number of turnings
          numOfTurning = numOfTurning + astar.getNumberOfTurning ( path );

          string encoding = to_string ( target.getX() ) + to_string ( target.getY() );
          visitedCell.emplace ( encoding,0 );
          // Set the previous cell to be the same of the current one
          previous = target;

          // Calculate how much time it takes to scan the current area
          target.setScanAngles ( ray.getSensingTime ( &map,x,y,orientation,FOV,range ) );
          // Get the scanning angle
          double scanAngle = target.getScanAngles().second - target.getScanAngles().first;
          // Update the overall scanned angle
          totalAngle += scanAngle;
          // ...and the overall scan time
          totalScanTime += calculateScanTime ( scanAngle*180/PI );
          // Calculate the relative coordinate to the robot of the RFID tag
//            relTagCoord = map.getRelativeTagCoord(absTagX, absTagY, target.getX(), target.getY());
          // Calculate received power and phase
//            double rxPower = received_power_friis(relTagCoord.first, relTagCoord.second, freq, txtPower);
//            double phase = phaseDifference(relTagCoord.first, relTagCoord.second, freq);
          map.updatePathPlanningGrid ( x, y, range, rxPower - SENSITIVITY );
//            myGrid.addEllipse(rxPower - SENSITIVITY, map.getNumGridCols() - target.getX(), target.getY(), target.getOrientation(), -0.5, 7.0);
          // Remove the current pose from the list of possible candidate cells
          cleanPossibleDestination2 ( nearCandidates,target );
          // Get the list of the candidate cells with their evaluation
          EvaluationRecords *record = function.evaluateFrontiers ( nearCandidates, &map, threshold );

          // If there are candidate cells
          if ( record->size() != 0 )
          {
            // Find the new destination
            std::pair<Pose,double> result = function.selectNewPose ( record );
            target = result.first;
            // If this cells has not been visited before
            if ( ! contains ( tabuList,target ) )
            {
              // Add it to the list of visited cells as first-view
              encodedKeyValue = 1;
              updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                                &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle);
              // Leave the backtracking branch
              btMode = false;
              nearCandidates.clear();
              cout << "[BT-MODE4] Go back to previous positions in the graph" << endl;
            }
              // ... otherwise, if the cells has already been visisted
            else
            {
              // If there are other candidates
              if ( nearCandidates.size() != 0 )
              {
                cout << "[BT-MODE1]Already visited, but there are other candidates" << endl;

                // Remove the destination from the candidate list
                cleanPossibleDestination2 ( nearCandidates,target );
                // Get the candidates with their evaluation
                EvaluationRecords *record = function.evaluateFrontiers ( nearCandidates, &map, threshold );
                // Select the new destination
                std::pair<Pose,double> result = function.selectNewPose ( record );
                target = result.first;

                // Add it to the list of visited cells as first-view
                encodedKeyValue = 1;
                updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                                  &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle);
              }
                // ...otherwise, if there are no more candidates
              else
              {
                cout << "[BT-MODE2] Go back to previous positions in the graph" << endl;
                // Select as target the last element in the graph
                string targetString = graph2.at ( graph2.size()-1 ).first;
                // And remove from the graph
                graph2.pop_back();
                target = record->getPoseFromEncoding ( targetString );
                // Add it to the history of cell as already more than once
                encodedKeyValue = 2;
                updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                                  &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle);
                // Leave backtracking
                btMode = false;
                // Clear candidate list
                nearCandidates.clear();
              }
            }
          }
            // ... if there are not candidate cells
          else
          {
            // Select as new pose, the last cell in the graph
            string targetString = graph2.at ( graph2.size()-1 ).first;
            // and the remove it form the graph
            graph2.pop_back();
            target = record->getPoseFromEncoding ( targetString );

            // Add it in history as cell visited more than once
            encodedKeyValue = 2;
            updatePathMetrics(&count, &target, &previous, actualPose, &nearCandidates, &graph2, &map, &function,
                              &tabuList, &history, encodedKeyValue, &astar, &numConfiguration, &totalAngle, &travelledDistance, &numOfTurning, scanAngle);
            // Leave backtracking
            btMode = false;
            cout << "[BT-MODE3] Go back to previous positions in the graph" << endl;
          }
          delete record;
        }
      }
        // Perform exploration until a certain coverage is achieved
      while ( sensedCells < precision * totalFreeCells );
      // Plotting utilities
      map.drawVisitedCells ();
      map.printVisitedCells ( history );
//        map.drawRFIDScan();
//        map.drawRFIDGridScan(myGrid);
//        myGrid.saveAs(("/home/pulver/Desktop/MCDM/rfid_result_gridmap.pgm"));

      cout << "Num configuration: " << numConfiguration << endl;
      cout << "Travelled distance calculated during the algorithm: " << travelledDistance << endl;

      cout << "------------------ HISTORY -----------------" << endl;
      // Calculate which cells have been visited only once
      list<Pose> tmp_history = cleanHistory(&history, &record);
      calculateDistance(tmp_history, map, &astar );

      cout << "------------------ TABULIST -----------------" << endl;
      calculateDistance(tabuList, map, &astar );

      // Trasform distance in meters
      if ( resolution == 1.0 ) // Corridor map has a resolution of 0.5 meter per cell
      {
        travelledDistance = travelledDistance/2;
      }


      printResult(newSensedCells, totalFreeCells, precision, numConfiguration, travelledDistance, numOfTurning,
                  totalAngle, totalScanTime);
      // Find the tag
//        std::pair<int,int> tag = map.findTag();
//        cout << "RFID pose: [" << tag.second << "," << tag.first << "]" << endl;
//        tag = map.findTagfromGridMap(myGrid);
//        cout << "[Grid]RFID pose: [" << tag.second << "," << tag.first << "]" << endl;
      cout << "-----------------------------------------------------------------"<<endl;
      auto endMCDM= chrono::high_resolution_clock::now();

      double totalTimeMCDM = chrono::duration<double,milli> ( endMCDM - startMCDM ).count();
      cout << "Total time for MCDM algorithm : " << totalTimeMCDM << "ms, " << totalTimeMCDM/1000 <<" s, " <<
           totalTimeMCDM/60000 << " m "<< endl;
      cout << "Spinning at the end" << endl;
      sleep(1);
      exit ( 0 );
    }

    ros::spinOnce();
    r.sleep();


  }// end while ros::ok
}// end main

bool contains ( std::list<Pose>& list, Pose& p )
{
  bool result = false;
  MCDMFunction function;

  std::list<Pose>::iterator findIter = std::find ( list.begin(), list.end(), p );
  if ( findIter != list.end() )
  {
    //cout << "Found it: "<< function.getEncodedKey(p,0) <<endl;
    result = true;
  }

  return result;
}

void cleanPossibleDestination2 ( std::list< Pose >& possibleDestinations, Pose& p )
{
  MCDMFunction function;
  //cout<<"I remove "<< function.getEncodedKey(p,0) << endl;
  //cout << possibleDestinations->size() << endl;
  std::list<Pose>::iterator findIter = std::find ( possibleDestinations.begin(), possibleDestinations.end(), p );
  if ( findIter != possibleDestinations.end() )
  {
    //cout << function.getEncodedKey(*findIter,0) << endl;
    possibleDestinations.erase ( findIter );
  }
  else cout<< "not found" << endl;

}


void pushInitialPositions ( dummy::Map map, int x, int y, int orientation, int range, int FOV, double threshold, string actualPose, vector< pair< string, list< Pose > > >* graph2 )
{
  NewRay ray;
  MCDMFunction function;
  ray.findCandidatePositions ( &map,x,y,orientation ,FOV,range );
  vector<pair<long,long> >candidatePosition = ray.getCandidatePositions();
  ray.emptyCandidatePositions();
  list<Pose> frontiers;
  vector<pair<long,long> >::iterator it =candidatePosition.begin();
  for ( it; it != candidatePosition.end(); it++ )
  {
    Pose p1 = Pose ( ( *it ).first, ( *it ).second,0 ,range,FOV );
    Pose p2 = Pose ( ( *it ).first, ( *it ).second,180,range,FOV );
    Pose p3 = Pose ( ( *it ).first, ( *it ).second,90,range,FOV );
    Pose p4 = Pose ( ( *it ).first, ( *it ).second,270,range,FOV );
    frontiers.push_back ( p1 );
    frontiers.push_back ( p2 );
    frontiers.push_back ( p3 );
    frontiers.push_back ( p4 );
  }
  EvaluationRecords *record = function.evaluateFrontiers ( frontiers, &map, threshold );
  list<Pose>nearCandidates = record->getFrontiers();
  cout << "Number of candidates:" << nearCandidates.size() << endl;
  std::pair<string,list<Pose>> pair = make_pair ( actualPose,nearCandidates );
  graph2->push_back ( pair );
}

double calculateScanTime ( double scanAngle )
{
  return ( -7.2847174296449998e-006*scanAngle*scanAngle*scanAngle + 2.2131847908245512e-003*scanAngle*scanAngle + 1.5987873410233613e-001*scanAngle + 10 );
}

Pose createFromInitialPose ( Pose pose, int variation, int range, int FOV )
{
  Pose tmp = Pose ( pose.getX(), pose.getY(), ( pose.getOrientation() + variation ) %360,FOV,range );
  return tmp;
}


void calculateDistance(list<Pose> history, dummy::Map& map, Astar* astar)
{
  std::list<Pose>::iterator it = history.begin();
  double travelledDistance = 0;
  int numOfTurning = 0;
  // Calculate the overall path connecting these cells
  for ( it; it != prev ( history.end(),1 ); it++ )
  {
//        cout << function->getEncodedKey(*it,1) << endl; // print cell in the tabulist
    std::list<Pose>::iterator it2 = next ( it,1 );
    string path = astar->pathFind ( ( *it2 ).getX(), ( *it2 ).getY(), ( *it ).getX(), ( *it ).getY(), &map );
    travelledDistance = travelledDistance + astar->lenghtPath ( path );
    numOfTurning = numOfTurning + astar->getNumberOfTurning ( path );
    //cout << astar.lengthPath ( path ) << endl;
  }
  cout << "Number of cells: " << history.size() << endl;
  cout << "Num of Turning: " << numOfTurning << endl;
  cout << "Travelled distance (cells): " << travelledDistance << endl;
  cout << "Travelled distance (meters): " << travelledDistance / 2.0 << endl; // Valid only if resolution == 1.0 (cell side is 0.5m)
}

void updatePathMetrics(int* count, Pose* target, Pose* previous, string actualPose, list<Pose>* nearCandidates, vector<pair<string,list<Pose>>>* graph2,
                       dummy::Map* map, MCDMFunction* function, list<Pose>* tabuList, vector<string>* history, int encodedKeyValue, Astar* astar , long* numConfiguration,
                       double* totalAngle, double* travelledDistance, int* numOfTurning , double scanAngle)
{
  // Add it to the list of visited cells as first-view
  history->push_back ( function->getEncodedKey ( *target, encodedKeyValue ) );
//  cout << function->getEncodedKey ( *target,1 ) << endl;
  // Add it to the list of visited cells from which acting
  tabuList->push_back ( *target );
  // Remove it from the list of candidate position
  cleanPossibleDestination2 ( *nearCandidates, *target );
  // Push in the graph the previous robot pose and the new list of candidate position, without the current pose of the robot
  // We don't want to visit this cell again
  std::pair<string,list<Pose>> pair = make_pair ( actualPose, *nearCandidates );
  graph2->push_back ( pair );
  // Calculate the path from the previous robot pose to the current one
  string path = astar->pathFind ( target->getX(), target->getY(), previous->getX(), previous->getY(), map );
//  cout << "1: " << *travelledDistance << endl;
  // Update the distance counting
  *travelledDistance = *travelledDistance + astar->lenghtPath(path);
//  cout << "2: " << *travelledDistance << endl;
  // Update the turning counting
  *numOfTurning = *numOfTurning + astar->getNumberOfTurning(path);
  // Update the scanning angle
  *totalAngle += scanAngle;
  // Update the number of configurations of the robot along the task
  (*numConfiguration)++;
  // Update counter of iterations
  (*count)++;
}

list<Pose> cleanHistory(vector<string>* history, EvaluationRecords* record_history){
  vector<string>::iterator it_history = history->begin();
  list<Pose> tmp_history;
  for ( it_history; it_history!=prev(history->end(),1); it_history++)
  {
    if ((*it_history).back() == '1')
    {
      tmp_history.push_back(record_history->getPoseFromEncoding(*it_history));
    }
  }
  return tmp_history;
}

void printResult(long newSensedCells, long totalFreeCells, double precision, long numConfiguration, double travelledDistance,
                 int numOfTurning, double totalAngle, double totalScanTime)
{
  cout << "-----------------------------------------------------------------"<<endl;
  cout << "Area sensed: " << newSensedCells << " / " << totalFreeCells<< endl;
  cout << "Total cell visited :" << numConfiguration <<endl;
  cout << "Total travelled distance (meters): " << travelledDistance << endl;
  cout << "Total travel time: " << travelledDistance / 0.5 << "s, " << ( travelledDistance/0.5) /60 << " m"<< endl;
  cout << "I came back to the original position since i don't have any other candidate position"<< endl;
  cout << "Total exploration time (s): " << travelledDistance / 0.5 << endl;
  cout << "Total number of turning: " << numOfTurning << endl;
  cout << "Sum of scan angles (radians): " << totalAngle << endl;
  cout << "Total time for scanning: " << totalScanTime << endl;
  cout << "Total time for exploration: " << travelledDistance/0.5 + totalScanTime << "s, " <<
       ( travelledDistance/0.5 + totalScanTime ) /60 << " m" << endl;
  if (newSensedCells < precision * totalFreeCells)
  {
    cout << "FINAL: MAP NOT EXPLORED! :(" << endl;
  } else
  {
    cout << "FINAL: MAP EXPLORED!" << endl;
  }

  cout << "-----------------------------------------------------------------"<<endl;
}


Pose getCurrentPose(float resolution, float costresolution, dummy::Map* map, double initFov, int initRange)
{
  ros::Time _now_stamp_ = ros::Time(0);

  tf::StampedTransform start_pose_in_tf;
  tf::TransformListener _tf_listener;

  _tf_listener.waitForTransform("map", "base_link", _now_stamp_, ros::Duration(2.0));
  try
  {
    _tf_listener.lookupTransform("map", "base_link", _now_stamp_, start_pose_in_tf);
  }
  catch(tf::TransformException& ex)
  {
    ROS_INFO("TRANSFORMS ARE COCKED-UP PAL! Why is that :=> %s", ex.what());
  }

  tf::Vector3 start_position = start_pose_in_tf.getOrigin();
  tf::Quaternion start_orientation = start_pose_in_tf.getRotation();

  geometry_msgs::PoseStamped start_pose;
  start_pose.header.stamp = start_pose_in_tf.stamp_;
  start_pose.header.frame_id = start_pose_in_tf.frame_id_;

  tf::pointTFToMsg(start_position, start_pose.pose.position);
  tf::quaternionTFToMsg(start_orientation, start_pose.pose.orientation);

  double initX = start_pose.pose.position.x;
  double initY = start_pose.pose.position.y;
  tf::Quaternion quat = tf::Quaternion(start_pose.pose.orientation.x, start_pose.pose.orientation.y,
                                       start_pose.pose.orientation.z, start_pose.pose.orientation.w);
  tfScalar angle = 2 * atan2(quat[2], quat[3]);

  cout << endl << "Current position in the map frame:" << initX << "," << initY << " with orientation :" << angle
       << endl;

  int initOrientation = angle * 180 / PI;
  cout << "Orientation after casting: " << initOrientation << endl;


  //ATTENTION: should be adapted for cells different from 1mx1m
  //convert from map frame to image
  tf::Vector3 pose = tf::Vector3(initX, initY, 0.0);
  if (resolution >= 0 && resolution < 1 && resolution != costresolution)
  {
    //full resolution and scaling
    cout << "Full resolution and scaling" << endl;
    pose = pose / costresolution;
//    cout << "[BEFORE]Initial position in the Gazebo frame: " << pose.getX() * costresolution<< "," << pose.getY() * costresolution << endl;
    Pose initialPose = Pose(map->getPathPlanningNumRows() - (long) pose.getY() + costorigin.position.y / costresolution, (long) pose.getX() - costorigin.position.x / costresolution,
                            initOrientation, initRange, initFov);
    return initialPose;
  }
  else
  {
    //1mx1m
    cout << endl << "1mx1m" << endl;
    //cout << "[BEFORE]Initial position in the image frame: " << pose.getX()<< "," << map.getPathPlanningNumRows() - (long)pose.getY() << endl;
    //NOTE: Y in map are X in image
    Pose initialPose = Pose( (long) pose.getX() , (long) pose.getY(), initOrientation, initRange, initFov);
    return initialPose;
  }

}

void grid_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  //ROS_INFO("RECEIVED A MAP!");
  if(costmapReceived == 0)
  {
    ROS_INFO("CALLBACK FIRST!");
    //costmap_grid = msg.get();
    costresolution = msg->info.resolution;
    costwidth = msg->info.width;
    costheight = msg->info.height;
    costorigin = msg->info.origin;
    for(int i = 0; i < msg.get()->data.size(); ++i)
    {
      occdata.push_back(msg->data.at(i));
    }
    std::cout << "size of occdata " << occdata.size() << " size of message data " << msg->data.size() << std::endl;
    std::cout << "height " << msg->info.height << " width " << msg->info.width << " resolution " << msg->info.resolution << std::endl;
    costmapReceived = 1;
  }

}

void update_callback(const map_msgs::OccupancyGridUpdateConstPtr& msg)
{
  //NOTE: everything is commented because we don't want update the costmap since the environment is
  //assumed static

  //std::cout << "CALLBACK SECOND" << std::endl;

  /*int index = 0;
      for(int y=msg->y; y< msg->y+msg->height; y++) {
              for(int x=msg->x; x< msg->x+msg->width; x++) {
                      costmap_grid.data[ getIndex(x,y) ] = msg->data[ index++ ];
              }
      }*/
}


int getIndex(int x, int y){
  int sx = costmap_grid.info.width;
  return y * sx + x;
}


void move(float x, float y, float orZ, float orW, int resolution, int costresolution, float time_travel){
  move_base_msgs::MoveBaseGoal goal;

  MoveBaseClient ac ("move_base", true);
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();


//  // TODO: merge
//  // Clustered map
//  if (resolution == costresolution)
//  {
//    goal.target_pose.pose.position.x = x ;//+ costorigin.position.x;
//    goal.target_pose.pose.position.y = y ;//+ costorigin.position.y;
//  } else
//  {
    // Full resolution
    goal.target_pose.pose.position.x = x;//  * costresolution + costorigin.position.x;
    goal.target_pose.pose.position.y = y; // * costresolution + costorigin.position.y;
//  }

  goal.target_pose.pose.orientation.z = orZ;
  goal.target_pose.pose.orientation.w = orW;

  ROS_INFO("Sending goal");
  cout << "   [map]goal: (" << float(x) << "," << float(y) << ")" << endl;
  ac.sendGoal(goal);

  cout << "     Waiting for " << time_travel << " seconds" << endl;
  ac.waitForResult(ros::Duration(time_travel));

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("I'm moving...");
  else
    ROS_INFO("The base failed to move");

  cout << endl;
}
