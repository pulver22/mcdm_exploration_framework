
#include "RFIDGridmap.h"



RFIDGridmap::RFIDGridmap(nav_msgs::OccupancyGrid occGrid, double mapResolution,
                         double gridResolution, bool debug)
    : global_frame_("world"), layer_name_("rfid"), format_("mono8") {


  ROS_DEBUG("[RFIDGridmap.cpp@constructor] Creating RFIDGridmap");
  std::string fileURI="tmp/rfid_in_grid.png";
  GridMapRosConverter::fromOccupancyGrid(occGrid, layer_name_, RFIDGridmap_);
  saveLayer(RFIDGridmap_, layer_name_, fileURI, debug_, format_);
  createGrid(RFIDGridmap_, layer_name_, fileURI, mapResolution, gridResolution,
             debug_, global_frame_, format_);
  ROS_DEBUG("[RFIDGridmap.cpp@main] grid created");

};





RFIDGridmap::RFIDGridmap(std::string fileURI, double mapResolution,
                         double gridResolution, bool debug)
    : global_frame_("world"), layer_name_("rfid"), format_("mono8") {
  ROS_DEBUG("[RFIDGridmap.cpp@main] Creating RFIDGridmap");
  debug_ = debug;
  createGrid(RFIDGridmap_, layer_name_, fileURI, mapResolution, gridResolution,
             debug_, global_frame_, format_);
  ROS_DEBUG("[RFIDGridmap.cpp@main] grid created");
};

RFIDGridmap::RFIDGridmap(std::string fileURI, double mapResolution,
                         double gridResolution)
    : RFIDGridmap::RFIDGridmap(fileURI, mapResolution, gridResolution, true){};

void RFIDGridmap::saveAs(std::string fileURI) {
  saveLayer(RFIDGridmap_, layer_name_, fileURI, debug_, format_);
};

void RFIDGridmap::addEllipse(double likelihood, double poseX, double poseY,
                             double poseHeading, double minX, double maxX) {

  addEllipse(RFIDGridmap_, layer_name_, likelihood, poseX, poseY, poseHeading,
             minX, maxX, debug_);
};

void RFIDGridmap::addCircle(double likelihood, double poseX, double poseY,
                            double poseR) {
  addCircle(RFIDGridmap_, layer_name_, likelihood, poseX, poseY, poseR, debug_);
};

void RFIDGridmap::addLine(double likelihood, double X1, double Y1, double X2,
                          double Y2) {
  addLine(RFIDGridmap_, layer_name_, likelihood, X1, Y1, X2, Y2, debug_);
};

double RFIDGridmap::getCell(int i, int j) {
  return getCell(RFIDGridmap_, layer_name_, i, j, debug_);
};

void RFIDGridmap::setCell(double val, int i, int j) {
  setCell(RFIDGridmap_, layer_name_, val, i, j, debug_);
};

double RFIDGridmap::getPosition(double x, double y) {
  return getPosition(RFIDGridmap_, layer_name_, x, y, debug_);
};

void RFIDGridmap::setPosition(double val, double x, double y) {
  setPosition(RFIDGridmap_, layer_name_, val, x, y, debug_);
};

long RFIDGridmap::getNumCols() { return RFIDGridmap_.getSize()(1); }

long RFIDGridmap::getNumRows() { return RFIDGridmap_.getSize()(0); }

// Internal functions to handle gridmap. .......................................

void RFIDGridmap::createGrid(grid_map::GridMap &map_, std::string layerName,
                             std::string fileURI, double mapResolution,
                             double gridResolution, bool debug,
                             std::string global_frame, std::string format) {

  // load map file into a gridmap ..............................................
  // debug
  if (debug) {
    std::cout << "Using file [" << fileURI << "]\n";
  }

  // 2D position of the grid map in the grid map frame [m].
  double orig_x;
  double orig_y;

  // grid size in pixels
  double num_rows;
  double num_cols;

  // cell value ranges
  double minValue;
  double maxValue;

  // load an image from cv
  if (debug) {
    std::cout << "Loading image into cv mat. \n";
  }
  cv::Mat imageCV = cv::imread(fileURI, CV_LOAD_IMAGE_UNCHANGED);
  num_rows = imageCV.rows;
  num_cols = imageCV.cols;

  // orig will be placed at bottom left position
  orig_x = num_rows * mapResolution / 2.0;
  orig_y = -num_cols * mapResolution / 2.0;

  cv::minMaxLoc(imageCV, &minValue, &maxValue);

  if (debug) {
    std::cout << "Image size [" << num_rows << ", " << num_cols << "]\n";
    std::cout << "Min, max values [" << minValue << ", " << maxValue << "]\n";
    std::cout << "Channels [" << imageCV.channels() << "]\n";
    std::cout << "Encoding  [" << type2str(imageCV.type()) << "]\n";
  }

  // create empty grid map
  if (debug) {
    std::cout << "Creating empty grid \n";
  }

  grid_map::GridMap tempMap(vector<string>({std::string(layerName)}));
  tempMap.setGeometry(Length(num_rows, num_cols), mapResolution,
                      Position(orig_x, orig_y));
  tempMap.setFrameId(global_frame);
  tempMap.clearAll();

  // Convert cv image to grid map.
  if (debug) {
    std::cout << "Storing cv mat into emtpy grid \n";
  }
  sensor_msgs::ImagePtr imageROS =
      cv_bridge::CvImage(std_msgs::Header(), format, imageCV).toImageMsg();
  GridMapRosConverter::addLayerFromImage(*imageROS, layerName, tempMap);

  // binarize: mark obstacles
  // If the value in the map is below 250, set it to 1 to represent a free

  if (debug) {
    std::cout << "Binarize occupancy probs.  \n";
  }

  double countOnes = 0;
  double countZeros = 0;

  // note. For some reason, upon creating the grid, it casts from [0,255] to
  // [0,1]. So instead of looking for 250, look for 250/255~0.98
  for (grid_map::GridMapIterator iterator(tempMap); !iterator.isPastEnd();
       ++iterator) {
    if (tempMap.at(layerName, *iterator) < 0.98) {
      tempMap.at(layerName, *iterator) = 1;
      countOnes++;
    } else {
      tempMap.at(layerName, *iterator) = 0;
      countZeros++;
    }
  }

  if (debug) {
    std::cout << 100 * countOnes / (countOnes + countZeros)
              << " % of cells are obstcles \n";
    std::cout << 100 * countZeros / (countOnes + countZeros)
              << " % of cells are empty \n";
  }

  // change gridmap resolution from mapResolution to gridResolution
  if (debug) {
    std::cout << "Changing grid resolution.  \n";
  }

  GridMapCvProcessing::changeResolution(tempMap, map_, gridResolution);
};

void RFIDGridmap::addEllipse(grid_map::GridMap &map_, std::string layerName,
                             double likelihood, double antennaX,
                             double antennaY, double antennaHeading,
                             double minX, double maxX, bool debug) {

  // 1.-  Get elipsoid iterator.
  // Antenna is at one of the focus of the ellipse with center at antennaX,
  // antennaY, tilted antennaHeading .
  // http://www.softschools.com/math/calculus/finding_the_foci_of_an_ellipse/
  // if a is mayor axis and b is minor axis
  // a-c= minX
  // a+c= maxX
  // a = (maxX + minX)/2
  // c  = maxX/2 + minX
  // b  = sqrt(a^2-c^2)

  // mirror y axis!!!!
  //  antennaX = -antennaX;
  antennaY = -antennaY;
  antennaHeading = -antennaHeading;

  double a = (abs(maxX) + abs(minX)) / 2.0;
  double c = (abs(maxX) - abs(minX)) / 2;
  double b = sqrt((a * a) - (c * c));
  double xc = antennaX + (c * cos(antennaHeading));
  double yc = antennaY + (c * sin(antennaHeading));

  Position center(xc, yc); // meters
  Length length(2 * a, 2 * b);

  for (grid_map::EllipseIterator iterator(map_, center, length, antennaHeading);
       !iterator.isPastEnd(); ++iterator) {
    // map_.at(layerName, *iterator)=0.5;
    if (!isnan(map_.at(layerName, *iterator))) {
      map_.at(layerName, *iterator) += likelihood;
    } else {
      map_.at(layerName, *iterator) = likelihood;
    }
  }

  // debug
  if (debug) {
    std::cout << "Ellipse data \n";
    std::cout << "maxX = " << maxX << "\n";
    std::cout << "minX = " << minX << "\n";
    std::cout << "Axes [ " << a << ", " << b << ", " << c << " ]\n";
    std::cout << "Focal point [ " << antennaX << ", " << antennaY << " ]\n";
    std::cout << "Centre [ " << xc << ", " << yc << " ]\n";

    // CAREFUL!!!
    // Position focal(antennaX, antennaY); // meters
    // grid_map::Index cIndex;
    // map_.getIndex(focal,cIndex);
    // map_.at(layerName, cIndex)=0.0;
  }
};

void RFIDGridmap::addLine(grid_map::GridMap &map_, std::string layerName,
                          double likelihood, double X1, double Y1, double X2,
                          double Y2, bool debug) {
  // mirror y axis!!!
  Y1 = -Y1;
  Y2 = -Y2;

  grid_map::Index start;
  grid_map::Index end;
  map_.getIndex(Position(X1, Y1), start);
  map_.getIndex(Position(X2, Y2), end);

  for (grid_map::LineIterator iterator(map_, start, end); !iterator.isPastEnd();
       ++iterator) {
    if (!isnan(map_.at(layerName, *iterator))) {
      map_.at(layerName, *iterator) += likelihood;
    } else {
      map_.at(layerName, *iterator) = likelihood;
    }
  }
};

void RFIDGridmap::addCircle(grid_map::GridMap &map_, std::string layerName,
                            double likelihood, double X, double Y, double R,
                            bool debug) {

  // necessary mirroring ...
  Y = -Y;
  Position center(X, Y);

  for (grid_map::CircleIterator iterator(map_, center, R);
       !iterator.isPastEnd(); ++iterator) {
    if (!isnan(map_.at(layerName, *iterator))) {
      map_.at(layerName, *iterator) += likelihood;
    } else {
      map_.at(layerName, *iterator) = likelihood;
    }
  }
};

void RFIDGridmap::saveLayer(grid_map::GridMap map_, std::string layerName,
                            std::string fileURI, bool debug,
                            std::string format) {
  if (debug) {
    std::cout << "Reconverting into ros msg"
              << "\n";
  }

  double min = INFINITY;
  double max = -INFINITY;
  double val;
  // find max and min
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd();
       ++iterator) {
    val = map_.at(layerName, *iterator);
    if (val < min) {
      min = val;
    }
    if (val > max) {
      max = val;
    }
  }

  // and map everything between 0 and 1
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd();
       ++iterator) {
    val = (map_.at(layerName, *iterator) - min) / (max - min);
    map_.at(layerName, *iterator) = val;
  }

  sensor_msgs::Image imageROSout;
  GridMapRosConverter::toImage(map_, layerName, format, imageROSout);

  if (debug) {
    std::cout << "Reconverting into cv"
              << "\n";
  }

  cv_bridge::CvImagePtr imageCVout;
  imageCVout = cv_bridge::toCvCopy(imageROSout, format);

  if (debug) {
    std::cout << "And saving as [" << fileURI << "]\n";
  }

  cv::imwrite(fileURI, imageCVout->image);
};

double RFIDGridmap::getCell(grid_map::GridMap &map_, std::string layerName,
                            int i, int j, bool debug) {

  grid_map::Index index(i, j);
  double val = map_.at(layerName, index);

  return val;
};

void RFIDGridmap::setCell(grid_map::GridMap &map_, std::string layerName,
                          double value, int i, int j, bool debug) {
  grid_map::Index index(i, j);
  map_.at(layerName, index) = value;
};

double RFIDGridmap::getPosition(grid_map::GridMap &map_, std::string layerName,
                                double x, double y, bool debug) {

  Position focal(x, y);
  grid_map::Index index;
  map_.getIndex(focal, index);
  double val = map_.at(layerName, index);

  return val;
};

void RFIDGridmap::setPosition(grid_map::GridMap &map_, std::string layerName,
                              double value, double x, double y, bool debug) {

  Position focal(x, y);
  grid_map::Index index;
  map_.getIndex(focal, index);
  map_.at(layerName, index) = value;
};
// Auxiliary functions.........................................................


// not needed ?
// void RFIDGridmap::encodeGrid(grid_map::GridMap *gm, std::string layerName, int obstValue, int freeValue) {
//   long n_obsts = 0;
//   long n_others = 0;
//   long total = 0;
//   for (grid_map::GridMapIterator iterator(*gm); !iterator.isPastEnd();
//        ++iterator) {
//     if (gm->at(layerName, *iterator) >= obstValue) {
//       gm->at(layerName, *iterator) = -1;
//       n_obsts++;
//     } else
//     {
//       gm->at(layerName, *iterator) = 0;
//       n_others++;
//     }
//   }
//
//   total = n_obsts +  n_others;
//   ROS_DEBUG("[RFIDGridmap.cpp@encodeGrid] Encoded grid with %3.3f %% of occupied cells CASTED to  (-1)", 100.0 * n_obsts / (1.0 * total));
//   ROS_DEBUG("[RFIDGridmap.cpp@encodeGrid] %3.3f %% cells under obst threshold (%d), CASTED to (0)", 100.0 * n_others / (1.0 * total), obstValue);
// }


string RFIDGridmap::type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
  case CV_8U:
    r = "8U";
    break;
  case CV_8S:
    r = "8S";
    break;
  case CV_16U:
    r = "16U";
    break;
  case CV_16S:
    r = "16S";
    break;
  case CV_32S:
    r = "32S";
    break;
  case CV_32F:
    r = "32F";
    break;
  case CV_64F:
    r = "64F";
    break;
  default:
    r = "User";
    break;
  }

  r += "C";
  r += (chans + '0');

  return r;
};
