#include <map.h>

namespace dummy{

    Map::Map():map_grid_(vector<string>({"layer"})), nav_grid_(vector<string>({"layer"})), planning_grid_(vector<string>({"layer"})), rfid_grid_(vector<string>({"layer"}))
    {


    }

    Map::Map(std::ifstream& infile, float resolution):map_grid_(vector<string>({"layer"})), nav_grid_(vector<string>({"layer"})), planning_grid_(vector<string>({"layer"})), rfid_grid_(vector<string>({"layer"}))
    {
      Map::createMap(infile);
      Map::createGrid(resolution);
      Map::createNewMap();
      ROS_DEBUG("[Map.cpp@map] map created from ifstream");
    }

    Map::Map(float resolution, float costresolution, int width, int height, vector<int> data, geometry_msgs::Pose origin):map_grid_(vector<string>({"layer"})), nav_grid_(vector<string>({"layer"})), planning_grid_(vector<string>({"layer"})), rfid_grid_(vector<string>({"layer"}))
    {
      Map::createMap(width, height, data, origin);
      Map::createGrid(resolution);
      Map::createNewMap();
      Map::createPathPlanningGrid(costresolution);
      // once both are created, we can obtain the ratio
      gridToPathGridScale = planning_grid_.getResolution()/nav_grid_.getResolution();
      ROS_ASSERT_MSG(gridToPathGridScale > 0, "[Map.cpp@map] Planning resolution lower than navigation resolution = %d", gridToPathGridScale);


      Map::createRFIDGrid(costresolution);
      ROS_DEBUG("[Map.cpp@map] map created from data vector");
    }

    void Map::createMap(std::ifstream& infile)
    {
      // load an image from cv
      ROS_DEBUG("[Map.cpp@createMap] Loading image into cv mat.");
      cv::Mat imageCV = MreadImage(infile);
      geometry_msgs::Pose origin;
      // orig will be placed at bottom left position
      //TODO : CHECK THIS!!!
      origin.position.x=0;
      origin.position.y=0;
      // harcoded values!!!!
      std::string map_frame_id="map";
      double map_resolution = 0.1;

      createMap(imageCV,origin,map_frame_id,map_resolution);
    }

    void Map::createMap(cv::Mat imageCV,geometry_msgs::Pose origin, std::string map_frame_id, double map_resolution)
    {

      // map data WAS stored in map internal var, now is map_grid_


      // this method is simiarl to RFIDGridmap::createGrid

      //2D position of the grid map in the grid map frame [m].
      double orig_x;
      double orig_y;


      // cell value ranges
      double  minValue;
      double  maxValue;

      // grid size in pixels
      Map::numRows = imageCV.rows;
      Map::numCols = imageCV.cols;

      // map origin in rangeInMeters
      orig_x=origin.position.x;
      orig_y=origin.position.y;


      cv::minMaxLoc(imageCV, &minValue, &maxValue);

      ROS_DEBUG("[Map.cpp@createMap] Image size [%lu, %lu]",numRows, numCols);
      ROS_DEBUG("[Map.cpp@createMap] Min, max values [%3.3f %3.3f]",minValue, maxValue);
      ROS_DEBUG("[Map.cpp@createMap] Channels [%d]",imageCV.channels());
      ROS_DEBUG("[Map.cpp@createMap] Encoding [%s]",type2str(imageCV.type()).c_str() );
      ROS_DEBUG("[Map.cpp@createMap] Dummy map resolution [%3.3f]",map_resolution);

      // create empty grid map
      ROS_DEBUG("[Map.cpp@createMap] Creating empty grid");

      grid_map::GridMap tempMap(vector<string>({"layer"}));
      tempMap.setGeometry(Length(numRows, numCols), map_resolution, Position(orig_x, orig_y));
      tempMap.setFrameId(map_frame_id);
      tempMap.clearAll();

      // Convert cv image to grid map.
      ROS_DEBUG("[Map.cpp@createMap] Storing cv mat into emtpy grid");
      string format=("mono8");
      sensor_msgs::ImagePtr imageROS = cv_bridge::CvImage(std_msgs::Header(), format, imageCV).toImageMsg();
      GridMapRosConverter::addLayerFromImage(*imageROS, "layer", tempMap);

      // binarize: mark obstacles
      // If the value in the map is below 250, set it to 1 to represent a free

      ROS_DEBUG("[Map.cpp@createMap] Binarize occupancy probs.");

      double countOnes=0;
      double countZeros=0;

      // note. For some reason, upon creating the grid, it casts from [0,255] to
      // [0,1]. So instead of looking for 250, look for 250/255~0.98
      for (grid_map::GridMapIterator iterator(tempMap); !iterator.isPastEnd(); ++iterator) {
        if (tempMap.at("layer", *iterator)<0.98){
               tempMap.at("layer", *iterator)=1;
               countOnes++;
        } else {
               tempMap.at("layer", *iterator)=0;
               countZeros++;
        }
      }

      ROS_DEBUG("[Map.cpp@map] [%3.3f] percent of cells are obstacles ",100*countOnes/(countOnes+countZeros)  );
      ROS_DEBUG("[Map.cpp@map] [%3.3f] percent of cells are empty ",100*countZeros/(countOnes+countZeros)  );


      //GridMapCvProcessing::changeResolution(tempMap, map_, gridResolution);
      map_grid_ = tempMap;

    }

    void Map::createMap(int width,int height,vector<int> data, geometry_msgs::Pose origin)
    {
      // load an image from vector
      ROS_DEBUG("[Map.cpp@createMap] Loading vector into cv mat.");
      cv::Mat imageCV = MreadImage(width,height,data);

      //TODO : CHECK THIS!!!
      // harcoded values!!!!
      std::string map_frame_id="map";
      double map_resolution = 0.1;

      createMap(imageCV,origin,map_frame_id,map_resolution);
    }

    cv::Mat Map::MreadImage(int width,int height,vector<int> data)
    {
        cv::Mat mImg( width, height, CV_32S ) ;

        uchar pv[data.size()];
        for(unsigned int i = 0; i < data.size(); i++) {
            pv[i] = (uchar) data.at(i);
        }

        memcpy(mImg.data, &pv, data.size()*sizeof(uchar));

        return mImg;
    }

    // solution proposed https://codeday.me/es/qa/20190427/576956.html
    cv::Mat Map::MreadImage(std::ifstream& input)
    {
        input.seekg(0, std::ios::end);
        size_t fileSize = input.tellg();
        input.seekg(0, std::ios::beg);

        if (fileSize == 0) {
            return cv::Mat();
            ROS_FATAL("[Map.cpp@MreadImage] Image input stream size is 0.");
        }

        std::vector<unsigned char> data(fileSize);
        input.read( reinterpret_cast<char*>(&data[0]), sizeof(unsigned char) * fileSize);

        if (!input) {
            return cv::Mat();
            ROS_FATAL("[Map.cpp@MreadImage] Cant cast input data");
        }

        cv::Mat mImg = cv::imdecode(cv::Mat(data), CV_LOAD_IMAGE_UNCHANGED);

        return mImg;
    }

    void Map::createGrid(float resolution)
    {
      // change gridmap resolution from map_resolution to gridResolution
      ROS_DEBUG("[Map.cpp@createGrid] creating grid with resolution %3.3f",resolution);

      GridMapCvProcessing::changeResolution(map_grid_, nav_grid_, resolution);
      Map::numGridRows = nav_grid_.getSize()(0);
      Map::numGridCols = nav_grid_.getSize()(1);
    }

    void Map::createPathPlanningGrid(float resolution)
    {
      // change gridmap resolution from map_resolution to gridResolution
      ROS_DEBUG("[Map.cpp@createGrid] creating planning grid with resolution %3.3f",resolution);

      GridMapCvProcessing::changeResolution(map_grid_, planning_grid_, resolution);
      Map::numPathPlanningGridRows = planning_grid_.getSize()(0);
      Map::numPathPlanningGridCols = planning_grid_.getSize()(1);
    }

    void Map::createRFIDGrid(float resolution)
    {
      // change gridmap resolution from map_resolution to gridResolution
      ROS_DEBUG("[Map.cpp@createGrid] creating RFID grid with resolution %3.3f",resolution);

      GridMapCvProcessing::changeResolution(map_grid_, rfid_grid_, resolution);
      rfid_grid_["layer"].setConstant(0.0);
    }

   void Map::updatePathPlanningGrid(int cellX_pp, int cellY_pp, int rangeInCells_pp, double power)
   {
           int minX_pp = cellX_pp - rangeInCells_pp;
           int maxX_pp = cellX_pp + rangeInCells_pp;
           if(minX_pp < 0) minX_pp = 0;
           if(maxX_pp > numPathPlanningGridRows - 1) maxX_pp = numPathPlanningGridRows - 1;
           int minY_pp = cellY_pp - rangeInCells_pp;
           int maxY = cellY_pp + rangeInCells_pp;
           if(minY_pp < 0) minY_pp = 0;
           if(maxY > numPathPlanningGridCols - 1) maxY = numPathPlanningGridCols - 1;

           grid_map::Index planningStartIndex(minX_pp, minX_pp);
           grid_map::Index planningBufferSize(2*rangeInCells_pp, 2*rangeInCells_pp);

           Position position_pp, upper_left_pp;

           // this is the number of navigation cells inside a planning cell
           grid_map::Index navBufferSize(gridToPathGridScale, gridToPathGridScale);

           grid_map::Index navStartIndex,rfid_index;

           int countScanned = 0;
           int setToOne = 0;

           // iterate over the submap in the planning grid (lower res)
           for (grid_map::SubmapIterator planning_iterator(planning_grid_, planningStartIndex, planningBufferSize);
                 !planning_iterator.isPastEnd(); ++planning_iterator) {

                      //get the centre of the current planning cell
                      planning_grid_.getPosition(*planning_iterator, position_pp);

                      countScanned = 0;
                      setToOne = 0;

                      // obtain the position of the upper-left  navigation cell INSIDe current planning cell
                      upper_left_pp.x()=position_pp.x() - gridToPathGridScale/2;
                      upper_left_pp.y()=position_pp.y() - gridToPathGridScale/2;
                      // and corresponding index in nav_grid_
                      nav_grid_.getIndex(upper_left_pp, navStartIndex);

                      for (grid_map::SubmapIterator nav_iterator(nav_grid_, navStartIndex, navBufferSize);
                              !nav_iterator.isPastEnd(); ++nav_iterator) {

                                   if( nav_grid_.at("layer", *nav_iterator) == 1)
                                   {
                                       setToOne = 1;
                                   }

                                   if( nav_grid_.at("layer", *nav_iterator) == 2)
                                   {
                                       countScanned++;
                                   }
                      }
                      if(countScanned == gridToPathGridScale*gridToPathGridScale)
                      {
                        planning_grid_.at("layer", *planning_iterator)=2;
                        rfid_grid_.atPosition("layer", position_pp)+=power;
                      }
                      if(setToOne == 1)
                        planning_grid_.at("layer", *planning_iterator)=1;
          }
      }

    int Map::getPathPlanningGridValue(long i,long j) const
    {
        grid_map::Index index(i,j);
        double val = planning_grid_.at("layer", index);
        return (int) val;
    }

    void Map::setPathPlanningGridValue(int value, int i, int j)
    {
      grid_map::Index index(i,j);
      planning_grid_.at("layer", index)=value;
    }

    int Map::getPathPlanningNumCols() const
    {
        return numPathPlanningGridCols;
    }

    int Map::getPathPlanningNumRows() const
    {
        return numPathPlanningGridRows;
    }

    int Map::getGridToPathGridScale() const
    {
      return gridToPathGridScale;
    }

    void Map::createNewMap()
    {
      storeBinary("/tmp/test.pgm");
      storeAscii("/tmp/freeCell.txt");
    }

  void Map::storeBinary(std::string fileURI) {
      std::ofstream imgNew(fileURI.c_str(), ios::out);

      long columns = numGridCols;
      long rows = numGridRows;

      imgNew << "P2\n" << columns << " " << rows << "\n255\n";

      for(long row = 0; row < rows; ++row)
      {
          for(long col = 0; col < columns; ++col)
          {
              if(getGridValue(row,col) == 0)
              {
                  imgNew<<  255 << " ";
              }
              else
              {
                  imgNew <<  0 << " ";
              }
          }
      }
      imgNew.close();


    }

  void Map::storeAscii(std::string fileURI) {
        std::ofstream txt(fileURI.c_str());
        long columns = numGridCols;
        long rows = numGridRows;

        for(long row = 0; row < rows; ++row)
        {
            for(long col = 0; col < columns; ++col)
            {
                if(getGridValue(row,col) == 0) {
                    txt <<  col << ": " << row << endl;
                }
            }
        }
        txt.close();
      }


    void Map::setGridValue(int value, long i, long j)
    {
      grid_map::Index index(i,j);
      nav_grid_.at("layer", index)=value;
    }

    void Map::addEdgePoint(int x, int y)
    {
      std::pair<int,int> pair(x,y);
      edgePoints.push_back(pair);
    }

    std::vector<vector<long> > Map::getMap2D(){
      vector<vector<long> > map2D;

      for (long gridRow = 0; gridRow < numGridRows; ++gridRow)
      {
          for (long gridCol = 0; gridCol < numGridCols; ++gridCol)
          {
              map2D[gridRow].at(gridCol) = getGridValue(gridRow,gridCol);
          }
      }

      return map2D;

    }

    int Map::getGridValue(long i,long j) const
    {
      grid_map::Index index(i,j);
      double val = nav_grid_.at("layer", index);
      return (int) val;
    }

    int Map::getMapValue(long i, long j)
    {
      grid_map::Index index(i,j);
      double val = map_grid_.at("layer", index);
      return (int) val;
    }

    long Map::getNumGridCols() const
    {
        return numGridCols;
    }

    long Map::getNumGridRows() const
    {
        return numGridRows;
    }

    long Map::getNumCols()
    {
        return numCols;
    }

    long Map::getNumRows()
    {
        return numRows;
    }

    int dummy::Map::getGridValue(long i) const
    {
      long rows = getNumGridRows();
      std::ldiv_t result = std::div(i , rows);
      rows = result.quot;
      long cols = result.rem;

      grid_map::Index index(rows,cols);
      double val = nav_grid_.at("layer", index);
      return (int) val;
    }

    Pose Map::getRobotPosition()
    {
        return currentPose;
    }

    void Map::setCurrentPose(Pose& p)
    {
        currentPose = p;
    }

    long Map::getTotalFreeCells()
    {
      long columns = getNumGridCols();
      long rows = getNumGridRows();
      totalFreeCells = columns * rows;
      for(long row = 0; row < rows; ++row)
      {
          for(long col = 0; col < columns; ++col)
          {
              if(getGridValue(row,col) == 1) totalFreeCells--;
          }
      }
      return totalFreeCells;
    }

    void Map::decreaseFreeCells()
    {
        totalFreeCells--;
    }

    void Map::drawVisitedCells()
    {
      storeBinary("/tmp/result.pgm");
    }

    void Map::printVisitedCells(vector< string >& history)
    {
      std::ofstream txt("/tmp/finalResult.txt");
      for (int i =0 ; i < history.size(); i++){
          string encoding = history.at(i);
          string s ;
          stringstream ss;
          ss << s;
          char delimiter('/');
          string x,y,orientation,r,phi;
          std::string::size_type pos = encoding.find('/');
          x = encoding.substr(0,pos);
          int xValue = atoi(x.c_str());
          string newString = encoding.substr(pos+1,encoding.size());
          std::string::size_type newPos = newString.find('/');
          y = newString.substr(0,newPos);
          int yValue = atoi(y.c_str());
          newString = newString.substr(newPos+1,encoding.size());
          newPos = newString.find('/');
          orientation = newString.substr(0,newPos);
          int orientationValue = atoi(orientation.c_str());
          newString = newString.substr(newPos+1,encoding.size());
          newPos = newString.find('/');
          r = newString.substr(0,newPos);
          int rValue = atoi(r.c_str());
          newString = newString.substr(newPos+1,encoding.size());
          newPos = newString.find('/');
          phi = newString.substr(0,newPos);
          double phiValue = atoi(phi.c_str());

          txt << xValue << "  " << yValue << " " << orientationValue << " " << r <<endl;
      }

    }

    void Map::drawRFIDScan()
    {
        std::ofstream resultMap("/tmp/rfid_result.pgm", ios::out);
        long columns = numPathPlanningGridCols;
        long rows = numPathPlanningGridRows;

        resultMap << "P2\n" << columns << " " << rows << "\n255\n";

        for(long row = 0; row < rows; ++row)
        {
            for(long col = 0; col < columns; ++col)
            {
                int value = getRFIDGridValue(row, col);
                value = std::min(value, 255);
                value = std::max(value, 0);
                resultMap << value  << " ";
            }
        }
        resultMap.close();
    }

    void Map::drawRFIDGridScan(RFIDGridmap grid)
    {
      std::ofstream resultMap("/tmp/rfid_result_gridmap.pgm", ios::out);
      long columns = numPathPlanningGridCols;
      long rows = numPathPlanningGridRows;

      resultMap << "P2\n" << columns << " " << rows << "\n255\n";

      for(long row = 0; row < rows; ++row)
      {
          for(long col = 0; col < columns; ++col)
          {
              int value = static_cast<int>(grid.getCell(row, col));
//     cout << value << endl;
              value = std::min(value, 255);
              value = std::max(value, 0);
              resultMap << value  << " ";
          }
      }

      resultMap.close();
    }


    std::pair<int, int> Map::getRelativeTagCoord(int absTagX, int absTagY, int antennaX, int antennaY)
    {
       std::pair<int, int> relTagCoord;
       relTagCoord.first = std::abs(absTagX - antennaX);
       relTagCoord.second = std::abs(absTagY - antennaY);
       return relTagCoord;
    }

    std::pair<int, int> Map::findTag()
    {
       std::pair<int,int> tag(0,0);
       double powerRead = 0;
       int numRFIDRows = rfid_grid_.getSize()(0);
       int numRFIDCols = rfid_grid_.getSize()(1);

       for(int row=0; row < numRFIDRows; row++)
       {
           for(int col=0; col < numRFIDCols; col++)
           {
               if(getRFIDGridValue(row, col) > powerRead)
               {
                   powerRead = getRFIDGridValue(row, col);
                   tag.first = row;
                   tag.second = col;
               }
           }
       }
       return tag;
    }

    std::pair<int, int> Map::findTagfromGridMap(RFIDGridmap grid)
    {
       std::pair<int,int> tag(0,0);
       double powerRead = 0;
       int numRFIDRows = grid.getNumCols();
       int numRFIDCols = grid.getNumCols();

       for(int row=0; row < numRFIDRows; row++)
       {
           for(int col=0; col < numRFIDCols; col++)
           {
               if(grid.getCell(row, col) > powerRead)
               {
                   powerRead = getRFIDGridValue(row, col);
    //        cout << "Value read: " << powerRead << endl;
                   tag.first = row;
                   tag.second = col;
               }
           }
       }
       return tag;
    }

    int Map::getRFIDGridValue(long i,long j) const
    {
      grid_map::Index index(i,j);
      double val = rfid_grid_.at("layer", index);
      return (int) val;
    }

    void Map::setRFIDGridValue(float power, int i, int j)
    {
       grid_map::Index index(i,j);
       rfid_grid_.at("layer", index) += power;
    }

    string Map::type2str(int type)
    {
        string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
        }

        r += "C";
        r += (chans+'0');

        return r;
    }

}
