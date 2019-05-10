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

    Map::Map(float plan_resolution, float map_resolution, int width, int height, vector<int> data, geometry_msgs::Pose origin):map_grid_(vector<string>({"layer"})), nav_grid_(vector<string>({"layer"})), planning_grid_(vector<string>({"layer"})), rfid_grid_(vector<string>({"layer"}))
    {
      Map::createMap(width, height, map_resolution, data, origin);
      plotMyGrid("/tmp/initial_map.pgm", &map_grid_);
      Map::createGrid(map_resolution);
      plotMyGrid("/tmp/initial_nav.pgm", &nav_grid_);
      Map::createNewMap();

      Map::createPathPlanningGrid(plan_resolution);
      plotMyGrid("/tmp/initial_planning.pgm", &planning_grid_);

      // once both are created, we can obtain the ratio
      gridToPathGridScale = planning_grid_.getResolution()/nav_grid_.getResolution();
      ROS_ASSERT_MSG(gridToPathGridScale >= 1, "[Map.cpp@map] Planning resolution lower than navigation resolution = %d", gridToPathGridScale);


      Map::createRFIDGrid(map_resolution);
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
      ROS_WARN("[Map.cpp@createMap] MAP RESOLUTION MUST BE 0.1 m/cell !!!.");

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

      imageCV = binarizeImage(imageCV);

      // grid size in pixels
      Map::numRows = imageCV.rows;
      Map::numCols = imageCV.cols;

      // map origin in rangeInMeters
      orig_x=origin.position.x+imageCV.rows*map_resolution/2;
      orig_y=origin.position.y+imageCV.cols*map_resolution/2;

      ROS_DEBUG("[Map.cpp@createMap] Image size [%lu, %lu]",numRows, numCols);

      cv::minMaxLoc(imageCV, &minValue, &maxValue);
      ROS_DEBUG("[Map.cpp@createMap] Min, max values [%3.3f %3.3f]",minValue, maxValue);
      ROS_DEBUG("[Map.cpp@createMap] Channels [%d]",imageCV.channels());
      ROS_DEBUG("[Map.cpp@createMap] Encoding [%s]",type2str(imageCV.type()).c_str() );
      ROS_DEBUG("[Map.cpp@createMap] Map resolution [%3.3f]",map_resolution);
      ROS_DEBUG("[Map.cpp@createMap] Map resolution [%3.3f]",map_resolution);

      // create empty grid map
      ROS_DEBUG("[Map.cpp@createMap] Creating empty grid");

      grid_map::GridMap tempMap(vector<string>({"layer"}));
      // map lenth MUST be given in meters!
      tempMap.setGeometry(Length(numRows*map_resolution, numCols*map_resolution), map_resolution, Position(orig_x, orig_y));
      tempMap.setFrameId(map_frame_id);
      tempMap.clearAll();

      // Convert cv image to grid map.
      ROS_DEBUG("[Map.cpp@createMap] Storing cv mat into emtpy grid");
      string format=("mono8");
      sensor_msgs::ImagePtr imageROS = cv_bridge::CvImage(std_msgs::Header(), format, imageCV).toImageMsg();
      GridMapRosConverter::addLayerFromImage(*imageROS, "layer", tempMap);

      map_grid_ = tempMap;
      printGridData("map", &map_grid_ );
    }

    void Map::createMap(int width,int height, double resolution , vector<int> data, geometry_msgs::Pose origin)
    {
      // load an image from vector
      ROS_DEBUG("[Map.cpp@createMap] Loading vector into cv mat.");
      cv::Mat imageCV = MreadImage(width,height,data);

      //TODO : CHECK THIS!!!
      // harcoded values!!!!
      std::string map_frame_id="map";

      createMap(imageCV,origin,map_frame_id, resolution);
    }

    cv::Mat Map::MreadImage(int width,int height,vector<int> data)
    {
        cv::Mat mImg( width, height, CV_8UC1 ) ;

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

    cv::Mat Map::binarizeImage(cv::Mat mImg)
    {


            // how many different values does it have?
            std::vector<uchar> diffValues;
            for (int y = 0; y < mImg.rows; ++y)
            {
                const uchar* row_ptr = mImg.ptr<uchar>(y);
                for (int x = 0; x < mImg.cols; ++x)
                {
                    uchar value = row_ptr[x];

                    if ( std::find(diffValues.begin(), diffValues.end(), value) == diffValues.end() ){
                        diffValues.push_back(value);
                        ROS_DEBUG("[Map.cpp@binarize] Map val 0x%x",value);
                    }
                }
            }


      //Apply thresholding to binarize!
      //cv::adaptiveThreshold(mImg, mImg, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,3,5);
      cv::threshold(mImg, mImg,99,255,cv::THRESH_BINARY);
      diffValues.clear();
      // how many different values does it have?
      for (int y = 0; y < mImg.rows; ++y)
      {
          const uchar* row_ptr = mImg.ptr<uchar>(y);
          for (int x = 0; x < mImg.cols; ++x)
          {
              uchar value = row_ptr[x];

              if ( std::find(diffValues.begin(), diffValues.end(), value) == diffValues.end() )
                  diffValues.push_back(value);
          }
      }

      if (diffValues.size()!=2)
      {
          ROS_ERROR("[Map.cpp@createMap] [%lu] different values: ",diffValues.size());
          for (int i = 0; i < diffValues.size(); i++)
          {
            ROS_ERROR("[Map.cpp@createMap] \t\t [0x%x]==[%d] ",diffValues[i],diffValues[i]);
          }
      }
      return mImg;

    }

    void Map::createGrid(float resolution)
    {
      // change gridmap resolution from map_resolution to gridResolution
      ROS_DEBUG("[Map.cpp@createGrid] creating grid with resolution %3.3f",resolution);

      GridMapCvProcessing::changeResolution(map_grid_, nav_grid_, resolution);

      //cv::resize(src, dst, Size(), 0.5, 0.5, cv::CV_INTER_AREA);


      Map::numGridRows = nav_grid_.getSize()(0);
      Map::numGridCols = nav_grid_.getSize()(1);
      printGridData("nav",&nav_grid_ );
    }

    void Map::createPathPlanningGrid(float resolution)
    {
      // change gridmap resolution from map_resolution to gridResolution
      ROS_DEBUG("[Map.cpp@createGrid] creating planning grid with resolution %3.3f",resolution);

      GridMapCvProcessing::changeResolution(map_grid_, planning_grid_, resolution);
      Map::numPathPlanningGridRows = planning_grid_.getSize()(0);
      Map::numPathPlanningGridCols = planning_grid_.getSize()(1);
      printGridData("plan",&planning_grid_ );
    }

    void Map::createRFIDGrid(float resolution)
    {
      // change gridmap resolution from map_resolution to gridResolution
      ROS_DEBUG("[Map.cpp@createGrid] creating RFID grid with resolution %3.3f",resolution);

      GridMapCvProcessing::changeResolution(map_grid_, rfid_grid_, resolution);
      rfid_grid_["layer"].setConstant(0.0);
      printGridData("rfid",&rfid_grid_ );
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
//                        std::cout << "[Map.cpp@updatePathPlanningGrid] Inside : " << nav_grid_.at("layer", *nav_iterator) << endl;

                                   if( nav_grid_.at("layer", *nav_iterator) == 1)
                                   {
                                       setToOne = 1;
                                     std::cout << "[Map.cpp@updatePathPlanningGrid] 1" << endl;
                                   }

                                   if( nav_grid_.at("layer", *nav_iterator) == 2)
                                   {
                                       countScanned++;
                                     std::cout << "[Map.cpp@updatePathPlanningGrid] 2" << endl;
                                   }
                      }
                      if(countScanned == gridToPathGridScale*gridToPathGridScale)
                      {
                        planning_grid_.at("layer", *planning_iterator)=2;
                        rfid_grid_.atPosition("layer", position_pp)+=power;
                        std::cout << "[Map.cpp@updatePathPlanningGrid] CountScanned" << endl;
                      }
                      if(setToOne == 1)
                      {
                        planning_grid_.at("layer", *planning_iterator)=1;
                        std::cout << "[Map.cpp@updatePathPlanningGrid] SetToOne" << endl;
                      }

          }
      }

    float Map::getGridToPathGridScale() const
    {
      return planning_grid_.getResolution()/nav_grid_.getResolution();
    }

    // Getter shortcuts ........................................................
    bool  Map::getPathPlanningPosition(double &x, double &y, long i, long j)
    {
      return getPosition(x, y,  i, j, &planning_grid_);
    }

    bool  Map::getPathPlanningIndex(double x, double y, long &i, long &j)
    {
      return getIndex(x, y,  i, j, &planning_grid_);
    }

    int Map::getPathPlanningGridValue(long i,long j) const
    {
        return getValue(i,j, &planning_grid_);
    }

    int  Map::getPathPlanningGridValue(geometry_msgs::PoseStamped ps) const
    {
      return getValue(ps, &planning_grid_);
    }

    int Map::getPathPlanningGridValue(long i) const
    {
        return getValue(i, &planning_grid_);
    }

    int Map::getPathPlanningNumCols() const
    {
        return getGridNumCols(&planning_grid_);
    }

    int Map::getPathPlanningNumRows() const
    {
        return getGridNumRows(&planning_grid_);
    }

    bool  Map::getGridPosition(double &x, double &y, long i, long j)
    {
      return getPosition(x, y,  i, j, &nav_grid_);
    }

    bool  Map::getGridIndex(double x, double y, long &i, long &j)
    {
      return getIndex(x, y,  i, j, &nav_grid_);
    }


    int Map::getGridValue(long i,long j) const
    {
      return getValue(i,j, &nav_grid_);
    }

    int  Map::getGridValue(geometry_msgs::PoseStamped ps) const
    {
      return getValue(ps, &nav_grid_);
    }

    int Map::getGridValue(long i) const
    {
        return getValue(i, &nav_grid_);
    }

    long Map::getNumGridCols() const
    {
        return getGridNumCols(&nav_grid_);
    }

    long Map::getNumGridRows() const
    {
        return getGridNumRows(&nav_grid_);
    }

    bool  Map::getMapPosition(double &x, double &y, long i, long j)
    {
      return getPosition(x, y,  i, j, &map_grid_);
    }

    bool  Map::getMapIndex(double x, double y, long &i, long &j)
    {
      return getIndex(x, y,  i, j, &map_grid_);
    }

    int Map::getMapValue(long i, long j)
    {
      return getValue(i,j, &map_grid_);
    }

    int  Map::getMapValue(geometry_msgs::PoseStamped ps) const
    {
      return getValue(ps, &map_grid_);
    }

    int Map::getMapValue(long i) const
    {
        return getValue(i, &map_grid_);
    }

    long Map::getNumCols()
    {
        return getGridNumCols(&map_grid_);
    }

    long Map::getNumRows()
    {
        return getGridNumRows(&map_grid_);
    }

    bool  Map::getRFIDPosition(double &x, double &y, long i, long j)
    {
      return getPosition(x, y,  i, j, &rfid_grid_);
    }

    bool  Map::getRFIDIndex(double x, double y, long &i, long &j)
    {
      return getIndex(x, y,  i, j, &rfid_grid_);
    }

    int Map::getRFIDGridValue(long i, long j) const
    {
      return getValue(i,j, &rfid_grid_);
    }

    int Map::getRFIDGridValue(geometry_msgs::PoseStamped ps) const
    {
      return getValue(ps, &rfid_grid_);
    }

    int Map::getRFIDGridValue(long i) const
    {
        return getValue(i, &rfid_grid_);
    }

    long Map::getRFIDGridNumCols()
    {
        return getGridNumCols(&rfid_grid_);
    }

    long Map::getRFIDGridNumRows()
    {
        return getGridNumRows(&map_grid_);
    }

    // Generic (internal) getters ..............................................
    bool  Map::getIndex(double x, double y, long &i, long &j, grid_map::GridMap *gm){
      grid_map::Index resIndex;
      Position inPose(x,y);
      bool success;
      if (gm->isInside(inPose))
      {
        gm->getIndex(inPose, resIndex);
        i=resIndex(0);
        j=resIndex(1);
        success=true;
      }
      else
      {
        printErrorReason(inPose,gm);
        i=-1;
        j=-1;
        success=false;
      }
      return success;
    }

    bool  Map::getPosition(double &x, double &y, long i, long j, grid_map::GridMap *gm){
      grid_map::Index inIndex(i,j);
      Position inPose;
      bool success;
      if (gm->getPosition(inIndex, inPose))
      {
        x=inPose.x();
        y=inPose.y();
        success=true;
      }
      else
      {
        ROS_ERROR("Requested [%lu, %lu] index is outside grid boundaries: [0,0] - [%d, %d]", i,j,gm->getSize()(0)-1,gm->getSize()(1)-1);
        x=std::numeric_limits<double>::max();
        y=std::numeric_limits<double>::max();
        success=false;
      }
      return success;
    }



    int  Map::getValue(long i,long j, const grid_map::GridMap *gm) const
    {
      int val;
      grid_map::Index index(i,j);
      Position position;

      if (gm->getPosition(index, position))
      {
        val = (int) gm->at("layer", index);
      }
      else
      {
        ROS_ERROR("Requested [%lu, %lu] index is outside grid boundaries: [0,0] - [%d, %d]", i,j,gm->getSize()(0)-1,gm->getSize()(1)-1);
        val = -1;
      }
      return val;
    }

    int  Map::getValue(geometry_msgs::PoseStamped ps, const grid_map::GridMap *gm) const
    {
        double val;
        if (ps.header.frame_id!=gm->getFrameId())
        {
          ROS_ERROR("Pose given in different frame id: grid==[%s], point ==[%s]",ps.header.frame_id.c_str(),gm->getFrameId().c_str() );
          val= -1;
        }
        else
        {
          grid_map::Position point(ps.pose.position.x,ps.pose.position.y);

          if (gm->isInside(point))
          {
            val = gm->atPosition("layer", point);
          }
          else
          {
            printErrorReason(point,gm);
            val = -1;

          }
        }
        return (int) val;
    }

    int Map::getValue(long i, const  grid_map::GridMap *gm) const
    {
      long rows = getGridNumRows(gm);
      std::ldiv_t result = std::div(i , rows);
      rows = result.quot;
      long cols = result.rem;

      return getValue(rows,cols, gm);
    }

    int Map::getGridNumCols(const grid_map::GridMap *gm) const
    {
      return gm->getSize()(1);
    }

    int Map::getGridNumRows(const grid_map::GridMap *gm) const
    {
      return gm->getSize()(0);
    }
    // E.O. Generic (internal) getters .........................................


    // Generic (internal) setters ..............................................
    void Map::setValue(int value, long i,long j, grid_map::GridMap *gm)
    {
      grid_map::Index index(i,j);
      Position position;

      if (gm->getPosition(index, position))
      {
        (gm->at("layer", index)) = value;
      }
      else
      {
        ROS_ERROR("Requested [%lu, %lu] index is outside grid boundaries: [0,0] - [%d, %d]", i,j,gm->getSize()(0)-1,gm->getSize()(1)-1);
      }
    }

    void  Map::setValue(int value, geometry_msgs::PoseStamped ps, grid_map::GridMap *gm)
    {
        if (ps.header.frame_id!=gm->getFrameId())
        {
          ROS_ERROR("Pose given in different frame id: grid==[%s], point ==[%s]",ps.header.frame_id.c_str(),gm->getFrameId().c_str() );
        }
        else
        {
          grid_map::Position point(ps.pose.position.x,ps.pose.position.y);

          if (gm->isInside(point))
          {
            gm->atPosition("layer", point)=value;
          }
          else
          {
            printErrorReason(point,gm);
          }
        }
    }

    void Map::setValue(int value, long i, grid_map::GridMap *gm)
    {
      long rows = getGridNumRows(gm);
      std::ldiv_t result = std::div(i , rows);
      rows = result.quot;
      long cols = result.rem;

      setValue(value, rows,cols,gm);
    }

    // E.O. Generic (internal) setters .........................................


    void Map::setPathPlanningGridValue(int value, int i, int j)
    {
      setValue( value,  i, j, &planning_grid_);
    }

    void Map::setPathPlanningGridValue(int value, geometry_msgs::PoseStamped ps)
    {
      setValue( value, ps, &planning_grid_);
    }

    void Map::setPathPlanningGridValue(int value,long i)
    {
      setValue( value, i, &planning_grid_);
    }

    void Map::setGridValue(int value, long i, long j)
    {
      setValue( value,  i, j, &nav_grid_);
    }

    void Map::setGridValue(int value, geometry_msgs::PoseStamped ps)
    {
      setValue( value, ps, &nav_grid_);
    }

    void Map::setGridValue(int value,long i)
    {
      setValue( value, i, &nav_grid_);
    }

    void Map::setRFIDGridValue(float power, int i, int j)
    {
       setValue( getValue(i,j, &rfid_grid_) + power,  i, j, &rfid_grid_);
    }

    void Map::setRFIDGridValue(float power, geometry_msgs::PoseStamped ps)
    {
      setValue( getValue(ps, &rfid_grid_) + power,  ps, &rfid_grid_);
    }

    void Map::setRFIDGridValue(float power,long i)
    {
      setValue( getValue(i, &rfid_grid_) + power,  i, &rfid_grid_);
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
       int numRFIDRows = grid.getNumRows();
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

    void Map::printErrorReason(grid_map::Position point, const grid_map::GridMap *gm) const
    {
      int maxRow = gm->getSize()(0)-1;
      int maxCol = gm->getSize()(1)-1;
      grid_map::Index topLeftI(0,0);
      grid_map::Index topRightI(0,maxCol);
      grid_map::Index bottomRightI(maxRow,maxCol);
      grid_map::Index bottomLeftI(maxRow,0);

      grid_map::Position topLeftP,topRightP,bottomRightP,bottomLeftP;
      gm->getPosition(topLeftI, topLeftP);
      gm->getPosition(topRightI, topRightP);
      gm->getPosition(bottomRightI, bottomRightP);
      gm->getPosition(bottomLeftI, bottomLeftP);
      ROS_ERROR("Point [%3.3f, %3.3f] is outside grid boundaries: grid: [%3.3f, %3.3f], [%3.3f, %3.3f], [%3.3f, %3.3f], [%3.3f, %3.3f]",point.x(),point.y(), topLeftP.x(),topLeftP.y(),  topRightP.x(),topRightP.y(),  bottomRightP.x(),bottomRightP.y(),  bottomLeftP.x(),bottomLeftP.y()  );
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

    void Map::printGridData(std::string grid_name , const grid_map::GridMap *gm )
    {
      int nRow = gm->getSize()(0);
      int nCol = gm->getSize()(1);
      grid_map::Index topLeftI(0,0);
      grid_map::Index topRightI(0,nCol-1);
      grid_map::Index bottomRightI(nRow-1,nCol-1);
      grid_map::Index bottomLeftI(nRow-1,0);

      grid_map::Position topLeftP,topRightP,bottomRightP,bottomLeftP;
      gm->getPosition(topLeftI, topLeftP);
      gm->getPosition(topRightI, topRightP);
      gm->getPosition(bottomRightI, bottomRightP);
      gm->getPosition(bottomLeftI, bottomLeftP);

      ROS_DEBUG("[Map.cpp@printGridData] Grid Name: [%s]",grid_name.c_str());
      ROS_DEBUG("[Map.cpp@printGridData] Num Cols, Num Rows [%d,  %d]",nCol,nRow);
      ROS_DEBUG("[Map.cpp@printGridData] Frame id [%s]",gm->getFrameId().c_str());
      ROS_DEBUG("[Map.cpp@printGridData] Grid Center [%3.3f %3.3f] m. ", gm->getPosition().x(),gm->getPosition().y());
      ROS_DEBUG("[Map.cpp@printGridData] Resolution [%3.3f] m./cell ", gm->getResolution());
      ROS_DEBUG("[Map.cpp@printGridData] Map Size [%3.3f x %3.3f] m.", gm->getLength()(0),gm->getLength()(1)  );
      ROS_DEBUG("[Map.cpp@printGridData] Bounding box positions (m.):");
      ROS_DEBUG(" \t\t topLeft cell: [%d, %d] == [%3.3f, %3.3f] m.",topLeftI.x(),topLeftI.y() ,topLeftP.x(),topLeftP.y() );
      ROS_DEBUG(" \t\t topRight cell: [%d, %d] == [%3.3f, %3.3f] m.", topRightI.x(), topRightI.y(), topRightP.x(), topRightP.y()  );
      ROS_DEBUG(" \t\t bottomRight cell: [%d, %d] == [%3.3f, %3.3f] m.", bottomRightI.x(),bottomRightI.y(), bottomRightP.x(),bottomRightP.y()  );
      ROS_DEBUG(" \t\t bottomLeft cell: [%d, %d] == [%3.3f, %3.3f] m.",bottomLeftI.x(),bottomLeftI.y(),bottomLeftP.x(),bottomLeftP.y()  );

    }

    void Map::plotMyGrid(std::string fileURI, const grid_map::GridMap * gm)
    {

      cv::Mat imageCV;
      double  minValue=1e20;
      double  maxValue=-1e20;
      sensor_msgs::ImagePtr imageROS;
      cv_bridge::CvImagePtr cv_ptr;


      for (grid_map::GridMapIterator iterator(*gm); !iterator.isPastEnd(); ++iterator) {

          if (gm->at("layer",*iterator)>maxValue)
            maxValue=gm->at("layer",*iterator);
          if (gm->at("layer",*iterator)<minValue)
              minValue=gm->at("layer",*iterator);
      }
      ROS_DEBUG("Ranges [%3.3f, %3.3f]",minValue,maxValue);
      GridMapCvConverter::toImage<unsigned short, 1>(*gm, "layer", CV_16UC1, minValue, maxValue, imageCV);
      cv::imwrite(fileURI.c_str(), imageCV);

    }
}
