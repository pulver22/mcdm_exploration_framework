#include <map.h>
#include <fstream>
#include <iostream>
using namespace std;

namespace dummy{

    Map::Map(std::ifstream& infile, int resolution)
    {
        Map::createMap(infile);
        Map::createGrid(resolution);
        Map::createNewMap();
        //cout << "ciao" << endl;
    }

    Map::Map()
    {

    }

    Map::Map(float resolution, float costresolution, int width,int height,vector<int> data, geometry_msgs::Pose origin){

        if(resolution > 0 && resolution <= 1){
            //NOTE: to be used with 1mx1m cells
            for(int i = 0; i < width*height; ++i){
                map.push_back(0);
            }

            numCols = width;
            numRows = height;
            cout <<" numRows: " << numRows <<", numCols: "<< numCols << endl;
            //set 1 in the grid cells corrisponding to obstacles according to percentage value
            // contained in data vector
            for(long row = 0; row < height; ++row){
                for(long col = 0; col < width; ++col){

                    //if(map[row*numCols + col] == 0)

                    if(data.at(row*width + col) > 60) {
                        map[( height - row)*width + col] = 1;
                        //grid[(height - row)*width + col] = 1;


                    }

                    if(data.at(row*width + col) <20) {
                        map[(height - row)*width + col] = 0;
                        //grid[(height - row)*width + col] = 0;

                    }

                    if(data.at(row*width + col) < 0) {
                        map[(height - row)*width + col] = 1;
                        //grid[(height - row)*width + col] = 1;

                    }

                }
            }


            //int tmpRes = resolution * 100;
            Map::createGrid(resolution);
            ROS_DEBUG("[Map.cpp@map] Grid created");
            Map::createPathPlanningGrid(costresolution);
            ROS_DEBUG("[Map.cpp@map] Planning Grid created");
            Map::createNewMap();
            ROS_DEBUG("[Map.cpp@map] Map created");

        }else if(resolution == 0){

            //NOTE: to be used with default resolution
            for(int i = 0; i < width*height; ++i){
                grid.push_back(0);
            }

            numCols = width;
            numRows = height;
            numGridCols = width;
            numGridRows = height;
            //set 1 in the grid cells corrisponding to obstacles according to percentage value
            // contained in data vector
            for(long row = 0; row < height; ++row){
                for(long col = 0; col < width; ++col){



                    if(data.at(row*width + col) > 60) {
                        grid[(height - row)*width + col] = 1;
                    }

                    if(data.at(row*width + col) <20) {
                        grid[(height - row)*width + col] = 0;
                    }

                    if(data.at(row*width + col) < 0) {
                        grid[(height - row)*width + col] = 1;
                    }

                }
            }

            Map::createNewMap();
            Map::createPathPlanningGrid(1.0);

        }
    }


// create a monodimensional vector containing the map
    void Map::createMap(std::ifstream& infile)
    {
        long row = 0, col = 0;
        std::stringstream ss;
        std::string inputLine = "";

        // First line : version
        getline(infile,inputLine);
        if(inputLine.compare("P2") != 0) {

            long rows, cols, size, greylevels;

            //Second line : comment
            char comment_char = infile.get();
            if(comment_char == '#') {
                getline(infile,inputLine);
            } else {
                //cout << "No comment in the header" << endl;
                ss << comment_char;
            }

            // Continue with a stringstream
            ss << infile.rdbuf();
            // Third line : size
            ss >> numCols >> numRows >> greylevels;
            // cout << numCols << " columns and " << numRows << " rows" << endl;

            size = numCols * numRows;


            long* data = new long[size];
            for(long* ptr = data; ptr < data+size; ptr++) {
                // read in binary char
                unsigned char t_ch = ss.get();
                // convert to integer
                long t_data = static_cast<long>(t_ch);
                // if passes add value to data array
                *ptr = t_data;
            }
            // close the stream
            infile.close();

            Map::map.reserve(numRows*numCols);

            // Following lines : data

            for(long* ptr = data, i = 0; ptr < data+size; ptr++, i++) {

                map[i] = *ptr ;
                //cout << map[i];
            }
            delete data;

        }else{


            //Second line : comment
            char comment_char = infile.get();
            if(comment_char == '#') {
                getline(infile,inputLine);
            } else {
                //cout << "No comment in the header" << endl;
                ss << comment_char;
            }
            //Following lines not useful anymore
            //getline(infile,inputLine);
            //cout << "Comment : " << inputLine << endl;

            // Continue with a stringstream
            ss << infile.rdbuf();
            // Third line : size
            ss >> numCols >> numRows;
            // cout << numCols << " columns and " << numRows << " rows" << endl;

            //max value of color
            getline(infile,inputLine);


            //std::vector<int> array(numRows*numCols);
            Map::map.reserve(numRows*numCols);

            // Following lines : data
            for(row = 0; row < numRows; ++row)
                for (col = 0; col < numCols; ++col) ss >> Map::map[row*numCols + col];

            //infile.close();
        }
    }

// Create a grid 1mx1m
    void Map::createGrid(float resolution)
    {
        //cluster cells into grid
        float clusterSize = (float)((1/resolution));
        Map::numGridRows = (long)numRows/clusterSize;
        Map::numGridCols = (long)numCols/clusterSize;
        cout <<" numGridRows: " << numGridRows <<", numGridCols: "<< numGridCols << endl;

        for(int i = 0; i < numGridCols*numGridRows; ++i)
        {
            grid.push_back(0);
        }

        //set 1 in the grid cells corrisponding to obstacles
        for(long row = 0; row < numRows; ++row)
        {
            for(long col = 0; col < numCols; ++col)
            {

                if(getMapValue(row,col) == 1)
                {
                    grid[(long)(row/clusterSize)*numGridCols + (long)(col/clusterSize)] = 1;
                    //NOTE: i don't remember when it should be used
                    //map[(long)(row/clusterSize)*numGridCols + (long)(col/clusterSize)] = 1;
                }
            }
        }


    }

    void Map::createPathPlanningGrid(float resolution)
    {
        //cluster cells into grid

        float clusterSize = (float)((1/resolution));
        //std::cout << "imgResolution: " << resolution << " clusterSize: " << clusterSize << std::endl;
        Map::numPathPlanningGridRows = (int)(numRows/clusterSize);
        Map::numPathPlanningGridCols = (int)(numCols/clusterSize);
        cout <<"numPathPlanningGridRows: " << numPathPlanningGridRows <<", numPathPlanningGridCols: "<< numPathPlanningGridCols << endl;

        for(int i = 0; i < numPathPlanningGridCols*numPathPlanningGridRows; ++i)
        {
            pathPlanningGrid.push_back(0);
        }

        //set 1 in the grid cells corrisponding to obstacles
        for(long row = 0; row < numRows; ++row)
        {
            for(long col = 0; col < numCols; ++col)
            {
                if(resolution == 1.0){
                    if(getGridValue(row,col) == 1)
                        pathPlanningGrid[(long)(row/clusterSize)*numPathPlanningGridCols + (long)(col/clusterSize)] = 1;
//                    cout << "Empty cell in pathPlanningGrid: " << row << ":" << col << endl;


                }else if(getMapValue(row,col) == 1){
                    pathPlanningGrid[(long)(row/clusterSize)*numPathPlanningGridCols + (long)(col/clusterSize)] = 1;
//                    cout << "Empty cell in pathPlanningGrid: " << row << ":" << col << endl;
                    //NOTE: i don't remember when it should be used
                    //map[(long)(row/clusterSize)*numGridCols + (long)(col/clusterSize)] = 1;
                }
            }
        }
        Map::gridToPathGridScale = (int)(numGridRows / numPathPlanningGridRows);


    }


    void Map::updatePathPlanningGrid(int posX, int posY, int rangeInMeters, double power)
    {

        //int ppX = (int)(posX/gridToPathGridScale);
        //int ppY = (int)(posY/gridToPathGridScale);
        int minX = posX - rangeInMeters;
        int maxX = posX + rangeInMeters;
        if(minX < 0) minX = 0;
        if(maxX > numPathPlanningGridRows - 1) maxX = numPathPlanningGridRows - 1;
        int minY = posY - rangeInMeters;
        int maxY = posY + rangeInMeters;
        if(minY < 0) minY = 0;
        if(maxY > numPathPlanningGridCols - 1) maxY = numPathPlanningGridCols - 1;

        for(int row = minX; row <= maxX; ++row)
        {
            for(int col = minY; col <= maxY; ++col)
            {
                int countScanned = 0;
                int setToOne = 0;
                for(int gridRow = row*gridToPathGridScale; gridRow < row*gridToPathGridScale + gridToPathGridScale; ++gridRow)
                {
                    for(int gridCol = col*gridToPathGridScale; gridCol < col*gridToPathGridScale + gridToPathGridScale; ++gridCol)
                    {
                        //std::cout << "X: " << row << " Y: " << col << " gridX: " << gridRow << " gridY: " << gridCol << std::endl;
                        if(getGridValue(gridRow, gridCol) == 1)
                        {
                            setToOne = 1;
                        }

                        if(getGridValue(gridRow, gridCol) == 2)
                        {
                            countScanned++;
                        }
                    }
                }
                if(countScanned == gridToPathGridScale*gridToPathGridScale)
                {
                    cout << "Setting as scanned cell (" << row << "," << col << ")" << endl;
                    setPathPlanningGridValue(2, row, col);
//                    setRFIDGridValue(power, row, col);
                }
                if(setToOne == 1) setPathPlanningGridValue(1, row, col);
            }
        }


    }

    int Map::getPathPlanningGridValue(long i,long j) const
    {
        return pathPlanningGrid[i*numPathPlanningGridCols + j];
    }

    void Map::setPathPlanningGridValue(int value, int i, int j)
    {
        pathPlanningGrid[i*numPathPlanningGridCols + j] = value;
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
        //cout << "ciao" << endl;
        std::ofstream imgNew("/home/pulver/Desktop/test.pgm", ios::out);

        //imgNew << "h"<<endl;

        std::ofstream txt("/home/pulver/Desktop/freeCell.txt");
        long columns = numGridCols;
        long rows = numGridRows;

        imgNew << "P2\n" << columns << " " << rows << "\n255\n";

        for(long row = 0; row < rows; ++row)
        {
            for(long col = 0; col < columns; ++col)
            {
                //if an obstacle is present put 255 as black
                if(getGridValue(row,col) == 0) {
                    imgNew<<  255 << " ";
                    txt <<  col << ": " << row << endl;
                }
                    //else put 0 as free cell
                else {
                    imgNew <<  0 << " ";

                }
            }
            //imgNew << "\n";
        }

        imgNew.close();
        txt.close();
    }



// values: 0 -> unscanned free cell, 1 -> obstacle cell, 2 -> scanned free cell
    void Map::setGridValue(int value, long i, long j)
    {
        if(value == 0 || value == 1 || value == 2)
        {
            grid[i*numGridCols + j] = value;
        }
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

//various getters

    int Map::getGridValue(long i,long j) const
    {
        return grid[i*numGridCols + j];
    }

    int Map::getMapValue(long i, long j)
    {
        return map[i*numCols + j];
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
        return Map::grid[i];
    }

    Pose Map::getRobotPosition()
    {
        return currentPose;
    }

    void Map::setCurrentPose(Pose& p)
    {
        currentPose = p;
    }


    long Map::getTotalFreeCells(){
        long columns = getNumGridCols();
        long rows = getNumGridRows();
        totalFreeCells = columns * rows;
        for(long row = 0; row < rows; ++row)
        {
            for(long col = 0; col < columns; ++col)
            {
                //if an obstacle is present put 255 as black
                if(getGridValue(row,col) == 1) totalFreeCells--;
            }
        }
        //cout << "Total free cells: " << totalFreeCells << endl;
        return totalFreeCells;
    }

    void Map::decreaseFreeCells(){
        this->totalFreeCells--;
    }

    void Map::drawVisitedCells()
    {
        std::ofstream resultMap("/home/pulver/Desktop/result.pgm", ios::out);
        long columns = numGridCols;
        long rows = numGridRows;

        resultMap << "P2\n" << columns << " " << rows << "\n255\n";

        for(long row = 0; row < rows; ++row)
        {
            for(long col = 0; col < columns; ++col)
            {
                //string encoding = to_string(col) + to_string(row);
                //if an obstacle is present put 255 as black
                if(getGridValue(row,col) == 0) {
                    resultMap<<  255 << " ";
                }
                    //else put 0 as free cell
                else {
                    resultMap <<  0 << " ";

                }
            }
            //imgNew << "\n";
        }

        resultMap.close();

    }

    void Map::printVisitedCells(vector< string >& history)
    {
        std::ofstream txt("/home/pulver/Desktop/finalResult.txt");
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
            //cout << x << endl;
            string newString = encoding.substr(pos+1,encoding.size());
            //cout << newString << endl;
            std::string::size_type newPos = newString.find('/');
            y = newString.substr(0,newPos);
            int yValue = atoi(y.c_str());
            //cout << y << endl;
            newString = newString.substr(newPos+1,encoding.size());
            //cout << newString << endl;
            newPos = newString.find('/');
            orientation = newString.substr(0,newPos);
            int orientationValue = atoi(orientation.c_str());
            //orientationValue = 180 - orientationValue;
            //cout << orientation << endl;
            newString = newString.substr(newPos+1,encoding.size());
            //cout << newString << endl;
            newPos = newString.find('/');
            r = newString.substr(0,newPos);
            int rValue = atoi(r.c_str());
            //cout << r << endl;
            newString = newString.substr(newPos+1,encoding.size());
            //cout << newString << endl;
            newPos = newString.find('/');
            phi = newString.substr(0,newPos);
            double phiValue = atoi(phi.c_str());

            txt << xValue << "  " << yValue << " " << orientationValue << " " << r <<endl;
        }

    }


    void Map::drawRFIDScan()
    {
        std::ofstream resultMap("/home/pulver/Desktop/MCDM/rfid_result.pgm", ios::out);
        long columns = numPathPlanningGridCols;
        long rows = numPathPlanningGridRows;

        resultMap << "P2\n" << columns << " " << rows << "\n255\n";

        for(long row = 0; row < rows; ++row)
        {
            for(long col = 0; col < columns; ++col)
            {
                int value = getRFIDGridValue(row, col);
//      cout << value << endl;
                value = std::min(value, 255);
                value = std::max(value, 0);
                resultMap << value  << " ";
            }
        }

        resultMap.close();
    }

/**
 * @brief Map::drawRFIDScan save on disk an image representing the scanned environment. Lighter zone represents
 * higher probability for the presence of the RFID tag
 */
    void Map::drawRFIDGridScan(RFIDGridmap grid)
    {
        std::ofstream resultMap("/home/pulver/Desktop/MCDM/rfid_result_gridmap.pgm", ios::out);
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

    /**
 * @brief Map::getRelativeTagCoord calculate the relative position of the RFID tag wrt the antenna
 * @param absTagX: absolute x-position of the RFID tag
 * @param absTagY: absolute x-position of the RFID tag
 * @param antennaX: absolute x-position of the antenna
 * @param antennaY: absolute y-position of the antenna
 * @return a pair containing the relative position of the tag to the antenna
 */
    std::pair<int, int> Map::getRelativeTagCoord(int absTagX, int absTagY, int antennaX, int antennaY)
    {
//  cout << "TAG = [" << absTagX << "," << absTagY <<"] -- ANTENNA = [" << antennaX << "," << antennaY << "]" << endl;
        std::pair<int, int> relTagCoord;
        relTagCoord.first = std::abs(absTagX - antennaX);
        relTagCoord.second = std::abs(absTagY - antennaY);
//  cout << "RELTAG = [" << relTagCoord.first << "," << relTagCoord.second << "]" << endl;
        return relTagCoord;
    }


    std::pair<int, int> Map::findTag()
    {
        std::pair<int,int> tag(0,0);
        double powerRead = 0;
        for(int row=0; row < numPathPlanningGridRows; row++)
        {
            for(int col=0; col < numPathPlanningGridCols; col++)
            {
                if(getRFIDGridValue(row, col) > powerRead)
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

    std::pair<int, int> Map::findTagfromGridMap(RFIDGridmap grid)
    {
        std::pair<int,int> tag(0,0);
        double powerRead = 0;
        for(int row=0; row < numPathPlanningGridRows; row++)
        {
            for(int col=0; col < numPathPlanningGridCols; col++)
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
        return RFIDGrid[i * numPathPlanningGridCols + j];
    }

    /**
 * @brief Map::setRFIDGridValue: update the grid cell summing the current value with a new reading
 * @param power: the sensed power by the antenna
 * @param i: the x-position in the grid
 * @param j: the y-position in the grid
 */
    void Map::setRFIDGridValue(float power, int i, int j)
    {
        //  cout << "-----" << endl;
//  if( power < 0) power = 0;
//    cout << RFIDGrid[i * numPathPlanningGridCols + j] << endl;
        RFIDGrid[i * numPathPlanningGridCols + j] += power;
//    cout << RFIDGrid[i * numPathPlanningGridCols + j] << endl;
    }


}
