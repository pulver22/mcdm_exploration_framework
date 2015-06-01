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

// get the map as a monodimension vector with 0 and 1
void Map::createGrid(int resolution)
{
  //cluster cells into grid
  float clusterSize = (float)((100.0/resolution));
  Map::numGridRows = (long)numRows/clusterSize;
  Map::numGridCols = (long)numCols/clusterSize;
  long gridRow = 0, gridCol = 0;
  //cout << numGridCols << " : "<< numGridRows << endl;
  

  //cout << "Total cell: " << totalFreeCells << endl;
  
  //get the size of the array and initialize to 0
  
  /*
    Map::grid.reserve(numGridRows*numGridCols);
  
    for (gridRow = 0; gridRow < numGridRows; ++gridRow)
    {
	    for (gridCol = 0; gridCol < numGridCols; ++gridCol)
	    {
	    grid[gridRow*numGridCols + gridCol] = 0;
	    }
    }*/
  
   for(int i = 0; i < numGridCols*numGridRows; ++i)
  {
    grid.push_back(0);
  }
  
  //set 1 in the grid cells corrisponding to obstacles
    for(long row = 0; row < numRows; ++row)
    {
	    for(long col = 0; col < numCols; ++col)
	    {
		//if(map[row*numCols + col] == 0) 
		if(map[row*numCols + col] < 250) 
		{
			grid[(long)(numRows-row/clusterSize)*numGridCols + (long)(col/clusterSize)] = 1;
			
		}
	    }
    }
    
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

void Map::drawVisitedCells(unordered_map<string,int>& visitedCells,int resolution)
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



}