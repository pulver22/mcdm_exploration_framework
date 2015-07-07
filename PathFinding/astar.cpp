// Astar.cpp
// http://en.wikipedia.org/wiki/A*
// Compiler: Dev-C++ 4.9.9.2
// FB - 201012256
#include "PathFinding/astar.h"
#include "PathFinding/node.h"

using namespace std;

Astar::Astar()
{
}

Astar::~Astar()
{
}


bool operator<(const Node & a,const Node & b){
    Node tmp1 = a;
    Node tmp2 = b;
    //return tmp1.getPriority() > tmp2.getPriority();

    return a.getPriority() > b.getPriority();
}



// A-star algorithm.
// The route returned is a string of direction digits.
string Astar::pathFind( const int & xStart, const int & yStart, const int & xFinish, const int & yFinish,
		    Map& originalMap )
{
    const int m = originalMap.getNumGridCols();
    const int n = originalMap.getNumGridRows();
    int map[n][m];
    int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
    int open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
    int dir_map[n][m]; // map of directions
    // create map
    for(int y=0;y<m;y++)
    {
        for(int x=0;x<m;x++) map[x][y]= originalMap.getGridValue(x,y);
    }
    //cout << "alive in pathfind"<< endl;
    static priority_queue<Node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static Node* n0;
    static Node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi=0;

    // reset the node maps
    for(y=0;y<m;y++)
    {
        for(x=0;x<n;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }

    // create the start node and push into list of open nodes
    n0=new Node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[xStart][yStart]=n0->getPriority(); // mark it on the open nodes map
    delete n0;

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new Node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish)
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map[xdx][ydy]==1
                || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new Node( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    delete m0;
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                           pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                    delete m0;
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}

double Astar::lenghtPath(string path)
{

    double lenght = 0.0;    
    //calculate the distance expressed in cells
    for(char& c : path) {
	if(c =='0' || c=='2' || c=='4' || c=='6'){
	    lenght = lenght + 1;
	}else lenght = lenght + sqrt(2);
    }
    
    return lenght;
}

int Astar::getNumberOfTurning(string path){
    int numberOfTurning = 0;  
    //calculate the number of turning during the path
    
    if(path.size()==0){
	return 0;
    }else{
	for(int i=0; i< path.size()-1; i++){
	    if(path.at(i+1) == path.at(i) +1 || path.at(i+1) == path.at(i) -1 ){
		numberOfTurning =+ 1;    
	    }else if(path.at(i+1) == path.at(i) +2 || path.at(i+1) == path.at(i) -2 ){
		numberOfTurning =+ 2;
	    }else if(path.at(i+1) == path.at(i) +3 || path.at(i+1) == path.at(i) -3 ){
		numberOfTurning =+ 3;
	    }
	}
    
	return numberOfTurning;
    }
}
