#include "evaluationrecords.h"
#include <functional>
#include "graphpose.h"
#include <sstream>
#include <iostream>

using namespace std;
EvaluationRecords::EvaluationRecords()
//:    evaluations(new unordered_map<Pose, double>())
{
}

EvaluationRecords::~EvaluationRecords()
{
    //delete evaluations;
}

void EvaluationRecords::putEvaluation(Pose& frontier, double value)
{
    if(evaluations.empty()){
	minValue = value;
	maxValue = value;
    }
    
    string s =getEncodedKey(frontier);
    evaluations.emplace(s,value);

    if(value >= maxValue)
	maxValue = value;
    if(value <= minValue)
	minValue = value;


}

double EvaluationRecords::getEvaluation(Pose& frontier)
{
    string s = getEncodedKey(frontier);
    return evaluations[s];
}

unordered_map<string, double> EvaluationRecords::getEvaluations()
{
    return evaluations;
}

bool EvaluationRecords::contains(Pose& frontier)
{
    string s = getEncodedKey(frontier);
    unordered_map<string,double>::const_iterator got = evaluations.find(s);
    if (got == evaluations.end()){
	return false;
    }else return true;
}

int EvaluationRecords::size()
{
    return evaluations.size();
}



vector<Pose> EvaluationRecords::getFrontiers() 
{
    
    vector<string> list;
    list.reserve(evaluations.size());

    for(unordered_map<string,double>::iterator it = evaluations.begin(); it != evaluations.end(); it++ ) {
	list.push_back((*it).first);
	
    } 
    
    vector<Pose> toRet ;

    for(vector<string>::iterator it = list.begin(); it != list.end(); it++){
	// create a pose object for every string string in the list
	
	string s = *it;
	Pose p = getPoseFromEncoding(s);
	
	toRet.push_back(p);
    }

    return toRet;
}




void EvaluationRecords::removeFrontier(Pose& frontier)
{        
    for(unordered_map<string,double>::iterator it = evaluations.begin(); it != evaluations.end(); it++){
	string s1 = (*it).first;
	string s2 = getEncodedKey(frontier);
	if( s1 == s2 ){
	    evaluations.erase(s2);
	    break;
	}
    }
}

void EvaluationRecords::normalize(){
    vector<string> list ;
    list.reserve(evaluations.size());

    for(unordered_map<string,double>::iterator it = evaluations.begin(); it != evaluations.end(); it++ ) {
	list.push_back((*it).first);
    } 
    
    for(vector<string>::iterator it = list.begin(); it != list.end(); it++ ){
	for(unordered_map<string,double>::iterator it2 = evaluations.begin(); it2 != evaluations.end(); it2++){
	    if ((*it) == (*it2).first){
		double value = (*it2).second;
		value = (value - minValue)/(maxValue-minValue);
		evaluations.erase((*it2).first);
		pair<string,double> newPair (*it, value);
		evaluations.insert(newPair);
	     }
	 }
    }


}

string EvaluationRecords::getEncodedKey(Pose& p)
{
    string key =  to_string(p.getX()) + "/" + to_string( p.getY()) + "/" + to_string( (int)p.getOrientation()) + "/" + to_string(p.getRange()) +"/" + to_string(p.getFOV());
    return key;
}

Pose EvaluationRecords::getPoseFromEncoding(string &encoding)
{
	
	stringstream ss;
	string s ;
	ss << s;
	char delimiter('/');
	string x,y,orientation,r,phi;
	//cout << encoding << endl;
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
	double orientationValue = atoi(orientation.c_str());
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
	//cout << phi << endl;
	Pose p(xValue,yValue,orientationValue,rValue,phiValue);
	return p;
}

