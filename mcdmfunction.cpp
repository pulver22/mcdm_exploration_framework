#include "mcdmfunction.h"
#include "explorationconstants.h"
#include "Criteria/criterion.h"
#include "Criteria/criteriaName.h"
#include "Criteria/traveldistancecriterion.h"
#include "Criteria/informationgaincriterion.h"
#include "Criteria/sensingtimecriterion.h"
#include "Criteria/mcdmweightreader.h"
#include "Criteria/criterioncomparator.h"
#include <string>
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include "newray.h"


using namespace std;
using namespace dummy;
/* create a list of criteria with name and <encoded_name,weight> pair after reading that from a file
 */
MCDMFunction::MCDMFunction() //:
     //criteria(new unordered_map<string, Criterion* >())
     //activeCriteria(new vector<Criterion >() ) 
{
    
    // Initialization ad-hoc: create a weightmatrix for 3 criteria with predefined weight
    MCDMWeightReader reader;
    //cout << "test" << endl;
    matrix = reader.getMatrix();
    //cout << "test2" << endl;

    // get the list of all criteria to be considered
    list<string> listCriteria = matrix->getKnownCriteria();
    for (list< string >::iterator it = listCriteria.begin(); it != listCriteria.end(); ++it){
	string name = *it;
	// retrieve the weight of the criterion using the encoded version of the name
	double weight = matrix->getWeight(matrix->getNameEncoding(name));
	Criterion *c = createCriterion(name, weight);
	if(c != NULL){
	    criteria.emplace(name, c);
	}
    }

}

MCDMFunction::~MCDMFunction()
{	
    //delete matrix;

}

//Create a criterion starting from its name and weight
Criterion * MCDMFunction::createCriterion(string name, double weight)
{
    Criterion *toRet = NULL;
    if(name == (SENSING_TIME)){
	toRet =  new SensingTimeCriterion(weight);
    } else if (name == (INFORMATION_GAIN)) {
	toRet = new InformationGainCriterion(weight);
    } else if (name == (TRAVEL_DISTANCE)){
	toRet = new  TravelDistanceCriterion(weight);
    }
    return toRet;
}

// For a candidate frontier, calculate its evaluation regarding to considered criteria and put it in the evaluation record (through 
//the evaluate method provided by Criterion class)
double MCDMFunction::evaluateFrontier( Pose& p,  dummy::Map &map)
{
    //NewRay ray;
    //ray.calculateInfoGainSensingTime(map,p.getX(),p.getY(),p.getOrientation(),p.getFOV(),p.getRange());
   
      for (int i =0; i < activeCriteria.size(); i++){
	Criterion *c = activeCriteria.at(i);
	//string name = c->getName();
	//double weight = c->getWeight();
	//cout << name <<" " << weight <<  endl;
	c->evaluate(p,map);
	//cout << "Alive in evaluateFrontier" << endl;
   }
   
    //for loop, over the criteria, to compute the utility of the frontier.

    return 0.0;
}


// Scan a list of candidate positions,then apply the choquet fuzzy algorithm
EvaluationRecords* MCDMFunction::evaluateFrontiers( std::list< Pose >& frontiers,  Map& map,double threshold)
{         
    //myMutex.lock();
   
    //Clean the last evaluation
    //         lprint << "clean evaluations" << endl;
    // ATTENTION : to be fixed -> you need to clear criteria[ i ].Criterion ...
    //NOTE: probably working
    unordered_map<string,Criterion *>::iterator it;
    for(it = criteria.begin(); it != criteria.end(); it++){
	std::pair<string,Criterion*> pair = *it;
	(criteria.at(pair.first))->clean();
    }
    
    
    /*
    //Get the list of activeCriteria
    if(activeCriteria.size() == 0){
	activeCriteria = new vector<Criterion >();
    }
    */
    
    // listActiveCriteria contains the name of the criteria while "criteria struct" contain the pairs <name, criterion>
    vector<string> listActiveCriteria = matrix->getActiveCriteria();
    for(vector<string>::iterator it = listActiveCriteria.begin(); it != listActiveCriteria.end(); it++){
	activeCriteria.push_back(criteria[*it]);
    }
    
    //ldbg << "number of active criteria: "<<activeCriteria->size() << endl;
  
    //Evaluate the frontiers
    list<Pose>::iterator it2 ;
    for (it2 = frontiers.begin(); it2 != frontiers.end(); it2++){
	Pose f = *it2;
	double value = 0.0;
	value = evaluateFrontier(f, map);
    }

    
    //Normalize the values
    for(vector<Criterion *>::iterator it = activeCriteria.begin(); it != activeCriteria.end(); ++it){
	(*it)->normalize();
    }
    
    //Create the EvaluationRecords
    //         lprint << "#number of frontier to evaluate: "<<frontiers.size()<<endl;
    EvaluationRecords *toRet = new EvaluationRecords();
    
    
    // analyze every single frontier f, and add in the evaluationRecords <frontier, evaluation>
    for(list<Pose>::iterator i=frontiers.begin(); i!=frontiers.end(); i++){
	
	Pose f = *i;
	// order criteria depending on the considered frontier
	//qsort(&activeCriteria,activeCriteria.size(),sizeof(Criterion),CriterionComparator(f));
	
	//ATTENTION: doesn't work
	//NOTE: maybe work
	sort(activeCriteria.begin(),activeCriteria.end(),CriterionComparator(f));
	/*
	cout << "criteria ordered: " << endl;
	for (int i =0; i < activeCriteria.size(); i++){
	    Criterion *c = activeCriteria.at(i);
	    cout << c->getName() << ": " << c->getEvaluation(f) << endl;
	}*/
	   
	//apply the choquet integral
	Criterion *lastCrit = NULL;
	double finalValue = 0.0;
	for(vector<Criterion *>::iterator i = activeCriteria.begin(); i != activeCriteria.end(); i++){
	    Criterion *c = NULL ;
	    double weight = 0.0;
	    //Get the list of criterion whose evaluation is >= than the one's considered
	    list<string> names;
	   for(vector<Criterion *>::iterator j = i+1; j != activeCriteria.end(); j++){
	       //CHECK IF THE ITERATOR RETURN THE COUPLE <STRING,CRITERION>
	       Criterion *next = (*j);
	       names.push_back(next->getName());
	   }
	    weight = matrix->getWeight(names);
	
//               lprint << "#"<<names << " - w = " << weight << " - ";
//               lprint << names <<" with weight "<<weight<<endl;
	    if(i==activeCriteria.begin()){
		c = (*i);
		finalValue += c->getEvaluation(f) * weight;
//                         lprint << "#crit "<<c->getName()<<" - eval = "<<c->getEvaluation(f) << endl;
	    } else {
		c = (*i);
		double tmpValue = c->getEvaluation(f)-lastCrit->getEvaluation(f);
//                         lprint << "#crit "<<c->getName()<<" - eval = "<<c->getEvaluation(f) << endl;
		finalValue += tmpValue*weight;
	    }
	    lastCrit = c;
	    //delete c;
	}
	
	//cout <<"X: "<< f.getX() <<"; Y : " <<f.getY()<<", Orientation :"<<f.getOrientation() <<", Evaluation : "<<finalValue << endl;
	if(finalValue > threshold){
	    toRet->putEvaluation(f, finalValue);
	}
	//delete lastCrit;
    }
//         }
    
    
    activeCriteria.clear();
    /*
    for(vector<Criterion*>::iterator itActive= activeCriteria.begin();itActive !=activeCriteria.end();itActive++){
	delete *itActive;
    }*/
    
    //delete activeCriteria;
    //activeCriteria = NULL;
    //myMutex.unlock();
    return toRet;
}

pair<Pose,double> MCDMFunction::selectNewPose(EvaluationRecords *evaluationRecords)
{	
    
    Pose newTarget;
    double value = 0;
    unordered_map<string,double> evaluation = evaluationRecords->getEvaluations();
    for(unordered_map<string,double>::iterator it = evaluation.begin(); it != evaluation.end(); it++){
	string tmp = (*it).first;
	Pose p = evaluationRecords->getPoseFromEncoding(tmp);
	if(value < (*it).second){
		newTarget = p;
		value = (*it).second;
	    }//else continue;
    }
    pair<Pose,double> result = make_pair(newTarget,value);
    
    // i switch x and y to allow debugging graphically looking the image
    cout << "New target : " << "x = "<<newTarget.getY() <<", y = "<< newTarget.getX() << ", orientation = " 
	    <<newTarget.getOrientation() << ", Evaluation: "<< value << endl;
    return result;
}

string MCDMFunction::getEncodedKey(Pose& p, int value)
{
    string key;
    //value = 0 : encode everything
    //value = 1 : encode x,y,orientation, take first 
    //value = 2 : encode x,y,orientation, take multiple time
    if(value == 0){
	key =  to_string(p.getX()) + "/" + to_string( p.getY()) + "/" + to_string( (int)p.getOrientation()) + "/"  + to_string(p.getRange()) + "/" + to_string((int)p.getFOV());
    }else if(value == 1){
	key = to_string(p.getX()) + "/" + to_string( p.getY()) + "/" + to_string( (int)p.getOrientation()) + "/" + "1";
    } else if (value ==2){
	key = to_string(p.getX()) + "/" + to_string( p.getY()) + "/" + to_string( (int)p.getOrientation()) + "/" + "0";
    }
    return key;
}





