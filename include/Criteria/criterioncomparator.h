#ifndef CRITERIONCOMPARATOR_H
#define CRITERIONCOMPARATOR_H


#include "criterion.h"
#include "pose.h"


class CriterionComparator
{

public:
    explicit CriterionComparator(Pose& p);
    virtual ~CriterionComparator();
    bool operator()(Criterion* c1, Criterion* c2 );

private:
    Pose p;
    
};

#endif // CRITERIONCOMPARATOR_H
