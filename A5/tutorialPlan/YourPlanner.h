#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "RrtConConBase.h"
#include "YourSampler.h"

using namespace ::rl::plan;

/**
*	The implementation of your planner.
*	modify any of the existing methods to improve planning performance.
*/
class YourPlanner : public RrtConConBase
{
public:
  YourPlanner();

  virtual ~YourPlanner();

  YourSampler* sampler;

  virtual ::std::string getName() const;

  bool solve();


protected:
  void choose(::rl::math::Vector& chosen);

  RrtConConBase::Vertex addVertex(Tree& tree, const ::rl::plan::VectorPtr& q);

  RrtConConBase::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);
  
  RrtConConBase::Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen);
  

  int max_failures;
  int count_failures;
  

private:

};

#endif // _YOUR_PLANNER_H_
