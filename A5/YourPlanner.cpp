#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>
#include <iostream>


YourPlanner::YourPlanner() :
  RrtConConBase(),
  max_failures(5),
  count_failures(0)
{
  // Set max_failures to 5 and count_failures to 0.
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Rollback";
}

RrtConConBase::Vertex 
YourPlanner::addVertex(Tree& tree, const ::rl::plan::VectorPtr& q)
{
    RrtConConBase::Vertex v = RrtConConBase::addVertex(tree, q);

    // Everytime when add new vertex, it means connect successfully. 
    // Therefore, set the count_failures to 0.
    this->count_failures = 0;
    return v;
}


void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  //your modifications here
  this->sampler->setEnd(*this->start,*this->goal);
  chosen = this->sampler->generate_gaussian_center_biased();
  //chosen = this->sampler->generate();
}

RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  RrtConConBase::Vertex connected = RrtConConBase::connect(tree, nearest, chosen);

  if (connected != NULL)
  {
    // If connected not Null, it means it connect successfully, set count_failures to 0.
      this->count_failures = 0;
  }
  else
  {
    // If connected is Null, it means it connect unsuccessfully, add 1 to count_failures.
      ++this->count_failures;
    
    // If current count_failures is greater than max_failures, rollback.
      if (this->count_failures >= this->max_failures)
      {
          RrtConConBase::Vertex last = ::boost::vertex(::boost::num_vertices(tree) - 1, tree);
          ::boost::remove_vertex(last, tree);
          this->count_failures = 0;
      }
  }

  return connected;
}


bool
YourPlanner::solve()
{
  return RrtConConBase::solve();
}


RrtConConBase::Neighbor
YourPlanner::nearest(const Tree& tree, const rl::math::Vector& chosen) {
  return RrtConConBase::nearest(tree, chosen);
  
}
