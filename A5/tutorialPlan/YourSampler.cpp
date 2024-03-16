#include <chrono>
#include <rl/plan/SimpleModel.h>
#include "YourSampler.h"
#include <iostream>

namespace rl
{
    namespace plan
    {
        YourSampler::YourSampler() :
            Sampler(),
            randDistribution(0, 1),
            gaussDistribution(0.0,1.0),
            gaussEngine(::std::random_device()()),
            randEngine(::std::random_device()()),
            goal_bias_prob(0.9), 
            goal(::rl::math::Vector()),
            start(::rl::math::Vector()),
            center(::rl::math::Vector())
        {
        }
        
        YourSampler::~YourSampler()
        {
        }
        

        int
        YourSampler::col(::rl::math::Vector& a) {
            this->model->setPosition(a);
            this->model->updateFrames();
            return this->model->isColliding();
        }

        ::rl::math::Vector
        YourSampler::generate()
        {
            // Our template code performs uniform sampling.
            // You are welcome to change any or all parts of the sampler.
            // BUT PLEASE MAKE SURE YOU CONFORM TO JOINT LIMITS,
            // AS SPECIFIED BY THE ROBOT MODEL!

            ::rl::math::Vector sampleq(this->model->getDof());

            ::rl::math::Vector maximum(this->model->getMaximum());
            ::rl::math::Vector minimum(this->model->getMinimum());

            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
                sampleq(i) = minimum(i) + this->rand() * (maximum(i) - minimum(i));
            }

            // It is a good practice to generate samples in the
            // the allowed configuration space as done above.
            // Alternatively, to make sure generated joint 
            // configuration values are clipped to the robot model's 
            // joint limits, you may use the clip() function like this: 
            // this->model->clip(sampleq);

            return sampleq;
        }
		
		::std::uniform_real_distribution<::rl::math::Real>::result_type
		YourSampler::rand()
		{
			return this->randDistribution(this->randEngine);
		}

        void
		YourSampler::seed(const ::std::mt19937::result_type& value)
		{
			this->gaussEngine.seed(value);
			this->randEngine.seed(value);
		}

        void
        YourSampler::setEnd(const ::rl::math::Vector& start, const ::rl::math::Vector& goal)
        {
            // Get the goal and start configuration, compute the center configuration using computerCenter method,
            // and store the result in this->center.
            this->goal = goal;
            this->start = start;
            this->center = this->computeCenter(this->start,this->goal);

        }
        
        
        ::rl::math::Vector
        YourSampler::transferToPosition(const ::rl::math::Vector& a){
            // This method used for transfering the rotation of robot joints to an end effector pose, 
            // which is the 3-D position of the end effector.
            this->model->setPosition(a);
            this->model->updateFrames();
            rl::math::Transform endEffectorPose = this->model->forwardPosition(0);
            rl::math::Vector pose = endEffectorPose.translation();
            return pose;
        }

        ::rl::math::Vector
        YourSampler::computeCenter(const ::rl::math::Vector& start, const ::rl::math::Vector& goal){
            // Based on given 2 configurations, calculate their mid configuration.
            // Firstly, use the transferToPosition() method to get each end configuration's 3-D position.
            auto goal_pose = this->transferToPosition(goal);
            auto start_pose = this->transferToPosition(start);

            // Secondly, get the direction from start to goal.
            auto direction = goal_pose - start_pose;

            // If the direction from the initial to the end position is the same as the positive direction of the x-axis,
            // the center configuration is the (goal - start) / 2.0. Otherwise, is the (start - goal) / 2.0
            if(direction(0) > 0.0 ){
                return (goal-start)/2.0;
            }
            else{
                return (start-goal)/2.0;
            }

        }

        ::rl::math::Vector
        YourSampler::generate_gaussian_center_biased()
        {
            // Generate sample

            ::rl::math::Vector maximum(this->model->getMaximum());
            ::rl::math::Vector minimum(this->model->getMinimum());
            ::rl::math::Vector sample(this->model->getDof());
            for (::std::size_t i = 0; i < this->model->getDof(); ++i)
            {
            	float sigma = (maximum(i) - minimum(i)) / 6.0;
                std::normal_distribution<::rl::math::Real> d(0.0, sigma);
                sample(i) =  this->center(i) + d(this->gaussEngine) * this->goal_bias_prob;
            }

            this->model->clip(sample);

            return sample;
        }
    }
}
