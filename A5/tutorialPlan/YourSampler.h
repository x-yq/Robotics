#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_


#include <rl/plan/Sampler.h>
#include <random>
#include <rl/kin/Kinematics.h>


namespace rl
{
	namespace plan
	{
		class YourSampler : public Sampler
		{
		public:
			YourSampler();
			
			virtual ~YourSampler();
			
			::rl::math::Vector generate();

			::std::uniform_real_distribution<::rl::math::Real>::result_type rand();
			
			int col(::rl::math::Vector& a);
			
			virtual void seed(const ::std::mt19937::result_type& value);
			
			void setEnd(const ::rl::math::Vector& start, const ::rl::math::Vector& goal);

			::rl::math::Vector transferToPosition(const ::rl::math::Vector& a);
			
			::rl::math::Vector computeCenter(const ::rl::math::Vector& start, const ::rl::math::Vector& goal);
			
			::rl::math::Vector generate_gaussian_center_biased();
			
			::rl::math::Vector goal;

			::rl::math::Vector start;

			::rl::math::Vector center;
			
			::rl::math::Real goal_bias_prob;
			
		protected:
			
			::std::normal_distribution<::rl::math::Real> gaussDistribution;
			
			::std::uniform_real_distribution< ::rl::math::Real> randDistribution;
			
			::std::mt19937 gaussEngine;

            ::std::mt19937 randEngine;	
			
		private:
			
		};
	}
}

#endif // _YOURSAMPLER_H_