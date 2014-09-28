/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Max-Planck-Institute for Intelligent Systems,
 *                     University of Southern California
 *    Manuel Wuthrich (manuel.wuthrich@gmail.com)
 *    Jan Issac (jan.issac@gmail.com)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @date 05/25/2014
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California
 */

#ifndef FAST_FILTERING_DISTRIBUTIONS_TRUNCATED_GAUSSIAN_HPP
#define FAST_FILTERING_DISTRIBUTIONS_TRUNCATED_GAUSSIAN_HPP

// eigen
#include <limits>
#include <cmath>
#include <boost/math/special_functions/erf.hpp>

// state_filtering
#include <fast_filtering/distributions/interfaces/evaluation.hpp>
#include <fast_filtering/distributions/interfaces/gaussian_map.hpp>

namespace ff
{

class TruncatedGaussian:
        public Evaluation<double, double>,
        public GaussianMap<double, double>
{

public:
    TruncatedGaussian(double mean = 0.0,
                      double sigma = 1.0,
                      double min = -std::numeric_limits<double>::infinity(),
                      double max = std::numeric_limits<double>::infinity()):
                                                        mean_(mean),
                                                        sigma_(sigma),
                                                        min_(min),
                                                        max_(max)
    {
        ComputeAuxiliaryParameters();
    }

    virtual ~TruncatedGaussian() { }

    virtual void SetParameters(double mean, double sigma, double min, double max)
    {
        mean_ =     mean;
        sigma_ =    sigma;
        min_ =      min;
        max_ =      max;

        ComputeAuxiliaryParameters();
    }

    virtual double Probability(const double& input) const
    {
        if(input < min_ || input > max_)
            return 0;

        return normalization_factor_ *
                std::exp(-0.5 * std::pow((input-mean_)/sigma_, 2));
    }

    virtual double LogProbability(const double& input) const
    {
        return std::log(Probability(input));
    }

    virtual double MapStandardGaussian(const double& gaussian_sample) const
    {
        // map from a gaussian to a uniform distribution
        double standard_uniform_sample = 0.5 *
                (1.0 + std::erf(gaussian_sample / std::sqrt(2.0)));
        // map onto truncated uniform distribution
        double truncated_uniform_sample = cumulative_min_ +
                  standard_uniform_sample * (cumulative_max_ - cumulative_min_);
        // map onto truncated gaussian
        return mean_ + sigma_ * std::sqrt(2.0) *
                  boost::math::erf_inv(2.0 * truncated_uniform_sample - 1.0);
    }

private:
    virtual void ComputeAuxiliaryParameters()
    {
        cumulative_min_ = 0.5 +
                0.5 * std::erf( (min_-mean_) / (sigma_*std::sqrt(2)) );
        cumulative_max_ = 0.5 +
                0.5 * std::erf( (max_-mean_) / (sigma_*std::sqrt(2)) );

        normalization_factor_ = 1.0 /
             (sigma_ * (cumulative_max_-cumulative_min_) * std::sqrt(2.0*M_PI));
    }

private:
    double mean_;
    double sigma_;
    double min_;
    double max_;

    double cumulative_min_;
    double cumulative_max_;
    double normalization_factor_;
};

}

#endif
