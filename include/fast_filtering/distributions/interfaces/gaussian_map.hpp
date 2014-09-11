/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Max-Planck-Institute for Intelligent Systems,
 *                     University of Southern California
 *    Manuel Wuthrich (manuel.wuthrich@gmail.com)
 *    Jan Issac (jan.issac@gmail.com)
 *
 *
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

#ifndef FAST_FILTERING_DISTRIBUTIONS_INTERFACES_GAUSSIAN_MAP_HPP
#define FAST_FILTERING_DISTRIBUTIONS_INTERFACES_GAUSSIAN_MAP_HPP

#include <Eigen/Dense>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include <fast_filtering/utils/assertions.hpp>
#include <fast_filtering/distributions/interfaces/sampling.hpp>
#include <fast_filtering/distributions/standard_gaussian.hpp>

namespace ff
{

namespace internal
{
struct Empty { };
}


template <typename Vector, typename Noise = internal::Empty>
class GaussianMap:
        public Sampling<Vector>
{
public:
    explicit GaussianMap(const unsigned& noise_dimension = Noise::SizeAtCompileTime):
        standard_gaussian_(noise_dimension)
    {
        // make sure that noise is derived from eigen
        REQUIRE_INTERFACE(Noise, Eigen::Matrix<typename Noise::Scalar, Noise::SizeAtCompileTime, 1>);
    }

    virtual ~GaussianMap() { }

    virtual Vector MapStandardGaussian(const Noise& sample) const = 0;

    virtual Vector Sample()
    {
        return MapStandardGaussian(standard_gaussian_.Sample());
    }

    virtual int NoiseDimension() const
    {
        return standard_gaussian_.Dimension();
    }

private:
    StandardGaussian<Noise> standard_gaussian_;
};

// specialization for scalar noise
template <typename Vector>
class GaussianMap<Vector, double>:
        public Sampling<Vector>
{
public:
    virtual ~GaussianMap() { }

    virtual Vector MapStandardGaussian(const double& sample) const = 0;

    virtual Vector Sample()
    {
        return MapStandardGaussian(standard_gaussian_.Sample());
    }

    virtual int NoiseDimension() const
    {
        return 1;
    }
private:
    StandardGaussian<double> standard_gaussian_;
};



// specialization for no noise
template <typename Vector>
class GaussianMap<Vector, internal::Empty>:
        public Sampling<Vector>
{
public:
    virtual ~GaussianMap() { }

    virtual Vector MapStandardGaussian() const = 0;

    virtual Vector Sample()
    {
        return MapStandardGaussian();
    }

    virtual int NoiseDimension() const
    {
        return 0;
    }
};


}

#endif
