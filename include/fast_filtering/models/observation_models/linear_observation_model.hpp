/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Max-Planck-Institute for Intelligent Systems,
 *                     University of Southern California
 *    Jan Issac (jan.issac@gmail.com)
 *    Manuel Wuthrich (manuel.wuthrich@gmail.com)
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
 * @date 2014
 * @author Jan Issac (jan.issac@gmail.com)
 * @author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California
 */

#ifndef FAST_FILTERING_MODELS_PROCESS_MODELS_LINEAR_GAUSSIAN_OBSERVATION_MODEL_HPP
#define FAST_FILTERING_MODELS_PROCESS_MODELS_LINEAR_GAUSSIAN_OBSERVATION_MODEL_HPP

#include <fast_filtering/utils/traits.hpp>
#include <fast_filtering/distributions/gaussian.hpp>
#include <fast_filtering/models/process_models/interfaces/stationary_process_model.hpp>

namespace ff
{

// Forward declarations
template <typename State, typename Input> class LinearGaussianOservationModel;

namespace internal
{
template <typename State, typename Input>
struct Traits<LinearGaussianOservationModel<State, Input> >
{
    typedef Gaussian<State> GaussianBase;

    typedef typename internal::Traits<GaussianBase>::Scalar Scalar;
    typedef typename internal::Traits<GaussianBase>::Operator Operator;
    typedef typename internal::Traits<GaussianBase>::Noise Noise;

    typedef Eigen::Matrix<Scalar,
                          State::SizeAtCompileTime,
                          State::SizeAtCompileTime> SensorMatrix;

    typedef Eigen::Matrix<Scalar,
                          State::SizeAtCompileTime,
                          Input::SizeAtCompileTime> FeedthroughMatrix;
};
}


template <typename State_, typename Input_ = internal::Empty>
class LinearGaussianOservationModel:
    public internal::Traits<LinearGaussianOservationModel<State_, Input_> >::GaussianBase
{
public:
    typedef internal::Traits<LinearGaussianOservationModel<State_, Input_> > Traits;

    typedef State_ State;
    typedef Input_ Input;
    typedef typename Traits::Noise Noise;
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Operator Operator;
    typedef typename Traits::SensorMatrix SensorMatrix;
    typedef typename Traits::FeedthroughMatrix FeedthroughMatrix;

    using Traits::GaussianBase::Mean;
    using Traits::GaussianBase::Covariance;
    using Traits::GaussianBase::Dimension;

public:
    LinearGaussianOservationModel(
            const Operator& noise_covariance,
            const size_t dimension = State::SizeAtCompileTime,
            const size_t observation_dimension = Input::SizeAtCompileTime):
        Traits::GaussianBase(dimension),
        observation_dimension_(observation_dimension == Eigen::Dynamic? 0 : observation_dimension),
        H_(SensorMatrix::Identity(Dimension(), Dimension())),
        C_(FeedthroughMatrix::Zero(InputDimension(), Dimension())),
        delta_time_(0.)
    {
        Covariance(noise_covariance);
    }

    ~LinearGaussianOservationModel() { }

    virtual State MapStandardGaussian(const Noise& sample) const
    {
        return Mean() + delta_time_ * this->cholesky_factor_ * sample;
    }

    virtual void Condition(const double& delta_time,
                           const State& x,
                           const Input& u = Input())
    {
        delta_time_ = delta_time;

        Mean(H_ * x);
    }

    virtual const SensorMatrix& H() const
    {
        return H_;
    }

    virtual const FeedthroughMatrix& C() const
    {
        return C_;
    }

    virtual void H(const SensorMatrix& sensor_matrix)
    {
        H_ = sensor_matrix;
    }

    virtual void C(const FeedthroughMatrix& feedthrough_matrix)
    {
        C_ = feedthrough_matrix;
    }


    virtual size_t ObservationDimension() const
    {
        return observation_dimension_;
    }

    virtual size_t StateDimension() const
    {

    }

    virtual size_t InputDimension() const
    {
        return observation_dimension_;
    }

protected:
    size_t observation_dimension_;
    SensorMatrix H_;
    FeedthroughMatrix C_;
    double delta_time_;
};

}

#endif
