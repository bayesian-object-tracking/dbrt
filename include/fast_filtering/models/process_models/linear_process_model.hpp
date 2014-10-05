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

#ifndef FAST_FILTERING_MODELS_PROCESS_MODELS_LIBEAR_GAUSSIAN_PROCESS_MODEL_HPP
#define FAST_FILTERING_MODELS_PROCESS_MODELS_LIBEAR_GAUSSIAN_PROCESS_MODEL_HPP

#include <fast_filtering/utils/traits.hpp>
#include <fast_filtering/distributions/gaussian.hpp>
#include <fast_filtering/models/process_models/interfaces/stationary_process_model.hpp>

namespace ff
{

// Forward declarations
template <typename State, typename Input> class LinearGaussianProcessModel;

namespace internal
{
template <typename State, typename Input>
struct Traits<LinearGaussianProcessModel<State, Input> >
{
    typedef Gaussian<State> GaussianBase;
    typedef StationaryProcessModel<State, Input> ProcessModelBase;

    typedef typename internal::Traits<GaussianBase>::Scalar Scalar;
    typedef typename internal::Traits<GaussianBase>::Operator Operator;
    typedef typename internal::Traits<GaussianBase>::Noise Noise;

    typedef Eigen::Matrix<Scalar,
                          State::SizeAtCompileTime,
                          State::SizeAtCompileTime> DynamicsMatrix;

    typedef Eigen::Matrix<Scalar,
                          State::SizeAtCompileTime,
                          Input::SizeAtCompileTime> InputMatrix;
};
}


template <typename State_, typename Input_ = internal::Empty>
class LinearGaussianProcessModel:
    public internal::Traits<LinearGaussianProcessModel<State_, Input_> >::ProcessModelBase,
    public internal::Traits<LinearGaussianProcessModel<State_, Input_> >::GaussianBase
{
public:
    typedef internal::Traits<LinearGaussianProcessModel<State_, Input_> > Traits;

    typedef State_ State;
    typedef Input_ Input;
    typedef typename Traits::Noise Noise;
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::Operator Operator;
    typedef typename Traits::DynamicsMatrix DynamicsMatrix;
    typedef typename Traits::InputMatrix InputMatrix;

    using Traits::GaussianBase::Mean;
    using Traits::GaussianBase::Covariance;
    using Traits::GaussianBase::Dimension;

public:
    LinearGaussianProcessModel(
            const Operator& noise_covariance,
            const size_t dimension = State::SizeAtCompileTime,
            const size_t input_dimension = Input::SizeAtCompileTime):
        Traits::GaussianBase(dimension),
        input_dimension_(input_dimension == Eigen::Dynamic? 0 : input_dimension),
        A_(DynamicsMatrix::Identity(Dimension(), Dimension())),
        B_(InputMatrix::Zero(InputDimension(), Dimension())),
        delta_time_(0.)
    {
        Covariance(noise_covariance);
    }

    ~LinearGaussianProcessModel() { }

    virtual State MapStandardGaussian(const Noise& sample) const
    {
        return Mean() + delta_time_ * this->cholesky_factor_ * sample;
    }

    virtual void Condition(const double& delta_time,
                           const State& x,
                           const Input& u = Input())
    {
        delta_time_ = delta_time;

        Mean(A_ * x);
    }

    virtual const DynamicsMatrix& A() const
    {
        return A_;
    }

    virtual const InputMatrix& B() const
    {
        return B_;
    }

    virtual void A(const DynamicsMatrix& dynamics_matrix)
    {
        A_ = dynamics_matrix;
    }

    virtual void B(const InputMatrix& input_matrix)
    {
        B_ = input_matrix;
    }

    virtual size_t InputDimension() const
    {
        return input_dimension_;
    }

protected:
    size_t input_dimension_;
    DynamicsMatrix A_;
    InputMatrix B_;
    double delta_time_;    
};

}

#endif
