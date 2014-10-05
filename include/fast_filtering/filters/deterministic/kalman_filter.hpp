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

#ifndef FAST_FILTERING_FILTERS_DETERMINISTIC_KALMAN_FILTER_HPP
#define FAST_FILTERING_FILTERS_DETERMINISTIC_KALMAN_FILTER_HPP

#include <fast_filtering/utils/traits.hpp>

#include <boost/shared_ptr.hpp>

#include <fast_filtering/distributions/gaussian.hpp>
#include <fast_filtering/models/process_models/linear_process_model.hpp>
#include <fast_filtering/models/observation_models/linear_observation_model.hpp>

namespace ff
{

template <typename ProcessModel, typename ObservationModel>
class KalmanFilter
{
public:
    typedef boost::shared_ptr<ProcessModel> ProcessModelPtr;
    typedef boost::shared_ptr<ObservationModel> ObserationModelPtr;

    typedef typename ProcessModel::Scalar Scalar;
    typedef typename ProcessModel::State State;
    typedef typename ProcessModel::Input Input;
    typedef typename ProcessModel::State State;
    typedef typename ProcessModel::DynamicsMatrix DynamicsMatrix;
    typedef typename ProcessModel::Operator DynamicsCovariance;

    typedef typename ObservationModel::Observation Observation;
    typedef typename ObservationModel::SensorMatrix SensorMatrix;
    typedef typename ObservationModel::Operator SensorCovariance;

    typedef Gaussian<State> StateDistribution;

public:
    KalmanFilter(const ProcessModelPtr& process_model,
                 const ProcessModelPtr& observation_model):
        process_model_(process_model),
        observation_model_(observation_model)
    {
    }

    void Predict(const double delta_time,
                 const StateDistribution& prior,
                 StateDistribution& predicted)
    {
        const DynamicsMatrix& A = process_model_->A();
        const DynamicsCovariance Q = delta_time * process_model_->Covariance();

        predicted.Mean(A * prior.Mean());
        predicted.Covariance(A * prior.Covariance() * A.transpose() + Q);
    }

    void Update(const StateDistribution& predicted,
                const Observation& y,
                StateDistribution& posterior)
    {
        typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

        const SensorMatrix& H = observation_model_->H();
        const SensorMatrix R = observation_model_->Covariance();

        const Matrix S = H * predicted.Covariance() * H.transpose() + R;
        const Matrix K = predicted.Covariance() * H.transpose() * S.inverse();

        posterior.Mean(
                    predicted.Mean() + K * (y - H * predicted.Mean()));
        posterior.Covariance(
                    predicted.Covariance() - K * H * predicted.Covariance());
    }


protected:
    ProcessModelPtr process_model_;
    ObserationModelPtr observation_model_;
};

}

#endif
