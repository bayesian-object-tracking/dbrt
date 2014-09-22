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

#ifndef FAST_FILTERING_STATES_COMPOSED_STATE_DISTRIBUTION_HPP
#define FAST_FILTERING_STATES_COMPOSED_STATE_DISTRIBUTION_HPP

#include <Eigen/Dense>
#include <vector>
#include <boost/static_assert.hpp>

namespace ff
{

// Forward declarations
template <typename CohesiveState, typename FactorizedState, size_t FACTORIZED_STATES>
class ComposedStateDistribution;

namespace internal
{
/**
 * SumOfDeltas distribution traits specialization
 * \internal
 */
template <typename CohesiveState, typename FactorizedState, size_t FACTORIZED_STATES>
struct Traits<ComposedStateDistribution<CohesiveState, FactorizedState, FACTORIZED_STATES> >
{
    typedef typename CohesiveState::Scalar Scalar;

    typedef Eigen::Matrix<Scalar,
                          CohesiveState::SizeAtCompileTime,
                          CohesiveState::SizeAtCompileTime> CovAA;

    typedef Eigen::Matrix<Scalar,
                          CohesiveState::SizeAtCompileTime,
                          FactorizedState::SizeAtCompileTime> CovAB;

    typedef Eigen::Matrix<Scalar,
                          CohesiveState::SizeAtCompileTime,
                          1> CovAY;

    typedef Eigen::Matrix<Scalar,
                          FactorizedState::SizeAtCompileTime,
                          FactorizedState::SizeAtCompileTime> CovBB;

    typedef Eigen::Matrix<Scalar,
                          FactorizedState::SizeAtCompileTime,
                          1> CovBY;

    typedef Eigen::Matrix<Scalar, 1, 1> Y;
    typedef Eigen::Matrix<Scalar, 1, 1> CovYY;
};
}

/**
 * \class ComposedStateDistribution
 * \ingroup states
 */
template <typename CohesiveState,
          typename FactorizedState,
          size_t FACTORIZED_STATES = -1>
class ComposedStateDistribution
{
public:
    typedef internal::Traits<
                ComposedStateDistribution<CohesiveState,
                                          FactorizedState,
                                          FACTORIZED_STATES> > Traits;
    typedef typename Traits::Scalar Scalar;
    typedef typename Traits::CovAA CovAA;
    typedef typename Traits::CovAB CovAB;
    typedef typename Traits::CovAY CovAY;
    typedef typename Traits::CovBB CovBB;
    typedef typename Traits::CovBY CovBY;
    typedef typename Traits::CovYY CovYY;
    typedef typename Traits::Y Y;

    struct JointPartitions
    {
        FactorizedState b;
        Y y;

        CovAB cov_ab;
        CovAY cov_ay;
        CovBB cov_bb;
        CovBY cov_by;
        CovYY cov_yy;
    };

public:
    /**
     * Creates a composed state distribution. This distribution harbors a
     * cohesive state part and a number of factorized states. The total state
     * dimension is \f$\dim(CohesiveState) + FACTORIZED_STATES *
     * \dim(FactorizedState) \f$.
     *
     * @param cohesive_state_dimension      Dimension of the cohesive state part
     * @param factorized_state_dimension    Dimension of single part of the
     *                                      factorized state segment
     * @param factorized_states             Dimension of number of factored
     *                                      states.
     */
    ComposedStateDistribution(
            const size_t& cohesive_state_dimension = CohesiveState::SizeAtCompileTime,
            const size_t& factorized_state_dimension = FactorizedState::SizeAtCompileTime,
            const size_t& factorized_states_count = FACTORIZED_STATES):
        cohesive_state_dimension_(cohesive_state_dimension),
        factorized_state_dimension_(factorized_state_dimension),
        factorized_states_count_(factorized_states_count)
    {

    }

    virtual ~ComposedStateDistribution() { }

    size_t CohesiveStatesDimension()
    {
        return cohesive_state_dimension_;
    }

    size_t FactorizedStateDimension()
    {
        return factorized_state_dimension_;
    }

    size_t FactorizedStatesCount()
    {
        return factorized_states_count_;
    }

public:
    CohesiveState state_a_;
    CovAA cov_aa_;
    CovAA cov_aa_inverse_;
    std::vector<JointPartitions> joint_partitions_;

    size_t cohesive_state_dimension_;
    size_t factorized_state_dimension_;
    size_t factorized_states_count_;
};

}

#endif
