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

#include <fast_filtering/utils/traits.hpp>

namespace ff
{

// Forward declarations
template <typename CohesiveState, typename FactorizedState>
class ComposedStateDistribution;

namespace internal
{
/**
 * SumOfDeltas distribution traits specialization
 * \internal
 */
template <typename CohesiveState, typename FactorizedState>
struct Traits<ComposedStateDistribution<CohesiveState, FactorizedState> >
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
template <typename CohesiveState, typename FactorizedState>
class ComposedStateDistribution
{
public:
    typedef internal::Traits<
                ComposedStateDistribution<CohesiveState,
                                          FactorizedState> > Traits;
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
     * dimension is \f$\dim(CohesiveState) + FACTORIZED\_STATES *
     * \dim(FactorizedState) \f$.
     */
    ComposedStateDistribution()
    {
    }    

    virtual ~ComposedStateDistribution() { }

    void initialize(const CohesiveState& initial_a,
                    const size_t factorized_states_count,
                    const FactorizedState& initial_b_i,
                    const Scalar sigma_a,
                    const Scalar sigma_b_i)
    {
        a = initial_a;
        cov_aa = CovAA::Identity(a_dimension(), a_dimension()) * sigma_a;

        joint_partitions.resize(factorized_states_count);
        for (auto& partition: joint_partitions)
        {
            partition.b = initial_b_i;
            partition.cov_bb = CovBB::Identity(
                        b_i_dimension(), b_i_dimension()) * sigma_b_i;
        }
    }

    size_t a_dimension()
    {
        return a.rows();
    }

    size_t b_i_dimension()
    {
        if (joint_partitions.size() > 0)
        {
            return joint_partitions[0].b.rows();
        }

        return 0;
    }

    size_t b_dimension()
    {
        return joint_partitions.size();
    }

public:
    CohesiveState a;
    CovAA cov_aa;
    CovAA cov_aa_inverse;
    std::vector<JointPartitions> joint_partitions;
};

}

#endif
