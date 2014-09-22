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

#ifndef FAST_FILTERING_FILTERS_DETERMINISTIC_FACTORIZED_UNSCENTED_KALMAN_FILTER_HPP
#define FAST_FILTERING_FILTERS_DETERMINISTIC_FACTORIZED_UNSCENTED_KALMAN_FILTER_HPP


#include <fast_filtering/distributions/sum_of_deltas.hpp>
#include <fast_filtering/filters/deterministic/composed_state_distribution.hpp>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

namespace ff
{

/**
 * \class FactorizedUnscentedKalmanFilter
 * \ingroup filters
 *
 * The Factorized UKF filters high dimensional states using high dimensional
 * measurements. The state is composed of a unfactorized low-dimensional part
 * \f$a_t\f$ and a high-dimensional fully factorised segment
 * \f$b^{[1]}_t, b^{[2]}_t, \ldots, b^{[M]}_t \f$. The two parts are predicted
 * using two different process models ProcessModelA and ProcessModelB.
 */
template<typename CohesiveStateProcessModel,
         typename FactorizedStateProcessModel,
         typename ObservationModel>
class FactorizedUnscentedKalmanFilter
{
public:
    typedef ComposedStateDistribution<typename CohesiveStateProcessModel::State,
                                      typename FactorizedStateProcessModel::State,
                                      1> StateDistribution;


    typedef Eigen::Matrix<typename StateDistribution::Scalar,
                          Eigen::Dynamic,
                          Eigen::Dynamic> JointCovariance;

    typedef Eigen::Matrix<typename StateDistribution::Scalar,
                          Eigen::Dynamic,
                          1> JointState;

    //typedef SumOfDeltas<JointState> SigmaPoints;
    typedef Eigen::Matrix<typename StateDistribution::Scalar,
                          Eigen::Dynamic,
                          Eigen::Dynamic> SigmaPoints;

    typedef boost::shared_ptr<CohesiveStateProcessModel> CohesiveStateProcessModelPtr;
    typedef boost::shared_ptr<FactorizedStateProcessModel> FactorizedStateProcessModelPtr;
    typedef boost::shared_ptr<ObservationModel> ObservationModelPtr;

public:
    FactorizedUnscentedKalmanFilter()
    {

    }

    virtual ~FactorizedUnscentedKalmanFilter() { }

    void predict(const StateDistribution& prior_state,
                 StateDistribution& predicted_state)
    {
        size_t joint_dimension = prior_state.CohesiveStatesDimension()
                                  + prior_state.FactorizedStateDimension()
                                  + 1;

        size_t number_of_points = 2 * joint_dimension + 1;
        size_t dim_a = cohesive_state_process_model_->Dimension();
        size_t dim_Qa = dim_a;
        size_t dim_b = factorized_state_process_model_->Dimension();
        size_t dim_Qb = dim_b;
        size_t dim_y = 1;


        // cohesive state sigma points Xa
        SigmaPoints Xa(dim_a, number_of_points);
        SigmaPoints XQa(dim_Qa, number_of_points);

        // factorized state sigma points Xb_i
        std::vector<SigmaPoints> Xb(prior_state.FactorizedStatesDimension(),
                                    SigmaPoints(dim_b, number_of_points));
        SigmaPoints XQb(dim_Qb, number_of_points);

        // measurement noise X_R
        SigmaPoints XR(1, number_of_points);


        ComputeSigmaPoints(prior_state.state_a_, prior_state.cov_aa_, 0, Xa);


    }

    void update(const StateDistribution& predicted_state,
                StateDistribution& posterior_state)
    {

    }

public:
    template <typename MeanVector, typename CovarianceMatrix>
    void ComputeSigmaPoints(const MeanVector& mean,
                            const CovarianceMatrix& covariance,
                            const size_t offset,
                            SigmaPoints& sigma_points)
    {
        // asster sigma_points.rows() == mean.rows()
        size_t joint_dimension = (sigma_points.cols() - 1) / 2;
        CovarianceMatrix covarianceSqr = covariance.llt().matrixL();

        sigma_points.setZero();
        sigma_points.col(0) = mean;

        MeanVector pointShift;
        for (size_t i = 1; i <= joint_dimension; ++i)
        {
            if (offset + 1 <= i && i < offset + 1 + covariance.rows())
            {
                pointShift = covarianceSqr.col(i - (offset + 1));

                sigma_points.col(i) = mean + pointShift;
                sigma_points.col(joint_dimension + i) = mean - pointShift;
            }
            else
            {
                sigma_points.col(i) = mean;
                sigma_points.col(joint_dimension + i) = mean;
            }
        }
    }

    template <typename MatrixAInv,
              typename MatrixB,
              typename MatrixC,
              typename MatrixD,
              typename ResultMatrix>
    void SMWInversion(const MatrixAInv& A_inv,
                      const MatrixB& B,
                      const MatrixC& C,
                      const MatrixD& D,
                      ResultMatrix& inv)
    {
        Eigen::MatrixXd CAinv = C * A_inv;
        Eigen::MatrixXd AinvB = A_inv * B;
        Eigen::MatrixXd D_m_CAinvB__inv = D - C * AinvB;
        D_m_CAinvB__inv = D_m_CAinvB__inv.inverse();

        Eigen::MatrixXd D_m_CAinvB__inv_CAinv = D_m_CAinvB__inv * CAinv;

        inv.resize(A_inv.rows() + C.rows(), A_inv.cols() + B.cols());

        inv.block(0, 0, A_inv.rows(), A_inv.rows()) = A_inv + AinvB * D_m_CAinvB__inv_CAinv;
        inv.block(0, A_inv.cols(), B.rows(), B.cols()) = -(AinvB * D_m_CAinvB__inv);
        inv.block(A_inv.rows(), 0, C.rows(), C.cols()) = -(D_m_CAinvB__inv_CAinv);
        inv.block(A_inv.rows(), A_inv.cols(), D.rows(), D.cols()) = D_m_CAinvB__inv;
    }

    template <typename MatrixAInv,
              typename MatrixB,
              typename MatrixC,
              typename MatrixD,
              typename MatrixLA,
              typename MatrixLB,
              typename MatrixLC,
              typename MatrixLD>
    void SMWInversion(const MatrixAInv& A_inv,
                      const MatrixB& B,
                      const MatrixC& C,
                      const MatrixD& D,
                      MatrixLA& L_A,
                      MatrixLB& L_B,
                      MatrixLC& L_C,
                      MatrixLD& L_D)
    {
        Eigen::MatrixXd CAinv = C * A_inv;
        Eigen::MatrixXd AinvB = A_inv * B;

        L_D = (D - C * AinvB).inverse();

        Eigen::MatrixXd L_D_CAinv = L_D * CAinv;

        L_A = A_inv + AinvB * L_D_CAinv;
        L_B = -(AinvB * L_D);
        L_C = -L_D_CAinv;
    }


protected:
    CohesiveStateProcessModelPtr cohesive_state_process_model_;
    FactorizedStateProcessModelPtr factorized_state_process_model_;
    ObservationModelPtr observation_model_;
};

}

#endif

