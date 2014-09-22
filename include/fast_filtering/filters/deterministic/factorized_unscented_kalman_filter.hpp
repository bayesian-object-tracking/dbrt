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

#include <cmath>

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

    typedef std::vector<double> Weights;

//    struct SigmaPoints:
//        public SigmaPointMatrix
//    {
//        SigmaPoints() { }
//        SigmaPoints(size_t rows, size_t cols): SigmaPointMatrix(rows, cols) { }

//        SigmaPoints operator=(const SigmaPoints& sigma_points)
//        {
//            w_m = sigma_points.w_m;
//            w_c = sigma_points.w_c;
//        }

//        SigmaPoints operator=(const SigmaPointMatrix& sigma_points)
//        {
//            w_m = 1;
//            w_c = 1;
//        }

//        double w_c;
//        double w_m;
//    };

    typedef boost::shared_ptr<CohesiveStateProcessModel> CohesiveStateProcessModelPtr;
    typedef boost::shared_ptr<FactorizedStateProcessModel> FactorizedStateProcessModelPtr;
    typedef boost::shared_ptr<ObservationModel> ObservationModelPtr;

public:
    FactorizedUnscentedKalmanFilter(
            const CohesiveStateProcessModelPtr cohesive_state_process_model,
            const FactorizedStateProcessModelPtr factorized_state_process_model,
            const ObservationModelPtr observation_model):
        cohesive_state_process_model_(cohesive_state_process_model),
        factorized_state_process_model_(factorized_state_process_model),
        observation_model_(observation_model)
    {
        dim_a_ = cohesive_state_process_model_->Dimension();
        dim_Qa_ = cohesive_state_process_model_->NoiseDimension();

        dim_b_i_ = factorized_state_process_model_->Dimension();
        dim_Qb_i_ = factorized_state_process_model_->NoiseDimension();

        dim_y_i_ = 1;
        dim_R_ = observation_model_->NoiseDimension();

        dim_joint_ = dim_a_ + dim_b_i_ + dim_y_i_;
        number_of_points_ = 2 * dim_joint_ + 1;

        // cohesive state sigma points Xa
        Xa_ = SigmaPoints(dim_a_, number_of_points_);
        XQa_ = SigmaPoints(dim_Qa_, number_of_points_);

        // factorized state sigma points Xb_i
        XQb_ = SigmaPoints(dim_Qb_i_, number_of_points_);

        // measurement noise X_R
        Xy_ = SigmaPoints(1, number_of_points_);
        XR_ = SigmaPoints(1, number_of_points_);

        zero_noise_a_ = Eigen::MatrixXd::Zero(dim_Qa_, 1);
        zero_noise_b_ = Eigen::MatrixXd::Zero(dim_Qb_i_, 1);
        zero_noise_y_ = Eigen::MatrixXd::Zero(dim_R_, 1);

        dummy_b_i_ = Eigen::MatrixXd(dim_b_i_, 0);
        dummy_cov_bb_i_ = Eigen::MatrixXd(dim_b_i_, 0);

        w_m_.resize(number_of_points_, 1./double(number_of_points_));
        w_c_.resize(number_of_points_, 1./double(number_of_points_));
    }

    virtual ~FactorizedUnscentedKalmanFilter() { }

    void predict(const StateDistribution& prior_state,
                 StateDistribution& predicted_state)
    {
        if (Xb_.size() == 0)
        {
            size_t dim_b = prior_state.FactorizedStatesCount();
            Xb_.resize(dim_b, SigmaPoints(dim_b_i_, number_of_points_));
        }

        // compute the sigma point partitions X = [Xa  XQa  0(b^[i])  XQb  XR]
        // 0(b^[i]) is the place holder for the a b^[i]
        ComputeSigmaPointPartitions(
            { {prior_state.state_a_, prior_state.cov_aa_},
              {zero_noise_a_, cohesive_state_process_model_->NoiseCovariance()},
              {dummy_b_i_, dummy_cov_bb_i_},
              {zero_noise_b_, factorized_state_process_model_->NoiseCovariance()},
              {zero_noise_y_, observation_model_->NoiseCovariance()} },
            X_partitions_);

        Xa_ = X_partitions_[0];
        XQa_ = X_partitions_[1];
        XQb_ = X_partitions_[3];
        XR_ = X_partitions_[4];

        /*  PROCESS Xa[j] = f_a(Xa[j], XQa[j])  */

        // predict the cohersive state segment a
        SigmaPointsMean(Xa_, w_m_, predicted_state.state_a_);
        NormalizeSigmaPoints(predicted_state.state_a_,  w_c_, Xa_);
        predicted_state.cov_aa_ = Xa_ * Xa_.transpose();
        predicted_state.cov_aa_inverse_ = predicted_state.cov_aa_.inverse();

        // predict the joint state [a  b_i  y_i]
        for (size_t i = 0; i < prior_state.joint_partitions_.size(); ++i)
        {
            typename StateDistribution::JointPartitions& predicted_partition =
                    predicted_state.joint_partitions_[i];

            ComputeSigmaPoints(prior_state.joint_partitions_[i].b,
                               prior_state.joint_partitions_[i].cov_bb,
                               dim_a_ + dim_Qa_,
                               Xb_[i]);

            /*  PROCESS Xb[i, j] = f_a(Xb[i, j], XQb[i, j])  */
            /*  PROCESS Xy_[j] = h(Xa[j], Xb[i, j], XR_[j])  */

            SigmaPointsMean(Xb_[i], w_m_, predicted_partition.b);
            SigmaPointsMean(Xy_, w_m_, predicted_partition.y);

            NormalizeSigmaPoints(predicted_partition.b, w_c_, Xb_[i]);
            NormalizeSigmaPoints(predicted_partition.y, w_c_, Xy_);

            predicted_partition.cov_ab = Xa_ * Xb_[i].transpose();
            predicted_partition.cov_ay = Xa_ * Xy_.transpose();
            predicted_partition.cov_bb = Xb_[i] * Xb_[i].transpose();
            predicted_partition.cov_by = Xb_[i] * Xy_.transpose();
            predicted_partition.cov_yy = Xy_ * Xy_.transpose();
        }
    }

    void update(const StateDistribution& predicted_state,
                StateDistribution& posterior_state,
                Eigen::MatrixXd y)
    {
        size_t dim_b = predicted_state.FactorizedStatesCount();

        Eigen::MatrixXd A(dim_b, dim_a_);
        Eigen::MatrixXd b(dim_b, 1);
        Eigen::MatrixXd Cov_yy_given_a(dim_b, 1);
        Eigen::MatrixXd Cov_yy_given_a_inv(dim_b, 1);
        Eigen::MatrixXd AT_Cov_yy_given_a;

        typedef typename StateDistribution::CovAY CovAA;
        typedef typename StateDistribution::CovAY CovAY;
        typedef typename StateDistribution::CovYY CovYY;

        CovAA& cov_aa = predicted_state.cov_aa_;
        CovAA& cov_aa_inv = predicted_state.cov_aa_inverse_;

        for (size_t i = 0; i < dim_b; ++i)
        {
            CovAY& cov_ay = predicted_state.joint_partitions_[i].cov_ay_;
            CovYY& cov_yy = predicted_state.joint_partitions_[i].cov_yy_;

            A.block(i, 0, 1, dim_a_) = cov_ay.transpose();
            b.block(i, 0, 1, 1) = predicted_state.joint_partitions_[i].y;

            Cov_yy_given_a.block(i, 0, 1, 1) =
                    cov_yy - cov_ay.transpose() * cov_aa_inv * cov_ay;
        }
        A = A * predicted_state.cov_aa_inverse_;
        b -= A * predicted_state.state_a_;

        InvertDiagonalAsVector(Cov_yy_given_a, Cov_yy_given_a_inv);

        AT_Cov_yy_given_a = A.transpose() * Cov_yy_given_a_inv;

        // update cohesive state segment
        posterior_state.state_a_ =
                predicted_state.state_a_
                + cov_aa * AT_Cov_yy_given_a * (y - A * predicted_state.state_a_ + b);

        posterior_state.cov_aa_ = (cov_aa_inv + AT_Cov_yy_given_a * A).inverse();
    }

public:
    template <typename MeanVector>
    void SigmaPointsMean(const SigmaPoints& sigma_points,
                         const Weights& w,
                         MeanVector& mean)
    {
        mean.setZero();

        for (size_t i = 0; i < sigma_points.cols(); ++i)
        {
            mean += w[i] * sigma_points[i];
        }
    }

    template <typename MeanVector>
    void NormalizeSigmaPoints(const MeanVector& mean,
                              const Weights& w,
                              SigmaPoints& sigma_points)
    {
        for (size_t i = 0; i < sigma_points.cols(); ++i)
        {
            sigma_points[i] = std::sqrt(w[i]) * (sigma_points[i] - mean);
        }
    }

    void ComputeSigmaPointPartitions(
            const std::vector<std::pair<Eigen::MatrixXd,
                                        Eigen::MatrixXd> >& moments_list,
            std::vector<SigmaPoints>& sigma_point_partitions)
    {
        size_t dim = 0;
        for (auto& moments : moments_list)
        {
            dim += moments.first.rows();
        }
        size_t number_of_points = 2 * dim + 1;

        Eigen::MatrixXd sigma_points;
        size_t offset = 0;
        for (auto& moments : moments_list)
        {
            sigma_points = SigmaPoints(moments.first.rows(), number_of_points);
            // check whether this is only a place holder
            if (moments.first.cols() > 0)
            {
                ComputeSigmaPoints(moments.first,
                                   moments.second,
                                   offset,
                                   sigma_points);
            }
            sigma_point_partitions.push_back(sigma_points);

            offset += moments.first.rows();
        }
    }

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

    template <typename RegularMatrix, typename SquareRootMatrix>
    void SquareRoot(const RegularMatrix& regular_matrix,
                    SquareRootMatrix& square_root)
    {
        square_root = regular_matrix.llt().matrixL();
    }

    template <typename RegularMatrix>
    void SquareRootDiagonal(const RegularMatrix& regular_matrix,
                            RegularMatrix& square_root)
    {
        square_root = regular_matrix;
        for (size_t i = 0; i < square_root.rows(); ++i)
        {
            square_root(i, i) = std::sqrt(square_root(i, i));
        }
    }

    template <typename RegularMatrix, typename SquareRootVector>
    void SquareRootDiagonalAsVector(const RegularMatrix& regular_matrix,
                                  SquareRootVector& square_root)
    {
        square_root = regular_matrix;
        for (size_t i = 0; i < square_root.rows(); ++i)
        {
            square_root(i, 0) = std::sqrt(square_root(i, 0));
        }
    }

    template <typename DiagonalMatrix>
    void InvertDiagonalAsVector(const DiagonalMatrix& diagonal,
                                DiagonalMatrix& diagonal_inverse)
    {
        diagonal_inverse.resize(diagonal.rows(), 1);

        for (size_t i = 0; i < diagonal.rows(); ++i)
        {
            diagonal_inverse(i, 0) = 1./diagonal(i, 0);
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

    size_t dim_a_;
    size_t dim_Qa_;
    size_t dim_b_i_;
    size_t dim_Qb_i_;
    size_t dim_y_i_;
    size_t dim_R_;
    size_t dim_joint_;
    size_t number_of_points_;

    // sigma points
    SigmaPoints Xa_;
    SigmaPoints XQa_;
    std::vector<SigmaPoints> Xb_;
    SigmaPoints XQb_;
    SigmaPoints Xy_;
    SigmaPoints XR_;

    Eigen::MatrixXd zero_noise_a_;
    Eigen::MatrixXd zero_noise_b_;
    Eigen::MatrixXd zero_noise_y_;
    Eigen::MatrixXd dummy_b_i_;
    Eigen::MatrixXd dummy_cov_bb_i_;

    std::vector<SigmaPoints> X_partitions_;

    Weights w_m_;
    Weights w_c_;
};

}

#endif

