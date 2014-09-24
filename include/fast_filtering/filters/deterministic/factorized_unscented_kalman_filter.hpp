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
         typename ObservationModel,
         size_t FACTORIZED_STATES = -1>
class FactorizedUnscentedKalmanFilter
{
public:
    typedef ComposedStateDistribution<typename CohesiveStateProcessModel::State,
                                      typename FactorizedStateProcessModel::State,
                                      FACTORIZED_STATES> StateDistribution;

    typedef typename StateDistribution::CovAA CovAA;
    typedef typename StateDistribution::CovBB CovBB;
    typedef typename StateDistribution::CovYY CovYY;
    typedef typename StateDistribution::CovAY CovAY;
    typedef typename StateDistribution::CovAB CovAB;
    typedef typename StateDistribution::CovBY CovBY;

    typedef Eigen::Matrix<typename StateDistribution::Scalar,
                          Eigen::Dynamic,
                          Eigen::Dynamic> SigmaPoints;

    typedef typename CohesiveStateProcessModel::State State_a;
    typedef typename FactorizedStateProcessModel::State State_b_i;

    typedef boost::shared_ptr<CohesiveStateProcessModel> CohesiveStateProcessModelPtr;
    typedef boost::shared_ptr<FactorizedStateProcessModel> FactorizedStateProcessModelPtr;
    typedef boost::shared_ptr<ObservationModel> ObservationModelPtr;

    enum RandomVariableIndex { a = 0, Q_a , b_i, Q_bi, R_yi, y_i = R_yi };

public:
    FactorizedUnscentedKalmanFilter(
            const CohesiveStateProcessModelPtr cohesive_state_process_model,
            const FactorizedStateProcessModelPtr factorized_state_process_model,
            const ObservationModelPtr observation_model,
            double kappa = 1.):
        f_a_(cohesive_state_process_model),
        f_b_(factorized_state_process_model),
        h_(observation_model),
        kappa_(kappa)
    {
        alpha_ = 1.2;
        beta_ = 2.;
        kappa_ = 0.;
    }

    virtual ~FactorizedUnscentedKalmanFilter() { }

    /**
     * Predicts the state for the next time step
     *
     * @param [in]  prior_state         State prior distribution
     * @param [out] predicted_state     Predicted state posterior distribution
     */
    void predict(const StateDistribution& prior_state,
                 StateDistribution& predicted_state)
    {
        // compute the sigma point partitions X = [Xa  XQa  0(b^[i])  XQb  XR]
        // 0(b^[i]) is the place holder for the a b^[i]
        ComputeSigmaPointPartitions(
            {
                { prior_state.a_,                      prior_state.cov_aa_ },
                { Eigen::MatrixXd::Zero(Dim(Q_a),  1), f_a_->NoiseCovariance() },
                { Eigen::MatrixXd::Zero(Dim(b_i),  1), Eigen::MatrixXd::Zero(Dim(b_i), Dim(b_i)) },
                { Eigen::MatrixXd::Zero(Dim(Q_bi), 1), f_b_->NoiseCovariance() },
                { Eigen::MatrixXd::Zero(Dim(R_yi), 1), h_->NoiseCovariance() }
            },
            X_);

        // FOR ALL X_[a]
        // X_[a] = f_a_ -> predict(X_[a], X_[Q_bi]);

        // predict the cohesive state segment a
        Mean(X_[a], predicted_state.a_);
        Normalize(predicted_state.a_, X_[a]);
        predicted_state.cov_aa_ = X_[a] * X_[a].transpose();

        // predict the joint state [a  b_i  y_i]
        for (size_t i = 0; i < prior_state.joint_partitions_.size(); ++i)
        {
            ComputeSigmaPoints(prior_state.joint_partitions_[i].b,
                               prior_state.joint_partitions_[i].cov_bb,
                               Dim(a) + Dim(Q_a),
                               X_[b_i]);

            // FOR ALL X_[b_i] and X_[y_i]
            //X_[b_i] = f_b_ -> predict(X_[b_i], X_[Q_bi]);
            //X_[y_i] = h_   -> predict(X_[a],   X_[Q_bi], X_[R_yi]);

            typename StateDistribution::JointPartitions& predicted_partition =
                    predicted_state.joint_partitions_[i];

            Mean(X_[b_i], predicted_partition.b);
            Mean(X_[y_i], predicted_partition.y);

            Normalize(predicted_partition.b, X_[b_i]);
            Normalize(predicted_partition.y, X_[y_i]);

            predicted_partition.cov_ab = X_[a] * X_[b_i].transpose();
            predicted_partition.cov_ay = X_[a] * X_[y_i].transpose();
            predicted_partition.cov_bb = X_[b_i] * X_[b_i].transpose();
            predicted_partition.cov_by = X_[b_i] * X_[y_i].transpose();
            predicted_partition.cov_yy = X_[y_i] * X_[y_i].transpose();
        }
    }

    /**
     * Update the predicted_state and store the result in the posterior_state
     * The update step involves updating the cohesive state followed by the
     * update of the factorized part
     *
     * @param [in]  predicted_state     Propagated state
     * @param [in]  y                   Measurement
     * @param [out] posterior_state     Updated posterior state
     */
    void update(const StateDistribution& predicted_state,
                const Eigen::MatrixXd& y,
                StateDistribution& posterior_state)
    {
        update_a(predicted_state, y, posterior_state);
        update_b(predicted_state, y, posterior_state);
    }


    /**
     * Update the cohesive predicted_state part a
     *
     * @param [in]  predicted_state     Propagated state
     * @param [in]  y                   Measurement
     * @param [out] posterior_state     Updated posterior state
     */
    void update_a(const StateDistribution& predicted_state,
                  const Eigen::MatrixXd& y,
                  StateDistribution& posterior_state)
    {
        size_t dim_b = predicted_state.b_dimension();

        Eigen::MatrixXd A(dim_b, Dim(a));
        Eigen::MatrixXd mu_y(dim_b, 1);
        Eigen::MatrixXd cov_yy_given_a_inv(dim_b, 1);

        predicted_state.cov_aa_inverse_ = predicted_state.cov_aa_.inverse();
        CovAA& cov_aa_inv = predicted_state.cov_aa_inverse_;

        for (size_t i = 0; i < dim_b; ++i)
        {
            CovAY& cov_ay = predicted_state.joint_partitions_[i].cov_ay_;
            CovYY& cov_yy = predicted_state.joint_partitions_[i].cov_yy_;

            A.block(i, 0, 1, Dim(a)) = cov_ay.transpose();
            mu_y.block(i, 0, 1, 1) = predicted_state.joint_partitions_[i].y;

            cov_yy_given_a_inv.block(i, 0, 1, 1) =
                    cov_yy - cov_ay.transpose() * cov_aa_inv * cov_ay;
        }
        A = A * cov_aa_inv;

        InvertDiagonalAsVector(cov_yy_given_a_inv, cov_yy_given_a_inv);
        Eigen::MatrixXd AT_Cov_yy_given_a = A.transpose() * cov_yy_given_a_inv.asDiagonal();

        // update cohesive state segment
        posterior_state.cov_aa_ = (cov_aa_inv + AT_Cov_yy_given_a * A).inverse();

        posterior_state.a_ =
                predicted_state.a_ +
                posterior_state.cov_aa_ * AT_Cov_yy_given_a * (y - mu_y);
    }

    /**
     * Update the factorized predicted_state part b given the updated cohesive
     * part a.
     *
     * @param [in]  predicted_state     Propagated state
     * @param [in]  y                   Measurement
     * @param [out] posterior_state     Updated posterior state
     */
    void update_b(const StateDistribution& predicted_state,
                  const Eigen::MatrixXd& y,
                  StateDistribution& posterior_state)
    {
        Eigen::MatrixXd L;
        Eigen::MatrixXd L_aa;
        Eigen::MatrixXd L_ay;
        Eigen::MatrixXd L_ya;
        Eigen::MatrixXd L_yy;

        size_t dim_b = predicted_state.b_dimension();

        Eigen::MatrixXd B;
        Eigen::MatrixXd c;
        Eigen::MatrixXd ba_by;
        Eigen::MatrixXd K;
        Eigen::MatrixXd innov;
        Eigen::MatrixXd cov_b_given_a_y;
        ba_by.resize(Dim(b_i), Dim(a) + Dim(y_i));
        innov.resize(Dim(a) + Dim(y_i), 1);
        innov.block(0, 0, Dim(a), 1) = -predicted_state.a_;

        CovAA& cov_aa_inv = predicted_state.cov_aa_inverse_;

        for (size_t i = 0; i < dim_b; ++i)
        {
            CovAY& cov_ay = predicted_state.joint_partitions_[i].cov_ay_;
            CovAB& cov_ab = predicted_state.joint_partitions_[i].cov_ab_;
            CovBY& cov_by = predicted_state.joint_partitions_[i].cov_by_;
            CovBB& cov_bb = predicted_state.joint_partitions_[i].cov_bb_;
            CovYY& cov_yy = predicted_state.joint_partitions_[i].cov_yy_;

            SMWInversion(cov_aa_inv, cov_ay, cov_ay.transpose(), cov_yy,
                         L_aa, L_ay, L_ya, L_yy,
                         L);

            B = cov_ab.transpose() * L_aa  +  cov_by * L_ya;

            ba_by.block(0, 0,      Dim(b_i), Dim(a)) = cov_ab.transpose();
            ba_by.block(0, Dim(a), Dim(b_i), Dim(y_i)) = cov_by;

            K = ba_by * L;
            innov.block(Dim(a), 0, Dim(y_i), 1) =
                    y(i, 0) - predicted_state.joint_partitions_[i].y;

            c = predicted_state.joint_partitions_[i].b + K * innov;

            cov_b_given_a_y = cov_bb - K * ba_by.transpose();

            // update b_[i]
            posterior_state.joint_partitions_[i].b =
                    B * posterior_state.a_ + c;
            posterior_state.joint_partitions_[i].cov_bb_ =
                    cov_b_given_a_y
                    - B * posterior_state.cov_aa_ * B.transpose();
        }
    }


public:
    /**
     * @brief Dim Returns the dimension of the specified random variable ID
     *
     * @param var_id    ID of the requested random variable
     */
    size_t Dim(RandomVariableIndex var_id)
    {
        switch(var_id)
        {
        case a:     return f_a_->Dimension();
        case Q_a:   return f_a_->NoiseDimension();
        case b_i:   return f_b_->Dimension();
        case Q_bi:  return f_b_->NoiseDimension();
        case y_i:   return 1;
        //case R_yi:  return 1;
        }
    }

    /**
     * Computes the weighted mean of the given sigma points
     *
     * @note TESTED
     */
    template <typename MeanVector>
    void Mean(const SigmaPoints& sigma_points, MeanVector& mean)
    {
        double w_0;
        double w_i;
        ComputeWeights(sigma_points.cols(), w_0, w_i);

        mean = w_0 * sigma_points.col(0);

        for (size_t i = 1; i < sigma_points.cols(); ++i)
        {
            mean +=  w_i * sigma_points.col(i);
        }
    }

    /**
     * Normalizes the given sigma point such that they represent zero mean
     * weighted points.
     * @param [in]  mean          Mean of the sigma points
     * @param [in]  w             Weights of the points used to determine the
     *                            covariance
     * @param [out] sigma_points  The sigma point collection
     *
     * @note TESTED
     */
    template <typename MeanVector>    
    void Normalize(const MeanVector& mean, SigmaPoints& sigma_points)
    {
        double w_0_sqrt;
        double w_i_sqrt;
        ComputeWeights(sigma_points.cols(), w_0_sqrt, w_i_sqrt);

        w_0_sqrt += (1 - alpha_ * alpha_ + beta_);

        w_0_sqrt = std::sqrt(w_0_sqrt);
        w_i_sqrt = std::sqrt(w_i_sqrt);

        sigma_points.col(0) = w_0_sqrt * (sigma_points.col(0) - mean);

        for (size_t i = 1; i < sigma_points.cols(); ++i)
        {
            sigma_points.col(i) = w_i_sqrt * (sigma_points.col(i) - mean);
        }
    }

    /**
     * Computes the Unscented Transform weights according to the specified
     * sigmapoints
     *
     * @param [in]  number_of_sigma_points
     * @param [out] w_0_sqrt    The weight for the first sigma point
     * @param [out] w_i_sqrt    The weight for the remaining sigma points
     *
     * @note TESTED
     */
    void ComputeWeights(size_t number_of_sigma_points,
                        double& w_0,
                        double& w_i)
    {
        size_t dimension = (number_of_sigma_points - 1) / 2;

        double lambda = alpha_ * alpha_ * (double(dimension) + kappa_)
                        - double(dimension);
        w_0 = lambda / (double(dimension) + lambda);
        w_i = 1. / (2. * (double(dimension) + lambda));
    }

    /**
     * Computes the sigma point partitions for all specified moment pairs
     *
     * @param moments_list              Moment pair of each partition.
     *                                  For a place holder, the first moment
     *                                  (the mean) must have the required number
     *                                  of rows and 0 columns.
     * @param sigma_point_partitions    sigma point partitions
     *
     * @note TESTED
     */
    void ComputeSigmaPointPartitions(
            const std::vector<std::pair<Eigen::MatrixXd,
                                        Eigen::MatrixXd> >& moment_pairs,
            std::vector<SigmaPoints>& X)
    {
        size_t dim = 0;
        for (auto& moments : moment_pairs) { dim += moments.first.rows(); }
        size_t number_of_points = 2 * dim + 1;

        X.resize(moment_pairs.size());

        size_t offset = 0;
        size_t i = 0;
        for (auto& moments : moment_pairs)
        {
            X[i].resize(moments.first.rows(), number_of_points);

            // check whether this is only a place holder
            if (!moments.second.isZero())
            {
                ComputeSigmaPoints(moments.first, moments.second, offset, X[i]);
            }

            offset += moments.first.rows();
            i++;
        }
    }

    /**
     * Computes the sigma point partition. Given a random variable X and its
     * partitions [Xa  Xb  Xc] this function computes the sigma points of
     * a single partition, say Xb. This is done such that the number of sigma
     * points in Xb is 2*dim([Xa  Xb  Xc])+1. In so doing, the sigma points of X
     * can be computed partition wise, and ultimately augmented together.
     *
     * @param [in]  mean          First moment
     * @param [in]  covariance    Second centered moment
     * @param [in]  offset        Offset dimension if this transform is a
     *                            partition of a larger one
     * @param [out] sigma_points  Selected sigma points
     *
     * @note TESTED
     */
    template <typename MeanVector, typename CovarianceMatrix>
    void ComputeSigmaPoints(const MeanVector& mean,
                            const CovarianceMatrix& covariance,
                            const size_t offset,
                            SigmaPoints& sigma_points)
    {
        // assert sigma_points.rows() == mean.rows()
        size_t joint_dimension = (sigma_points.cols() - 1) / 2;
        CovarianceMatrix covarianceSqr = covariance.llt().matrixL();

        covarianceSqr *= std::sqrt((double(joint_dimension)
                                    + alpha_ * alpha_ * (double(joint_dimension) + kappa_)
                                    - double(joint_dimension)));

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

    /**
     * @note TESTED
     */
    template <typename RegularMatrix, typename SquareRootMatrix>
    void SquareRoot(const RegularMatrix& regular_matrix,
                    SquareRootMatrix& square_root)
    {
        square_root = regular_matrix.llt().matrixL();
    }

    /**
     * @note TESTED
     */
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

    /**
     * @note TESTED
     */
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

    /**
     * Blockweise matrix inversion using the Sherman-Morrision-Woodbury
     * indentity given that \f$\Sigma^{-1}_{aa}\f$ of
     *
     * \f$
     *  \begin{pmatrix} \Sigma_{aa} & \Sigma_{ab} \\
     *                  \Sigma_{ba} & \Sigma_{bb} \end{pmatrix}^{-1}
     * \f$
     *
     * is already available.
     *
     * \f$
     *  \begin{pmatrix} \Sigma_{aa} & \Sigma_{ab} \\
     *                  \Sigma_{ba} & \Sigma_{bb} \end{pmatrix}^{-1} =
     * \begin{pmatrix} \Lambda_{aa} & \Lambda_{ab} \\
     *                 \Lambda_{ba} & \Lambda_{bb} \end{pmatrix} = \Lambda
     * \f$
     *
     * @param [in]  A_inv   \f$ \Sigma^{-1}_{aa} \f$
     * @param [in]  B       \f$ \Sigma_{ab} \f$
     * @param [in]  C       \f$ \Sigma_{ba} \f$
     * @param [in]  D       \f$ \Sigma_{bb} \f$
     * @param [out] L_A     \f$ \Lambda_{aa} \f$
     * @param [out] L_B     \f$ \Lambda_{ab} \f$
     * @param [out] L_C     \f$ \Lambda_{ba} \f$
     * @param [out] L_D     \f$ \Lambda_{bb} \f$
     * @param [out] L       \f$ \Lambda \f$
     *
     * @note TESTED
     */
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

    /**
     * Blockweise matrix inversion using the Sherman-Morrision-Woodbury
     * indentity given that \f$\Sigma^{-1}_{aa}\f$ of
     *
     * \f$
     *  \begin{pmatrix} \Sigma_{aa} & \Sigma_{ab} \\
     *                  \Sigma_{ba} & \Sigma_{bb} \end{pmatrix}^{-1}
     * \f$
     *
     * is already available.
     *
     * \f$
     *  \begin{pmatrix} \Sigma_{aa} & \Sigma_{ab} \\
     *                  \Sigma_{ba} & \Sigma_{bb} \end{pmatrix}^{-1} =
     * \begin{pmatrix} \Lambda_{aa} & \Lambda_{ab} \\
     *                 \Lambda_{ba} & \Lambda_{bb} \end{pmatrix} = \Lambda
     * \f$
     *
     * @param [in]  A_inv   \f$ \Sigma^{-1}_{aa} \f$
     * @param [in]  B       \f$ \Sigma_{ab} \f$
     * @param [in]  C       \f$ \Sigma_{ba} \f$
     * @param [in]  D       \f$ \Sigma_{bb} \f$
     * @param [out] L_A     \f$ \Lambda_{aa} \f$
     * @param [out] L_B     \f$ \Lambda_{ab} \f$
     * @param [out] L_C     \f$ \Lambda_{ba} \f$
     * @param [out] L_D     \f$ \Lambda_{bb} \f$
     * @param [out] L       \f$ \Lambda \f$
     *
     *  @note TESTED
     */
    template <typename MatrixAInv,
              typename MatrixB,
              typename MatrixC,
              typename MatrixD,
              typename MatrixLA,
              typename MatrixLB,
              typename MatrixLC,
              typename MatrixLD,
              typename ResultMatrix>
    void SMWInversion(const MatrixAInv& A_inv,
                      const MatrixB& B,
                      const MatrixC& C,
                      const MatrixD& D,
                      MatrixLA& L_A,
                      MatrixLB& L_B,
                      MatrixLC& L_C,
                      MatrixLD& L_D,
                      ResultMatrix& L)
    {
        SMWInversion(A_inv, B, C, D, L_A, L_B, L_C, L_D);

        L.resize(L_A.rows() + L_C.rows(), L_A.cols() + L_B.cols());

        L.block(0,          0,          L_A.rows(), L_A.cols()) = L_A;
        L.block(0,          L_A.cols(), L_B.rows(), L_B.cols()) = L_B;
        L.block(L_A.rows(), 0,          L_C.rows(), L_C.cols()) = L_C;
        L.block(L_A.rows(), L_A.cols(), L_D.rows(), L_D.cols()) = L_D;
    }

protected:
    CohesiveStateProcessModelPtr f_a_;
    FactorizedStateProcessModelPtr f_b_;
    ObservationModelPtr h_;

    double kappa_;
    double beta_;
    double alpha_;

    // sigma points
    std::vector<SigmaPoints> X_;
};

}

#endif
