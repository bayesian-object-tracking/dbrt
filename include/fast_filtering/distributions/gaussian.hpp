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

#ifndef FAST_FILTERING_DISTRIBUTIONS_GAUSSIAN_HPP
#define FAST_FILTERING_DISTRIBUTIONS_GAUSSIAN_HPP

// eigen
#include <Eigen/Dense>

// state_filtering
#include <fl/util/assertions.hpp>
#include <fast_filtering/distributions/interfaces/moments.hpp>
#include <fast_filtering/distributions/interfaces/evaluation.hpp>
#include <fast_filtering/distributions/interfaces/gaussian_map.hpp>

namespace ff
{

// Forward declarations
template <typename Vector> class Gaussian;

namespace internal
{
/**
 * Gaussian distribution traits specialization
 * \internal
 */
template <typename Vector_>
struct Traits<Gaussian<Vector_> >
{
    typedef Vector_ Vector;
    typedef typename Vector::Scalar Scalar;
    typedef Eigen::Matrix<Scalar, Vector::SizeAtCompileTime, 1>  Noise;

    typedef Eigen::Matrix<Scalar, Vector::SizeAtCompileTime,
                                  Vector::SizeAtCompileTime> Operator;

    typedef Moments<Vector, Operator>          MomentsBase;
    typedef Evaluation<Vector, Scalar>         EvaluationBase;
    typedef GaussianMap<Vector, Noise>         GaussianMapBase;

};
}


/**
 * \class Gaussian
 * \ingroup distributions
 */
template <typename Vector_>
class Gaussian:
        public internal::Traits<Gaussian<Vector_> >::MomentsBase,
        public internal::Traits<Gaussian<Vector_> >::EvaluationBase,
        public internal::Traits<Gaussian<Vector_> >::GaussianMapBase
{
public:
    typedef internal::Traits<Gaussian<Vector_> > Traits;

    typedef typename Traits::Vector     Vector;
    typedef typename Traits::Scalar     Scalar;
    typedef typename Traits::Operator   SecondMoment;
    typedef typename Traits::Noise      Noise;

public:
    explicit Gaussian(const unsigned& new_dimension = Vector::SizeAtCompileTime):
        Traits::GaussianMapBase(new_dimension)
    {
        //static_assert_dynamic_sized(Vector);

//        static_assert_base(Vector, Eigen::Matrix<Scalar, Eigen::Dynamic, 1>);
        static_assert_base(Vector,
                           Eigen::Matrix<Scalar, Vector::SizeAtCompileTime, 1>);

        mean_.resize(dimension(), 1);
        covariance_.resize(dimension(), dimension());
        precision_.resize(dimension(), dimension());
        cholesky_factor_.resize(dimension(), dimension());

        set_standard();
    }

//    Gaussian()
//    {
//        //static_assert_const_sized(Vector);

//        // make sure that vector is derived from eigen
//        static_assert_base(Vector,
//                           Eigen::Matrix<Scalar, Vector::SizeAtCompileTime, 1>);

//        mean_.resize(Dimension(), 1);
//        covariance_.resize(Dimension(), Dimension());
//        precision_.resize(Dimension(), Dimension());
//        cholesky_factor_.resize(Dimension(), Dimension());

//        SetStandard();
//    }


    virtual ~Gaussian() { }

    virtual Vector map_standard_normal(const Noise& sample) const
    {
        return mean_ + cholesky_factor_ * sample;
    }

    virtual void set_standard()
    {
        full_rank_ = true;
        mean(Vector::Zero(dimension()));
        covariance(SecondMoment::Identity(dimension(), dimension()));
    }

    virtual void mean(const Vector& new_mean)
    {
        mean_ = new_mean;
    }

    virtual void covariance(const SecondMoment& new_covariance)
    {
        covariance_ = new_covariance;

        // we assume that the input matrix is positive semidefinite
        Eigen::LDLT<SecondMoment> ldlt;
        ldlt.compute(covariance_);
        SecondMoment L = ldlt.matrixL();
        Vector D_sqrt = ldlt.vectorD();
        for(size_t i = 0; i < D_sqrt.rows(); i++)
            D_sqrt(i) = std::sqrt(std::fabs(D_sqrt(i)));
        cholesky_factor_ = ldlt.transpositionsP().transpose()*L*D_sqrt.asDiagonal();

        if(new_covariance.colPivHouseholderQr().rank() == new_covariance.rows())
        {
            full_rank_ = true;
            precision_ = covariance_.inverse();
            log_normalizer_ = -0.5 * ( log(covariance_.determinant()) + double(new_covariance.rows()) * log(2.0 * M_PI) );
        }
        else
            full_rank_ = false;
    }


    virtual void diagonal_covariance(const SecondMoment& covariance)
    {
        covariance_ = covariance;

        double determinant = 1;
        precision_ = SecondMoment::Zero(covariance_.rows(), covariance_.cols());
        cholesky_factor_ = SecondMoment::Zero(covariance_.rows(), covariance_.cols());
        full_rank_ = true;
        for(size_t i = 0; i < covariance_.rows(); i++)
        {
            determinant *= covariance(i, i);
            precision_(i,i) =  1.0 / covariance_(i, i);
            if(!std::isfinite(precision_(i,i)))
                full_rank_ = false;

            cholesky_factor_(i,i) = std::sqrt(covariance_(i,i));
        }

        log_normalizer_ = -0.5 * ( std::log(determinant)
                                + double(covariance.rows()) * std::log(2.0 * M_PI) );
    }


    virtual Vector mean() const
    {
        return mean_;
    }

    virtual SecondMoment covariance() const
    {
        return covariance_;
    }

    virtual Scalar LogProbability(const Vector& vector) const
    {
        if(full_rank_)
            return log_normalizer_ - 0.5 * (vector - mean_).transpose() * precision_ * (vector - mean_);
        else
            return -std::numeric_limits<Scalar>::infinity();
    }

    virtual int dimension() const
    {
        return this->standard_variate_dimension(); // all dimensions are the same
    }


protected:
    Vector mean_;
    SecondMoment covariance_;
    bool full_rank_;
    SecondMoment precision_;
    SecondMoment cholesky_factor_;
    Scalar log_normalizer_;
};

}

#endif
