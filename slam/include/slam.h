#pragma once

#include <algorithm>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <chrono>
#include <Eigen/Dense>
#include <exception>
#include <iterator>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>
#include <vector>

namespace Eigen
{
namespace internal
{
template <typename Scalar>
struct scalar_normal_dist_op
{
    static boost::mt19937                      rng;  // The uniform pseudo-random algorithm
    mutable boost::normal_distribution<Scalar> norm; // The gaussian combinator

    EIGEN_EMPTY_STRUCT_CTOR(scalar_normal_dist_op)

    template <typename Index>
    inline const Scalar operator()(Index, Index = 0) const
    {
        return norm(rng);
    }
};

template <typename Scalar>
boost::mt19937 scalar_normal_dist_op<Scalar>::rng;

template <typename Scalar>
struct functor_traits<scalar_normal_dist_op<Scalar>>
{
    enum
    {
        Cost         = 50 * NumTraits<Scalar>::MulCost,
        PacketAccess = false,
        IsRepeatable = false
    };
};
} // end namespace internal
} // end namespace Eigen

using Vector2f_t = Eigen::Vector2f;
using Vector2d_t = Eigen::Vector2d;
using Vector3f_t = Eigen::Vector3f;
using Vector3d_t = Eigen::Vector3d;

class Slam
{
  private:
    Eigen::MatrixXf mLM; // land marks
    Eigen::MatrixXf mWP; // way points

  public:
    // Configuration for Air bus 380
    // Basic information: Length = 73.0 m, Speed 1,185 km/h = 329.1667 m/s (136.11 m/s is perfect)
    float  mVelocity   = 83.33F;                          // m/s
    float  mMaxSWA     = std::_Pi_val / (4.0F);           // radians, maximum steering angle (-MAXG < g < MAXG)
    float  mRateSWA    = 70.0F * std::_Pi_val / (180.0F); // rad / s, maximum rate of change in steer angle
    float  mWheelBase  = 73.0F;                           // metres, vehicle wheel-base
    double mDtControls = 0.01;                            // seconds, time interval between control signals

    // control noises
    float mSigmaV   = 0.3F;                           // m/s
    float mSigmaSWA = (1.0F * std::_Pi_val / 180.0F); // radians

    // observation parameters
    float  mMaxRange  = 2000.0F;              // metres
    double mDtObserve = 5.058F * mDtControls; // seconds, time interval between observations

    // observation noises
    float mSigmaR = 0.1F;                           // metres
    float mSigmaB = (1.0F * std::_Pi_val / 180.0F); // radians

    // data association innovation gates (Mahalanobis distances)
    float mGateReject  = 50.0F;   // maximum distance for association
    float mGateAugment = 1000.0F; // minimum distance for creation of new feature

    // waypoint proximity
    float mAtWaypoint  = 1.0F; // metres, distance from current waypoint at which to switch to next waypoint
    float mNumberLoops = 1.0F; // number of loops through the waypoint list

    // resampling
    int mNumParticles = 100;                  // number of particles.
    int mNumEffective = 0.75 * mNumParticles; // minimum number of effective particles before resampling

    // switches
    bool mSwitchControlNoise     = true; // if 0, velocity and gamma are perfect
    bool mSwitchSensorNoise      = true; // if 0, measurements are perfect
    bool mSwitchInflateNoise     = true; // if 1, the estimated Q and R are inflated (ie, add stabilising noise)
    bool mSwitchHeadingKnown     = true; // if 1, the vehicle heading is observed directly at each iteration
    bool mSwitchAssociationKnown = true; // if 1, associations are given, if 0, they are estimated using gates
    bool mSwitchBatchUpdate      = true; // if 1, process scan in batch, if 0, process sequentially
    bool mSwitchSampleProposal   = true; // sample from proposal
    bool mSwitchResample         = true; // need resample or not

    Eigen::VectorXi mTABLE; // data association table

    /// @brief       EKF based SLAM for remote / valet parking / gps-denied nav.
    ///
    /// @param [in]  landMarks
    /// @param [in]  wayPoints
    /// @param [out] slam state and covariance
    Slam(const Eigen::MatrixXf& landMarks, const Eigen::MatrixXf& wayPoints)
        : mLM(landMarks)
        , mWP(wayPoints)
    {
        mTABLE = Eigen::VectorXi::Zero(mLM.cols());
    }
    ~Slam() = default;

    struct Particle_t
    {
        float                        w;  // particle weight
        Eigen::VectorXf              X;  // ego state
        Eigen::MatrixXf              P;  // ego covariance
        Eigen::VectorXf              XF; // feature state
        std::vector<Eigen::MatrixXf> PF; // feature covariance
    };
    /// @brief       add a new features to state
    ///
    /// @param [in]  particle prior system states and state covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [out] particle augmentd system states and state covariances
    virtual void addOneNewFeature(Particle_t& particle, const Eigen::MatrixXf& Z, const Eigen::MatrixXf& R) = 0;

    struct ControlNoiseState_t
    {
        float v;
        float swa;
    };
    /// @brief       add random noise to nominal control values
    ///
    /// @param [in]  v - host velocity [m/s]
    /// @param [in]  swa - steering wheel angle (SWA) [rad]
    /// @param [in]  Q - system noise covariance
    /// @param [in]  addNoise - include additive noise or not, by default no
    /// @param [out] ControlNoiseState_t - v with noise and swa with noises
    /// @note        Assume Q is diagonal
    ControlNoiseState_t addControlNoise(const float& v, const float& swa, const Eigen::MatrixXf& Q, bool additiveNoise)
    {
        float iv   = v;
        float iswa = swa;
        if (additiveNoise)
        {
            iv   = iv + generateRandomNumber<float>() * std::sqrt(Q(0, 0));
            iswa = iswa + generateRandomNumber<float>() * std::sqrt(Q(1, 1));
        }
        return {iv, iswa};
    }

    /// @brief       add random measurement noise
    ///
    /// @param [in]  Z - measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  addNoise - include additive noise or not, by default no
    /// @param [out] Z - measurements
    /// @note        Assume R is diagonal
    void addObservationNoise(Eigen::MatrixXf& Z, const Eigen::MatrixXf& R, bool additiveNoise)
    {
        if (additiveNoise && (Z.cols() > 0))
        {
            for (int col = 0; col < Z.cols(); col++)
            {
                Z(0, col) = Z(0, col) + (generateRandomNumber<float>() * std::sqrt(R(0, 0)));
                Z(1, col) = Z(1, col) + (generateRandomNumber<float>() * std::sqrt(R(1, 1)));
            }
        }
    }

    /// @brief       augment slam's state and covariance
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - slam covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [out] X - augmented state
    /// @param [out] P - augmented covariance
    /// @note        assume the number of vehicle pose states is three, Only one value for R is used, as all
    /// measurements are assumed to have same noise properties
    virtual void
        augment(Eigen::VectorXf& X, Eigen::MatrixXf& P, const Eigen::MatrixXf& Z, const Eigen::MatrixXf& R) = 0;

    /// @brief       add a new features to state
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - slam covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [out] X - augmented state
    /// @param [out] P - augmented covariance
    virtual void addOneNewFeature(Eigen::VectorXf&       X,
                                  Eigen::MatrixXf&       P,
                                  const Eigen::MatrixXf& Z,
                                  const Eigen::MatrixXf& R) = 0;

    /// @brief       batch update
    /// @param [in]  X - predicted slam state
    /// @param [in]  P - predicted state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  IDF - feature index for each z
    /// @param [out] updated state and covariance
    virtual void batchUpdate(Eigen::VectorXf&       X,
                             Eigen::MatrixXf&       P,
                             const Eigen::MatrixXf& Z,
                             const Eigen::MatrixXf& R,
                             const Eigen::VectorXi& idf) = 0;

    /// @brief       compute innovation between two state estimates, normalizing the heading components.
    ///
    /// @param [in] states
    /// @param [out] innovation
    virtual Eigen::MatrixXf computeDelta(const Eigen::MatrixXf& X1, const Eigen::MatrixXf& X2) = 0;

    /// @brief       calculate the EKF update given the prior state [X,P]
    ///
    /// @param [in]  X - prior system state
    /// @param [in]  P - prior system covariances
    /// @param [in]  V - innovation
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  H - linearised observation model
    /// @param [out] updated mean and covariances
    /// @note        the(linearised) observation model H. The result is calculated using Cholesky factorisation, which
    /// is more numerically stable than a naive implementation.
    void choleskyUpdate(Eigen::VectorXf&       X,
                        Eigen::MatrixXf&       P,
                        const Eigen::VectorXf& V,
                        const Eigen::MatrixXf& R,
                        const Eigen::MatrixXf& H)
    {
        try
        {
            Eigen::MatrixXf PHT = P * H.transpose();
            Eigen::MatrixXf S   = H * PHT + R;

            // note S matrix should be symmetric and pos-definite
            S = makeSymmetric(S); // make symmetric

            // cholesky decomposition
            Eigen::MatrixXf SCHOL    = choleskyDecomposition(S);
            Eigen::MatrixXf SCHOLINV = SCHOL.inverse();
            if (!SCHOLINV.allFinite())
            {
                SCHOLINV = Eigen::MatrixXf::Zero(SCHOL.rows(), SCHOL.cols());
            }

            Eigen::MatrixXf W1 = PHT * SCHOLINV;
            Eigen::MatrixXf W  = W1 * SCHOLINV.transpose();
            X                  = X + W * V; // update
            P                  = P - W1 * W1.transpose();
        }
        catch (std::exception& e)
        {
            std::cout << e.what() << "\t" << "choleskyUpdate" << std::endl;
        }
    }

    /// @brief       compute steering wheel angle
    ///
    /// @param [in]  X - slam state
    /// @param [in]  WP - waypoints
    /// @param [in]  iwp - index to current waypoint
    /// @param [in]  minD - minimum distance to current waypoint before switching to next
    /// @param [in]  swa - current steering wheel angle
    /// @param [in]  rateSWA - max steering rate (rad/s)
    /// @param [in]  maxSWA - max steering angle (rad)
    /// @param [in]  dt - timestep
    /// @param [out] swa - new current wheel steering angle and iwp - new current waypoint
    void computeSWA(const Eigen::VectorXf& X,
                    const Eigen::MatrixXf& WP,
                    int&                   iwp,
                    const float&           minD,
                    float&                 swa,
                    const float&           rateSWA,
                    const float&           maxSWA,
                    const float&           dt)
    {
        try
        {
            // determine if current waypoint reached
            if (WP.cols() > 0)
            {
                Eigen::MatrixXf CWP = Eigen::MatrixXf::Zero(2, 1);
                CWP                 = WP.block(0, iwp - 1, 2, 1);
                float d2            = std::pow((CWP(0, 0) - X(0)), 2.0F) + std::pow((CWP(1, 0) - X(1)), 2.0F);
                if (d2 < std::pow(minD, 2.0F))
                {
                    iwp = iwp + 1;       // switch to next
                    if (iwp > WP.cols()) // rached final waypoint, flag and return
                    {
                        iwp = 0;
                        return;
                    }

                    // next waypoint
                    CWP.setZero(CWP.rows(), CWP.cols());
                    CWP = WP.block(0, iwp - 1, 2, 1);
                }

                // compute change in steering wheel angle to point towardss current waypoints
                auto deltaG = pi2Pi(std::atan2(CWP(1, 0) - X(1), CWP(0, 0) - X(0)) - X(2) - swa);

                // limit rate
                auto maxDelta = rateSWA * dt;
                if (std::abs(deltaG) > maxDelta)
                {
                    deltaG = maxDelta * signum<int>(deltaG);
                }

                // limit angle
                swa = swa + deltaG;
                if (std::abs(swa) > maxSWA)
                {
                    swa = signum<int>(swa) * maxSWA;
                }
            }
        }
        catch (std::exception& e)
        {
            std::cout << e.what() << "\t" << "computeSteering" << std::endl;
        }
    }

    /// @brief       compute range and bearing
    ///
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  LM - set of all landmarks
    /// @param [out] Observation_t - set of range-bearing observations
    Eigen::MatrixXf computeRangeBearing(const Eigen::VectorXf& X, const Eigen::MatrixXf& LM)
    {
        Eigen::MatrixXf Z = Eigen::MatrixXf::Zero(0, 0);
        try
        {
            // Compute exact observation
            std::vector<Vector2f_t> dpos = {};
            for (int index = 0; index < LM.cols(); index++)
            {
                dpos.push_back(Vector2f_t{LM(0, index) - X(0), LM(1, index) - X(1)});
            }
            float phi = X(2);

            if (LM.cols() > 0)
            {
                Z.resize(2, LM.cols());
                Z = Eigen::MatrixXf::Zero(2, LM.cols());
                for (int index = 0; index < dpos.size(); index++)
                {
                    Z(0, index) = std::sqrt(std::pow(dpos[index].x(), 2.0F) + std::pow(dpos[index].y(), 2.0F));
                    Z(1, index) = std::atan2(dpos[index].y(), dpos[index].x()) - phi;
                }
            }
        }
        catch (std::exception& e)
        {
            std::cout << e.what() << "\t" << "computeRangeBearing" << std::endl;
        }
        return Z;
    }

    struct NormalizedInnovation_t
    {
        float nis; // normalised innovation squared (ie, Mahalanobis distance)
        float nd;  // normalised distance
    };

    /// @brief       compute data association
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  idf - extracted features id
    /// @param [out] NormalizedInnovation_t - return normalised innovation squared (ie, Mahalanobis distance) and
    /// normalised distance
    virtual NormalizedInnovation_t computeAssociation(const Eigen::VectorXf& X,
                                                      const Eigen::MatrixXf& P,
                                                      const Eigen::MatrixXf& Z,
                                                      const Eigen::MatrixXf& R,
                                                      int                    idf) = 0;

    struct Jacobians_t
    {
        Eigen::MatrixXf              ZP;
        std::vector<Eigen::MatrixXf> HV;
        std::vector<Eigen::MatrixXf> HF;
        std::vector<Eigen::MatrixXf> SF;
    };

    /// @brief       compute Jacobian
    ///
    /// @param [in]  particle system states and state covariances
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  R - measurement noise covariance
    /// @param [out] Jacobians
    virtual Jacobians_t
        computeJacobians(const Particle_t& particle, const Eigen::VectorXi& idf, const Eigen::MatrixXf& R) = 0;

    /// @brief       positive-definite matrix into the product of a lower triangular matrix and its conjugate transpose,
    ///              which is useful for efficient numerical solutions
    ///
    /// @param [in]  M - matrix
    /// @param [out] Cholesky output
    Eigen::MatrixXf choleskyDecomposition(const Eigen::MatrixXf& M)
    {
        int                         size = M.rows(); // Dimensionality (rows)
        Eigen::MatrixXf             normTransform(size, size);
        Eigen::LLT<Eigen::MatrixXf> cholSolver(M);

        // We can only use the cholesky decomposition if the covariance matrix is symmetric, pos-definite.
        // But a covariance matrix might be pos-semi-definite. In that case, we'll go to an EigenSolver
        if (cholSolver.info() == Eigen::Success)
        {
            normTransform = cholSolver.matrixL(); // Use cholesky solver
        }
        else // Use eigen solver
        {
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigenSolver(M);
            normTransform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
        }

        if (!normTransform.allFinite())
        {
            normTransform = Eigen::MatrixXf::Zero(M.rows(), M.cols());
        }
        return normTransform;
    }

    struct Association_t
    {
        Eigen::MatrixXf ZF;
        Eigen::MatrixXf ZN;
        Eigen::VectorXi idf;
    };

    /// @brief       For simulations with known data-associations, this function maintains a feature / observation
    /// lookup table.It returns the updated table,the set of associated observations and the set of observations to new
    /// features
    ///
    /// @param [in]  X - slam state
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  IDZ - associted id with meas
    /// @param [in]  TABLE - feature/observation lookup table
    /// @param [out] Association_t - data associated information
    virtual Association_t dataAssociateTable(const Eigen::VectorXf& X,
                                             const Eigen::MatrixXf& Z,
                                             const Eigen::VectorXi& idz,
                                             Eigen::VectorXi&       table) = 0;

    /// @brief       For simulations with known data-associations, this function maintains a feature / observation
    /// lookup table.It returns the updated table,the set of associated observations and the set of observations to new
    /// features
    ///
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  IDZ - associted id with meas
    /// @param [in]  TABLE - feature/observation lookup table
    /// @param [in]  numParticles - number of particles
    /// @param [out] Association_t - data associated information
    virtual Association_t dataAssociateTable(const Eigen::MatrixXf& Z,
                                             const Eigen::VectorXi& idz,
                                             Eigen::VectorXi&       table,
                                             int                    numParticles = 1000) = 0;

    /// @brief       Simple gated nearest-neighbour data-association. No clever feature caching tricks to speed up
    /// association, so computation is O(N), where N is the number of features in the state
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  gate1 and 2 - nearest-neighbour gates
    /// @param [out] Association_t - data associated information
    virtual Association_t dataAssociate(const Eigen::VectorXf& X,
                                        const Eigen::MatrixXf& P,
                                        const Eigen::MatrixXf& Z,
                                        const Eigen::MatrixXf& R,
                                        const float&           gate1,
                                        const float&           gate2) = 0;

    /// @brief       Having selected a new pose from the proposal distribution, this pose is assumed perfect and each
    ///              feature update may be computed independently and without pose uncertainty.
    ///
    /// @param [in]  particle system states and state covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  R - measurement noise covariance
    /// @param [out] updated state and covariance of particles
    virtual void featureUpdate(Particle_t&            particle,
                               const Eigen::MatrixXf& Z,
                               const Eigen::VectorXi& idf,
                               const Eigen::MatrixXf& R) = 0;

    /// @brief       Gaussian evaluvation
    ///
    /// @param [in]  V - a set of innovation vectors
    /// @param [in]  S - covariance matrix for the innovation
    /// @param [in]  logFlag - if 1 computes the log-likelihood, otherwise computes the likelihood.
    /// @param [out] set of Gaussian likelihoods or log-likelihood for each V elements
    virtual float gaussEvaluate(const Eigen::VectorXf& V, const Eigen::MatrixXf& S, bool logFlag = false) = 0;

    struct Observation_t
    {
        Eigen::MatrixXf Z;
        Eigen::VectorXi idf;
    };

    /// @brief       compute observation
    ///
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  LM - set of all landmarks
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  rmax - maximum range of range-bearing sensor
    /// @param [out] Observation_t - set of range-bearing observations & landmark index tag for each observation
    Observation_t getObservations(const Eigen::VectorXf& X,
                                  const Eigen::MatrixXf& LM,
                                  const Eigen::VectorXi& idf,
                                  const float&           rmax)
    {
        auto [oLM, oidf] = getVisibleLandmarks(X, LM, idf, rmax);
        return {computeRangeBearing(X, oLM), oidf};
    }

    /// @brief       generate a random number between given lower and upper boundaries
    ///
    /// @param [out] random number
    template <typename T>
    T generateRandomNumber()
    {
        unsigned                    seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine  generator(seed);
        std::normal_distribution<T> distribution(0.0, 1.0);
        return distribution(generator);
    };

    struct VisibleLandmarks_t
    {
        Eigen::MatrixXf LM;
        Eigen::VectorXi idf;
    };
    /// @brief       select set of landmarks that are visible within vehicle's semi-circular field-of-view
    ///
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  LM - set of all landmarks
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  rmax - maximum range of range-bearing sensor
    /// @param [out] VisibleLandmarks_t set & landmark index tag for each observation
    VisibleLandmarks_t getVisibleLandmarks(const Eigen::VectorXf& X,
                                           const Eigen::MatrixXf& LM,
                                           const Eigen::VectorXi& idf,
                                           const float&           rmax)
    {
        Eigen::MatrixXf LMO;
        LMO.resize(0, 0);
        LMO = Eigen::MatrixXf::Zero(0, 0);

        Eigen::VectorXi idfo;
        idfo.resize(0);
        idfo = Eigen::VectorXi::Zero(0);

        try
        {
            std::vector<double> dx_v = {};
            std::vector<double> dy_v = {};
            for (int i = 0; i < LM.cols(); i++)
            {
                dx_v.push_back(LM(0, i) - X(0));
                dy_v.push_back(LM(1, i) - X(1));
            }
            double phi = X(2);

            // incremental tests for bounding semi-circle
            std::vector<double> dr_t;
            std::fill(dr_t.begin(), dr_t.end(), 0.0);
            int              counter = 0;
            std::vector<int> tracker = {};
            std::transform(dx_v.begin(),
                           dx_v.end(),
                           dy_v.begin(),
                           std::back_inserter(dr_t),
                           [&](double i, double j)
                           {
                               counter++;
                               if ((std::abs(i) < static_cast<double>(rmax) &&
                                    std::abs(j) < static_cast<double>(rmax)) &&       // bounding box
                                   ((i * std::cos(phi) + j * std::sin(phi)) > 0.0) && // bounding line
                                   ((std::pow(i, 2.0) + std::pow(j, 2.0)) <
                                    std::pow(static_cast<double>(rmax), 2.0))) // bounding circle
                               {
                                   tracker.push_back(counter);
                                   return counter;
                               }
                               else
                               {
                                   return -1;
                               }
                           });

            // the bounding box test is unnecessary but illustrates a possible speedup technique as it quickly
            // eliminates distant points.Ordering the landmark set would make this operation %O(logN) rather that O(N).
            if (tracker.size() > 0)
            {
                LMO.resize(2, tracker.size());
                LMO = Eigen::MatrixXf::Zero(2, tracker.size());

                idfo.resize(tracker.size());
                idfo = Eigen::VectorXi::Zero(tracker.size());

                int index = 0;
                for (const auto& t : tracker)
                {
                    LMO.block(0, index, 2, 1)  = LM.block(0, t - 1, 2, 1);
                    idfo.block(index, 0, 1, 1) = idf.block(t - 1, 0, 1, 1);
                    index++;
                }
            }
        }
        catch (std::exception& e)
        {
            std::cout << e.what() << "\t" << "getVisibleLandmarks" << std::endl;
        }
        return {LMO, idfo};
    }

    /// @brief       initialize the particles
    ///
    /// @param [out] prior state and covariances
    virtual std::vector<Particle_t> initializeParticles(int numParticles) = 0;

    /// @brief       joseph update
    ///
    /// @param [in]  X - prior system state
    /// @param [in]  P - prior system covariances
    /// @param [in]  V - innovation
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  H - linearised observation model
    /// @param [out] updated mean and covariances
    /// @note        This module is identical to KF_simple_update() except that it uses the Joseph - form covariance
    /// update, as shown in Bar - Shalom "Estimation with Applications...", 2001,p302.
    void josephUpdate(Eigen::VectorXf&       X,
                      Eigen::MatrixXf&       P,
                      const Eigen::VectorXf& V,
                      const Eigen::MatrixXf& R,
                      const Eigen::MatrixXf& H)
    {
        try
        {
            const auto      PHT = P * H.transpose();
            Eigen::MatrixXf S   = H * PHT + R;
            Eigen::MatrixXf SI  = S.inverse();
            SI                  = makeSymmetric(SI);
            Eigen::MatrixXf W   = PHT * SI;

            X = X + W * V;

            // Joseph-form covariance update
            Eigen::MatrixXf C = Eigen::MatrixXf::Identity(P.rows(), P.cols()) - W * H;
            P                 = C * P * C.transpose() + W * R * W.transpose();
            P                 = P + Eigen::MatrixXf::Identity(P.rows(), P.cols()) * std::numeric_limits<float>::min();
        }
        catch (std::exception& e)
        {
            std::cout << e.what() << "\t" << "josephUpdate" << std::endl;
        }
    }

    /// @brief       compute sample weight w
    ///
    /// @param [in]  particle system states and state covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  R - measurement noise covariance
    /// @param [out] likelihood give states
    virtual float likelihood(const Particle_t&      particles,
                             const Eigen::MatrixXf& Z,
                             const Eigen::VectorXi& idf,
                             const Eigen::MatrixXf& R) = 0;

    /// @brief       sample from proposal distribution
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - state covariance
    /// @param [in]  n - number of samples
    /// @param [out] proposed distribution
    virtual Eigen::MatrixXf multivariateGauss(const Eigen::VectorXf& X, const Eigen::MatrixXf& P, int n) = 0;

    /// @brief       Draw nn samples from a size-dimensional normal distribution with a specified mean and covariance
    ///
    /// @param [in]  mean - state
    /// @param [in]  covar - state covariance
    /// @param [in]  numSamples - How many samples (columns) to draw
    /// @param [out] proposed distribution
    Eigen::MatrixXf multivariateNormalGaussianDistribution(const Eigen::VectorXf& mean,
                                                           const Eigen::MatrixXf& covar,
                                                           int                    numSamples)
    {
        int                                           size = mean.rows(); // Dimensionality (rows)
        Eigen::internal::scalar_normal_dist_op<float> randN;              // Gaussian functor
        Eigen::internal::scalar_normal_dist_op<float>::rng.seed(1);       // Seed the rng

        // cholesky decomposition
        Eigen::MatrixXf normTransform = choleskyDecomposition(covar);
        return (normTransform * Eigen::MatrixXf::NullaryExpr(size, numSamples, randN)).colwise() + mean;
    }

    struct State_t
    {
        Eigen::VectorXf X;
        Eigen::MatrixXf P;
    };

    /// @brief       make a symmetric matrix
    ///
    /// @param [in]  P - covariance
    /// @param [out] symmetrified covariance
    Eigen::MatrixXf makeSymmetric(const Eigen::MatrixXf& P)
    {
        return (P + P.transpose()) * 0.5F;
    }

    /// @brief       Perform state update for a given heading measurement, phi, with fixed measurement noise sigmaPhi
    ///
    /// @param [in]  X - system state
    /// @param [in]  P - state covariance
    /// @param [in]  phi - bearing measurements
    /// @param [in]  useHeadings - by default false
    /// @param [out] state and covariance
    virtual void observeHeading(Eigen::VectorXf& X, Eigen::MatrixXf& P, const float& phi, bool useHeading = false) = 0;

    /// @brief       Perform state update for a given heading measurement, phi, with fixed measurement noise sigmaPhi
    ///
    /// @param[in]   particle system states and state covariances
    /// @param [in]  phi - bearing measurements
    /// @param [in]  useHeadings - by default false
    /// @param [out] state and covariance
    virtual void observeHeading(Particle_t& particle, const float& phi, bool useHeading = false) = 0;

    struct ObserveModel_t
    {
        Eigen::MatrixXf Z;
        Eigen::MatrixXf H;
    };

    /// @brief       Given a feature index (ie, the order of the feature in the state vector), predict the expected
    /// range - bearing observation of this feature and its Jacobian.
    ///
    /// @param [in]  X - state vector
    /// @param [in]  idf - index of feature order in state
    /// @param [out] ObserveModel_t - predicted observation and observation Jacobian
    virtual ObserveModel_t observeModel(const Eigen::VectorXf& X, int idf) = 0;

    /// @brief       format give angle
    ///
    /// @param [in]  angle
    /// @param [out] formated angle
    float pi2Pi(float angle)
    {
        angle = std::fmod(angle, static_cast<float>(2 * std::_Pi_val));
        if (angle > std::_Pi_val)
        {
            angle = angle - (2.0F * std::_Pi_val);
        }
        if (angle < -std::_Pi_val)
        {
            angle = angle + (2 * std::_Pi_val);
        }

        return angle;
    }

    /// @brief       Predict the prior state and covariance
    ///
    /// @param [in]  X - system state
    /// @param [in]  P - state covariance
    /// @param [in]  v - host velocity
    /// @param [in]  swa -  steering wheel angle
    /// @param [in]  Q - covariance matrix for velocity and gamma
    /// @param [in]  wb - vehicle wheelbase
    /// @param [in]  dt - timestep
    /// @param [out] Xn, Pn - predicted state and covariance
    virtual void predict(Eigen::VectorXf&       X,
                         Eigen::MatrixXf&       P,
                         const float&           v,
                         const float&           swa,
                         const Eigen::MatrixXf& Q,
                         const float&           wb,
                         const float&           dt) = 0;

    /// @brief       Predict the prior state and covariance of particles
    ///
    /// @param [in]  particle prior system states and state covariances
    /// @param [in]  v - host velocity
    /// @param [in]  swa -  steering wheel angle
    /// @param [in]  Q - covariance matrix for velocity and gamma
    /// @param [in]  wb - vehicle wheelbase
    /// @param [in]  dt - timestep
    /// @param [out] Xn, Pn - predicted state and covariance of particles
    virtual void predict(Particle_t&            particle,
                         const float&           v,
                         const float&           swa,
                         const Eigen::MatrixXf& Q,
                         const float&           wb,
                         const float&           dt) = 0;

    /// @brief       resample the particles if their weight variance is such that N effective is less thatn Nmin
    ///
    /// @param [in]  particle system states and state covariances
    /// @param [in]  numEffective - effective numbef of particles
    /// @param [in]  resampleStatus - 1 means resample is on
    /// @param [out] resampled state and covariance of particles
    virtual void
        resampleParticles(std::vector<Particle_t>& particles, int numEffective, bool resampleStatus = false) = 0;

    /// @brief       compute proposal distribution, then sample from it, and compute new particle weight
    ///
    /// @param [in]  particle system states and state covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  R - measurement noise covariance
    /// @param [out] sampled state and covariance of particles
    virtual void sampleProposal(Particle_t&            particle,
                                const Eigen::MatrixXf& Z,
                                const Eigen::VectorXi& idf,
                                const Eigen::MatrixXf& R) = 0;

    struct Stratified_t
    {
        Eigen::MatrixXf keep;
        float           neff;
    };

    /// @brief       compute effective particles
    ///
    /// @param [in]  w - set of N weights [w1,..wN]
    /// @param [out] keep - N indices of particles to keep
    /// @param [out] Neff - number of effective particles (measure of weight variance)
    virtual Stratified_t stratifiedResample(Eigen::MatrixXf& w) = 0;

    /// @brief       Generate N uniform random numbers stratified within interval (0,1)
    ///
    /// @param [in]  N - number of samples
    /// @param [out] set of samples are in ascending order
    virtual Eigen::MatrixXf stratifiedRandom(int n) = 0;

    /// @brief       instance update
    /// @param [in]  X - predicted slam state
    /// @param [in]  P - predicted state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  IDF - feature index for each z
    /// @param [out] updated state and covariance
    virtual void singleUpdate(Eigen::VectorXf&       X,
                              Eigen::MatrixXf&       P,
                              const Eigen::MatrixXf& Z,
                              const Eigen::MatrixXf& R,
                              const Eigen::VectorXi& idf) = 0;

    /// @brief       signum function
    ///
    /// @param [in]  value
    /// @param [out] calculated signum value
    /// @note        for each element of X, SIGN(X) returns 1 if the element % is greater than zero, 0 if it equals zero
    /// and -1 if it is % less than zero
    template <typename T>
    inline constexpr int signum(T x)
    {
        return (T(0) < x) - (x < T(0));
    }

    /// @brief       Update the predcited state and covariance with observation
    /// @param [in]  X - predicted slam state
    /// @param [in]  P - predicted state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  IDF - feature index for each z
    /// @param [in]  batch - switch to specify whether to process measurements together or sequentially
    /// @param [out] updated state and covariance
    virtual void update(Eigen::VectorXf&       X,
                        Eigen::MatrixXf&       P,
                        const Eigen::MatrixXf& Z,
                        const Eigen::MatrixXf& R,
                        const Eigen::VectorXi& idf,
                        bool                   batch = false) = 0;

    /// @brief       compute vehicle pose
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  v - velocity
    /// @param [in]  swa - steer angle
    /// @param [in]  wb - wheelbase
    /// @param [in]  dt - change in time
    /// @param [out] new vehicle pose
    void vehicleModel(Eigen::VectorXf& X, const float& v, const float& swa, const float& wb, const float& dt)
    {
        try
        {
            const auto TX = X;
            X.setZero(X.rows());
            X(0) = TX(0) + v * dt * std::cos(swa + TX(2));
            X(1) = TX(1) + v * dt * std::sin(swa + TX(2));
            X(2) = pi2Pi(TX(2) + v * dt * std::sin(swa) / wb);
        }
        catch (std::exception& e)
        {
            std::cout << e.what() << "\t" << "vehicleModel" << std::endl;
        }
    }

    // Miscellaneous
    Eigen::MatrixXf getLandMarks() const
    {
        return mLM;
    }
    Eigen::MatrixXf getWayPoints() const
    {
        return mWP;
    }
};
