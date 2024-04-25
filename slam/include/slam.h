#pragma once

#include <algorithm>
#include <Eigen/Dense>
#include <exception>
#include <iterator>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>
#include <vector>

class Slam
{
  private:
    Eigen::MatrixXf mLM; // land marks
    Eigen::MatrixXf mWP; // way points

  public:
    // Configuration for Air bus 380
    // Basic information: Length = 73.0 m, Speed 1,185 km/h = 329.1667 m/s (136.11 m/s is perfect)
    float mVelocity   = 83.33F;                          // m/s
    float mMaxSWA     = std::_Pi_val / 4.0F;             // radians, maximum steering angle (-MAXG < g < MAXG)
    float mRateSWA    = 90.0F * std::_Pi_val / (180.0F); // rad / s, maximum rate of change in steer angle
    float mWheelBase  = 73.0F;                           // metres, vehicle wheel-base
    float mDtControls = 0.01F;                           // seconds, time interval between control signals

    // control noises
    float mSigmaV   = 0.3F;                           // m/s
    float mSigmaSWA = (1.0F * std::_Pi_val / 180.0F); // radians

    // observation parameters
    float mMaxRange  = 2000.0F;              // metres
    float mDtObserve = 0.058F * mDtControls; // seconds, time interval between observations

    // observation noises
    float mSigmaR = 0.1F;                           // metres
    float mSigmaB = (1.0F * std::_Pi_val / 180.0F); // radians

    // data association innovation gates (Mahalanobis distances)
    float mGateReject  = 50.0F;  // maximum distance for association
    float mGateAugment = 750.0F; // minimum distance for creation of new feature

    // waypoint proximity
    float mAtWaypoint  = 1.0F; // metres, distance from current waypoint at which to switch to next waypoint
    float mNumberLoops = 1.0F; // number of loops through the waypoint list

    // switches
    bool mSwitchControlNoise     = true;  // if 0, velocity and gamma are perfect
    bool mSwitchSensorNoise      = true;  // if 0, measurements are perfect
    bool mSwitchInflateNoise     = true;  // if 1, the estimated Q and R are inflated (ie, add stabilising noise)
    bool mSwitchHeadingKnown     = true;  // if 1, the vehicle heading is observed directly at each iteration
    bool mSwitchAssociationKnown = false; // if 1, associations are given, if 0, they are estimated using gates
    bool mSwitchBatchUpdate      = true;  // if 1, process scan in batch, if 0, process sequentially

    // Noise selection
    float mVelocityNoiseLowerBound = -0.55F;
    float mVelocityNoiseUpperBound = 0.55F;
    float mSwaNoiseLowerBound      = -std::_Pi_val / 180.0F;
    float mSwaNoiseUpperBound      = std::_Pi_val / 180.0F;

    Eigen::MatrixXf mTABLE; // data association table

    /// @brief       EKF based SLAM for remote / valet parking / gps-denied nav.
    ///
    /// @param [in]  landMarks
    /// @param [in]  wayPoints
    /// @param [out] slam state and covariance
    Slam(const Eigen::MatrixXf& landMarks, const Eigen::MatrixXf& wayPoints)
        : mLM(landMarks)
        , mWP(wayPoints)
    {
        mTABLE = Eigen::MatrixXf::Zero(1, mLM.cols());
    }
    ~Slam() = default;

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
    virtual ControlNoiseState_t
        addControlNoise(const float& v, const float& swa, const Eigen::MatrixXf& Q, bool additiveNoise = false) = 0;

    /// @brief       add random measurement noise
    ///
    /// @param [in]  Z - measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  addNoise - include additive noise or not, by default no
    /// @param [out] Z - measurements
    /// @note        Assume R is diagonal
    virtual void addObservationNoise(Eigen::MatrixXf& Z, const Eigen::MatrixXf& R, bool additiveNoise = false) = 0;

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
        augment(Eigen::MatrixXf& X, Eigen::MatrixXf& P, const Eigen::MatrixXf& Z, const Eigen::MatrixXf& R) = 0;

    /// @brief       add a new features to state
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - slam covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [out] X - augmented state
    /// @param [out] P - augmented covariance
    virtual void addOneNewFeature(Eigen::MatrixXf&       X,
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
    virtual void batchUpdate(Eigen::MatrixXf&       X,
                             Eigen::MatrixXf&       P,
                             const Eigen::MatrixXf& Z,
                             const Eigen::MatrixXf& R,
                             const Eigen::MatrixXf& IDF) = 0;

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
    virtual void choleskyUpdate(Eigen::MatrixXf&       X,
                                Eigen::MatrixXf&       P,
                                const Eigen::MatrixXf& V,
                                const Eigen::MatrixXf& R,
                                const Eigen::MatrixXf& H) = 0;

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
    virtual void computeSWA(const Eigen::MatrixXf& X,
                            const Eigen::MatrixXf& WP,
                            int&                   iwp,
                            const float&           minD,
                            float&                 swa,
                            const float&           rateSWA,
                            const float&           maxSWA,
                            const float&           dt) = 0;

    /// @brief       compute range and bearing
    ///
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  LM - set of all landmarks
    /// @param [out] Observation_t - set of range-bearing observations
    virtual Eigen::MatrixXf computeRangeBearing(const Eigen::MatrixXf& X, const Eigen::MatrixXf& LM) = 0;

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
    virtual NormalizedInnovation_t computeAssociation(const Eigen::MatrixXf& X,
                                                      const Eigen::MatrixXf& P,
                                                      const Eigen::MatrixXf& Z,
                                                      const Eigen::MatrixXf& R,
                                                      int                    idf) = 0;

    struct Association_t
    {
        Eigen::MatrixXf ZF;
        Eigen::MatrixXf ZN;
        Eigen::MatrixXf IDF;
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
    virtual Association_t dataAssociateTable(const Eigen::MatrixXf& X,
                                             const Eigen::MatrixXf& Z,
                                             const Eigen::MatrixXf& IDZ,
                                             Eigen::MatrixXf&       TABLE) = 0;

    /// @brief       Simple gated nearest-neighbour data-association. No clever feature caching tricks to speed up
    /// association, so computation is O(N), where N is the number of features in the state
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  gate1 and 2 - nearest-neighbour gates
    /// @param [out] Association_t - data associated information
    virtual Association_t dataAssociate(const Eigen::MatrixXf& X,
                                        const Eigen::MatrixXf& P,
                                        const Eigen::MatrixXf& Z,
                                        const Eigen::MatrixXf& R,
                                        const float&           gate1,
                                        const float&           gate2) = 0;

    struct Observation_t
    {
        Eigen::MatrixXf Z;
        Eigen::MatrixXf IDF;
    };

    /// @brief       compute observation
    ///
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  LM - set of all landmarks
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  rmax - maximum range of range-bearing sensor
    /// @param [out] Observation_t - set of range-bearing observations & landmark index tag for each observation
    virtual Observation_t getObservations(const Eigen::MatrixXf& X,
                                          const Eigen::MatrixXf& LM,
                                          const Eigen::MatrixXf& IDF,
                                          const float&           rmax) = 0;

    /// @brief       generate a random number between given lower and upper boundaries
    ///
    /// @param [in]  lower boundary
    /// @param [in]  upper boundary
    /// @param [out] random number
    template <typename T>
    T generateRandomNumer(T lowerBound, T upperBound)
    {
        std::random_device                 rseed;
        std::mt19937                       rng(rseed());
        std::uniform_int_distribution<int> dist(lowerBound, upperBound);
        return dist(rng);
    };

    struct VisibleLandmarks_t
    {
        Eigen::MatrixXf LM;
        Eigen::MatrixXf IDF;
    };
    /// @brief       select set of landmarks that are visible within vehicle's semi-circular field-of-view
    ///
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  LM - set of all landmarks
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  rmax - maximum range of range-bearing sensor
    /// @param [out] VisibleLandmarks_t set & landmark index tag for each observation
    virtual VisibleLandmarks_t getVisibleLandmarks(const Eigen::MatrixXf& X,
                                                   const Eigen::MatrixXf& LM,
                                                   const Eigen::MatrixXf& IDF,
                                                   const float&           rmax) = 0;

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
    virtual void josephUpdate(Eigen::MatrixXf&       X,
                              Eigen::MatrixXf&       P,
                              const Eigen::MatrixXf& V,
                              const Eigen::MatrixXf& R,
                              const Eigen::MatrixXf& H) = 0;

    struct State_t
    {
        Eigen::MatrixXf X;
        Eigen::MatrixXf P;
    };
    ///// @brief       perform landmarks based navigation
    // virtual void landmarkBasedNavigation() = 0;

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
    virtual void observeHeading(Eigen::MatrixXf& X, Eigen::MatrixXf& P, const float& phi, bool useHeading = false) = 0;

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
    virtual ObserveModel_t observeModel(const Eigen::MatrixXf& X, int idf) = 0;

    /// @brief       format give angle
    ///
    /// @param [in]  angle
    /// @param [out] formated angle
    virtual float pi2Pi(float angle) = 0;

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
    virtual void predict(Eigen::MatrixXf&       X,
                         Eigen::MatrixXf&       P,
                         const float&           v,
                         const float&           swa,
                         const Eigen::MatrixXf& Q,
                         const float&           wb,
                         const float&           dt) = 0;

    /// @brief       instance update
    /// @param [in]  X - predicted slam state
    /// @param [in]  P - predicted state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  IDF - feature index for each z
    /// @param [out] updated state and covariance
    virtual void singleUpdate(Eigen::MatrixXf&       X,
                              Eigen::MatrixXf&       P,
                              const Eigen::MatrixXf& Z,
                              const Eigen::MatrixXf& R,
                              const Eigen::MatrixXf& IDF) = 0;

    /// @brief       signum function
    ///
    /// @param [in]  value
    /// @param [out] calculated signum value
    /// @note        for each element of X, SIGN(X) returns 1 if the element % is greater than zero, 0 if it equals zero
    /// and -1 if it is % less than zero
    template <typename T>
    int sign(T val)
    {
        auto out = val;
        if (val > 0)
        {
            out = 1;
        }
        else if (val < 0)
        {
            out = -1;
        }
        else
        {
            out = 0;
        }
        return static_cast<int>(out);
    }

    /// @brief       Update the predcited state and covariance with observation
    /// @param [in]  X - predicted slam state
    /// @param [in]  P - predicted state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  IDF - feature index for each z
    /// @param [in]  batch - switch to specify whether to process measurements together or sequentially
    /// @param [out] updated state and covariance
    virtual void update(Eigen::MatrixXf&       X,
                        Eigen::MatrixXf&       P,
                        const Eigen::MatrixXf& Z,
                        const Eigen::MatrixXf& R,
                        const Eigen::MatrixXf& IDF,
                        bool                   batch = false) = 0;

    /// @brief       compute vehicle pose
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  v - velocity
    /// @param [in]  swa - steer angle
    /// @param [in]  wb - wheelbase
    /// @param [in]  dt - change in time
    /// @param [out] new vehicle pose
    virtual void
        vehicleModel(Eigen::MatrixXf& X, const float& v, const float& swa, const float& wb, const float& dt) = 0;

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
