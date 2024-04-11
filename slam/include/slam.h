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

    // Configuration for F35
    // Basic information: Length = 15.4 m, Speed 1,930 km/h = 536.11 m/s (136.11 m/s is good)
    float mVelocity   = 536.11F;                       // m/s
    float mMaxSWA     = std::_Pi_val / 2.0F;           // radians, maximum steering angle (-MAXG < g < MAXG)
    float mRateSWA    = 90.0F * std::_Pi_val / 180.0F; // rad / s, maximum rate of change in steer angle
    float mWheelBase  = 15.4F;                         // metres, vehicle wheel-base
    float mDtControls = 0.01F;                         // seconds, time interval between control signals

    // control noises
    float mSigmaV   = 0.3F;                           // m/s
    float mSigmaSWA = (3.0F * std::_Pi_val / 180.0F); // radians

    // observation parameters
    float mMaxRange  = 10000.0F;             // metres
    float mDtObserve = 0.058F * mDtControls; // seconds, time interval between observations

    // observation noises
    float mSigmaR = 0.1F;                           // metres
    float mSigmaB = (1.0F * std::_Pi_val / 180.0F); // radians

    // data association innovation gates (Mahalanobis distances)
    float mGateReject  = 750.0F; // maximum distance for association
    float mGateAugment = 750.0F; // minimum distance for creation of new feature

    // waypoint proximity
    float mAtWaypoint  = 1.0F; // metres, distance from current waypoint at which to switch to next waypoint
    float mNumberLoops = 1.0F; // number of loops through the waypoint list

    // switches (TO DO LIST..)
    bool mSwitchControlNoise     = true;  // if 0, velocity and gamma are perfect
    bool mSwitchSensorNoise      = true;  // if 0, measurements are perfect
    bool mSwitchInflateNoise     = false; // if 1, the estimated Q and R are inflated (ie, add stabilising noise)
    bool mSwitchHeadingKnown     = false; // if 1, the vehicle heading is observed directly at each iteration
    bool mSwitchAssociationKnown = false; // if 1, associations are given, if 0, they are estimated using gates
    bool mSwitchBatchUpdate      = true;  // if 1, process scan in batch, if 0, process sequentially
    bool mSwitchSeedRandom       = false; // if not 0, seed the randn() with its value at beginning of simulation

    // Noise selection
    float mNoiseLowerBound = -2.0F;
    float mNoiseUpperBound = 2.0F;

  public:
    /// @brief       EKF based SLAM for remote / valet parking / gps-denied nav.
    ///
    /// @param [in]  landMarks
    /// @param [in]  wayPoints
    /// @param [out] slam state and covariance
    Slam(const Eigen::MatrixXf& landMarks, const Eigen::MatrixXf& wayPoints);
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
    ControlNoiseState_t addControlNoise(float& v, float& swa, const Eigen::MatrixXf& Q, bool additiveNoise = false);

    /// @brief       add random measurement noise
    ///
    /// @param [in]  Z - measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  addNoise - include additive noise or not, by default no
    /// @param [out] Z - measurements
    /// @note        Assume R is diagonal
    void addObservationNoise(Eigen::MatrixXf& Z, const Eigen::MatrixXf& R, bool additiveNoise = false);

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
    void augment(Eigen::MatrixXf& X, Eigen::MatrixXf& P, const Eigen::MatrixXf& Z, const Eigen::MatrixXf& R);

    /// @brief       add a new features to state
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - slam covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [out] X - augmented state
    /// @param [out] P - augmented covariance
    void addOneNewFeature(Eigen::MatrixXf& X, Eigen::MatrixXf& P, const Eigen::MatrixXf& Z, const Eigen::MatrixXf& R);

    /// @brief       batch update
    /// @param [in]  X - predicted slam state
    /// @param [in]  P - predicted state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  IDF - feature index for each z
    /// @param [out] updated state and covariance
    void batchUpdate(Eigen::MatrixXf&       X,
                     Eigen::MatrixXf&       P,
                     const Eigen::MatrixXf& Z,
                     const Eigen::MatrixXf& R,
                     const Eigen::MatrixXf& IDF);

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
    void choleskyUpdate(Eigen::MatrixXf&       X,
                        Eigen::MatrixXf&       P,
                        const Eigen::MatrixXf& V,
                        const Eigen::MatrixXf& R,
                        const Eigen::MatrixXf& H);

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
    /// @param [out] swa - new current wheel steering angle
    /// @param [out] iwp - new current waypoint
    void computeSteering(const Eigen::MatrixXf& X,
                         const Eigen::MatrixXf& WP,
                         int&                   iwp,
                         const float            minD,
                         float&                 swa,
                         const float            rateSWA,
                         const float            maxSWA,
                         const float            dt);

    /// @brief       compute range and bearing
    ///
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  LM - set of all landmarks
    /// @param [out] Observation_t - set of range-bearing observations
    Eigen::MatrixXf computeRangeBearing(Eigen::MatrixXf& X, Eigen::MatrixXf& LM);

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
    NormalizedInnovation_t computeAssociation(const Eigen::MatrixXf& X,
                                              const Eigen::MatrixXf& P,
                                              const Eigen::MatrixXf& Z,
                                              const Eigen::MatrixXf& R,
                                              int                    idf);

    struct Association_t
    {
        Eigen::MatrixXf ZF;
        Eigen::MatrixXf ZN;
        Eigen::MatrixXf IDF;
    };

    /// @brief       Simple gated nearest-neighbour data-association. No clever feature caching tricks to speed up
    /// association, so computation is O(N), where N is the number of features in the state
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  gate1 and 2 - nearest-neighbour gates
    /// @param [out] Association_t - data associated information
    Association_t dataAssociate(const Eigen::MatrixXf& X,
                                const Eigen::MatrixXf& P,
                                const Eigen::MatrixXf& Z,
                                const Eigen::MatrixXf& R,
                                const float            gate1,
                                const float            gate2);

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
    Observation_t getObservations(Eigen::MatrixXf& X, Eigen::MatrixXf& LM, Eigen::MatrixXf& IDF, float rmax);

    /// @brief       generate a random number between given lower and upper boundaries
    ///
    /// @param [in]  lower boundary
    /// @param [in]  upper boundary
    /// @param [out] random number
    template <typename T>
    T generateRandomNumer(T lowerBound, T upperBound)
    {
        return lowerBound + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (upperBound - lowerBound)));
    };

    /// @brief       select set of landmarks that are visible within vehicle's semi-circular field-of-view
    ///
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  LM - set of all landmarks
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  rmax - maximum range of range-bearing sensor
    /// @param [out] set of range-bearing observations & landmark index tag for each observation
    void getVisibleLandmarks(Eigen::MatrixXf& X, Eigen::MatrixXf& LM, Eigen::MatrixXf& IDF, float rmax);

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
    void josephUpdate(Eigen::MatrixXf&       X,
                      Eigen::MatrixXf&       P,
                      const Eigen::MatrixXf& V,
                      const Eigen::MatrixXf& R,
                      const Eigen::MatrixXf& H);

    struct State_t
    {
        Eigen::MatrixXf X;
        Eigen::MatrixXf P;
    };
    /// @brief       perform landmarks based navigation
    void landmarkBasedNavigation();

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
    void observeHeading(Eigen::MatrixXf& X, Eigen::MatrixXf& P, const float phi, bool useHeading = false);

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
    ObserveModel_t observeModel(const Eigen::MatrixXf& X, int idf);

    /// @brief       format give angle
    ///
    /// @param [in]  angle
    /// @param [out] formated angle
    float pi2Pi(float angle);

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
    void predict(Eigen::MatrixXf&       X,
                 Eigen::MatrixXf&       P,
                 float                  v,
                 float                  swa,
                 const Eigen::MatrixXf& Q,
                 float                  wb,
                 float                  dt);

    /// @brief       instance update
    /// @param [in]  X - predicted slam state
    /// @param [in]  P - predicted state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  IDF - feature index for each z
    /// @param [out] updated state and covariance
    void singleUpdate(Eigen::MatrixXf&       X,
                      Eigen::MatrixXf&       P,
                      const Eigen::MatrixXf& Z,
                      const Eigen::MatrixXf& R,
                      const Eigen::MatrixXf& IDF);

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

    /// @brief       transform a list of poses [x;y;phi] so that they are global wrt a base pose
    /// @param [in]  P - pose
    /// @param [in]  B - system state
    /// @param [out] pose in global
    Eigen::MatrixXf transform2Global(Eigen::MatrixXf& P, const Eigen::MatrixXf& B);

    /// @brief       Update the predcited state and covariance with observation
    /// @param [in]  X - predicted slam state
    /// @param [in]  P - predicted state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  IDF - feature index for each z
    /// @param [in]  batch - switch to specify whether to process measurements together or sequentially
    /// @param [out] updated state and covariance
    void update(Eigen::MatrixXf&       X,
                Eigen::MatrixXf&       P,
                const Eigen::MatrixXf& Z,
                const Eigen::MatrixXf& R,
                const Eigen::MatrixXf& IDF,
                bool                   batch = false);

    /// @brief       compute vehicle pose
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  v - velocity
    /// @param [in]  swa - steer angle
    /// @param [in]  wb - wheelbase
    /// @param [in]  dt - change in time
    /// @param [out] new vehicle pose
    void vehicleModel(Eigen::MatrixXf& X, float v, float swa, float wb, float dt);
};
