#pragma once

#include "Slam.h"

class EKF : public Slam
{
  public:
    /// @brief       EKF based SLAM for remote / valet parking / gps-denied nav.
    ///
    /// @param [in]  landMarks
    /// @param [in]  wayPoints
    /// @param [out] slam state and covariance
    EKF(const Eigen::MatrixXf& landMarks, const Eigen::MatrixXf& wayPoints);
    ~EKF() = default;

    /// @brief       add random noise to nominal control values
    ///
    /// @param [in]  v - host velocity [m/s]
    /// @param [in]  swa - steering wheel angle (SWA) [rad]
    /// @param [in]  Q - system noise covariance
    /// @param [in]  addNoise - include additive noise or not, by default no
    /// @param [out] ControlNoiseState_t - v with noise and swa with noises
    /// @note        Assume Q is diagonal
    ControlNoiseState_t addControlNoise(const float&           v,
                                        const float&           swa,
                                        const Eigen::MatrixXf& Q,
                                        bool                   additiveNoise = false) override;

    /// @brief       add random measurement noise
    ///
    /// @param [in]  Z - measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  addNoise - include additive noise or not, by default no
    /// @param [out] Z - measurements
    /// @note        Assume R is diagonal
    void addObservationNoise(Eigen::MatrixXf& Z, const Eigen::MatrixXf& R, bool additiveNoise = false) override;

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
    void augment(Eigen::MatrixXf& X, Eigen::MatrixXf& P, const Eigen::MatrixXf& Z, const Eigen::MatrixXf& R) override;

    /// @brief       add a new features to state
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - slam covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [out] X - augmented state
    /// @param [out] P - augmented covariance
    void addOneNewFeature(Eigen::MatrixXf&       X,
                          Eigen::MatrixXf&       P,
                          const Eigen::MatrixXf& Z,
                          const Eigen::MatrixXf& R) override;

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
                     const Eigen::MatrixXf& IDF) override;

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
                        const Eigen::MatrixXf& H) override;

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
    void computeSWA(const Eigen::MatrixXf& X,
                    const Eigen::MatrixXf& WP,
                    int&                   iwp,
                    const float&           minD,
                    float&                 swa,
                    const float&           rateSWA,
                    const float&           maxSWA,
                    const float&           dt) override;

    /// @brief       compute range and bearing
    ///
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  LM - set of all landmarks
    /// @param [out] Observation_t - set of range-bearing observations
    Eigen::MatrixXf computeRangeBearing(const Eigen::MatrixXf& X, const Eigen::MatrixXf& LM) override;

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
                                              int                    idf) override;

    /// @brief       For simulations with known data-associations, this function maintains a feature / observation
    /// lookup table.It returns the updated table,the set of associated observations and the set of observations to new
    /// features
    ///
    /// @param [in]  X - slam state
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  IDZ - associted id with meas
    /// @param [in]  TABLE - feature/observation lookup table
    /// @param [out] Association_t - data associated information
    Association_t dataAssociateTable(const Eigen::MatrixXf& X,
                                     const Eigen::MatrixXf& Z,
                                     const Eigen::MatrixXf& IDZ,
                                     Eigen::MatrixXf&       TABLE) override;

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
                                const float&           gate1,
                                const float&           gate2) override;

    /// @brief       compute observation
    ///
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  LM - set of all landmarks
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  rmax - maximum range of range-bearing sensor
    /// @param [out] Observation_t - set of range-bearing observations & landmark index tag for each observation
    Observation_t getObservations(const Eigen::MatrixXf& X,
                                  const Eigen::MatrixXf& LM,
                                  const Eigen::MatrixXf& IDF,
                                  const float&           rmax) override;

    /// @brief       select set of landmarks that are visible within vehicle's semi-circular field-of-view
    ///
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  LM - set of all landmarks
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  rmax - maximum range of range-bearing sensor
    /// @param [out] VisibleLandmarks_t set & landmark index tag for each observation
    VisibleLandmarks_t getVisibleLandmarks(const Eigen::MatrixXf& X,
                                           const Eigen::MatrixXf& LM,
                                           const Eigen::MatrixXf& IDF,
                                           const float&           rmax) override;

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
                      const Eigen::MatrixXf& H) override;

    /// @brief       Perform state update for a given heading measurement, phi, with fixed measurement noise sigmaPhi
    ///
    /// @param [in]  X - system state
    /// @param [in]  P - state covariance
    /// @param [in]  phi - bearing measurements
    /// @param [in]  useHeadings - by default false
    /// @param [out] state and covariance
    void observeHeading(Eigen::MatrixXf& X, Eigen::MatrixXf& P, const float& phi, bool useHeading = false) override;

    /// @brief       Given a feature index (ie, the order of the feature in the state vector), predict the expected
    /// range - bearing observation of this feature and its Jacobian.
    ///
    /// @param [in]  X - state vector
    /// @param [in]  idf - index of feature order in state
    /// @param [out] ObserveModel_t - predicted observation and observation Jacobian
    ObserveModel_t observeModel(const Eigen::MatrixXf& X, int idf) override;

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
                 const float&           v,
                 const float&           swa,
                 const Eigen::MatrixXf& Q,
                 const float&           wb,
                 const float&           dt) override;

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
                      const Eigen::MatrixXf& IDF) override;

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
                bool                   batch = false) override;

    /// @brief       compute vehicle pose
    /// @param [in]  X - vehicle pose [x;y;phi]
    /// @param [in]  v - velocity
    /// @param [in]  swa - steer angle
    /// @param [in]  wb - wheelbase
    /// @param [in]  dt - change in time
    /// @param [out] new vehicle pose
    void vehicleModel(Eigen::MatrixXf& X, const float& v, const float& swa, const float& wb, const float& dt) override;
};
