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

    /// @brief       add a new features to state
    ///
    /// @param [in]  particle prior system states and state covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [out] particle augmentd system states and state covariances
    void addOneNewFeature(Particle_t& particle, const Eigen::MatrixXf& Z, const Eigen::MatrixXf& R) override
    {
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
    void augment(Eigen::VectorXf& X, Eigen::MatrixXf& P, const Eigen::MatrixXf& Z, const Eigen::MatrixXf& R) override;

    /// @brief       add a new features to state
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - slam covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [out] X - augmented state
    /// @param [out] P - augmented covariance
    void addOneNewFeature(Eigen::VectorXf&       X,
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
    void batchUpdate(Eigen::VectorXf&       X,
                     Eigen::MatrixXf&       P,
                     const Eigen::MatrixXf& Z,
                     const Eigen::MatrixXf& R,
                     const Eigen::VectorXi& IDF) override;

    /// @brief       compute data association
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  idf - extracted features id
    /// @param [out] NormalizedInnovation_t - return normalised innovation squared (ie, Mahalanobis distance) and
    /// normalised distance
    NormalizedInnovation_t computeAssociation(const Eigen::VectorXf& X,
                                              const Eigen::MatrixXf& P,
                                              const Eigen::MatrixXf& Z,
                                              const Eigen::MatrixXf& R,
                                              int                    idf) override;

    /// @brief       compute innovation between two state estimates, normalizing the heading components.
    ///
    /// @param [in] states
    /// @param [out] innovation
    Eigen::MatrixXf computeDelta(const Eigen::MatrixXf& X1, const Eigen::MatrixXf& X2) override
    {
        return {};
    }

    /// @brief       compute Jacobian
    ///
    /// @param [in]  particle system states and state covariances
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  R - measurement noise covariance
    /// @param [out] Jacobians
    Jacobians_t
        computeJacobians(const Particle_t& particle, const Eigen::VectorXi& idf, const Eigen::MatrixXf& R) override
    {
        return {};
    }

    /// @brief       For simulations with known data-associations, this function maintains a feature / observation
    /// lookup table.It returns the updated table,the set of associated observations and the set of observations to new
    /// features
    ///
    /// @param [in]  X - slam state
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  IDZ - associted id with meas
    /// @param [in]  TABLE - feature/observation lookup table
    /// @param [out] Association_t - data associated information
    Association_t dataAssociateTable(const Eigen::VectorXf& X,
                                     const Eigen::MatrixXf& Z,
                                     const Eigen::VectorXi& idz,
                                     Eigen::VectorXi&       table) override;

    /// @brief       For simulations with known data-associations, this function maintains a feature / observation
    /// lookup table.It returns the updated table,the set of associated observations and the set of observations to new
    /// features
    ///
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  IDZ - associted id with meas
    /// @param [in]  TABLE - feature/observation lookup table
    /// @param [in]  numParticles - number of particles
    /// @param [out] Association_t - data associated information
    Association_t dataAssociateTable(const Eigen::MatrixXf& Z,
                                     const Eigen::VectorXi& idz,
                                     Eigen::VectorXi&       table,
                                     int                    numParticles) override
    {
        return {};
    }

    /// @brief       Simple gated nearest-neighbour data-association. No clever feature caching tricks to speed up
    /// association, so computation is O(N), where N is the number of features in the state
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  gate1 and 2 - nearest-neighbour gates
    /// @param [out] Association_t - data associated information
    Association_t dataAssociate(const Eigen::VectorXf& X,
                                const Eigen::MatrixXf& P,
                                const Eigen::MatrixXf& Z,
                                const Eigen::MatrixXf& R,
                                const float&           gate1,
                                const float&           gate2) override;

    /// @brief       Having selected a new pose from the proposal distribution, this pose is assumed perfect and each
    ///              feature update may be computed independently and without pose uncertainty.
    ///
    /// @param [in]  particle system states and state covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  R - measurement noise covariance
    /// @param [out] updated state and covariance of particles
    void featureUpdate(Particle_t&            particle,
                       const Eigen::MatrixXf& Z,
                       const Eigen::VectorXi& idf,
                       const Eigen::MatrixXf& R) override
    {
    }

    /// @brief       Gaussian evaluvation
    ///
    /// @param [in]  V - a set of innovation vectors
    /// @param [in]  S - covariance matrix for the innovation
    /// @param [in]  logFlag - if 1 computes the log-likelihood, otherwise computes the likelihood.
    /// @param [out] set of Gaussian likelihoods or log-likelihood for each V elements
    float gaussEvaluate(const Eigen::VectorXf& V, const Eigen::MatrixXf& S, bool logFlag = false) override
    {
        return 0;
    }

    /// @brief       initialize the particles
    ///
    /// @param [out] prior state and covariances
    std::vector<Particle_t> initializeParticles(int numParticles) override
    {
        return {};
    }

    /// @brief       compute sample weight w
    ///
    /// @param [in]  particle system states and state covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  R - measurement noise covariance
    /// @param [out] likelihood give states
    float likelihood(const Particle_t&      particles,
                     const Eigen::MatrixXf& Z,
                     const Eigen::VectorXi& idf,
                     const Eigen::MatrixXf& R) override
    {
        return 0;
    }

    /// @brief       sample from proposal distribution
    ///
    /// @param [in]  X - slam state
    /// @param [in]  P - state covariance
    /// @param [in]  n - number of samples
    /// @param [out] proposed distribution
    Eigen::MatrixXf multivariateGauss(const Eigen::VectorXf& X, const Eigen::MatrixXf& P, int n) override
    {
        return {};
    }

    /// @brief       Perform state update for a given heading measurement, phi, with fixed measurement noise sigmaPhi
    ///
    /// @param [in]  X - system state
    /// @param [in]  P - state covariance
    /// @param [in]  phi - bearing measurements
    /// @param [in]  useHeadings - by default false
    /// @param [out] state and covariance
    void observeHeading(Eigen::VectorXf& X, Eigen::MatrixXf& P, const float& phi, bool useHeading = false) override;

    /// @brief       Perform state update for a given heading measurement, phi, with fixed measurement noise sigmaPhi
    ///
    /// @param[in]   particle system states and state covariances
    /// @param [in]  phi - bearing measurements
    /// @param [in]  useHeadings - by default false
    /// @param [out] state and covariance
    void observeHeading(Particle_t& particle, const float& phi, bool useHeading = false) override
    {
    }

    /// @brief       Given a feature index (ie, the order of the feature in the state vector), predict the expected
    /// range - bearing observation of this feature and its Jacobian.
    ///
    /// @param [in]  X - state vector
    /// @param [in]  idf - index of feature order in state
    /// @param [out] ObserveModel_t - predicted observation and observation Jacobian
    ObserveModel_t observeModel(const Eigen::VectorXf& X, int idf) override;

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
    void predict(Eigen::VectorXf&       X,
                 Eigen::MatrixXf&       P,
                 const float&           v,
                 const float&           swa,
                 const Eigen::MatrixXf& Q,
                 const float&           wb,
                 const float&           dt) override;

    /// @brief       Predict the prior state and covariance of particles
    ///
    /// @param [in]  particle prior system states and state covariances
    /// @param [in]  v - host velocity
    /// @param [in]  swa -  steering wheel angle
    /// @param [in]  Q - covariance matrix for velocity and gamma
    /// @param [in]  wb - vehicle wheelbase
    /// @param [in]  dt - timestep
    /// @param [out] Xn, Pn - predicted state and covariance of particles
    void predict(Particle_t&            particle,
                 const float&           v,
                 const float&           swa,
                 const Eigen::MatrixXf& Q,
                 const float&           wb,
                 const float&           dt) override
    {
    }

    /// @brief       resample the particles if their weight variance is such that N effective is less thatn Nmin
    ///
    /// @param [in]  particle system states and state covariances
    /// @param [in]  numEffective - effective numbef of particles
    /// @param [in]  resampleStatus - 1 means resample is on
    /// @param [out] resampled state and covariance of particles
    void resampleParticles(std::vector<Particle_t>& particles, int numEffective, bool resampleStatus = false) override
    {
    }

    /// @brief       compute proposal distribution, then sample from it, and compute new particle weight
    ///
    /// @param [in]  particle system states and state covariances
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  IDF - index tags for each landmark
    /// @param [in]  R - measurement noise covariance
    /// @param [out] sampled state and covariance of particles
    void sampleProposal(Particle_t&            particle,
                        const Eigen::MatrixXf& Z,
                        const Eigen::VectorXi& idf,
                        const Eigen::MatrixXf& R) override
    {
    }

    /// @brief       compute effective particles
    ///
    /// @param [in]  w - set of N weights [w1,..wN]
    /// @param [out] keep - N indices of particles to keep
    /// @param [out] Neff - number of effective particles (measure of weight variance)
    Stratified_t stratifiedResample(Eigen::MatrixXf& w) override
    {
        return {};
    }

    /// @brief       Generate N uniform random numbers stratified within interval (0,1)
    ///
    /// @param [in]  N - number of samples
    /// @param [out] set of samples are in ascending order
    Eigen::MatrixXf stratifiedRandom(int n) override
    {
        return {};
    }

    /// @brief       instance update
    /// @param [in]  X - predicted slam state
    /// @param [in]  P - predicted state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  IDF - feature index for each z
    /// @param [out] updated state and covariance
    void singleUpdate(Eigen::VectorXf&       X,
                      Eigen::MatrixXf&       P,
                      const Eigen::MatrixXf& Z,
                      const Eigen::MatrixXf& R,
                      const Eigen::VectorXi& idf) override;

    /// @brief       Update the predcited state and covariance with observation
    /// @param [in]  X - predicted slam state
    /// @param [in]  P - predicted state covariance
    /// @param [in]  Z - range-bearing measurements
    /// @param [in]  R - measurement noise covariance
    /// @param [in]  IDF - feature index for each z
    /// @param [in]  batch - switch to specify whether to process measurements together or sequentially
    /// @param [out] updated state and covariance
    void update(Eigen::VectorXf&       X,
                Eigen::MatrixXf&       P,
                const Eigen::MatrixXf& Z,
                const Eigen::MatrixXf& R,
                const Eigen::VectorXi& idf,
                bool                   batch = false) override;
};
