#include <array>
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include "EKF.h"

using Vector2f_t = Eigen::Vector2f;
using Vector3f_t = Eigen::Vector3f;

using namespace std;

int main()
{
    try
    {
        // Landmarks info
        std::vector<float> lm1r = {1286.9623655913983384380117058754F,  -16.801075268817204301075268817204F,
                                   2879.7043010752677218988537788391F,  4042.3387096774185920367017388344F,
                                   2510.0806451612897944869473576546F,  -1871.6397849462364320061169564724F,
                                   -2120.2956989247313686064444482327F, -3618.9516129032253957120701670647F,
                                   -4210.3494623655915347626432776451F, -4317.8763440860211630933918058872F,
                                   534.2741935483870967741935483871F,   -910.61827956989236554363742470741F,
                                   -4290.9946236559135286370292305946F, 177.06919945726258447393774986267F,
                                   1044.0976933514302800176665186882F,  506.78426051560745690949261188507F,
                                   1813.4328358208986173849552869797F,  2656.0379918588914733845740556717F,
                                   3242.1981004070585186127573251724F,  3999.3215739484458026709035038948F,
                                   1532.5644504749034240376204252243F,  1117.3677069199529796605929732323F,
                                   -152.64586160108228796161711215973F, -2008.8195386702818723279051482677F,
                                   -3755.0881953867001357139088213444F, -3046.8113975576652592280879616737F,
                                   -4902.9850746268630246049724519253F, 1654.6811397557721647899597883224F,
                                   4194.7082767978317860979586839676F,  3278.8331071913198684342205524445F};

        std::vector<float> lm2r = {203.82165605095541401273885350318F,  -1095.5414012738865494611673057079F,
                                   -2942.6751592356704350095242261887F, -76.433121019108280254777070063694F,
                                   3108.2802547770697856321930885315F,  4076.4331210191066929837688803673F,
                                   191.08280254777070063694267515924F,  -3770.7006369426762830698862671852F,
                                   -1235.6687898089185182470828294754F, 4089.1719745222908386494964361191F,
                                   4789.8089171974515920737758278847F,  2420.3821656050940873683430254459F,
                                   1286.6242038216551009099930524826F,  -164.38356164383561643835616438356F,
                                   -1698.6301369863012951100245118141F, -1479.4520547945194266503676772118F,
                                   -821.91780821917808219178082191781F, -630.13698630136986301369863013699F,
                                   1041.0958904109589041095890410959F,  2054.7945205479445576202124357224F,
                                   2219.1780821917818684596568346024F,  1369.863013698630136986301369863F,
                                   1616.4383561643844586797058582306F,  2109.5890410958909342298284173012F,
                                   1945.2054794520554423797875642776F,  1342.4657534246575342465753424658F,
                                   1917.8082191780849825590848922729F,  -1616.4383561643826396903023123741F,
                                   1150.6849315068493150684931506849F,  2000.0F};

        std::vector<Vector2f_t> mLM = {};
        Eigen::MatrixXf         LM  = Eigen::MatrixXf::Zero(2, lm1r.size());
        LM.setZero(2, lm1r.size());
        for (int i = 0; i < lm1r.size(); i++)
        {
            mLM.push_back(Vector2f_t(lm1r[i], lm2r[i]));
            LM(0, i) = lm1r[i];
            LM(1, i) = lm2r[i];
        }

        // Waypoints info
        std::vector<float> wp1r = {0.0F,
                                   997.98387096774193548387096774194F,
                                   4028.897849462364320061169564724F,
                                   -1058.4677419354838709677419354839F,
                                   -4976.478494623655933537520468235F};
        std::vector<float> wp2r = {0.0F,
                                   -2038.2165605095560749759897589684F,
                                   1707.0063694267500977730378508568F,
                                   1987.2611464968140353448688983917F,
                                   1464.9681528662404161877930164337F};

        std::vector<Vector2f_t> mWP = {};
        Eigen::MatrixXf         WP  = Eigen::MatrixXf::Zero(2, wp1r.size());
        WP.setZero(2, wp1r.size());
        for (int i = 0; i < wp1r.size(); i++)
        {
            mWP.push_back(Vector2f_t(wp1r[i], wp2r[i]));
            WP(0, i) = wp1r[i];
            WP(1, i) = wp2r[i];
        }

        // perform EKF-SLAM landmarks based navigation (test)
        std::shared_ptr<Slam> ekfSlam(new EKF(LM, WP));

        if (ekfSlam != nullptr && true)
        {

            Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(2, 2);
            Q(0, 0)           = std::pow(ekfSlam->mSigmaV, 2.0F);
            Q(0, 1)           = 0.0F;
            Q(1, 0)           = 0.0F;
            Q(1, 1)           = std::pow(ekfSlam->mSigmaSWA, 2.0F);

            Eigen::MatrixXf R = Eigen::MatrixXf::Zero(2, 2);
            R(0, 0)           = std::pow(ekfSlam->mSigmaR, 2.0F);
            R(0, 1)           = 0.0F;
            R(1, 0)           = 0.0F;
            R(1, 1)           = std::pow(ekfSlam->mSigmaB, 2.0F);

            // initialise states
            Eigen::VectorXf XTrue = Eigen::VectorXf::Zero(3);
            Eigen::VectorXf X     = Eigen::VectorXf::Zero(3);
            Eigen::MatrixXf P     = Eigen::MatrixXf::Zero(3, 3);

            double dt    = ekfSlam->mDtControls; // change in time between predicts
            double dtsum = 0.0F;                 // change in time since last observation

            // identifier for each landmark
            Eigen::VectorXi FeatureTag = Eigen::VectorXi::Zero(ekfSlam->getLandMarks().cols());
            int             index      = 0;
            std::transform(FeatureTag.begin(), FeatureTag.end(), FeatureTag.begin(), [&](int& i) { return ++index; });

            int   iwp = 1;    //  index of first waypoint
            float swa = 0.0F; // initial steering wheel angle

            Eigen::MatrixXf QE = Q;
            Eigen::MatrixXf RE = R;

            // inflate estimated noises (ie, add stabilising noise)
            if (ekfSlam->mSwitchInflateNoise)
            {
                QE = 2 * Q;
                RE = 8 * R;
            }

            int indexlooper = 0;
            while (iwp <= ekfSlam->getWayPoints().cols() && iwp > 0)
            {
                std::cout << "indexlooper" << "\t" << ++indexlooper << std::endl;
                std::cout << "X" << std::endl;
                std::cout << X.transpose() << std::endl;
                std::cout << "\n" << std::endl;

                // compute steering wheel angle
                ekfSlam->computeSWA(XTrue,
                                    ekfSlam->getWayPoints(),
                                    iwp,
                                    ekfSlam->mAtWaypoint,
                                    swa,
                                    ekfSlam->mRateSWA,
                                    ekfSlam->mMaxSWA,
                                    dt);

                // perform loops : if final waypoint reached, go back to first
                if (iwp == 0 && ekfSlam->mNumberLoops > 1)
                {
                    iwp                   = 1;
                    ekfSlam->mNumberLoops = ekfSlam->mNumberLoops - 1;
                }

                ekfSlam->vehicleModel(XTrue,
                                      ekfSlam->mVelocity,
                                      swa,
                                      ekfSlam->mWheelBase,
                                      dt); // movment of the platform(Odo meterreading)

                auto [vn, swan] = ekfSlam->addControlNoise(ekfSlam->mVelocity, swa, Q, ekfSlam->mSwitchControlNoise);

                // EKF predict
                ekfSlam->predict(X, P, vn, swan, QE, ekfSlam->mWheelBase, dt);

                // if heading known, observe heading
                ekfSlam->observeHeading(X, P, XTrue(2), ekfSlam->mSwitchHeadingKnown);

                // EKF update step
                dtsum = dtsum + dt;
                if (dtsum >= ekfSlam->mDtObserve)
                {
                    dtsum = 0.0F;

                    auto [Z, FeatureTagVisible] =
                        ekfSlam->getObservations(XTrue, ekfSlam->getLandMarks(), FeatureTag, ekfSlam->mMaxRange);

                    ekfSlam->addObservationNoise(Z, R, ekfSlam->mSwitchSensorNoise);

                    if (Z.size() > 0)
                    {
                        if (ekfSlam->mSwitchAssociationKnown)
                        {

                            auto [ZF, ZN, IDF] = ekfSlam->dataAssociateTable(X, Z, FeatureTagVisible, ekfSlam->mTABLE);

                            ekfSlam->update(X, P, ZF, RE, IDF, ekfSlam->mSwitchBatchUpdate);
                            ekfSlam->augment(X, P, ZN, RE);
                        }
                        else
                        {
                            auto [ZF, ZN, IDF] =
                                ekfSlam->dataAssociate(X, P, Z, RE, ekfSlam->mGateReject, ekfSlam->mGateAugment);
                            ekfSlam->update(X, P, ZF, RE, IDF.transpose(), ekfSlam->mSwitchBatchUpdate);
                            ekfSlam->augment(X, P, ZN, RE);
                        }
                    }
                }
            }
        }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "landmarkBasedNavigation" << std::endl;
    }

    return EXIT_SUCCESS;
}
