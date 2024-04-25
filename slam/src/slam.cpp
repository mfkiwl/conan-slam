// #include "Slam.h"
//
// Slam::Slam(const Eigen::MatrixXf& landMarks, const Eigen::MatrixXf& wayPoints)
//     : mLM(landMarks)
//     , mWP(wayPoints)
//{
//     mTABLE = Eigen::MatrixXf::Zero(1, mLM.cols());
// }
//
// Slam::ControlNoiseState_t
//     Slam::addControlNoise(const float& v, const float& swa, const Eigen::MatrixXf& Q, bool additiveNoise)
//{
//     //float iv   = v;
//     //float iswa = swa;
//     //if (additiveNoise)
//     //{
//     //    iv   = iv + generateRandomNumer<float>(mVelocityNoiseLowerBound, mVelocityNoiseUpperBound) * std::sqrt(Q(0,
//     0));
//     //    iswa = iswa + generateRandomNumer<float>(mSwaNoiseLowerBound, mSwaNoiseUpperBound) * std::sqrt(Q(1, 1));
//     //}
//     //return {iv, iswa};
// }
//
// void Slam::addObservationNoise(Eigen::MatrixXf& Z, const Eigen::MatrixXf& R, bool additiveNoise)
//{
//     //if (additiveNoise && (Z.cols() > 0))
//     //{
//     //    for (int col = 0; col < Z.cols(); col++)
//     //    {
//     //        Z(0, col) = Z(0, col) + (generateRandomNumer<float>(mVelocityNoiseLowerBound, mVelocityNoiseUpperBound)
//     *
//     //                                 std::sqrt(R(0, 0)));
//     //        Z(1, col) =
//     //            Z(1, col) + (generateRandomNumer<float>(mSwaNoiseLowerBound, mSwaNoiseUpperBound) * std::sqrt(R(1,
//     1)));
//     //    }
//     //}
// }
//
// void Slam::augment(Eigen::MatrixXf& X, Eigen::MatrixXf& P, const Eigen::MatrixXf& Z, const Eigen::MatrixXf& R)
//{
//     //try
//     //{
//     //    // add new features to state
//     //    for (int index = 0; index < Z.cols(); index++)
//     //    {
//     //        Eigen::MatrixXf TZ = Eigen::MatrixXf::Zero(2, 1);
//     //        TZ(0, 0)           = Z(0, index);
//     //        TZ(1, 0)           = Z(1, index);
//     //        addOneNewFeature(X, P, TZ, R);
//     //    }
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "augment" << std::endl;
//     //}
// }
//
// void Slam::addOneNewFeature(Eigen::MatrixXf& X, Eigen::MatrixXf& P, const Eigen::MatrixXf& Z, const Eigen::MatrixXf&
// R)
//{
//     //try
//     //{
//     //    int  len = std::max(X.rows(), X.cols());
//     //    auto r   = Z(0, 0);
//     //    auto b   = Z(1, 0);
//     //    auto s   = std::sin(X(2, 0) + b);
//     //    auto c   = std::cos(X(2, 0) + b);
//
//     //    // augment X
//     //    const auto AX = X;
//     //    X.resize(AX.rows() + 2, AX.cols());
//     //    X.setZero(X.rows(), X.cols());
//     //    X.block(0, 0, AX.rows(), AX.cols()) = AX;
//     //    Eigen::MatrixXf SF;
//     //    SF.resize(2, 1);
//     //    SF.setZero(2, 1);
//     //    SF(0, 0)                    = AX(0, 0) + (r * c);
//     //    SF(1, 0)                    = AX(1, 0) + (r * s);
//     //    X.block(AX.rows(), 0, 2, 1) = SF;
//     //    SF.resize(0, 0);
//
//     //    Eigen::MatrixXf Gv = Eigen::MatrixXf::Zero(2, 3);
//     //    Gv(0, 0)           = 1.0F;
//     //    Gv(0, 1)           = 0.0F;
//     //    Gv(0, 2)           = -r * s;
//     //    Gv(1, 0)           = 0.0F;
//     //    Gv(1, 1)           = 1.0F;
//     //    Gv(1, 2)           = r * c;
//
//     //    Eigen::MatrixXf Gz = Eigen::MatrixXf::Zero(2, 2);
//     //    Gz(0, 0)           = c;
//     //    Gz(0, 1)           = -r * s;
//     //    Gz(1, 0)           = s;
//     //    Gz(1, 1)           = r * c;
//
//     //    // augment P
//     //    const auto AP          = P;
//     //    int        resizeLimit = 2;
//     //    P.resize(len + resizeLimit, len + resizeLimit);
//     //    P.setZero(P.rows(), P.cols());
//     //    P.block(0, 0, AP.rows(), AP.cols()) = AP;
//
//     //    // feature covariance
//     //    P.block(len, len, 2, 2) = (Gv * AP.block(0, 0, 3, 3) * Gv.transpose()) + (Gz * R * Gz.transpose());
//
//     //    // vehicle to feature xcorr
//     //    P.block(len, 0, 2, 3) = Gv * P.block(0, 0, 3, 3);
//     //    P.block(0, len, 3, 2) = P.block(len, 0, 2, 3).transpose();
//
//     //    if (len > 3)
//     //    {
//     //        // map to feature xcorr
//     //        P.block(len, 3, 2, len - 3) = Gv * P.block(0, 3, 3, len - 3);
//     //        P.block(3, len, len - 3, 2) = P.block(len, 3, 2, len - 3).transpose();
//     //    }
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "addOneNewFeature" << std::endl;
//     //}
// }
//
// void Slam::batchUpdate(Eigen::MatrixXf&       X,
//                        Eigen::MatrixXf&       P,
//                        const Eigen::MatrixXf& Z,
//                        const Eigen::MatrixXf& R,
//                        const Eigen::MatrixXf& IDF)
//{
//     //try
//     //{
//     //    int lenZ = Z.cols();
//     //    int lenX = std::max(X.rows(), X.cols());
//
//     //    Eigen::MatrixXf H  = Eigen::MatrixXf::Zero(2 * lenZ, lenX);
//     //    Eigen::MatrixXf V  = Eigen::MatrixXf::Zero(2 * lenZ, 1);
//     //    Eigen::MatrixXf RR = Eigen::MatrixXf::Zero(2 * lenZ, 2 * lenZ);
//
//     //    for (int i = 0; i < lenZ; i++)
//     //    {
//     //        Eigen::MatrixXf L = Eigen::MatrixXf::Zero(1, 2);
//     //        L(0, 0)           = 2 * (i + 1) - 1;
//     //        L(0, 1)           = 2 * (i + 1);
//
//     //        auto [Zp, HT] = observeModel(X, IDF(i)); // LAST KNOWN PROBLEM IS HERE
//
//     //        H.block(L(0, 0) - 1, 0, L.cols(), H.cols()) = HT;
//
//     //        Eigen::MatrixXf VT = Eigen::MatrixXf::Zero(2, 1);
//     //        VT(0, 0)           = Z(0, i) - Zp(0, 0);
//     //        VT(1, 0)           = pi2Pi(Z(1, i) - Zp(1, 0));
//
//     //        V.block(L(0, 0) - 1, 0, L.cols(), 1) = VT;
//
//     //        RR.block(L(0, 0) - 1, L(0, 0) - 1, L.cols(), L.cols()) = R;
//     //    }
//
//     //    choleskyUpdate(X, P, V, RR, H);
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "batchUpdate" << std::endl;
//     //}
// }
//
// void Slam::choleskyUpdate(Eigen::MatrixXf&       X,
//                           Eigen::MatrixXf&       P,
//                           const Eigen::MatrixXf& V,
//                           const Eigen::MatrixXf& R,
//                           const Eigen::MatrixXf& H)
//{
//     //try
//     //{
//     //    Eigen::MatrixXf PHT = P * H.transpose();
//     //    Eigen::MatrixXf S   = H * PHT + R;
//     //
//     //    // note S matrix should be symmetric and pos-definite
//     //    S = makeSymmetric(S); // make symmetric
//
//     //    Eigen::MatrixXf normTransform(S.rows(), S.cols());
//     //    Eigen::LLT<Eigen::MatrixXf> cholSolver(S);
//
//     //    if (cholSolver.info() == Eigen::Success)
//     //    {
//     //        normTransform = cholSolver.matrixL();
//     //    }
//     //    else // if S might be a pos-semi-definite
//     //    {
//     //        // use eigen solver
//     //        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigenSolver(S);
//     //        normTransform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
//     //    }
//     //    Eigen::MatrixXf SCHOL(normTransform);
//     //    Eigen::MatrixXf W1 = PHT * SCHOL.inverse();
//     //    Eigen::MatrixXf W  = W1 * SCHOL.inverse().transpose();
//
//     //    X = X + W * V; // update
//     //    P = P - W1 * W1.transpose();
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "choleskyUpdate" << std::endl;
//     //}
// }
//
// void Slam::computeSWA(const Eigen::MatrixXf& X,
//                       const Eigen::MatrixXf& WP,
//                       int&                   iwp,
//                       const float&           minD,
//                       float&                 swa,
//                       const float&           rateSWA,
//                       const float&           maxSWA,
//                       const float&           dt)
//{
//     //try
//     //{
//     //    // determine if current waypoint reached
//     //    if (WP.cols() > 0)
//     //    {
//     //        Eigen::MatrixXf CWP = Eigen::MatrixXf::Zero(2, 1);
//     //        CWP(0, 0)           = WP(0, iwp - 1);
//     //        CWP(1, 0)           = WP(1, iwp - 1);
//
//     //        float d2 = std::pow((CWP(0, 0) - X(0, 0)), 2.0F) + std::pow((CWP(1, 0) - X(1, 0)), 2.0F);
//     //        if (d2 < std::pow(minD, 2.0F))
//     //        {
//     //            iwp = iwp + 1;       // switch to next
//     //            if (iwp > WP.cols()) // rached final waypoint, flag and return
//     //            {
//     //                iwp = 0;
//     //                return;
//     //            }
//
//     //            // next waypoint
//     //            CWP.setZero(CWP.rows(), CWP.cols());
//     //            CWP(0, 0) = WP(0, iwp - 1);
//     //            CWP(1, 0) = WP(1, iwp - 1);
//     //        }
//
//     //        // compute change in steering wheel angle to point towardss current waypoints
//     //        auto deltaG = pi2Pi(std::atan2(CWP(1, 0) - X(1, 0), CWP(0, 0) - X(0, 0)) - X(2, 0) - swa);
//
//     //        // limit rate
//     //        auto maxDelta = rateSWA * dt;
//     //        if (std::abs(deltaG) > maxDelta)
//     //        {
//     //            deltaG = maxDelta * sign<int>(deltaG);
//     //        }
//
//     //        // limit angle
//     //        swa = swa + deltaG;
//     //        if (std::abs(swa) > maxSWA)
//     //        {
//     //            swa = sign<int>(swa) * maxSWA;
//     //        }
//     //    }
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "computeSteering" << std::endl;
//     //}
// }
//
// Eigen::MatrixXf Slam::computeRangeBearing(const Eigen::MatrixXf& X, const Eigen::MatrixXf& LM)
//{
//     //Eigen::MatrixXf Z = Eigen::MatrixXf::Zero(0, 0);
//     //try
//     //{
//     //    // Compute exact observation
//     //    std::vector<float> dx_v;
//     //    std::vector<float> dy_v;
//     //    for (int i = 0; i < LM.cols(); i++)
//     //    {
//     //        dx_v.push_back(LM(0, i) - X(0, 0));
//     //        dy_v.push_back(LM(1, i) - X(1, 0));
//     //    }
//     //    float phi = X(2, 0);
//
//     //    Z.resize(2, LM.cols());
//     //    Z = Eigen::MatrixXf::Zero(2, LM.cols());
//
//     //    for (int index = 0; index < dx_v.size(); index++)
//     //    {
//     //        Z(0, index) = std::sqrt(std::pow(dx_v[index], 2.0F) + std::pow(dy_v[index], 2.0F));
//     //        Z(1, index) = std::atan2(dy_v[index], dx_v[index]) - phi;
//     //    }
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "computeRangeBearing" << std::endl;
//     //}
//     //return Z;
// }
//
// Slam::NormalizedInnovation_t Slam::computeAssociation(const Eigen::MatrixXf& X,
//                                                       const Eigen::MatrixXf& P,
//                                                       const Eigen::MatrixXf& Z,
//                                                       const Eigen::MatrixXf& R,
//                                                       int                    idf)
//{
//     //auto [Zp, H]        = observeModel(X, idf);
//     //Eigen::MatrixXf V   = Z - Zp;
//     //V(1, 0)             = pi2Pi(V(1, 0));
//     //Eigen::MatrixXf S   = H * P * H.transpose() + R;
//     //auto            nis = V.transpose() * S.inverse() * V;
//     //auto            nd  = nis(0, 0) + std::log(S.determinant());
//     //return {nis(0, 0), nd};
// }
//
// Slam::Association_t Slam::dataAssociateTable(const Eigen::MatrixXf& X,
//                                              const Eigen::MatrixXf& Z,
//                                              const Eigen::MatrixXf& IDZ,
//                                              Eigen::MatrixXf&       TABLE)
//{
//     //Eigen::MatrixXf ZF;
//     //Eigen::MatrixXf ZN;
//     //Eigen::MatrixXf IDF;
//     //Eigen::MatrixXf IDN;
//
//     //ZF.resize(0, 0);
//     //ZN.resize(0, 0);
//     //IDF.resize(0, 0);
//     //IDN.resize(0, 0);
//
//     //// find associations (zf) and new features (zn)
//     //for (int i = 0; i < IDZ.cols(); i++)
//     //{
//     //    int id = IDZ(0, i);
//     //    if (TABLE(0, id - 1) == 0) // new feature
//     //    {
//
//     //        if (ZN.rows() == 0 && ZN.cols() == 0)
//     //        {
//     //            ZN.resize(2, 1);
//     //            ZN                   = Eigen::MatrixXf::Zero(2, 1);
//     //            ZN.block(0, 0, 2, 1) = Z.col(i); // Z.block(0, i, Z.rows(), 1);
//     //        }
//     //        else
//     //        {
//     //            const auto ZNT = ZN;
//     //            ZN.resize(ZNT.rows(), ZNT.cols() + 1);
//     //            ZN                            = Eigen::MatrixXf::Zero(ZNT.rows(), ZNT.cols() + 1);
//     //            ZN.block(0, 0, 2, ZNT.cols()) = ZNT;
//     //            ZN.block(0, ZNT.cols(), 2, 1) = Z.col(i); // Z.block(0, i, Z.rows(), 1);
//     //        }
//
//     //        if (IDN.rows() == 0 && IDN.cols() == 0)
//     //        {
//     //            IDN.resize(1, 1);
//     //            IDN       = Eigen::MatrixXf::Zero(1, 1);
//     //            IDN(0, 0) = id;
//     //        }
//     //        else
//     //        {
//     //            const auto IDNT = IDN;
//     //            IDN.resize(IDNT.rows(), IDNT.cols() + 1);
//     //            IDN                             = Eigen::MatrixXf::Zero(IDNT.rows(), IDNT.cols() + 1);
//     //            IDN.block(0, 0, 1, IDNT.cols()) = IDNT;
//     //            IDN(0, IDNT.cols())             = id;
//     //        }
//     //    }
//     //    else
//     //    {
//     //        if (ZF.rows() == 0 && ZF.cols() == 0)
//     //        {
//     //            ZF.resize(2, 1);
//     //            ZF                   = Eigen::MatrixXf::Zero(2, 1);
//     //            ZF.block(0, 0, 2, 1) = Z.col(i); // Z.block(0, i, Z.rows(), 1);
//     //        }
//     //        else
//     //        {
//     //            const auto ZFT = ZF;
//     //            ZF.resize(ZFT.rows(), ZFT.cols() + 1);
//     //            ZF                            = Eigen::MatrixXf::Zero(ZFT.rows(), ZFT.cols() + 1);
//     //            ZF.block(0, 0, 2, ZFT.cols()) = ZFT;
//     //            ZF.block(0, ZFT.cols(), 2, 1) = Z.col(i); // Z.block(0, i, Z.rows(), 1);
//     //        }
//
//     //        if (IDF.rows() == 0 && IDF.cols() == 0)
//     //        {
//     //            IDF.resize(1, 1);
//     //            IDF       = Eigen::MatrixXf::Zero(1, 1);
//     //            IDF(0, 0) = TABLE(id - 1);
//     //        }
//     //        else
//     //        {
//     //            const auto IDFT = IDF;
//     //            IDF.resize(IDFT.rows(), IDFT.cols() + 1);
//     //            IDF                             = Eigen::MatrixXf::Zero(IDFT.rows(), IDFT.cols() + 1);
//     //            IDF.block(0, 0, 1, IDFT.cols()) = IDFT;
//     //            IDF(0, IDFT.cols())             = TABLE(id - 1);
//     //        }
//     //    }
//     //}
//
//     //// add new feature IDs to lookup table
//     //int   nxv = 3;                                           // number of vehicle pose states
//     //float nf  = (std::max(X.rows(), X.cols()) - nxv) / 2.0F; // number of features already in map
//     //// table(idn) = Nf + (1 : size(zn, 2)); // add new feature positions to lookup table
//
//     //std::vector<float> temp = {};
//     //for (int i = 1; i <= ZN.cols(); i++)
//     //{
//     //    temp.push_back(nf + i);
//     //}
//     //for (int i = 0; i < IDN.cols(); i++)
//     //{
//     //    int id           = IDN(0, i);
//     //    TABLE(0, id - 1) = temp.at(i);
//     //}
//
//     //return {ZF, ZN, IDF};
// }
//
// Slam::Association_t Slam::dataAssociate(const Eigen::MatrixXf& X,
//                                         const Eigen::MatrixXf& P,
//                                         const Eigen::MatrixXf& Z,
//                                         const Eigen::MatrixXf& R,
//                                         const float&           gate1,
//                                         const float&           gate2)
//{
//     //Eigen::MatrixXf ZF;
//     //Eigen::MatrixXf ZN;
//     //Eigen::MatrixXf IDF;
//
//     //ZF.resize(0, 0);
//     //ZN.resize(0, 0);
//     //IDF.resize(0, 0);
//
//     //try
//     //{
//     //    int nxv = 3;                                        // number of vehicle pose state
//     //    int nf  = (std::max(X.rows(), X.cols()) - nxv) / 2; // number of features already in map
//
//     //    // linear search for nearest-neighbour, no clever tricks (like a quick bounding - box threshold to remove
//     //    // distant features; or, better yet, a balanced k-d tree lookup). TODO: implement clever tricks.
//     //    for (int i = 0; i < Z.cols(); i++)
//     //    {
//     //        int   jbest = 0;
//     //        float nbest = std::numeric_limits<float>::infinity();
//     //        float outer = std::numeric_limits<float>::infinity();
//
//     //        // search for neighbours
//     //        for (int j = 1; j <= nf; j++)
//     //        {
//     //            auto [nis, nd] = computeAssociation(X, P, Z.col(i), R, j); // root casue is here
//
//     //            // if within gate, store nearest-neighbour
//     //            if (nis < gate1 && nd < nbest)
//     //            {
//     //                nbest = nd;
//     //                jbest = j;
//     //            }
//     //            else if (nis < outer) // else store best nis value
//     //            {
//     //                outer = nis;
//     //            }
//     //        }
//
//     //        // add nearest-neighbour to association list
//     //        if (jbest != 0.0F)
//     //        {
//     //            if (ZF.rows() == 0 && ZF.cols() == 0)
//     //            {
//     //                ZF.resize(2, 1);
//     //                ZF                   = Eigen::MatrixXf::Zero(2, 1);
//     //                ZF.block(0, 0, 2, 1) = Z.col(i); // Z.block(0, i, Z.rows(), 1);
//     //            }
//     //            else
//     //            {
//     //                const auto ZFT = ZF;
//     //                ZF.resize(ZFT.rows(), ZFT.cols() + 1);
//     //                ZF                            = Eigen::MatrixXf::Zero(ZFT.rows(), ZFT.cols() + 1);
//     //                ZF.block(0, 0, 2, ZFT.cols()) = ZFT;
//     //                ZF.block(0, ZFT.cols(), 2, 1) = Z.col(i); // Z.block(0, i, Z.rows(), 1);
//     //            }
//
//     //            if (IDF.rows() == 0 && IDF.cols() == 0)
//     //            {
//     //                IDF.resize(1, 1);
//     //                IDF       = Eigen::MatrixXf::Zero(1, 1);
//     //                IDF(0, 0) = jbest;
//     //            }
//     //            else
//     //            {
//     //                const auto IDFT = IDF;
//     //                IDF.resize(IDFT.rows(), IDFT.cols() + 1);
//     //                IDF                             = Eigen::MatrixXf::Zero(IDFT.rows(), IDFT.cols() + 1);
//     //                IDF.block(0, 0, 1, IDFT.cols()) = IDFT;
//     //                IDF(0, IDFT.cols())             = jbest;
//     //            }
//     //        }
//     //        else if (outer > gate2) // Z too far to associate, but far enough to be a new feature
//     //        {
//     //            if (ZN.rows() == 0 && ZN.cols() == 0)
//     //            {
//     //                ZN.resize(2, 1);
//     //                ZN                   = Eigen::MatrixXf::Zero(2, 1);
//     //                ZN.block(0, 0, 2, 1) = Z.col(i); // Z.block(0, i, Z.rows(), 1);
//     //            }
//     //            else
//     //            {
//     //                const auto ZNT = ZN;
//     //                ZN.resize(ZNT.rows(), ZNT.cols() + 1);
//     //                ZN                            = Eigen::MatrixXf::Zero(ZNT.rows(), ZNT.cols() + 1);
//     //                ZN.block(0, 0, 2, ZNT.cols()) = ZNT;
//     //                ZN.block(0, ZNT.cols(), 2, 1) = Z.col(i); // Z.block(0, i, Z.rows(), 1);
//     //            }
//     //        }
//     //    }
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "dataAssociate" << std::endl;
//     //}
//     //return {ZF, ZN, IDF};
// }
//
// Slam::Observation_t Slam::getObservations(const Eigen::MatrixXf& X,
//                                           const Eigen::MatrixXf& LM,
//                                           const Eigen::MatrixXf& IDF,
//                                           const float&           rmax)
//{
//     //auto [iLM, iIDF] = getVisibleLandmarks(X, LM, IDF, rmax);
//     //return {computeRangeBearing(X, iLM), iIDF};
// }
//
// Slam::VisibleLandmarks_t Slam::getVisibleLandmarks(const Eigen::MatrixXf& X,
//                                                    const Eigen::MatrixXf& LM,
//                                                    const Eigen::MatrixXf& IDF,
//                                                    const float&           rmax)
//{
//     //Eigen::MatrixXf iLM  = LM;
//     //Eigen::MatrixXf iIDF = IDF;
//     //try
//     //{
//     //    std::vector<float> dx_v;
//     //    std::vector<float> dy_v;
//     //    for (int i = 0; i < LM.cols(); i++)
//     //    {
//     //        dx_v.push_back(LM(0, i) - X(0, 0));
//     //        dy_v.push_back(LM(1, i) - X(1, 0));
//     //    }
//     //    float phi = X(2, 0);
//
//     //    // incremental tests for bounding semi-circle
//     //    std::vector<float> dr_t;
//     //    std::fill(dr_t.begin(), dr_t.end(), 0);
//     //    int              counter = 0;
//     //    std::vector<int> tracker = {};
//     //    std::transform(dx_v.begin(),
//     //                   dx_v.end(),
//     //                   dy_v.begin(),
//     //                   std::back_inserter(dr_t),
//     //                   [&](float i, float j)
//     //                   {
//     //                       counter++;
//     //                       if ((std::abs(i) < rmax && std::abs(j) < rmax) &&                     // bounding box
//     //                           ((i * std::cos(phi) + j * std::sin(phi)) > 0) &&                  // bounding line
//     //                           ((std::pow(i, 2.0F) + std::pow(j, 2.0F)) < std::pow(rmax, 2.0F))) // bounding circle
//     //                       {
//     //                           tracker.push_back(counter);
//     //                           return counter;
//     //                       }
//     //                   });
//
//     //    // the bounding box test is unnecessary but illustrates a possible speedup technique as it quickly
//     eliminates
//     //    // distant points.Ordering the landmark set would make this operation %O(logN) rather that O(N).
//     //    const auto LMT  = iLM;
//     //    const auto IDFT = iIDF;
//
//     //    iLM.resize(2, tracker.size());
//     //    iLM = Eigen::MatrixXf::Zero(2, tracker.size());
//
//     //    iIDF.resize(1, tracker.size());
//     //    iIDF = Eigen::MatrixXf::Zero(1, tracker.size());
//
//     //    int index = 0;
//     //    for (const auto& t : tracker)
//     //    {
//     //        iLM.block(0, index, 2, 1)  = LMT.block(0, t - 1, 2, 1);
//     //        iIDF.block(0, index, 1, 1) = IDFT.block(0, t - 1, 1, 1);
//     //        index++;
//     //    }
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "getVisibleLandmarks" << std::endl;
//     //}
//     //return {iLM, iIDF};
// }
//
// void Slam::josephUpdate(Eigen::MatrixXf&       X,
//                         Eigen::MatrixXf&       P,
//                         const Eigen::MatrixXf& V,
//                         const Eigen::MatrixXf& R,
//                         const Eigen::MatrixXf& H)
//{
//     //try
//     //{
//     //    const auto      PHT = P * H.transpose();
//     //    Eigen::MatrixXf S   = H * PHT + R;
//     //    Eigen::MatrixXf SI  = S.inverse();
//     //    SI                  = makeSymmetric(SI);
//     //    Eigen::MatrixXf W   = PHT * SI;
//
//     //    X = X + W * V;
//
//     //    // Joseph-form covariance update
//     //    Eigen::MatrixXf C = Eigen::MatrixXf::Identity(P.rows(), P.cols()) - W * H;
//     //    P                 = C * P * C.transpose() + W * R * W.transpose();
//     //    P                 = P + Eigen::MatrixXf::Identity(P.rows(), P.cols()) * std::numeric_limits<float>::min();
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "josephUpdate" << std::endl;
//     //}
// }
//
// void Slam::landmarkBasedNavigation()
//{
//     //try
//     //{
//     //    Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(2, 2);
//     //    Q(0, 0)           = std::pow(mSigmaV, 2.0F);
//     //    Q(0, 1)           = 0.0F;
//     //    Q(1, 0)           = 0.0F;
//     //    Q(1, 1)           = std::pow(mSigmaSWA, 2.0F);
//
//     //    Eigen::MatrixXf R = Eigen::MatrixXf::Zero(2, 2);
//     //    R(0, 0)           = std::pow(mSigmaR, 2.0F);
//     //    R(0, 1)           = 0.0F;
//     //    R(1, 0)           = 0.0F;
//     //    R(1, 1)           = std::pow(mSigmaB, 2.0F);
//
//     //    // initialise states
//     //    Eigen::MatrixXf XTrue = Eigen::MatrixXf::Zero(3, 1);
//     //    Eigen::MatrixXf X     = Eigen::MatrixXf::Zero(3, 1);
//     //    Eigen::MatrixXf P     = Eigen::MatrixXf::Zero(3, 3);
//
//     //    auto            dt         = mDtControls;                          // change in time between predicts
//     //    float           dtsum      = 0.0F;                                 // change in time since last observation
//     //    Eigen::MatrixXf FeatureTag = Eigen::MatrixXf::Zero(1, mLM.cols()); // identifier for each landmark
//     //    for (int id = 0; id < mLM.cols(); id++)
//     //    {
//     //        FeatureTag(0, id) = static_cast<float>(id) + 1.0F;
//     //    }
//
//     //    int   iwp = 1;    //  index of first waypoint
//     //    float swa = 0.0F; // initial steering wheel angle
//
//     //    Eigen::MatrixXf QE = Q;
//     //    Eigen::MatrixXf RE = R;
//
//     //    // inflate estimated noises (ie, add stabilising noise)
//     //    if (mSwitchInflateNoise)
//     //    {
//     //        QE = 2 * Q;
//     //        RE = 8 * R;
//     //    }
//
//     //    while (iwp <= mWP.cols() && iwp > 0)
//     //    {
//     //        // compute steering wheel angle
//     //        computeSWA(XTrue, mWP, iwp, mAtWaypoint, swa, mRateSWA, mMaxSWA, dt);
//
//     //        // perform loops : if final waypoint reached, go back to first
//     //        if (iwp == 0 && mNumberLoops > 1)
//     //        {
//     //            iwp          = 1;
//     //            mNumberLoops = mNumberLoops - 1;
//     //        }
//
//     //        vehicleModel(XTrue, mVelocity, swa, mWheelBase, dt); // movment of the platform(Odo meter reading)
//
//     //        auto [vn, swan] = addControlNoise(mVelocity, swa, Q, mSwitchControlNoise);
//
//     //        // EKF predict
//     //        predict(X, P, vn, swan, QE, mWheelBase, dt);
//
//     //        // if heading known, observe heading
//     //        observeHeading(X, P, XTrue(2), mSwitchHeadingKnown);
//
//     //        // EKF update step
//     //        dtsum = dtsum + dt;
//     //        if (dtsum >= mDtObserve)
//     //        {
//     //            dtsum = 0.0F;
//
//     //            auto [Z, FeatureTagVisible] = getObservations(XTrue, mLM, FeatureTag, mMaxRange);
//
//     //            addObservationNoise(Z, R, mSwitchSensorNoise);
//
//     //            if (mSwitchAssociationKnown)
//     //            {
//
//     //                auto [ZF, ZN, IDF] = dataAssociateTable(X, Z, FeatureTagVisible, mTABLE);
//     //                update(X, P, ZF, RE, IDF, mSwitchBatchUpdate);
//     //                augment(X, P, ZN, RE);
//     //            }
//     //            else
//     //            {
//     //                auto [ZF, ZN, IDF] = dataAssociate(X, P, Z, RE, mGateReject, mGateAugment);
//     //                update(X, P, ZF, RE, IDF, mSwitchBatchUpdate);
//     //                augment(X, P, ZN, RE);
//     //            }
//     //        }
//     //    }
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "landmarkBasedNavigation" << std::endl;
//     //}
// }
//
// void Slam::observeHeading(Eigen::MatrixXf& X, Eigen::MatrixXf& P, const float& phi, bool useHeading)
//{
//     //try
//     //{
//     //    if (!useHeading)
//     //    {
//     //        return;
//     //    }
//     //    // heading uncertainty - radians
//     //    float           sigmaPhi = 0.01F * std::_Pi_val / 180.0F; // radians, heading uncertainty
//     //    Eigen::MatrixXf H        = Eigen::MatrixXf::Zero(1, std::max(X.rows(), X.cols()));
//     //    H(0, 2)                  = 1.0F;
//
//     //    Eigen::MatrixXf V = Eigen::MatrixXf::Zero(1, 1);
//     //    V(0, 0)           = pi2Pi(phi - X(2, 0));
//
//     //    Eigen::MatrixXf R = Eigen::MatrixXf::Zero(1, 1);
//     //    R(0, 0)           = std::pow(sigmaPhi, 2.0F);
//     //    josephUpdate(X, P, V, R, H);
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "observeHeading" << std::endl;
//     //}
// }
//
// Slam::ObserveModel_t Slam::observeModel(const Eigen::MatrixXf& X, int idf)
//{
//     //int nxv  = 3;                   // number of vehicle pose states
//     //int fpos = nxv + (idf * 2) - 1; // position of xf in state
//
//     //Eigen::MatrixXf H = Eigen::MatrixXf::Zero(2, std::max(X.rows(), X.cols()));
//     //Eigen::MatrixXf Z = Eigen::MatrixXf::Zero(2, 1);
//
//     //try
//     //{
//     //    if (X.rows() > 3)
//     //    {
//     //        // auxiliary values
//     //        float dx  = X(fpos - 1, 0) - X(0, 0);
//     //        float dy  = X(fpos, 0) - X(1, 0);
//     //        float d2  = std::pow(dx, 2.0F) + std::pow(dy, 2.0F);
//     //        float d   = std::sqrt(d2);
//     //        float xd  = dx / d;
//     //        float yd  = dy / d;
//     //        float xd2 = dx / d2;
//     //        float yd2 = dy / d2;
//
//     //        // predict Z
//     //        Z(0, 0) = d;
//     //        Z(1, 0) = std::atan2(dy, dx) - X(2, 0);
//
//     //        // calculate H
//     //        Eigen::MatrixXf HU = Eigen::MatrixXf::Zero(2, 3);
//     //        HU(0, 0)           = -xd;
//     //        HU(0, 1)           = -yd;
//     //        HU(0, 2)           = 0.0F;
//     //        HU(1, 0)           = yd2;
//     //        HU(1, 1)           = -xd2;
//     //        HU(1, 2)           = -1.0F;
//     //        Eigen::MatrixXf LU = Eigen::MatrixXf::Zero(2, 2);
//     //        LU(0, 0)           = xd;
//     //        LU(0, 1)           = yd;
//     //        LU(1, 0)           = -yd2;
//     //        LU(1, 1)           = xd2;
//
//     //        H.block(0, 0, 2, 3)        = HU;
//     //        H.block(0, fpos - 1, 2, 2) = LU;
//     //    }
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "observeModel" << std::endl;
//     //}
//
//     //return {Z, H};
// }
//
// float Slam::pi2Pi(float angle)
//{
//     //angle = std::fmod(angle, static_cast<float>(2 * std::_Pi_val));
//     //if (angle > std::_Pi_val)
//     //{
//     //    angle = angle - (2.0F * std::_Pi_val);
//     //}
//     //if (angle < -std::_Pi_val)
//     //{
//     //    angle = angle + (2 * std::_Pi_val);
//     //}
//
//     //return angle;
// }
//
// void Slam::predict(Eigen::MatrixXf&       X,
//                    Eigen::MatrixXf&       P,
//                    const float&           v,
//                    const float&           swa,
//                    const Eigen::MatrixXf& Q,
//                    const float&           wb,
//                    const float&           dt)
//{
//     //try
//     //{
//     //    float s   = std::sin(swa + X(2, 0));
//     //    float c   = std::cos(swa + X(2, 0));
//     //    float vts = v * dt * s;
//     //    float vtc = v * dt * c;
//
//     //    // jacobians
//     //    Eigen::MatrixXf Gv = Eigen::MatrixXf::Zero(3, 3);
//     //    Gv(0, 0)           = 1.0F;
//     //    Gv(0, 1)           = 0.0F;
//     //    Gv(0, 2)           = -vts;
//     //    Gv(1, 0)           = 0.0F;
//     //    Gv(1, 1)           = 1.0F;
//     //    Gv(1, 2)           = vtc;
//     //    Gv(2, 0)           = 0.0F;
//     //    Gv(2, 1)           = 0.0F;
//     //    Gv(2, 2)           = 1.0F;
//
//     //    Eigen::MatrixXf Gu = Eigen::MatrixXf::Zero(3, 2);
//     //    Gu(0, 0)           = dt * c;
//     //    Gu(0, 1)           = -vts;
//     //    Gu(1, 0)           = dt * s;
//     //    Gu(1, 1)           = vtc;
//     //    Gu(2, 0)           = dt * std::sin(swa) / wb;
//     //    Gu(2, 1)           = v * dt * std::cos(swa) / wb;
//
//     //    // predict covariance
//     //    P.block(0, 0, 3, 3) = Gv * P.block(0, 0, 3, 3) * Gv.transpose() + Gu * Q * Gu.transpose();
//     //    if (P.rows() > 3)
//     //    {
//     //        P.block(0, 3, 3, P.cols() - 4) = Gv * P.block(0, 3, 3, P.cols() - 4);
//     //        P.block(3, 0, P.cols() - 4, 3) = P.block(0, 3, 3, P.cols() - 4).transpose();
//     //    }
//
//     //    // predicted state
//     //    X(0, 0) = X(0, 0) + vtc;
//     //    X(1, 0) = X(1, 0) + vts;
//     //    X(2, 0) = pi2Pi(X(2, 0) + v * dt * std::sin(swa) / wb);
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "predict" << std::endl;
//     //}
// }
//
// void Slam::singleUpdate(Eigen::MatrixXf&       X,
//                         Eigen::MatrixXf&       P,
//                         const Eigen::MatrixXf& Z,
//                         const Eigen::MatrixXf& R,
//                         const Eigen::MatrixXf& IDF)
//{
//     //try
//     //{
//     //    int lenZ = Z.cols();
//     //    for (int i = 0; i < lenZ; i++)
//     //    {
//     //        auto [Zp, H] = observeModel(X, IDF(0, i));
//
//     //        Eigen::MatrixXf V = Eigen::MatrixXf::Zero(2, 1);
//     //        V(0, 0)           = Z(0, i) - Zp(0, 0);
//     //        V(1, 0)           = pi2Pi(Z(1, i) - Zp(1, 0));
//
//     //        choleskyUpdate(X, P, V, R, H);
//     //    }
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "singleUpdate" << std::endl;
//     //}
// }
//
// void Slam::update(Eigen::MatrixXf&       X,
//                   Eigen::MatrixXf&       P,
//                   const Eigen::MatrixXf& Z,
//                   const Eigen::MatrixXf& R,
//                   const Eigen::MatrixXf& IDF,
//                   bool                   batch)
//{
//     //if (batch)
//     //{
//     //    batchUpdate(X, P, Z, R, IDF);
//     //}
//     //else
//     //{
//     //    singleUpdate(X, P, Z, R, IDF);
//     //}
// }
//
// void Slam::vehicleModel(Eigen::MatrixXf& X, const float& v, const float& swa, const float& wb, const float& dt)
//{
//     //try
//     //{
//     //    const auto TX = X;
//     //    X.setZero(X.rows(), X.cols());
//     //    X(0, 0) = TX(0, 0) + v * dt * std::cos(swa + TX(2, 0));
//     //    X(1, 0) = TX(1, 0) + v * dt * std::sin(swa + TX(2, 0));
//     //    X(2, 0) = pi2Pi(TX(2, 0) + v * dt * std::sin(swa) / wb);
//     //}
//     //catch (std::exception& e)
//     //{
//     //    std::cout << e.what() << "\t" << "vehicleModel" << std::endl;
//     //}
// }
