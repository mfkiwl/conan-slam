#include "EKF.h"

EKF::EKF(const Eigen::MatrixXf& landMarks, const Eigen::MatrixXf& wayPoints)
    : Slam(landMarks, wayPoints)
{
    mTABLE = Eigen::VectorXi::Zero(getLandMarks().cols());
}

void EKF::augment(Eigen::VectorXf& X, Eigen::MatrixXf& P, const Eigen::MatrixXf& Z, const Eigen::MatrixXf& R)
{
    try
    {
        // add new features to state
        for (int index = 0; index < Z.cols(); index++)
        {
            Eigen::MatrixXf TZ = Eigen::MatrixXf::Zero(2, 1);
            TZ(0, 0)           = Z(0, index);
            TZ(1, 0)           = Z(1, index);
            addOneNewFeature(X, P, TZ, R);
        }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "augment" << std::endl;
    }
}

void EKF::addOneNewFeature(Eigen::VectorXf& X, Eigen::MatrixXf& P, const Eigen::MatrixXf& Z, const Eigen::MatrixXf& R)
{
    try
    {
        int len = X.rows();

        auto r = Z(0, 0);
        auto b = Z(1, 0);
        auto s = std::sin(X(2, 0) + b);
        auto c = std::cos(X(2, 0) + b);

        // augment X
        const auto AX = X;
        X.resize(AX.rows() + 2);
        X.setZero(X.rows());
        X.block(0, 0, AX.rows(), 1) = AX;
        Eigen::MatrixXf SF;
        SF.resize(2, 1);
        SF.setZero(2, 1);
        SF(0, 0)                    = AX(0, 0) + (r * c);
        SF(1, 0)                    = AX(1, 0) + (r * s);
        X.block(AX.rows(), 0, 2, 1) = SF;
        SF.resize(0, 0);

        Eigen::MatrixXf Gv = Eigen::MatrixXf::Zero(2, 3);
        Gv(0, 0)           = 1.0F;
        Gv(0, 1)           = 0.0F;
        Gv(0, 2)           = -r * s;
        Gv(1, 0)           = 0.0F;
        Gv(1, 1)           = 1.0F;
        Gv(1, 2)           = r * c;

        Eigen::MatrixXf Gz = Eigen::MatrixXf::Zero(2, 2);
        Gz(0, 0)           = c;
        Gz(0, 1)           = -r * s;
        Gz(1, 0)           = s;
        Gz(1, 1)           = r * c;

        // augment P
        const auto AP          = P;
        int        resizeLimit = 2;
        P.resize(len + resizeLimit, len + resizeLimit);
        P.setZero(P.rows(), P.cols());
        P.block(0, 0, AP.rows(), AP.cols()) = AP;

        // feature covariance
        P.block(len, len, 2, 2) = (Gv * AP.block(0, 0, 3, 3) * Gv.transpose()) + (Gz * R * Gz.transpose());

        // vehicle to feature xcorr
        P.block(len, 0, 2, 3) = Gv * P.block(0, 0, 3, 3);
        P.block(0, len, 3, 2) = P.block(len, 0, 2, 3).transpose();

        if (len > 3)
        {
            // map to feature xcorr
            P.block(len, 3, 2, len - 3) = Gv * P.block(0, 3, 3, len - 3);
            P.block(3, len, len - 3, 2) = P.block(len, 3, 2, len - 3).transpose();
        }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "addOneNewFeature" << std::endl;
    }
}

void EKF::batchUpdate(Eigen::VectorXf&       X,
                      Eigen::MatrixXf&       P,
                      const Eigen::MatrixXf& Z,
                      const Eigen::MatrixXf& R,
                      const Eigen::VectorXi& idf)
{
    try
    {
        int lenZ = Z.cols();
        int lenX = X.rows();

        Eigen::MatrixXf H  = Eigen::MatrixXf::Zero(2 * lenZ, lenX);
        Eigen::MatrixXf V  = Eigen::MatrixXf::Zero(2 * lenZ, 1);
        Eigen::MatrixXf RR = Eigen::MatrixXf::Zero(2 * lenZ, 2 * lenZ);

        for (int i = 0; i < lenZ; i++)
        {
            auto [Zp, HT] = observeModel(X, idf(i));

            Eigen::MatrixXf L                                      = Eigen::MatrixXf::Zero(1, 2);
            L(0, 0)                                                = 2 * (i + 1) - 1;
            L(0, 1)                                                = 2 * (i + 1);
            H.block(L(0, 0) - 1, 0, L.cols(), H.cols())            = HT;
            Eigen::MatrixXf VT                                     = Eigen::MatrixXf::Zero(2, 1);
            VT(0, 0)                                               = Z(0, i) - Zp(0, 0);
            VT(1, 0)                                               = pi2Pi(Z(1, i) - Zp(1, 0));
            V.block(L(0, 0) - 1, 0, L.cols(), 1)                   = VT;
            RR.block(L(0, 0) - 1, L(0, 0) - 1, L.cols(), L.cols()) = R;
        }

        choleskyUpdate(X, P, V, RR, H);
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "batchUpdate" << std::endl;
    }
}

EKF::NormalizedInnovation_t EKF::computeAssociation(const Eigen::VectorXf& X,
                                                    const Eigen::MatrixXf& P,
                                                    const Eigen::MatrixXf& Z,
                                                    const Eigen::MatrixXf& R,
                                                    int                    idf)
{
    auto [Zp, H]        = observeModel(X, idf);
    Eigen::MatrixXf V   = Z - Zp;
    V(1, 0)             = pi2Pi(V(1, 0));
    Eigen::MatrixXf S   = H * P * H.transpose() + R;
    auto            nis = V.transpose() * S.inverse() * V;
    auto            nd  = nis(0, 0) + std::log(S.determinant());
    return {nis(0, 0), nd};
}

EKF::Association_t EKF::dataAssociateTable(const Eigen::VectorXf& X,
                                           const Eigen::MatrixXf& Z,
                                           const Eigen::VectorXi& idz,
                                           Eigen::VectorXi&       table)
{
    Eigen::MatrixXf ZF;
    ZF.resize(0, 0);
    ZF = Eigen::MatrixXf::Zero(0, 0);
    Eigen::MatrixXf ZN;
    ZN.resize(0, 0);
    ZN = Eigen::MatrixXf::Zero(0, 0);
    Eigen::VectorXi idfo;
    idfo.resize(0);
    idfo = Eigen::VectorXi::Zero(0);

    try
    {
        std::vector<Eigen::MatrixXf> ZFi = {};
        std::vector<Eigen::MatrixXf> ZNi = {};
        std::vector<int>             idf = {};
        std::vector<int>             idn = {};

        // find associations (zf) and new features (zn)
        for (int i = 0; i < idz.rows(); i++)
        {
            int id = idz(i);
            if (table(id - 1) == 0) // new feature
            {
                ZNi.push_back(Z.col(i));
                idn.push_back(id);
            }
            else
            {
                ZFi.push_back(Z.col(i));
                idf.push_back(table(id - 1));
            }
        }

        int index = 0;
        if (ZFi.size() > 0)
        {
            ZF = Eigen::MatrixXf::Zero(2, ZFi.size());
            for (const auto& elem : ZFi)
            {
                ZF.block(0, index, 2, 1) = elem.block(0, 0, 2, 1);
                index++;
            }
        }

        if (ZNi.size() > 0)
        {
            ZN    = Eigen::MatrixXf::Zero(2, ZNi.size());
            index = 0;
            for (const auto& elem : ZNi)
            {
                ZN.block(0, index, 2, 1) = elem.block(0, 0, 2, 1);
                index++;
            }
        }

        if (idf.size() > 0)
        {
            idfo = Eigen::VectorXi::Zero(idf.size());
            std::copy(idf.begin(), idf.end(), idfo.begin());
        }

        // add new feature IDs to lookup table
        int nxv = 3;                       // number of vehicle pose states
        int nf  = (X.rows() - nxv) / 2.0F; // number of features already in map

        // add new feature positions to lookup table
        std::vector<float> gatherNewPos = {};
        for (int i = 1; i <= ZN.cols(); i++)
        {
            gatherNewPos.push_back(nf + i);
        }
        for (int i = 0; i < idn.size(); i++)
        {
            int id        = idn[i];
            table(id - 1) = gatherNewPos.at(i);
        }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "dataAssociateTable" << std::endl;
    }
    return {ZF, ZN, idfo};
}

EKF::Association_t EKF::dataAssociate(const Eigen::VectorXf& X,
                                      const Eigen::MatrixXf& P,
                                      const Eigen::MatrixXf& Z,
                                      const Eigen::MatrixXf& R,
                                      const float&           gate1,
                                      const float&           gate2)
{
    Eigen::MatrixXf ZF;
    ZF.resize(0, 0);
    ZF = Eigen::MatrixXf::Zero(0, 0);
    Eigen::MatrixXf ZN;
    ZN.resize(0, 0);
    ZN = Eigen::MatrixXf::Zero(0, 0);
    Eigen::VectorXi idfo;
    idfo.resize(0);
    idfo = Eigen::VectorXi::Zero(0);

    try
    {
        int nxv = 3;                                        // number of vehicle pose state
        int nf  = (std::max(X.rows(), X.cols()) - nxv) / 2; // number of features already in map

        // linear search for nearest-neighbour, no clever tricks (like a quick bounding - box threshold to remove
        // distant features; or, better yet, a balanced k-d tree lookup). TODO: implement clever tricks.
        std::vector<Eigen::MatrixXf> ZFi = {};
        std::vector<Eigen::MatrixXf> ZNi = {};
        std::vector<int>             idf = {};

        for (int i = 0; i < Z.cols(); i++)
        {
            int   jbest = 0;
            float nbest = std::numeric_limits<float>::infinity();
            float outer = std::numeric_limits<float>::infinity();

            // search for neighbours
            for (int j = 1; j <= nf; j++)
            {
                auto [nis, nd] = computeAssociation(X, P, Z.col(i), R, j); // root casue is here

                // if within gate, store nearest-neighbour
                if (nis < gate1 && nd < nbest)
                {
                    nbest = nd;
                    jbest = j;
                }
                else if (nis < outer) // else store best nis value
                {
                    outer = nis;
                }
            }

            // add nearest-neighbour to association list
            if (jbest != 0.0F)
            {
                ZFi.push_back(Z.col(i));
                idf.push_back(jbest);
            }
            else if (outer > gate2) // Z too far to associate, but far enough to be a new feature
            {
                ZNi.push_back(Z.col(i));
            }
        }

        // eigen format
        ZF.resize(0, 0);
        ZF        = Eigen::MatrixXf::Zero(2, ZFi.size());
        int index = 0;
        for (const auto& elem : ZFi)
        {
            ZF.block(0, index, 2, 1) = elem.block(0, 0, 2, 1);
            index++;
        }

        Eigen::MatrixXf ZN;
        ZN.resize(0, 0);
        ZN    = Eigen::MatrixXf::Zero(2, ZNi.size());
        index = 0;
        for (const auto& elem : ZNi)
        {
            ZN.block(0, index, 2, 1) = elem.block(0, 0, 2, 1);
        }

        idfo.resize(0);
        idfo = Eigen::VectorXi::Zero(idf.size());
        std::copy(idf.begin(), idf.end(), idfo.begin());
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "dataAssociate" << std::endl;
    }
    return {ZF, ZN, idfo};
}

void EKF::observeHeading(Eigen::VectorXf& X, Eigen::MatrixXf& P, const float& phi, bool useHeading)
{
    try
    {
        if (!useHeading)
        {
            return;
        }
        // heading uncertainty - radians
        float           sigmaPhi = 0.01F * std::_Pi_val / 180.0F; // radians, heading uncertainty
        Eigen::MatrixXf H        = Eigen::MatrixXf::Zero(1, X.rows());
        H(0, 2)                  = 1.0F;

        Eigen::VectorXf V = Eigen::VectorXf::Zero(1);
        V(0)              = pi2Pi(phi - X(2));

        Eigen::MatrixXf R = Eigen::MatrixXf::Zero(1, 1);
        R(0, 0)           = std::pow(sigmaPhi, 2.0F);
        josephUpdate(X, P, V, R, H);
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "observeHeading" << std::endl;
    }
}

EKF::ObserveModel_t EKF::observeModel(const Eigen::VectorXf& X, int idf)
{
    int nxv  = 3;                   // number of vehicle pose states
    int fpos = nxv + (idf * 2) - 1; // position of xf in state

    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(2, X.rows());
    Eigen::MatrixXf Z = Eigen::MatrixXf::Zero(2, 1);

    try
    {
        if (X.rows() > 3)
        {
            // auxiliary values
            float dx  = X(fpos - 1) - X(0);
            float dy  = X(fpos) - X(1);
            float d2  = std::pow(dx, 2.0F) + std::pow(dy, 2.0F);
            float d   = std::sqrt(d2);
            float xd  = dx / d;
            float yd  = dy / d;
            float xd2 = dx / d2;
            float yd2 = dy / d2;

            // predict Z
            Z(0, 0) = d;
            Z(1, 0) = std::atan2(dy, dx) - X(2);

            // calculate H
            Eigen::MatrixXf HU = Eigen::MatrixXf::Zero(2, 3);
            HU(0, 0)           = -xd;
            HU(0, 1)           = -yd;
            HU(0, 2)           = 0.0F;
            HU(1, 0)           = yd2;
            HU(1, 1)           = -xd2;
            HU(1, 2)           = -1.0F;
            Eigen::MatrixXf LU = Eigen::MatrixXf::Zero(2, 2);
            LU(0, 0)           = xd;
            LU(0, 1)           = yd;
            LU(1, 0)           = -yd2;
            LU(1, 1)           = xd2;

            H.block(0, 0, 2, 3)        = HU;
            H.block(0, fpos - 1, 2, 2) = LU;
        }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "observeModel" << std::endl;
    }

    return {Z, H};
}

void EKF::predict(Eigen::VectorXf&       X,
                  Eigen::MatrixXf&       P,
                  const float&           v,
                  const float&           swa,
                  const Eigen::MatrixXf& Q,
                  const float&           wb,
                  const float&           dt)
{
    try
    {
        float phi = X(2);

        // jacobians
        Eigen::MatrixXf Gv = Eigen::MatrixXf::Zero(3, 3);
        Gv(0, 0)           = 1.0F;
        Gv(0, 1)           = 0.0F;
        Gv(0, 2)           = -v * dt * std::sin(swa + phi);
        Gv(1, 0)           = 0.0F;
        Gv(1, 1)           = 1.0F;
        Gv(1, 2)           = v * dt * std::cos(swa + phi);
        Gv(2, 0)           = 0.0F;
        Gv(2, 1)           = 0.0F;
        Gv(2, 2)           = 1.0F;

        Eigen::MatrixXf Gu = Eigen::MatrixXf::Zero(3, 2);
        Gu(0, 0)           = dt * std::cos(swa + phi);
        Gu(0, 1)           = -v * dt * std::sin(swa + phi);
        Gu(1, 0)           = dt * std::sin(swa + phi);
        Gu(1, 1)           = v * dt * std::cos(swa + phi);
        Gu(2, 0)           = dt * std::sin(swa) / wb;
        Gu(2, 1)           = v * dt * std::cos(swa) / wb;

        // predict covariance
        P.block(0, 0, 3, 3) = Gv * P.block(0, 0, 3, 3) * Gv.transpose() + Gu * Q * Gu.transpose();
        if (P.rows() > 3)
        {
            P.block(0, 3, 3, P.cols() - 4) = Gv * P.block(0, 3, 3, P.cols() - 4);
            P.block(3, 0, P.cols() - 4, 3) = P.block(0, 3, 3, P.cols() - 4).transpose();
        }

        // predicted state
        X(0) = X(0) + v * dt * std::cos(swa + phi);
        X(1) = X(1) + v * dt * std::sin(swa + phi);
        X(2) = pi2Pi(X(2) + v * dt * std::sin(swa) / wb);
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "predict" << std::endl;
    }
}

void EKF::singleUpdate(Eigen::VectorXf&       X,
                       Eigen::MatrixXf&       P,
                       const Eigen::MatrixXf& Z,
                       const Eigen::MatrixXf& R,
                       const Eigen::VectorXi& idf)
{
    try
    {
        int lenZ = Z.cols();
        for (int i = 0; i < lenZ; i++)
        {
            auto [Zp, H]      = observeModel(X, idf(i));
            Eigen::MatrixXf V = Eigen::MatrixXf::Zero(2, 1);
            V(0, 0)           = Z(0, i) - Zp(0, 0);
            V(1, 0)           = pi2Pi(Z(1, i) - Zp(1, 0));
            choleskyUpdate(X, P, V, R, H);
        }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "singleUpdate" << std::endl;
    }
}

void EKF::update(Eigen::VectorXf&       X,
                 Eigen::MatrixXf&       P,
                 const Eigen::MatrixXf& Z,
                 const Eigen::MatrixXf& R,
                 const Eigen::VectorXi& idf,
                 bool                   batch)
{
    if (batch)
    {
        batchUpdate(X, P, Z, R, idf);
    }
    else
    {
        singleUpdate(X, P, Z, R, idf);
    }
}
