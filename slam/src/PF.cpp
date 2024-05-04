#include "PF.h"

PF::PF(const Eigen::MatrixXf& landMarks, const Eigen::MatrixXf& wayPoints)
    : Slam(landMarks, wayPoints)
{
    mTABLE = Eigen::VectorXi::Zero(getLandMarks().cols());
}

void PF::addOneNewFeature(PF::Particle_t& particle, const Eigen::MatrixXf& Z, const Eigen::MatrixXf& R)
{
    try
    {
        int lenMeas = Z.cols();
        if (lenMeas > 0)
        {
            Eigen::MatrixXf              XF = Eigen::MatrixXf::Zero(2, lenMeas);
            std::vector<Eigen::MatrixXf> PF = {};
            auto                         X  = particle.X;

            for (int i = 0; i < lenMeas; i++)
            {

                auto r = Z(0, i);
                auto b = Z(1, i);
                auto s = std::sin(X(2) + b);
                auto c = std::cos(X(2) + b);

                XF(0, i) = X(0) + (r * c);
                XF(1, i) = X(1) + (r * s);

                Eigen::MatrixXf Gz = Eigen::MatrixXf::Zero(2, 2);
                Gz(0, 0)           = c;
                Gz(0, 1)           = -r * s;
                Gz(1, 0)           = s;
                Gz(1, 1)           = r * c;

                Eigen::MatrixXf Temp = Gz * R * Gz.transpose();
                PF.push_back(Temp);
            }

            auto       lenState = particle.XF.cols();
            const auto tXF      = particle.XF;
            particle.XF.resize(0, 0);
            particle.XF                                     = Eigen::MatrixXf::Zero(XF.rows(), XF.cols() + tXF.cols());
            particle.XF.block(0, 0, tXF.rows(), tXF.cols()) = tXF.block(0, 0, tXF.rows(), tXF.cols());

            int indX = 0;
            for (int j = lenState; j < lenState + lenMeas; j++)
            {
                particle.XF.block(0, j, 2, 1) = XF.block(0, indX, 2, 1);
                particle.PF.push_back(PF[indX]);
                indX++;
            }
        }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "addOneNewFeature" << std::endl;
    }
}

Eigen::MatrixXf PF::computeDelta(const Eigen::MatrixXf& X1, const Eigen::MatrixXf& X2)
{
    // compute innovation between two state estimates, normalizing the heading components.
    Eigen::MatrixXf dx = X1 - X2;
    dx(2, 0)           = pi2Pi(dx(2, 0));
    return dx;
}

PF::Jacobians_t PF::computeJacobians(const Particle_t& particle, const Eigen::VectorXi& idf, const Eigen::MatrixXf& R)
{
    auto len = idf.rows();

    Eigen::MatrixXf ZP;
    ZP.resize(0, 0);

    std::vector<Eigen::MatrixXf> HV = {};
    std::vector<Eigen::MatrixXf> HF = {};
    std::vector<Eigen::MatrixXf> SF = {};

    auto X = particle.X;

    Eigen::MatrixXf XF = Eigen::MatrixXf::Zero(2, len);
    for (int i = 0; i < len; i++)
    {
        auto id              = idf(i) - 1;
        XF.block(0, i, 2, 1) = particle.XF.block(0, id, 2, 1);
    }

    std::vector<Eigen::MatrixXf> PF = {};
    for (int i = 0; i < len; i++)
    {
        auto id = idf(i) - 1;
        PF.push_back(particle.PF[id]);
    }

    ZP = Eigen::MatrixXf::Zero(2, len);
    for (int i = 0; i < len; i++)
    {
        auto dx = XF(0, i) - X(0, 0);
        auto dy = XF(1, i) - X(1, 0);
        auto d2 = std::pow(dx, 2.0F) + std::pow(dy, 2.0F);
        auto d  = std::sqrt(d2);

        // predicted observation
        ZP(0, i) = d;
        ZP(1, i) = pi2Pi(std::atan2(dy, dx) - X(2));

        // Jacobian wrt to vehicle states
        Eigen::MatrixXf THV;
        THV       = Eigen::MatrixXf::Zero(2, 3);
        THV(0, 0) = -dx / d;
        THV(0, 1) = -dy / d;
        THV(0, 2) = 0.0F;
        THV(1, 0) = dy / d2;
        THV(1, 1) = -dx / d2;
        THV(1, 2) = -1.0;
        HV.push_back(THV);

        // Jacobian wrt to feature states
        Eigen::MatrixXf THF;
        THF       = Eigen::MatrixXf::Zero(2, 2);
        THF(0, 0) = dx / d;
        THF(0, 1) = dy / d;
        THF(1, 0) = -dy / d2;
        THF(1, 1) = dx / d2;
        HF.push_back(THF);

        // innovation covariance of feature observation
        auto TSF = THF * PF[i] * THF.transpose() + R;
        SF.push_back(TSF);
    }

    return {ZP, HV, HF, SF};
}

PF::Association_t PF::dataAssociateTable(const Eigen::MatrixXf& Z,
                                         const Eigen::VectorXi& idz,
                                         Eigen::VectorXi&       table,
                                         int                    numParticles)
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
        std::vector<float> temp = {};
        for (int i = 1; i <= ZN.cols(); i++)
        {
            temp.push_back(numParticles + i);
        }
        for (int i = 0; i < idf.size(); i++)
        {
            int id        = idf[i];
            table(id - 1) = temp.at(i);
        }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "dataAssociateTable" << std::endl;
    }
    return {ZF, ZN, idfo};
}

void PF::featureUpdate(Particle_t&            particle,
                       const Eigen::MatrixXf& Z,
                       const Eigen::VectorXi& idf,
                       const Eigen::MatrixXf& R)
{
    auto            len = idf.rows();
    Eigen::MatrixXf XF  = Eigen::MatrixXf::Zero(2, len);
    for (int i = 0; i < len; i++)
    {
        auto id              = idf(i) - 1;
        XF.block(0, i, 2, 1) = particle.XF.block(0, id, 2, 1);
    }

    std::vector<Eigen::MatrixXf> PF = {};
    for (int i = 0; i < len; i++)
    {
        auto id = idf(i) - 1;
        PF.push_back(particle.PF[id]);
    }

    auto [ZP, HV, HF, SF] = computeJacobians(particle, idf, R);

    Eigen::MatrixXf V = Z - ZP;
    for (int j = 0; j < V.cols(); j++)
    {
        V(1, j) = pi2Pi(V(1, j));
    }

    for (int i = 0; i < len; i++)
    {
        Eigen::MatrixXf VI  = V.block(0, i, 2, 1);
        Eigen::MatrixXf HFT = HF[i];
        Eigen::MatrixXf PFT = PF[i];
        Eigen::VectorXf XFT = XF.block(0, i, 2, 1);

        choleskyUpdate(XFT, PFT, VI, R, HFT);
        XF.block(0, i, 2, 1) = XFT;
        PF[i]                = PFT;
    }

    int idxx = 0;
    for (int i = 0; i < len; i++)
    {
        int id                         = idf(i) - 1;
        particle.XF.block(0, id, 2, 1) = XF.block(0, idxx, 2, 1);
        idxx++;
    }

    idxx = 0;
    for (int i = 0; i < len; i++)
    {
        auto id         = idf(i) - 1;
        particle.PF[id] = PF[idxx];
        idxx++;
    }
}

float PF::gaussEvaluate(const Eigen::VectorXf& V, const Eigen::MatrixXf& S, bool logFlag)
{
    float w;
    auto  D = V.rows();

    // cholesky decomposition
    Eigen::MatrixXf SC = choleskyDecomposition(S);

    Eigen::MatrixXf TS = SC.transpose();
    SC                 = Eigen::MatrixXf::Zero(TS.rows(), TS.cols());
    SC                 = TS;

    // normalized innovation
    Eigen::MatrixXf nin = SC.inverse() * V;

    for (int i = 0; i < nin.rows(); i++)
    {
        nin(i, 0) = std::pow(nin(i, 0), 2.0F);
    }

    // Gaussian exponential term
    auto E = -0.5F * nin.sum();

    if (!logFlag)
    {
        // normalising term (make Gaussian hyper-volume equal 1)

        auto C = std::pow(2.0F * std::_Pi_val, D / 2.0F) * SC.diagonal().prod();
        w      = std::exp(E) / C; // likelihood
    }
    else
    {
        Eigen::MatrixXf ScT = SC.diagonal();

        auto C = 0.5F * D * std::log(2.0F * std::_Pi_val) + ScT.sum();
        w      = E - C; // log likelihood
    }
    return w;
}

std::vector<PF::Particle_t> PF::initializeParticles(int numParticles)
{
    std::vector<Particle_t> particles;
    try
    {
        for (int index = 0; index < numParticles; index++)
        {
            Particle_t p = {};
            p.w          = 1.0F / static_cast<float>(numParticles);
            p.X          = Eigen::VectorXf::Zero(3);
            p.P          = Eigen::MatrixXf::Zero(3, 3);
            p.XF.resize(0, 0);
            p.PF = {};

            particles.push_back(p);
        }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "PF: initializeParticles" << std::endl;
    }
    return particles;
}

float PF::likelihood(const Particle_t&      particle,
                     const Eigen::MatrixXf& Z,
                     const Eigen::VectorXi& idf,
                     const Eigen::MatrixXf& R)
{
    float w = 1.0F;
    for (int i = 0; i < idf.rows(); i++)
    {
        Eigen::VectorXi tidf  = Eigen::VectorXi::Zero(1);
        tidf(0)               = idf(i);
        auto [ZP, HV, HF, SF] = computeJacobians(particle, tidf, R);
        Eigen::MatrixXf V     = Z.block(0, i, 2, 1) - ZP;
        V(1, 0)               = pi2Pi(V(1, 0));
        w                     = w * gaussEvaluate(V, SF[0]);
    }
    return w;
}

Eigen::MatrixXf PF::multivariateGauss(const Eigen::VectorXf& X, const Eigen::MatrixXf& P, int n)
{
    auto len = X.rows();

    // cholesky decomposition
    Eigen::MatrixXf S  = choleskyDecomposition(P);
    Eigen::MatrixXf TS = S.transpose();
    S                  = Eigen::MatrixXf::Zero(TS.rows(), TS.cols());
    S                  = TS;

    Eigen::MatrixXf XT = Eigen::MatrixXf::Zero(len, n);
    for (int i = 0; i < len; i++)
    {
        for (int j = 0; j < n; j++)
        {
            XT(i, j) = generateRandomNumber<float>();
        }
    }
    return (S * XT + X * Eigen::MatrixXf::Zero(1, 3));
}

void PF::observeHeading(Particle_t& particle, const float& phi, bool useHeading)
{
    try
    {
        if (!useHeading)
        {
            return;
        }
        // heading uncertainty - radians
        float sigmaPhi = 0.01F * std::_Pi_val / 180.0F; // radians, heading uncertainty

        auto X = particle.X;
        auto P = particle.P;

        Eigen::MatrixXf H = Eigen::MatrixXf::Zero(1, std::max(X.rows(), X.cols()));
        H(0, 2)           = 1.0F;

        Eigen::MatrixXf V = Eigen::MatrixXf::Zero(1, 1);
        V(0, 0)           = pi2Pi(phi - X(2, 0));

        Eigen::MatrixXf R = Eigen::MatrixXf::Zero(1, 1);
        R(0, 0)           = std::pow(sigmaPhi, 2.0F);
        josephUpdate(X, P, V, R, H);

        particle.X.resize(0);
        particle.P.resize(0, 0);
        particle.X = Eigen::VectorXf::Zero(X.rows());
        particle.P = Eigen::MatrixXf::Zero(P.rows(), P.cols());
        particle.X = X;
        particle.P = P;
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "PF: observeHeading" << std::endl;
    }
}

void PF::predict(PF::Particle_t&        particle,
                 const float&           v,
                 const float&           swa,
                 const Eigen::MatrixXf& Q,
                 const float&           wb,
                 const float&           dt)
{
    try
    {
        auto  X   = particle.X;
        auto  P   = particle.P;
        float phi = X(2, 0);

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

        // predicted state
        X(0) = X(0) + v * dt * std::cos(swa + phi);
        X(1) = X(1) + v * dt * std::sin(swa + phi);
        X(2) = pi2Pi(X(2) + v * dt * std::sin(swa) / wb);

        particle.X.resize(0);
        particle.P.resize(0, 0);
        particle.X = Eigen::VectorXf::Zero(X.rows());
        particle.P = Eigen::MatrixXf::Zero(P.rows(), P.cols());
        particle.X = X;
        particle.P = P;
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << "\t" << "PF: predict" << std::endl;
    }
}

void PF::resampleParticles(std::vector<PF::Particle_t>& particles, int numEffective, bool resampleStatus)
{
    std::vector<PF::Particle_t> iP = {};
    auto                        n  = particles.size();
    Eigen::MatrixXf             W  = Eigen::MatrixXf::Zero(1, n);
    for (int i = 0; i < n; i++)
    {
        W(0, i) = particles[i].w;
    }
    auto ws = W.sum();
    W       = W / ws;
    for (int i = 0; i < n; i++)
    {
        particles[i].w = particles[i].w / ws;
    }

    auto [keep, neff] = stratifiedResample(W);
    if (neff < numEffective && resampleStatus)
    {
        for (int i = 0; i < n; i++)
        {
            auto id             = keep(0, i);
            particles[id - 1].w = 1.0F / n;
            iP.push_back(particles[id - 1]);
        }
        particles = iP;
    }
}

void PF::sampleProposal(Particle_t&            particle,
                        const Eigen::MatrixXf& Z,
                        const Eigen::VectorXi& idf,
                        const Eigen::MatrixXf& R)
{
    auto X = particle.X;
    auto P = particle.P;

    auto X0 = X;
    auto P0 = P;

    for (int index = 0; index < idf.rows(); index++)
    {
        Eigen::VectorXi TIDF     = Eigen::VectorXi::Zero(1);
        TIDF(0)                  = idf(index);
        auto [ZPI, HVI, HFI, SF] = computeJacobians(particle, TIDF, R); // note TIDF is a single value
        Eigen::MatrixXf SFI      = SF[0].inverse();
        Eigen::MatrixXf VI       = Z.block(0, index, 2, 1) - ZPI;
        VI(1, 0)                 = pi2Pi(VI(1, 0));

        // proposal covariance
        auto PT = HVI[0].transpose() * SFI * HVI[0] + P.inverse();
        P       = PT.inverse();

        // proposal mean
        X = X + P * HVI[0].transpose() * SFI * VI;

        particle.X = X;
        particle.P = P;
    }

    // sample from proposal distribution
    auto XS = multivariateNormalGaussianDistribution(X, P, 1);

    particle.X = XS;
    particle.P = Eigen::MatrixXf::Zero(3, 3);

    // compute sample weight: w = w * p(z|xk) p(xk|xk-1) / proposal
    auto like  = likelihood(particle, Z, idf, R);
    auto prior = gaussEvaluate(computeDelta(X0, XS), P0);
    auto prop  = gaussEvaluate(computeDelta(X, XS), P);
    particle.w = particle.w * like * prior / prop;
}

PF::Stratified_t PF::stratifiedResample(Eigen::MatrixXf& W)
{
    W                  = W / W.sum();
    Eigen::MatrixXf WT = Eigen::MatrixXf::Zero(W.rows(), W.cols());
    for (int i = 0; i < W.cols(); i++)
    {
        WT(0, i) = std::pow(W(0, i), 2.0F);
    }
    auto            neff   = 1.0F / WT.sum();
    auto            len    = W.cols();
    Eigen::MatrixXf Keep   = Eigen::MatrixXf::Zero(1, len);
    auto            select = stratifiedRandom(len);

    float summer = W(0, 0);
    for (int i = 1; i < len; i++)
    {
        summer  = summer + W(0, i);
        W(0, i) = summer;
    }

    int ctr = 1;
    for (int i = 0; i < len; i++)
    {
        while (ctr <= len && select(0, i) < W(i))
        {
            Keep(0, ctr - 1) = i;
            ctr++;
        }
    }

    return {Keep, neff};
}

Eigen::MatrixXf PF::stratifiedRandom(int n)
{
    float           k  = 1.0F / n;
    Eigen::MatrixXf DI = Eigen::MatrixXf::Zero(1, n);
    DI(0, 0)           = k / 2.0F;
    for (int i = 1; i < n; i++)
    {
        DI(0, i) = DI(0, i - 1) + k;
    }

    Eigen::MatrixXf Rn = Eigen::MatrixXf::Zero(1, n);
    for (int j = 0; j < n; j++)
    {
        Rn(0, j) = generateRandomNumber<float>() * (k - k / 2.0F);
    }

    return DI + Rn;
}
