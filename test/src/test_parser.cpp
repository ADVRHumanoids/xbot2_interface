#include "common.h"

using TestParser = TestCommon;

TEST_F(TestParser, testUrdfOnly)
{
    auto model = XBot::ModelInterface::getModel(urdf,
                                                 nullptr,
                                                 model_type);

    ASSERT_TRUE(bool(model));

}

TEST_F(TestParser, testUrdfSrdf)
{
    auto model = XBot::ModelInterface::getModel(urdf,
                                                 srdf,
                                                 model_type);

    ASSERT_TRUE(bool(model));

}

TEST_F(TestParser, checkSizes)
{
    auto model = XBot::ModelInterface::getModel(urdf,
                                                 srdf,
                                                 model_type);

    ASSERT_EQ(model->getJointPosition().size(), model->getNq());
    ASSERT_EQ(model->getJointVelocity().size(), model->getNv());
    ASSERT_EQ(model->getJointAcceleration().size(), model->getNv());
    ASSERT_EQ(model->getJointEffort().size(), model->getNv());
}

TEST_F(TestParser, checkModelGetSet)
{
    auto model = XBot::ModelInterface::getModel(urdf,
                                                 srdf,
                                                 model_type);

    Eigen::VectorXd q, v, a, tau;
    q.setRandom(model->getNq());
    v.setRandom(model->getNv());
    a.setRandom(model->getNv());
    tau.setRandom(model->getNv());

    model->setJointPosition(q);
    model->setJointVelocity(v);
    model->setJointAcceleration(a);
    model->setJointEffort(tau);

    EXPECT_TRUE(model->getJointPosition().isApprox(q));
    EXPECT_TRUE(model->getJointVelocity().isApprox(v));
    EXPECT_TRUE(model->getJointAcceleration().isApprox(a));
    EXPECT_TRUE(model->getJointEffort().isApprox(tau));
}

TEST_F(TestParser, checkJointSize)
{
    auto model = XBot::ModelInterface::getModel(urdf,
                                                 srdf,
                                                 model_type);
    int nj = 0;

    for(auto [jname, jptr] : urdf->joints_)
    {
        if(jptr->type == urdf::Joint::FIXED)
        {
            EXPECT_FALSE(model->hasJoint(jname));
            EXPECT_EQ(model->getJointId(jname), -1);
            continue;
        }

        ++nj;

        EXPECT_TRUE(model->hasJoint(jname));

        auto j = model->getJoint(jname);

        EXPECT_EQ(jptr->type, j->getType());

        int jid = model->getJointId(jname);

        EXPECT_GE(jid, 0);

        EXPECT_EQ(j, model->getJoint(jid));

        EXPECT_EQ(j->getName(), jname);

        EXPECT_GT(j->getNq(), 0);
        EXPECT_GT(j->getNv(), 0);

        EXPECT_EQ(j->getJointPosition().size(), j->getNq());
        EXPECT_EQ(j->getJointVelocity().size(), j->getNv());
        EXPECT_EQ(j->getJointAcceleration().size(), j->getNv());
        EXPECT_EQ(j->getJointEffort().size(), j->getNv());

        auto jinfo = model->getJointInfo(jname);

        EXPECT_EQ(jinfo.id, jid);

        EXPECT_LE(jinfo.iq + jinfo.nq, model->getNq());

        EXPECT_LE(jinfo.iv + jinfo.nv, model->getNv());

    }

    EXPECT_EQ(nj, model->getJointNum());
}

TEST_F(TestParser, checkJointsGetSet)
{
    auto model = XBot::ModelInterface::getModel(urdf,
                                                 srdf,
                                                 model_type);

    Eigen::VectorXd q, v, a, tau;
    q.setRandom(model->getNq());
    v.setRandom(model->getNv());
    a.setRandom(model->getNv());
    tau.setRandom(model->getNv());

    model->setJointPosition(q);
    model->setJointVelocity(v);
    model->setJointAcceleration(a);
    model->setJointEffort(tau);

    for(auto [jname, jptr] : urdf->joints_)
    {
        if(jptr->type == urdf::Joint::FIXED)
        {
            continue;
        }

        EXPECT_TRUE(model->hasJoint(jname));

        auto j = model->getJoint(jname);

        auto jinfo = model->getJointInfo(jname);

        auto qj = j->getJointPosition();
        EXPECT_TRUE(qj.isApprox(q.segment(jinfo.iq, jinfo.nq)));
        Eigen::VectorXd qjrand = Eigen::VectorXd::Random(j->getNq());
        j->setJointPosition(qjrand);
        EXPECT_TRUE(qjrand.isApprox(model->getJointPosition().segment(jinfo.iq, jinfo.nq)));

        auto vj = j->getJointVelocity();
        EXPECT_TRUE(vj.isApprox(v.segment(jinfo.iv, jinfo.nv)));
        Eigen::VectorXd vjrand = Eigen::VectorXd::Random(j->getNv());
        j->setJointVelocity(vjrand);
        EXPECT_TRUE(vjrand.isApprox(model->getJointVelocity().segment(jinfo.iv, jinfo.nv)));

        auto aj = j->getJointAcceleration();
        EXPECT_TRUE(aj.isApprox(a.segment(jinfo.iv, jinfo.nv)));
        Eigen::VectorXd ajrand = Eigen::VectorXd::Random(j->getNv());
        j->setJointAcceleration(ajrand);
        EXPECT_TRUE(ajrand.isApprox(model->getJointAcceleration().segment(jinfo.iv, jinfo.nv)));

        auto tj = j->getJointEffort();
        EXPECT_TRUE(tj.isApprox(tau.segment(jinfo.iv, jinfo.nv)));
        Eigen::VectorXd tjrand = Eigen::VectorXd::Random(j->getNv());
        j->setJointEffort(tjrand);
        EXPECT_TRUE(tjrand.isApprox(model->getJointEffort().segment(jinfo.iv, jinfo.nv)));

        // wrong size should throw
        Eigen::VectorXd xwrong = Eigen::VectorXd::Zero(j->getNq() + j->getNv());
        EXPECT_THROW(j->setJointPosition(xwrong), std::out_of_range);
        EXPECT_THROW(j->setJointVelocity(xwrong), std::out_of_range);
        EXPECT_THROW(j->setJointAcceleration(xwrong), std::out_of_range);
        EXPECT_THROW(j->setJointEffort(xwrong), std::out_of_range);

    }
}


TEST_F(TestParser, checkJointLimits)
{
    auto model = XBot::ModelInterface::getModel(urdf,
                                                srdf,
                                                model_type);

    auto [qmin, qmax] = model->getJointLimits();
    auto vmax = model->getVelocityLimits();
    auto taumax = model->getEffortLimits();

    for(auto [jname, jptr] : urdf->joints_)
    {
        if(jptr->type == urdf::Joint::FIXED)
        {
            EXPECT_FALSE(model->hasJoint(jname));
            EXPECT_EQ(model->getJointId(jname), -1);
            continue;
        }

        if(!jptr->limits)
        {
            continue;
        }

        auto xbj = model->getJoint(jname);

        auto jinfo = xbj->getJointInfo();

        if(jptr->type == urdf::Joint::REVOLUTE ||
            jptr->type == urdf::Joint::PRISMATIC)
        {
            EXPECT_EQ(jptr->limits->lower, xbj->getJointLimits().first[0]);
            EXPECT_EQ(jptr->limits->upper, xbj->getJointLimits().second[0]);
            EXPECT_EQ(jptr->limits->velocity, xbj->getVelocityLimits()[0]);
            EXPECT_EQ(jptr->limits->effort, xbj->getEffortLimits()[0]);

            EXPECT_EQ(jptr->limits->lower, qmin[jinfo.iv]);
            EXPECT_EQ(jptr->limits->upper, qmax[jinfo.iv]);
            EXPECT_EQ(jptr->limits->velocity, vmax[jinfo.iv]);
            EXPECT_EQ(jptr->limits->effort, taumax[jinfo.iv]);
        }

        if(jptr->type == urdf::Joint::CONTINUOUS)
        {
            EXPECT_EQ(-M_PI, xbj->getJointLimits().first[0]);
            EXPECT_EQ(M_PI, xbj->getJointLimits().second[0]);
            EXPECT_EQ(jptr->limits->velocity, xbj->getVelocityLimits()[0]);
            EXPECT_EQ(jptr->limits->effort, xbj->getEffortLimits()[0]);

            EXPECT_EQ(-M_PI, qmin[jinfo.iv]);
            EXPECT_EQ(M_PI, qmax[jinfo.iv]);
            EXPECT_EQ(jptr->limits->velocity, vmax[jinfo.iv]);
            EXPECT_EQ(jptr->limits->effort, taumax[jinfo.iv]);
        }

        if(jptr->type == urdf::Joint::FLOATING)
        {
            EXPECT_TRUE((xbj->getJointLimits().first.head<3>().array() < 1e6).all());
            EXPECT_TRUE((xbj->getJointLimits().second.head<3>().array() > 1e6).all());
            EXPECT_TRUE((xbj->getJointLimits().first.tail<3>().array() == -M_PI).all());
            EXPECT_TRUE((xbj->getJointLimits().second.tail<3>().array() == M_PI).all());
        }

    }
}

TEST_F(TestParser, checkJointLimits2)
{

    auto model = XBot::ModelInterface::getModel(urdf,
                                                srdf,
                                                model_type);

    auto [qmin, qmax] = model->getJointLimits();
    auto vmax = model->getVelocityLimits();
    auto taumax = model->getEffortLimits();

    for(int i = 0; i < 100000; i++)
    {
        auto q = model->generateRandomQ();
        bool ok = model->checkJointLimits(q);
        EXPECT_TRUE(ok);

        if(!ok)
        {
            auto dq = model->difference(q, model->getNeutralQ());

            std::cout << "q   : " << q.transpose().format(2) << "\n";
            std::cout << "qmin: " << qmin.transpose().format(2) << "\n";
            std::cout << "dq  : " << dq.transpose().format(2) << "\n";
            std::cout << "qmax: " << qmax.transpose().format(2) << "\n";
            std::cout << "qerr: " << (dq-qmin).cwiseMin(0).transpose().format(2) << "\n";
            std::cout << "qerr: " << (dq-qmax).cwiseMax(0).transpose().format(2) << "\n";
        }
    }


    for(int i = 0; i < 10000; i++)
    {
        auto q = model->generateRandomQ();
        model->setJointPosition(q);

        int ji = rand() % model->getJointNum();

        auto j = model->getJoint(ji);

    }
}

TEST_F(TestParser, checkMapConversions)
{
    auto model = XBot::ModelInterface::getModel(urdf,
                                                srdf,
                                                model_type);

    auto qrand = model->generateRandomQ();

    XBot::JointNameMap qmap, qmap1;

    model->qToMap(qrand, qmap);

    EXPECT_EQ(qmap.size(), model->getNq());

    model->setJointPosition(qmap);
    Eigen::VectorXd qout = model->getJointPosition();
    EXPECT_TRUE(qrand.cwiseEqual(qout).all());
    model->getJointPosition(qmap1);
    EXPECT_EQ(qmap1, qmap);

    Eigen::VectorXd vrand;
    vrand.setRandom(model->getNv());

    XBot::JointNameMap vmap, vmap1;
    model->vToMap(vrand, vmap);

    model->setJointVelocity(vmap);
    Eigen::VectorXd vout = model->getJointVelocity();
    EXPECT_TRUE(vrand.cwiseEqual(vout).all());
    model->getJointVelocity(vmap1);
    EXPECT_EQ(vmap1, vmap);
    vmap1.clear();

    model->setJointAcceleration(vmap);
    vout = model->getJointAcceleration();
    EXPECT_TRUE(vrand.cwiseEqual(vout).all());
    model->getJointAcceleration(vmap1);
    EXPECT_EQ(vmap1, vmap);
    vmap1.clear();

    model->setJointEffort(vmap);
    vout = model->getJointEffort();
    EXPECT_TRUE(vrand.cwiseEqual(vout).all());
    model->getJointEffort(vmap1);
    EXPECT_EQ(vmap1, vmap);
    vmap1.clear();

}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
