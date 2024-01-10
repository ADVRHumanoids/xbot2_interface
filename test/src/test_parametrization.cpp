#include "common.h"

using TestParametrization = TestWithModel;


TEST_F(TestParametrization, checkSumDiff)
{
    Eigen::VectorXd q0 = model->getJointPosition();
    Eigen::VectorXd vrand1 = Eigen::VectorXd::Random(model->getNv());
    Eigen::VectorXd vrand2 = Eigen::VectorXd::Random(model->getNv());

    Eigen::VectorXd q1; model->sum(q0, vrand1, q1);
    Eigen::VectorXd q2; model->sum(q0, vrand2, q2);
    Eigen::VectorXd vzero = Eigen::VectorXd::Zero(model->getNv());

    Eigen::VectorXd qdiff1; model->difference(q1, q1, qdiff1);
    Eigen::VectorXd qdiff2; model->difference(q2, q2, qdiff2);

    EXPECT_TRUE(qdiff1.norm() < 1e-12);
    EXPECT_TRUE(qdiff2.norm() < 1e-12);
    EXPECT_TRUE(model->sum(q1, vzero).isApprox(q1));
    EXPECT_TRUE(model->sum(q2, vzero).isApprox(q2));

    auto vdiff1 = model->difference(q2, q1).eval();
    auto vdiff2 = model->difference(q1, q2).eval();

    EXPECT_EQ(vdiff1.size(), model->getNv());
    EXPECT_TRUE(model->sum(q1, vdiff1).isApprox(q2));

    EXPECT_EQ(vdiff2.size(), model->getNv());
    EXPECT_TRUE(model->sum(q2, vdiff2).isApprox(q1));

}


TEST_F(TestParametrization, checkJointFk)
{
    for(int iter = 0; iter < 1000; iter++)
    {
        auto q = model->generateRandomQ();
        auto v = model->getJointVelocity();
        model->setJointPosition(q);
        model->update();

        for(int id = 0; id < model->getJointNum(); id++)
        {
            auto j = model->getJoint(id);
            auto jinfo = j->getJointInfo();

            Eigen::Affine3d Tc;
            Eigen::Vector6d vc;

            j->forwardKinematics(q.segment(jinfo.iq, jinfo.nq),
                                 v.segment(jinfo.iv, jinfo.nv),
                                 Tc, vc);

            Eigen::Affine3d Tok = model->getPose(j->getChildLink(), j->getParentLink());

            EXPECT_TRUE(Tok.isApprox(Tc, 0.001)) <<
                j->getName() << "\nTc = " << Tc.matrix().format(2) << "\nTok = " << Tok.matrix().format(2) << std::endl;
        }
    }
}


TEST_F(TestParametrization, checkJointFkIk)
{
    for(int iter = 0; iter < 1000; iter++)
    {
        for(int id = 0; id < model->getJointNum(); id++)
        {
            auto j = model->getJoint(id);

            if(j->getType() != urdf::Joint::FLOATING)
            {
                continue;
            }

            Eigen::Affine3d p_Tc;
            p_Tc.linear() = Eigen::Quaterniond(Eigen::Vector4d::Random().normalized()).toRotationMatrix();
            p_Tc.translation().setRandom();
            Eigen::Vector6d c_vc;
            c_vc.setRandom();

            Eigen::VectorXd qj, vj;
            j->inverseKinematics(p_Tc, c_vc, qj, vj);

            Eigen::Affine3d p_Tc_exp;
            Eigen::Vector6d c_vc_exp;
            j->forwardKinematics(qj, vj, p_Tc_exp, c_vc_exp);

            EXPECT_TRUE(p_Tc.isApprox(p_Tc_exp));

            EXPECT_TRUE(c_vc.isApprox(c_vc_exp));


        }
    }
}

TEST_F(TestParametrization, checkJointMinimalParam)
{
    for(int iter = 0; iter < 1000; iter++)
    {
        Eigen::VectorXd q0 = model->getJointPosition();
        Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
        Eigen::VectorXd q = model->sum(q0, v);
        model->setJointPosition(q);
        model->update();

        for(int id = 0; id < model->getJointNum(); id++)
        {
            auto j = model->getJoint(id);

            auto qj = q.segment(model->getJointInfo(id).iq,
                                j->getNq());

            Eigen::VectorXd qmin;
            j->positionToMinimal(qj, qmin);

            Eigen::VectorXd qj_exp;
            j->minimalToPosition(qmin, qj_exp);

            ASSERT_EQ(qj_exp.size(), j->getNq());

            // fk should not change
            auto T1 = model->getPose(j->getChildLink(), j->getParentLink());
            j->setJointPosition(qj_exp);
            model->update();
            auto T2 = model->getPose(j->getChildLink(), j->getParentLink());


            EXPECT_TRUE(T1.isApprox(T2, 1e-3)) <<
                "err   = " << (qj - qj_exp).lpNorm<Eigen::Infinity>() << "\n" <<
                "qj    = " << qj.transpose().format(2) << "\n" <<
                "qjexp = " << qj_exp.transpose().format(2) << "\n" <<
                "T1    =\n" << T1.matrix().format(2) << "\n" <<
                "T2    =\n" << T2.matrix().format(2) << "\n";



            // wrong size should throw
            Eigen::VectorXd qwrong(10);
            EXPECT_THROW(j->positionToMinimal(qwrong, qmin), std::out_of_range);
            EXPECT_THROW(j->minimalToPosition(qwrong, qj), std::out_of_range);
        }
    }
}

TEST_F(TestParametrization, checkRobotMinimalParam)
{
    double dtset = 0, dtget = 0;
    int count = 1000;

    auto m2 = model->clone();

    for(int i = 0; i < count; i++)
    {

        model->setJointPosition(model->generateRandomQ());
        model->update();

        TIC(get);
        auto qmin = model->getJointPositionMinimal();
        dtget += TOC(get);

        ASSERT_EQ(qmin.size(), model->getNv());

        TIC(set);
        m2->setJointPositionMinimal(qmin);
        dtset += TOC(set);
        m2->update();

        checkConfigurationIsEqual(*model, *m2);

    }

    std::cout << "getJointPositionMinimal requires " << dtget/count*1e6 << " us \n";
    std::cout << "setJointPositionMinimal requires " << dtset/count*1e6 << " us \n";

}

TEST_F(TestParametrization, checkReducedModel)
{
    auto qfix = model->generateRandomQ();

    std::vector<std::string> j_to_fix = {"j_arm1_4", "hip_pitch_3", "j_wheel_2"};

    auto redmodel = model->generateReducedModel(
        qfix, j_to_fix);

    EXPECT_EQ(model->getJointNum(), redmodel->getJointNum() + j_to_fix.size());

    for(auto j : j_to_fix)
    {
        EXPECT_TRUE(model->hasJoint(j));
        EXPECT_FALSE(redmodel->hasJoint(j));
        EXPECT_FALSE(bool(redmodel->getJoint(j)));
    }

    EXPECT_NEAR(model->getMass(), redmodel->getMass(), 1e-3);

    auto redq = redmodel->generateRandomQ();
    Eigen::VectorXd redv = Eigen::VectorXd::Random(redmodel->getNv());
    redmodel->setJointPosition(redq);
    redmodel->setJointVelocity(redv);
    redmodel->update();

    for(auto jname : model->getJointNames())
    {
        if(!redmodel->hasJoint(jname))
        {
            int iq = model->getJointInfo(jname).iq;
            int nq = model->getJointInfo(jname).nq;
            model->getJoint(jname)->setJointPosition(qfix.segment(iq, nq));
            continue;
        }

        model->getJoint(jname)->setJointPosition(
            redmodel->getJoint(jname)->getJointPosition()
            );

        model->getJoint(jname)->setJointVelocity(
            redmodel->getJoint(jname)->getJointVelocity()
            );
    }

    model->update();

    for(auto [lname, lptr] : model->getUrdf()->links_)
    {
        auto T1 = model->getPose(lname);
        auto T2 = redmodel->getPose(lname);
        EXPECT_TRUE(T1.isApprox(T2, 1e-3)) <<
            lname << "\nTc = " << T1.matrix().format(2) << "\nTok = " << T2.matrix().format(2) << std::endl;

    }
}


int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
