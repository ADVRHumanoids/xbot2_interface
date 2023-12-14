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
    Eigen::VectorXd q0 = model->getJointPosition();
    Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
    Eigen::VectorXd q = model->sum(q0, v);

    for(int iter = 0; iter < 1000; iter++)
    {
        for(int id = 0; id < model->getJointNum(); id++)
        {
            auto j = model->getJoint(id);

            auto qj = q.segment(model->getJointInfo(id).iq,
                                j->getNq());
            Eigen::VectorXd qmin;
            j->positionToMinimal(qj, qmin);

            Eigen::VectorXd qj_exp;
            j->minimalToPosition(qmin, qj_exp);

            EXPECT_TRUE(qj.isApprox(qj_exp));

            // wrong size should throw
            Eigen::VectorXd qwrong(10);
            EXPECT_THROW(j->positionToMinimal(qwrong, qmin), std::out_of_range);
            EXPECT_THROW(j->minimalToPosition(qwrong, qj), std::out_of_range);
        }
    }
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
