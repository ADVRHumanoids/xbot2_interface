#include "common.h"

using TestKinematics = TestWithModel;


TEST_F(TestKinematics, checkJacobianNumerical)
{
    auto check_jac = [this](std::string lname, Eigen::VectorXd q0)
    {
        model->setJointPosition(q0);
        model->update();

        auto T = model->getPose(lname);

        const double h = 1e-4;

        Eigen::MatrixXd Jhat(6, model->getNv());

        for(int i = 0; i < model->getNv(); i++)
        {
            auto vi = Eigen::VectorXd::Unit(model->getNv(), i);
            Eigen::VectorXd qplus = model->sum(q0, vi*h/2);
            Eigen::VectorXd qminus = model->sum(q0, -vi*h/2);

            model->setJointPosition(qplus);
            model->update();
            auto Tplus = model->getPose(lname);

            model->setJointPosition(qminus);
            model->update();
            auto Tminus = model->getPose(lname);

            Eigen::Vector3d dp = (Tplus.translation() - Tminus.translation())/h;
            Eigen::Matrix3d dR = (Tplus.linear() - Tminus.linear())*T.linear().transpose();

            Jhat.col(i).head<3>() = dp;
            Jhat.col(i).tail<3>() << dR(2, 1)/h,
                                     dR(0, 2)/h,
                                     dR(1, 0)/h;

        }

        auto J = model->getJacobian(lname);

        EXPECT_LT((J - Jhat).lpNorm<Eigen::Infinity>(), 1e-3);
    };

    for(int i = 0; i < 30; i++)
    {
        Eigen::VectorXd q0 = model->getJointPosition();
        Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
        Eigen::VectorXd q = model->sum(q0, v);

        for(auto [lname, lptr] : model->getUrdf()->links_)
        {
            check_jac(lname, q);
        }
    }
}


TEST_F(TestKinematics, checkVelocityVsJacobian)
{
    auto check_vel_vs_jac = [this](std::string lname, Eigen::VectorXd q0)
    {
        model->setJointPosition(q0);
        model->update();

        Eigen::MatrixXd J = model->getJacobian(lname);

        for(int i = 0; i < model->getNv(); i++)
        {
            auto vi = Eigen::VectorXd::Random(model->getNv()).eval();
            model->setJointVelocity(vi);
            model->update();

            auto vel = model->getVelocityTwist(lname);

            EXPECT_LT((vel - J*vi).lpNorm<Eigen::Infinity>(), 1e-9);

        }

    };

    for(int i = 0; i < 30; i++)
    {
        Eigen::VectorXd q0 = model->getJointPosition();
        Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
        Eigen::VectorXd q = model->sum(q0, v);

        for(auto [lname, lptr] : model->getUrdf()->links_)
        {
            check_vel_vs_jac(lname, q);
        }
    }
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
