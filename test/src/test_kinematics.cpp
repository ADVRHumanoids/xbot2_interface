#include "common.h"

using TestKinematics = TestWithModel;

TEST_F(TestKinematics, checkJointFk)
{
    double dt = 0;
    int count = 0;

    for(int iter = 0; iter < 100; iter++)
    {
        Eigen::VectorXd q0 = model->getJointPosition();
        Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
        Eigen::VectorXd q;
        model->sum(q0, v, q);

        model->setJointPosition(q);
        model->setJointVelocity(v);
        model->update();

        for(int i = 0; i < model->getJointNum(); i++)
        {
            auto j = model->getJoint(i);

            if(j->getType() != urdf::Joint::FLOATING)
            {
                continue;
            }

            auto qj = q.segment(model->getJointInfo(i).iq,
                                j->getNq());

            auto vj = v.segment(model->getJointInfo(i).iv,
                                j->getNv());


            Eigen::Affine3d T_j;
            Eigen::Vector6d v_fk_j;
            TIC();
            j->forwardKinematics(qj, vj, T_j, v_fk_j);
            dt += TOC();
            count++;

            auto T = model->getPose(j->getParentLink()).inverse()*
                    model->getPose(j->getChildLink());

            auto v_fk = model->getRelativeVelocityTwist(j->getChildLink(), j->getParentLink());
            XBot::Utils::rotate(v_fk, T.linear().transpose());

            EXPECT_TRUE( T.isApprox(T_j) )
                    << j->getName() << "\n" <<
                       "T  = \n" << T.matrix().format(3) << "\n" <<
                       "Tj = \n" << T_j.matrix().format(3) << "\n";

            EXPECT_TRUE( v_fk.isApprox(v_fk_j) )
                    << j->getName() << "\n" <<
                       "v_fk  = \n" << v_fk.transpose().format(3) << "\n" <<
                       "v_fkj = \n" << v_fk_j.transpose().format(3) << "\n";
        }
    }

    std::cout << "forwardKinematics requires " << dt/count*1e6 << " us \n";
}

TEST_F(TestKinematics, checkJacobianNumerical)
{
    int count = 0;
    double dt = 0;

    int count_upd = 0;
    double dt_upd = 0;

    int count_pose = 0;
    double dt_pose = 0;

    auto check_jac = [&](std::string lname, Eigen::VectorXd q0)
    {
        model->setJointPosition(q0);
        TIC(upd);
        model->update();
        dt_upd += TOC(upd);
        count_upd++;

        TIC(pose);
        auto T = model->getPose(lname);
        dt_pose += TOC(pose);
        count_pose++;

        const double h = 1e-4;

        Eigen::MatrixXd Jhat(6, model->getNv());

        for(int i = 0; i < model->getNv(); i++)
        {
            auto vi = Eigen::VectorXd::Unit(model->getNv(), i);
            Eigen::VectorXd qplus;
            model->sum(q0, vi*h/2, qplus);
            Eigen::VectorXd qminus;
            model->sum(q0, -vi*h/2, qminus);

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

        TIC();
        Eigen::MatrixXd J;
        ASSERT_TRUE(model->getJacobian(lname, J));
        dt += TOC();
        count++;

        EXPECT_LT((J - Jhat).lpNorm<Eigen::Infinity>(), 1e-3);
    };

    for(int i = 0; i < 30; i++)
    {
        Eigen::VectorXd q0 = model->getJointPosition();
        Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
        Eigen::VectorXd q;
        model->sum(q0, v, q);

        for(auto [lname, lptr] : model->getUrdf()->links_)
        {
            check_jac(lname, q);
        }
    }

    std::cout << "update requires " << dt_upd/count*1e6 << " us \n";
    std::cout << "getPose requires " << dt_pose/count*1e6 << " us \n";
    std::cout << "getJacobian requires " << dt/count*1e6 << " us \n";
}


TEST_F(TestKinematics, checkVelocityVsJacobian)
{
    int count = 0;
    double dt = 0;

    auto check_vel_vs_jac = [this, &count, &dt](std::string lname, Eigen::VectorXd q0)
    {
        model->setJointPosition(q0);
        model->update();

        Eigen::MatrixXd J;
        ASSERT_TRUE(model->getJacobian(lname, J));

        for(int i = 0; i < model->getNv(); i++)
        {
            auto vi = Eigen::VectorXd::Random(model->getNv()).eval();
            model->setJointVelocity(vi);
            model->update();

            TIC();
            auto vel = model->getVelocityTwist(lname);
            dt += TOC();
            count++;

            EXPECT_LT((vel - J*vi).lpNorm<Eigen::Infinity>(), 1e-9);

        }

    };

    for(int i = 0; i < 30; i++)
    {
        Eigen::VectorXd q0 = model->getJointPosition();
        Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
        Eigen::VectorXd q;
        model->sum(q0, v, q);

        for(auto [lname, lptr] : model->getUrdf()->links_)
        {
            check_vel_vs_jac(lname, q);
        }
    }

    std::cout << "getVelocityTwist requires " << dt/count*1e6 << " us \n";
}


TEST_F(TestKinematics, checkJdotTimesV)
{
    int count = 0;
    double dt = 0;

    auto check_jdot_times_v = [this, &count, &dt](std::string lname, Eigen::VectorXd q0, Eigen::VectorXd v)
    {
        model->setJointPosition(q0);
        model->setJointVelocity(v);
        model->setJointAcceleration(v*0);
        model->update();

        TIC();
        auto acc = model->getJdotTimesV(lname);
        dt += TOC();
        count += 1;

        const double h = 1e-4;

        Eigen::VectorXd qplus; model->sum(q0, v*h/2, qplus);
        Eigen::VectorXd qminus; model->sum(q0, -v*h/2, qminus);

        model->setJointPosition(qplus);
        model->update();
        auto vplus = model->getVelocityTwist(lname);

        model->setJointPosition(qminus);
        model->update();
        auto vminus = model->getVelocityTwist(lname);

        Eigen::Vector6d acc_hat = (vplus - vminus)/h;

        EXPECT_LT((acc_hat - acc).lpNorm<Eigen::Infinity>(), 1e-3);


    };

    for(int i = 0; i < 100; i++)
    {
        Eigen::VectorXd q0 = model->getJointPosition();
        Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
        Eigen::VectorXd q; model->sum(q0, v, q);

        for(auto [lname, lptr] : model->getUrdf()->links_)
        {
            check_jdot_times_v(lname, q, v);
        }
    }

    std::cout << "getJdotTimesV requires " << dt/count*1e6 << " us \n";
}

TEST_F(TestKinematics, checkRelativeJacobian)
{
    int count = 0;
    double dt = 0;

    auto check_jac = [&](std::string lname, std::string bname, Eigen::VectorXd q0)
    {
        model->setJointPosition(q0);
        model->update();

        auto T = model->getPose(bname).inverse() * model->getPose(lname);

        const double h = 1e-4;

        Eigen::MatrixXd Jhat(6, model->getNv());

        for(int i = 0; i < model->getNv(); i++)
        {
            auto vi = Eigen::VectorXd::Unit(model->getNv(), i);
            Eigen::VectorXd qplus; model->sum(q0, vi*h/2, qplus);
            Eigen::VectorXd qminus; model->sum(q0, -vi*h/2, qminus);

            model->setJointPosition(qplus);
            model->update();
            auto Tplus = model->getPose(bname).inverse() * model->getPose(lname);

            model->setJointPosition(qminus);
            model->update();
            auto Tminus = model->getPose(bname).inverse() * model->getPose(lname);

            Eigen::Vector3d dp = (Tplus.translation() - Tminus.translation())/h;
            Eigen::Matrix3d dR = (Tplus.linear() - Tminus.linear())*T.linear().transpose();

            Jhat.col(i).head<3>() = dp;
            Jhat.col(i).tail<3>() << dR(2, 1)/h,
                    dR(0, 2)/h,
                    dR(1, 0)/h;

        }

        TIC();
        Eigen::MatrixXd J;
        ASSERT_TRUE(model->getRelativeJacobian(lname, bname, J));
        dt += TOC();
        count++;

        EXPECT_LT((J - Jhat).lpNorm<Eigen::Infinity>(), 1e-3)
                << lname << " -> " << bname << "\n" <<
                   "Jhat = \n" << Jhat.format(3) << "\n" <<
                   "J    = \n" << J.format(3) << "\n";
    };

    for(int i = 0; i < 1; i++)
    {
        Eigen::VectorXd q0 = model->getJointPosition();
        Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
        Eigen::VectorXd q; model->sum(q0, v, q);

        for(auto [lname, lptr] : model->getUrdf()->links_)
        {
            for(auto [bname, bptr] : model->getUrdf()->links_)
            {
                check_jac(lname, bname, q);
            }
        }
    }

    std::cout << "getRelativeJacobian requires " << dt/count*1e6 << " us \n";
}

TEST_F(TestKinematics, checkRelativeVelocityVsJacobian)
{
    int count = 0;
    double dt = 0;

    auto check_vel_vs_jac = [this, &count, &dt](std::string lname, std::string bname, Eigen::VectorXd q0)
    {
        model->setJointPosition(q0);
        auto vi = Eigen::VectorXd::Random(model->getNv()).eval();
        model->setJointVelocity(vi);
        model->update();

        Eigen::MatrixXd J;
        ASSERT_TRUE(model->getRelativeJacobian(lname, bname, J));

        TIC();
        auto vel = model->getRelativeVelocityTwist(lname, bname);
        dt += TOC();
        count++;

        EXPECT_LT((vel - J*vi).lpNorm<Eigen::Infinity>(), 1e-9);


    };

    for(int i = 0; i < 30; i++)
    {
        Eigen::VectorXd q0 = model->getJointPosition();
        Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
        Eigen::VectorXd q; model->sum(q0, v, q);

        for(auto [lname, lptr] : model->getUrdf()->links_)
        {
            for(auto [bname, bptr] : model->getUrdf()->links_)
            {
                check_vel_vs_jac(lname, bname, q);
            }
        }
    }

    std::cout << "getRelativeVelocityTwist requires " << dt/count*1e6 << " us \n";
}

TEST_F(TestKinematics, checkRelativeJdotTimesV)
{
    int count = 0;
    double dt = 0;

    auto check_jdot_times_v = [this, &count, &dt](std::string lname, std::string bname,
            Eigen::VectorXd q0, Eigen::VectorXd v)
    {
        model->setJointPosition(q0);
        model->setJointVelocity(v);
        model->setJointAcceleration(v*0);
        model->update();

        TIC();
        auto acc = model->getRelativeJdotTimesV(lname, bname);
        dt += TOC();
        count += 1;

        const double h = 1e-4;

        Eigen::VectorXd qplus; model->sum(q0, v*h/2, qplus);
        Eigen::VectorXd qminus; model->sum(q0, -v*h/2, qminus);

        model->setJointPosition(qplus);
        model->update();
        auto vplus = model->getRelativeVelocityTwist(lname, bname);

        model->setJointPosition(qminus);
        model->update();
        auto vminus = model->getRelativeVelocityTwist(lname, bname);

        Eigen::Vector6d acc_hat = (vplus - vminus)/h;

        EXPECT_LT((acc_hat - acc).lpNorm<Eigen::Infinity>(), 1e-3)
                << lname << " -> " << bname << "\n" <<
                   "a0     = " << acc.transpose().format(3) << "\n" <<
                   "a0_hat = " << acc_hat.transpose().format(3) << "\n";


    };

    for(int i = 0; i < 30; i++)
    {
        Eigen::VectorXd q0 = model->getJointPosition();
        Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
        Eigen::VectorXd q; model->sum(q0, v, q);

        for(auto [lname, lptr] : model->getUrdf()->links_)
        {
            for(auto [bname, bptr] : model->getUrdf()->links_)
            {
                check_jdot_times_v(lname, bname, q, v);
            }
        }
    }

    std::cout << "getRelativeJdotTimesV requires " << dt/count*1e6 << " us \n";
}

TEST_F(TestKinematics, checkAcceleration)
{
    int count = 0;
    double dt = 0;

    auto check_acc = [this, &count, &dt](std::string lname,
            Eigen::VectorXd q0, Eigen::VectorXd v, Eigen::VectorXd a)
    {
        model->setJointPosition(q0);
        model->setJointVelocity(v);
        model->setJointAcceleration(a);
        model->update();

        TIC();
        auto acc = model->getAccelerationTwist(lname);
        dt += TOC();
        count += 1;

        Eigen::MatrixXd J;
        ASSERT_TRUE(model->getJacobian(lname, J));
        Eigen::Vector6d acc_J = J*model->getJointAcceleration() +
                model->getJdotTimesV(lname);

        const double h = 1e-4;

        Eigen::VectorXd qjplus; model->sum(q0, v*h/2 + a*h*h/8, qjplus);
        Eigen::VectorXd vjplus = v + a*h/2;
        Eigen::VectorXd qjminus; model->sum(q0, -v*h/2 - a*h*h/8, qjminus);
        Eigen::VectorXd vjminus = v - a*h/2;

        model->setJointPosition(qjplus);
        model->setJointVelocity(vjplus);
        model->update();
        auto vplus = model->getVelocityTwist(lname);

        model->setJointPosition(qjminus);
        model->setJointVelocity(vjminus);
        model->update();
        auto vminus = model->getVelocityTwist(lname);

        Eigen::Vector6d acc_hat = (vplus - vminus)/h;

        EXPECT_LT((acc_hat - acc).lpNorm<Eigen::Infinity>(), 1e-3);

        EXPECT_LT((acc_J - acc).lpNorm<Eigen::Infinity>(), 1e-6);


    };

    for(int i = 0; i < 100; i++)
    {
        Eigen::VectorXd q0 = model->getJointPosition();
        Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
        Eigen::VectorXd q; model->sum(q0, v, q);
        Eigen::VectorXd a = Eigen::VectorXd::Random(model->getNv());

        for(auto [lname, lptr] : model->getUrdf()->links_)
        {
            check_acc(lname, q, v, a);
        }
    }

    std::cout << "getAccelerationTwist requires " << dt/count*1e6 << " us \n";
}


TEST_F(TestKinematics, checkRelativeAcceleration)
{
    int count = 0;
    double dt = 0;

    auto check_acc = [this, &count, &dt](std::string lname, std::string bname,
            Eigen::VectorXd q0, Eigen::VectorXd v, Eigen::VectorXd a)
    {
        model->setJointPosition(q0);
        model->setJointVelocity(v);
        model->setJointAcceleration(a);
        model->update();

        TIC();
        auto acc = model->getRelativeAccelerationTwist(lname, bname);
        dt += TOC();
        count += 1;

        Eigen::MatrixXd J;
        ASSERT_TRUE(model->getRelativeJacobian(lname, bname, J));

        Eigen::Vector6d acc_J = J *
                model->getJointAcceleration() +
                model->getRelativeJdotTimesV(lname, bname);

        const double h = 1e-4;

        Eigen::VectorXd qjplus; model->sum(q0, v*h/2 + a*h*h/8, qjplus);
        Eigen::VectorXd vjplus = v + a*h/2;
        Eigen::VectorXd qjminus; model->sum(q0, -v*h/2 - a*h*h/8, qjminus);
        Eigen::VectorXd vjminus = v - a*h/2;

        model->setJointPosition(qjplus);
        model->setJointVelocity(vjplus);
        model->update();
        auto vplus = model->getRelativeVelocityTwist(lname, bname);

        model->setJointPosition(qjminus);
        model->setJointVelocity(vjminus);
        model->update();
        auto vminus = model->getRelativeVelocityTwist(lname, bname);

        Eigen::Vector6d acc_hat = (vplus - vminus)/h;

        EXPECT_LT((acc_hat - acc).lpNorm<Eigen::Infinity>(), 1e-3);

        EXPECT_LT((acc_J - acc).lpNorm<Eigen::Infinity>(), 1e-6);


    };

    for(int i = 0; i < 3; i++)
    {
        Eigen::VectorXd q0 = model->getJointPosition();
        Eigen::VectorXd v = Eigen::VectorXd::Random(model->getNv());
        Eigen::VectorXd q; model->sum(q0, v, q);
        Eigen::VectorXd a = Eigen::VectorXd::Random(model->getNv());

        for(auto [lname, lptr] : model->getUrdf()->links_)
        {
            for(auto [bname, bptr] : model->getUrdf()->links_)
            {
                check_acc(lname, bname, q, v, a);
            }
        }
    }

    std::cout << "getRelativeAccelerationTwist requires " << dt/count*1e6 << " us \n";
}


int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}