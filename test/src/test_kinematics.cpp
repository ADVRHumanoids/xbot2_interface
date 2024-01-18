#include "common.h"


using TestKinematics = TestWithModel;

TEST_F(TestKinematics, checkExceptions)
{
    EXPECT_THROW(model->getPose(-100), std::out_of_range);
    EXPECT_THROW(model->getPose("not_exist"), std::out_of_range);
    EXPECT_THROW(model->getPose(-100, -101), std::out_of_range);
    EXPECT_THROW(model->getPose("not_exist", "not_exist_1"), std::out_of_range);

    EXPECT_THROW(model->getVelocityTwist(-100), std::out_of_range);
    EXPECT_THROW(model->getVelocityTwist("not_exist"), std::out_of_range);
    EXPECT_THROW(model->getRelativeVelocityTwist(-100, -101), std::out_of_range);
    EXPECT_THROW(model->getRelativeVelocityTwist("not_exist", "not_exist_1"), std::out_of_range);

    EXPECT_THROW(model->getAccelerationTwist(-100), std::out_of_range);
    EXPECT_THROW(model->getAccelerationTwist("not_exist"), std::out_of_range);
    EXPECT_THROW(model->getRelativeAccelerationTwist(-100, -101), std::out_of_range);
    EXPECT_THROW(model->getRelativeAccelerationTwist("not_exist", "not_exist_1"), std::out_of_range);

    EXPECT_THROW(model->getJdotTimesV(-100), std::out_of_range);
    EXPECT_THROW(model->getJdotTimesV("not_exist"), std::out_of_range);
    EXPECT_THROW(model->getRelativeJdotTimesV(-100, -101), std::out_of_range);
    EXPECT_THROW(model->getRelativeJdotTimesV("not_exist", "not_exist_1"), std::out_of_range);

    EXPECT_THROW(model->getJacobian("not_exist"), std::out_of_range);
    EXPECT_THROW(model->getRelativeJacobian("not_exist", "not_exist_1"), std::out_of_range);
}

TEST_F(TestKinematics, checkMapAccessorsWheel)
{
    Eigen::VectorXd q0 = model->generateRandomQ();
    model->setJointPosition(q0);

    // just modify a wheel position
    XBot::JointNameMap wmap =
    {
        {"j_wheel_2", 2.0}
    };

    ASSERT_TRUE(model->hasJoint(wmap.begin()->first));

    model->setJointPositionMinimal(wmap);

    double qexp = model->getJoint(wmap.begin()->first)->getJointPositionMinimal().value();

    EXPECT_NEAR(qexp, wmap.begin()->second, 1e-3);


    Eigen::VectorXd q1 = model->getJointPosition();

    auto diff = model->difference(q0, q1);
    auto [iv, nv] = model->getJointInfo(wmap.begin()->first).inv();

    EXPECT_NEAR( diff.head(iv).norm(), 0, 1e-6);
    EXPECT_NEAR( diff.tail(model->getNv() - iv - nv).norm(), 0, 1e-6 );
    EXPECT_GT( diff.segment(iv, nv).norm(), 0 );


}

TEST_F(TestKinematics, checkMapMinimalToPosition)
{
    Eigen::VectorXd q0 = model->generateRandomQ();
    Eigen::VectorXd q1 = q0;

    // just modify a wheel position
    XBot::JointNameMap wmap = {{"reference@v5", 2.0}};

    model->minimalToPosition(wmap, q0);

    auto diff = model->difference(q0, q1);
    auto [iv, nv] = model->getJointInfo("reference").inv();

    EXPECT_NEAR( diff.head(iv).norm(), 0, 1e-6);
    EXPECT_NEAR( diff.tail(model->getNv() - iv - nv).norm(), 0, 1e-6 );
    EXPECT_GT( diff.segment(iv, nv).norm(), 0 );

}

TEST_F(TestKinematics, checkComInertia)
{

    Eigen::MatrixXd Jcom = model->getCOMJacobian();

    Eigen::Matrix3d Lcom = (Jcom * model->computeInertiaInverse() * Jcom.transpose()).inverse();

    EXPECT_NEAR(Lcom(0, 0), model->getMass(), 1e-3);
    EXPECT_NEAR(Lcom(1, 1), model->getMass(), 1e-3);
    EXPECT_NEAR(Lcom(2, 2), model->getMass(), 1e-3);

    Lcom.diagonal().setZero();
    EXPECT_LT(Lcom.norm(), 1e-6);

    // J*Minv*S
    Eigen::MatrixXd JMinvS = (Jcom*model->computeInertiaInverse()).rightCols(model->getActuatedNv());
    EXPECT_LT(JMinvS.norm(), 1e-6) << JMinvS.format(2);

}

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

TEST_F(TestKinematics, checkFloatingBase)
{
    auto fb = model->getJoint(0);
    EXPECT_EQ(model->getFloatingBaseLink(), fb->getChildLink());

    model->setJointPosition(model->generateRandomQ());
    model->update();

    Eigen::Affine3d T;
    T.translation().setRandom();
    T.linear() = Eigen::Quaterniond(Eigen::Vector4d::Random().normalized()).toRotationMatrix();

    ASSERT_TRUE(model->setFloatingBasePose(T));
    model->update();

    EXPECT_TRUE(model->getFloatingBasePose().isApprox(T, 1e-4));

    EXPECT_TRUE(model->getPose(fb->getChildLink()).isApprox(T, 1e-4));

    Eigen::Vector6d v;
    v.setRandom();
    ASSERT_TRUE(model->setFloatingBaseState(T, v));
    model->update();

    EXPECT_TRUE(model->getPose(fb->getChildLink()).isApprox(T, 1e-4));
    EXPECT_TRUE(model->getVelocityTwist(fb->getChildLink()).isApprox(v, 1e-4)) <<
        model->getVelocityTwist(fb->getChildLink()).transpose() << "\n" <<
        v.transpose();

}

TEST_F(TestKinematics, checkFloatingBaseImu)
{
    auto imu = std::const_pointer_cast<XBot::ImuSensor>(model->getImu("imu_link"));

    ASSERT_TRUE(bool(imu));

    for(int i = 0; i < 1000; i++)
    {

        Eigen::Vector3d omega;
        omega.setRandom();

        Eigen::Quaterniond w_R_imu = Eigen::Quaterniond(
            Eigen::Vector4d::Random().normalized());

        imu->setMeasurement(omega,
                            Eigen::Vector3d::Zero(),
                            w_R_imu,
                            XBot::wall_time::clock::now());

        ASSERT_TRUE(model->setFloatingBaseState(*imu));

        model->update();

        EXPECT_TRUE(model->getFloatingBasePose().linear().isApprox(w_R_imu.toRotationMatrix(), 1e-4));

        EXPECT_TRUE(model->getFloatingBaseTwist().tail<3>().isApprox(omega, 1e-4));

    }

}

TEST_F(TestKinematics, checkJacobianNumerical)
{
    int count = 0;
    double dt = 0;

    int count_upd = 0;
    double dt_upd = 0;

    int count_pose = 0;
    double dt_pose = 0;

    auto check_jac = [&](std::string lname,
                         Eigen::VectorXd q0,
                         Eigen::Vector3d offset = Eigen::Vector3d::Zero().eval())
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

        Eigen::Affine3d T_offset;
        T_offset.setIdentity();
        T_offset.translation() << offset;

        for(int i = 0; i < model->getNv(); i++)
        {
            auto vi = Eigen::VectorXd::Unit(model->getNv(), i);
            Eigen::VectorXd qplus;
            model->sum(q0, vi*h/2, qplus);
            Eigen::VectorXd qminus;
            model->sum(q0, -vi*h/2, qminus);

            model->setJointPosition(qplus);
            model->update();
            auto Tplus = model->getPose(lname)*T_offset;

            model->setJointPosition(qminus);
            model->update();
            auto Tminus = model->getPose(lname)*T_offset;

            Eigen::Vector3d dp = (Tplus.translation() - Tminus.translation())/h;
            Eigen::Matrix3d dR = (Tplus.linear() - Tminus.linear())*T.linear().transpose();

            Jhat.col(i).head<3>() = dp;
            Jhat.col(i).tail<3>() << dR(2, 1)/h,
                dR(0, 2)/h,
                dR(1, 0)/h;

        }

        TIC();
        Eigen::MatrixXd J;
        if(offset.isZero())
        {
            ASSERT_TRUE(model->getJacobian(lname, J));
        }
        else
        {
            ASSERT_TRUE(model->getJacobian(lname, offset, J));
        }
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
            Eigen::Vector3d offset;
            offset.setRandom();
            check_jac(lname, q);
            check_jac(lname, q, offset);
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

TEST_F(TestKinematics, checkSignatures)
{
    auto urdf = model->getUrdf();

    model->setJointPosition(model->generateRandomQ());
    model->update();

    for(auto [lname, lptr] : urdf->links_)
    {
        int lid = model->getLinkId(lname);
        ASSERT_GE(lid, 0);

        // return by reference by id
        Eigen::MatrixXd J;
        J.setZero(6, model->getNv());
        model->getJacobian(lid, J);

        // by value with link name
        auto Jret = model->getJacobian(lname);

        EXPECT_TRUE((J.cwiseEqual(Jret)).all());

        // with link name
        J.setZero(6, model->getNv());
        model->getJacobian(lname, J);

        EXPECT_TRUE((J.cwiseEqual(Jret)).all());

        // fill buffer
        Eigen::MatrixXd JJ;
        JJ.setZero(10, model->getNv());
        model->getJacobian(lname, JJ.topRows<6>());

        EXPECT_TRUE((JJ.topRows<6>().cwiseEqual(Jret)).all());

        for(auto [lname1, lptr1] : urdf->links_)
        {
            int lid1 = model->getLinkId(lname1);
            ASSERT_GE(lid1, 0);

            // return by reference by id
            Eigen::MatrixXd J;
            J.setZero(6, model->getNv());
            model->getRelativeJacobian(lid, lid1, J);

            // by value with link name
            auto Jret = model->getRelativeJacobian(lname, lname1);

            EXPECT_TRUE((J.cwiseEqual(Jret)).all());

            // with link name
            J.setZero(6, model->getNv());
            model->getRelativeJacobian(lname, lname1, J);

            EXPECT_TRUE((J.cwiseEqual(Jret)).all());

            // fill buffer
            Eigen::MatrixXd JJ;
            JJ.setZero(10, model->getNv());
            model->getRelativeJacobian(lname, lname1, JJ.topRows<6>());

            EXPECT_TRUE((JJ.topRows<6>().cwiseEqual(Jret)).all());
        }
    }
}

TEST_F(TestKinematics, checkGcomp)
{
    double dt_gcomp = 0;
    double dt_rnea = 0;
    int count = 0;

    for(int i = 0; i < 1000; i++)
    {
        model->setJointPosition(model->generateRandomQ());
        model->update();

        TIC(gc);
        auto gcomp = model->computeGravityCompensation();
        dt_gcomp += TOC(gc);

        auto Jcom = model->getCOMJacobian();
        Eigen::Vector3d mg(0, 0, 9.81*model->getMass());

        Eigen::VectorXd err = (gcomp - Jcom.transpose()*mg);

        EXPECT_LT(err.lpNorm<Eigen::Infinity>(), 1e-3) <<
            "err = " << err.transpose().format(2) << "\n";

        TIC(id);
        auto rnea = model->computeInverseDynamics();
        dt_rnea += TOC(id);

        err = (gcomp - rnea);

        EXPECT_LT(err.lpNorm<Eigen::Infinity>(), 1e-6) <<
            "err = " << err.transpose().format(2) << "\n";

        count++;
    }

    std::cout << "computeGravityCompensation requires " << dt_gcomp/count*1e6 << " us \n";
    std::cout << "computeInverseDynamics requires " << dt_rnea/count*1e6 << " us \n";

}

TEST_F(TestKinematics, checkRneaVsCrba)
{
    int count = 0;
    double dt = 0;

    for(int i = 0; i < 1000; i++)
    {
        model->setJointPosition(model->generateRandomQ());
        Eigen::VectorXd a;
        a.setRandom(model->getNv());
        model->setJointAcceleration(a);
        model->update();

        TIC();
        Eigen::MatrixXd M = model->computeInertiaMatrix();
        count++;
        dt += TOC();

        Eigen::VectorXd gcomp = model->computeGravityCompensation();
        Eigen::VectorXd rnea = model->computeInverseDynamics();

        Eigen::VectorXd err = M*a + gcomp - rnea;

        EXPECT_LT(err.lpNorm<Eigen::Infinity>(), 1e-6) <<
            "err = " << err.transpose().format(2) << "\n"
                                                       << "M*a + g = " << (M*a + gcomp).transpose().format(2) << "\n"
                                                       << "rnea    = " << (rnea).transpose().format(2) << "\n";
    }

    std::cout << "computeInertiaMatrix requires " << dt/count*1e6 << " us \n";

}


TEST_F(TestKinematics, checkInertiaInverse)
{
    int count = 0;
    double dt_fd = 0;
    double dt_minv = 0;

    for(int i = 0; i < 1000; i++)
    {
        model->setJointPosition(model->generateRandomQ());
        Eigen::VectorXd tau;
        tau.setRandom(model->getNv());
        model->setJointEffort(tau);
        model->update();

        Eigen::MatrixXd eye, Minv;
        eye.setIdentity(model->getNv(), model->getNv());
        Minv.resizeLike(eye);

        TIC();
        Minv = model->computeInertiaInverse();
        dt_minv += TOC();
        count++;

        Eigen::VectorXd fd;
        TIC(fd);
        fd = model->computeForwardDynamics();
        dt_fd += TOC(fd);
        Eigen::VectorXd gcomp = model->computeGravityCompensation();

        Eigen::VectorXd err = Minv*(tau - gcomp) - fd;

        EXPECT_LT(err.lpNorm<Eigen::Infinity>(), 1e-2) <<
            "err = " << err.transpose().format(2);
    }

    std::cout << "computeInertiaInverseTimesMatrix requires " << dt_minv/count*1e6 << " us \n";
    std::cout << "computeForwardDynamics requires " << dt_fd/count*1e6 << " us \n";

}

TEST_F(TestKinematics, checkInertiaInverse1)
{
    int count = 0;
    double dt_m_inv = 0;
    double dt_minv = 0;

    for(int i = 0; i < 1000; i++)
    {
        model->setJointPosition(model->generateRandomQ());
        Eigen::VectorXd tau;
        tau.setRandom(model->getNv());
        model->setJointEffort(tau);
        model->update();

        Eigen::MatrixXd eye, Minv1, Minv;
        eye.setIdentity(model->getNv(), model->getNv());
        Minv.resizeLike(eye);
        Minv1.resizeLike(Minv);

        TIC();
        Minv = model->computeInertiaInverse();
        dt_minv += TOC();

        TIC(1);
        Minv1 = model->computeInertiaMatrix().inverse();
        dt_m_inv += TOC(1);

        count++;


        EXPECT_LT((Minv - Minv1).lpNorm<Eigen::Infinity>(), 1e-3) <<
            "err = " << (Minv - Minv1).lpNorm<Eigen::Infinity>();
    }

    std::cout << "computeInertiaInverse()          requires " << dt_minv/count*1e6 << " us \n";
    std::cout << "computeInertiaMatrix().inverse() requires " << dt_m_inv/count*1e6 << " us \n";

}

TEST_F(TestKinematics, checkCmmVsCm)
{
    int count = 0;
    double dt_cmm = 0;
    double dt_cm = 0;
    double dt_cm_cached = 0;

    for(int i = 0; i < 1000; i++)
    {
        model->setJointPosition(model->generateRandomQ());
        Eigen::VectorXd v;
        v.setRandom(model->getNv());
        model->setJointVelocity(v);
        model->update();

        TIC();
        auto cm1 = model->computeCentroidalMomentum();
        dt_cm += TOC();

        TIC(cmm);
        auto cmm = model->computeCentroidalMomentumMatrix();
        Eigen::Vector6d cm2;
        cm2.noalias() = cmm*v;
        dt_cmm += TOC(cmm);

        TIC(cm_cached);
        auto cmc = model->computeCentroidalMomentum();
        dt_cm_cached += TOC(cm_cached);

        EXPECT_LT((cm2-cm1).lpNorm<Eigen::Infinity>(), 1e-3);
        EXPECT_LT((cm1-cmc).lpNorm<Eigen::Infinity>(), 1e-6);

        count++;

    }

    std::cout << "computeCentroidalMomentum requires " << dt_cm/count*1e6 << " us \n";
    std::cout << "computeCentroidalMomentumMatrix times V requires " << dt_cmm/count*1e6 << " us \n";
    std::cout << "computeCentroidalMomentum cached requires  " << dt_cm_cached/count*1e6 << " us \n";
}

TEST_F(TestKinematics, checkCmm)
{
    int count = 0;
    double dt_cmm = 0;

    for(int i = 0; i < 1000; i++)
    {
        model->setJointPosition(model->generateRandomQ());
        model->update();
        Eigen::VectorXd q = model->getJointPosition();

        Eigen::VectorXd tau;
        tau.setRandom(model->getNv());
        tau.head<6>().setZero();

        Eigen::Vector6d F;
        F.setRandom();

        Eigen::VectorXd v;
        v.setZero(model->getNv());

        Eigen::MatrixXd J = model->getJacobian("base_link");
        Eigen::Vector3d p_world;
        p_world.setRandom();
        XBot::Utils::changeRefPoint(J, p_world - model->getPose("base_link").translation());

        Eigen::Vector3d com = model->getCOM();

        Eigen::Vector6d dmom;
        dmom << F.head<3>(), F.tail<3>() + (p_world - com).cross(F.head<3>());

        dmom.head<3>() += Eigen::Vector3d(0, 0, -9.81)*model->getMass();

        model->setJointEffort(tau + J.transpose()*F);

        TIC();
        auto Ag = model->computeCentroidalMomentumMatrix();
        dt_cmm += TOC();
        count++;

        auto a = model->computeForwardDynamics();

        double dt = 0.0001;

        Eigen::VectorXd qnew = model->sum(q, 0.5*a*dt*dt);
        Eigen::VectorXd vnew = a*dt;

        // variation in momentum at zero initial velocity must equal
        // the applied wrench at the com
        EXPECT_LT((Ag*a - dmom).lpNorm<Eigen::Infinity>(), 1e-3) << "computeCentroidalMomentumMatrix \ns"
            << (Ag*a).transpose() << std::endl
            << (dmom).transpose() << std::endl;
    }

    std::cout << "computeCentroidalMomentumMatrix requires " << dt_cmm/count*1e6 << " us \n";

}

TEST_F(TestKinematics, checkAddBody)
{
    auto model_orig = model->clone();

    double mass_old = model->getMass();

    Eigen::Affine3d rel_T;
    rel_T.linear() = Eigen::Quaterniond(Eigen::Vector4d::Random().normalized()).toRotationMatrix();
    rel_T.translation().setRandom();

    Eigen::Matrix3d ine;
    ine.setIdentity();

    double dmass = 100;

    std::string parent = "arm1_4";

    int lid = model->addFixedLink("newlink", parent, dmass, ine, rel_T);
    model->update();

    EXPECT_DOUBLE_EQ(mass_old + dmass, model->getMass());

    EXPECT_EQ(model->getLinkId("newlink"), lid);

    model->setJointPosition(model->generateRandomQ());
    model->update();

    auto T = model->getPose("newlink");

    auto Tbase = model->getPose(parent);

    EXPECT_TRUE((Tbase * rel_T).isApprox(T));

    auto create_modified_urdf = [](urdf::ModelConstSharedPtr old_urdf,
                                   std::string link_name,
                                   std::string parent_name,
                                   double mass,
                                   Eigen::Matrix3d inertia,
                                   Eigen::Affine3d pose)
    {
        // create modified URDF
        auto durdf = std::make_shared<urdf::Model>();
        durdf->initString(XBot::Utils::urdfToString(*old_urdf));

        auto djoint = std::make_shared<urdf::Joint>();
        djoint->name = "aux_joint";
        djoint->type = urdf::Joint::FIXED;
        djoint->parent_link_name = parent_name;
        djoint->child_link_name = link_name ;
        djoint->parent_to_joint_origin_transform.position.x = pose.translation().x();
        djoint->parent_to_joint_origin_transform.position.y = pose.translation().y();
        djoint->parent_to_joint_origin_transform.position.z = pose.translation().z();
        djoint->parent_to_joint_origin_transform.rotation.setFromQuaternion(
            Eigen::Quaterniond(pose.linear()).x(),
            Eigen::Quaterniond(pose.linear()).y(),
            Eigen::Quaterniond(pose.linear()).z(),
            Eigen::Quaterniond(pose.linear()).w()
            );

        auto dlink = std::make_shared<urdf::Link>();
        dlink->name = "newlink";
        dlink->inertial = std::make_shared<urdf::Inertial>();
        dlink->inertial->mass = mass;
        dlink->inertial->ixx = inertia(0, 0);
        dlink->inertial->iyy = inertia(1, 1);
        dlink->inertial->izz = inertia(2, 2);
        dlink->inertial->ixy = inertia(0, 1);
        dlink->inertial->iyz = inertia(1, 2);
        dlink->inertial->ixz = inertia(0, 2);
        dlink->parent_joint = djoint;

        dlink->setParent(durdf->links_.at(djoint->parent_link_name));

        durdf->links_[dlink->name] = dlink;
        durdf->joints_[djoint->name] = djoint;
        durdf->links_.at(djoint->parent_link_name)->child_joints.push_back(djoint);
        durdf->links_.at(djoint->parent_link_name)->child_links.push_back(dlink);

        return durdf;
    };

    // new model
    auto durdf = create_modified_urdf(model_orig->getUrdf(), "newlink", parent, dmass, ine, rel_T);
    auto dmodel = XBot::ModelInterface::getModel(durdf, model->getSrdf(), model->getType());
    dmodel->setJointPosition(model->getJointPosition());
    dmodel->update();

    {
        auto M = model->computeInertiaMatrix();
        auto dM = dmodel->computeInertiaMatrix();

        EXPECT_TRUE((M - dM).isMuchSmallerThan(M, 1e-3)) << (M - dM).lpNorm<Eigen::Infinity>();
    }

    EXPECT_NEAR(dmodel->getMass(), model->getMass(), 1e-3);

    EXPECT_TRUE(dmodel->getPose("newlink").isApprox(model->getPose("newlink"), 1e-3));

    EXPECT_TRUE(dmodel->computeGravityCompensation().isApprox(model->computeGravityCompensation(), 1e-3));

    // test update
    dmass = 20.0;
    rel_T.linear() = Eigen::Quaterniond(Eigen::Vector4d::Random().normalized()).toRotationMatrix();
    rel_T.translation().setRandom();
    ine.setRandom();
    ine = ine.transpose() * ine;

    model->updateFixedLink(lid, dmass, ine, rel_T);
    model->update();
    auto durdf_upd = create_modified_urdf(model_orig->getUrdf(), "newlink", parent, dmass, ine, rel_T);
    auto dmodel_upd = XBot::ModelInterface::getModel(durdf_upd, model->getSrdf(), model->getType());
    dmodel_upd->setJointPosition(model->getJointPosition());
    dmodel_upd->update();

    {
        auto M = model->computeInertiaMatrix();
        auto dM = dmodel_upd->computeInertiaMatrix();

        EXPECT_TRUE((M - dM).isMuchSmallerThan(M, 1e-3)) << (M - dM).lpNorm<Eigen::Infinity>();
    }

    EXPECT_NEAR(dmodel_upd->getMass(), model->getMass(), 1e-3);

    EXPECT_TRUE(dmodel_upd->getPose("newlink").isApprox(model->getPose("newlink"), 1e-3));

    EXPECT_TRUE(dmodel_upd->computeGravityCompensation().isApprox(model->computeGravityCompensation(), 1e-3));


}


int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
