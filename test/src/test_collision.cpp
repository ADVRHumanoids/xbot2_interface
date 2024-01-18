#include "common.h"

#include <xbot2_interface/collision.h>


struct TestCollision : TestWithModel
{
    std::shared_ptr<XBot::Collision::CollisionModel> cm;

    TestCollision()
    {
        urdf_path = XBOT2_TEST_RESOURCE_DIR "collision_test_robot.urdf.xml";
        srdf_path = "";
    }

    virtual void SetUp()
    {
        TestWithModel::SetUp();

        cm = std::make_shared<XBot::Collision::CollisionModel>(model);
    }
};

TEST_F(TestCollision, checkSize)
{
    cm->update();

    ASSERT_EQ(cm->getDistance().size(), cm->getNumCollisionPairs());
    ASSERT_EQ(cm->getDistanceJacobian().rows(), cm->getNumCollisionPairs());
    ASSERT_EQ(cm->getCollisionPairs().size(), cm->getNumCollisionPairs());
    ASSERT_EQ(cm->getWitnessPoints().size(), cm->getNumCollisionPairs());
    ASSERT_EQ(cm->getNormals().size(), cm->getNumCollisionPairs());
    ASSERT_EQ(cm->getDistanceJacobian().cols(), model->getNv());

}

TEST_F(TestCollision, checkWpNormals)
{
    auto check_wp_normals = [&]()
    {
        auto qrand = model->sum(model->getNeutralQ(), 3*Eigen::VectorXd::Random(model->getNv()));
        model->setJointPosition(qrand);
        model->update();

        cm->update();

        auto wp = cm->getWitnessPoints();
        auto n = cm->getNormals();
        auto d = cm->getDistance();
        auto lp = cm->getCollisionPairs();

        for(int i = 0; i < cm->getNumCollisionPairs(); i++)
        {
            auto [l1, l2] = lp[i];
            auto [w1, w2] = wp[i];

            Eigen::Vector3d w1_global = model->getPose(l1)*w1;
            Eigen::Vector3d w2_global = model->getPose(l2)*w2;

            EXPECT_NEAR((w1_global - w2_global).norm(), fabs(d[i]), 1e-3) <<
                l1 << " vs " << l2 << ": d = " << d[i] << "\n" <<
                "d0 = " << (w1_global - w2_global).norm() << "\n" <<
                "d1 = " << fabs(d[i]) << "\n";

            Eigen::Vector3d n_from_wp = (w2_global - w1_global).normalized() * (d[i] < 0 ? -1 : 1);

            EXPECT_LT((n[i] - n_from_wp).lpNorm<Eigen::Infinity>(), 1e-3) <<
                l1 << " vs " << l2 << ": d = " << d[i] << "\n" <<
                "n0 = " << n[i].transpose() << "\n" <<
                "n1 = " << n_from_wp.transpose() << "\n";
        }

    };

    int count = 100001;

    for(int i = 0; i < count; i++)
    {
        check_wp_normals();
    }


}

TEST_F(TestCollision, checkJacobian)
{

    double dt_upd = 0, dt_dist = 0, dt_jac = 0;

    double th = -1;

    auto check_jac = [&]()
    {
        auto qrand = model->sum(model->getNeutralQ(), 3*Eigen::VectorXd::Random(model->getNv()));
        model->setJointPosition(qrand);
        model->update();

        TIC(upd);
        cm->update(th);
        dt_upd += TOC(upd);

        TIC(jac);
        auto J = cm->getDistanceJacobian();
        dt_jac += TOC(jac);

        Eigen::MatrixXd Jhat;
        Jhat.resizeLike(J);

        TIC(d);
        auto d0 = cm->getDistance();
        dt_dist += TOC(d);

        double h = 1e-4;

        for(int i = 0; i < model->getNv(); i++)
        {
            Eigen::VectorXd dq = Eigen::VectorXd::Unit(model->getNv(), i)*h;

            model->setJointPosition(model->sum(qrand, dq));
            // std::cout << i << " qplus  " << model->getJointPosition().transpose().format(5) << "\n";
            model->update();
            cm->update(th);
            auto dplus = cm->getDistance();

            model->setJointPosition(model->sum(qrand, -dq));
            // std::cout << i << " qminus " << model->getJointPosition().transpose().format(5) << "\n";
            model->update();
            cm->update(th);
            auto dminus = cm->getDistance();


            // std::cout << i << " dplus  " << dplus.transpose().format(5) << "\n";
            // std::cout << i << " d0     " << d0.transpose().format(5) << "\n";
            // std::cout << i << " dminus " << dminus.transpose().format(5) << "\n\n";

            Jhat.col(i) = (dplus - dminus) / (2*h);
        }

        double Jerr = (J - Jhat).lpNorm<Eigen::Infinity>();


        EXPECT_LT(Jerr, 0.5);

        int i = 0;
        for(auto [l1, l2] : cm->getCollisionPairs())
        {
            double err = (J.row(i) - Jhat.row(i)).lpNorm<Eigen::Infinity>();

            if(err > 1e-1)
            {
                std::cout << l1 << " vs " << l2 << ": err = " << err << ", dist = " << d0[i] << "\n";
                std::cout << "Jhat = " << Jhat.row(i).format(4) << "\n";
                std::cout << "J    = " << J.row(i).format(4) << "\n\n";
            }

            i++;
        }

    };

    int count = 10001;

    for(int i = 0; i < count; i++)
    {
        check_jac();
    }

    std::cout << "CollisionModel::update requires " << dt_upd/count*1e6 << " us \n";
    std::cout << "CollisionModel::getDistance requires " << dt_dist/count*1e6 << " us \n";
    std::cout << "CollisionModel::getDistanceJacobian requires " << dt_jac/count*1e6 << " us \n";

}


int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
