#include "common.h"

#include <xbot2_interface/collision.h>
#include <fmt/format.h>

struct TestCollision : TestWithModel
{
    std::shared_ptr<XBot::Collision::CollisionModel> cm;

    TestCollision()
    {
        // urdf_path = XBOT2_TEST_RESOURCE_DIR "collision_test_robot.urdf.xml";
        // srdf_path = "";
    }

    virtual void SetUp()
    {
        TestWithModel::SetUp();

        cm = std::make_shared<XBot::Collision::CollisionModel>(model);
    }
};

struct TestCollisionWithParam : TestCollision,
                                ::testing::WithParamInterface<bool>
{

};

TEST_F(TestCollision, checkSize)
{
    cm->update();

    ASSERT_EQ(cm->computeDistance().size(), cm->getNumCollisionPairs());
    ASSERT_EQ(cm->getDistanceJacobian().rows(), cm->getNumCollisionPairs());
    ASSERT_EQ(cm->getCollisionPairs().size(), cm->getNumCollisionPairs());
    ASSERT_EQ(cm->getWitnessPoints().size(), cm->getNumCollisionPairs());
    ASSERT_EQ(cm->getNormals().size(), cm->getNumCollisionPairs());
    ASSERT_EQ(cm->getDistanceJacobian().cols(), model->getNv());

}

TEST_F(TestCollision, checkBroadphase)
{
    model->setJointPosition(model->getNeutralQ());
    model->update();
    cm->update();

    std::vector<int> coll_pair_ids;
    EXPECT_TRUE(cm->checkSelfCollision(coll_pair_ids));
    EXPECT_TRUE((cm->computeDistance().array() < 0).any());
    EXPECT_GT(coll_pair_ids.size(), 0);

    for(auto id : coll_pair_ids)
    {
        auto [l1, l2] = cm->getCollisionPairs().at(id);

        std::cout << l1 << " vs " << l2 << "\n";
    }
}

TEST_P(TestCollisionWithParam, checkFixCollision)
{
    // add a sphere in the environment
    XBot::Collision::Shape::Sphere sp;
    sp.radius = 0.5;
    Eigen::Affine3d w_T_c = model->getPose("wheel_3");
    cm->addCollisionShape("mysphere", "world", sp, w_T_c);

    bool include_env = GetParam();

    // let's go
    model->setJointPosition(model->getNeutralQ());
    model->update();
    cm->update();

    double d_th = 0.10;

    double min_d = 0.04;

    double err_th = 0.01;

    double lam = 1.0;

    auto coll_pairs = cm->getCollisionPairs(include_env);

    Eigen::MatrixXd Jn(coll_pairs.size(), model->getNv());
    Eigen::VectorXd en(coll_pairs.size());  // des - actual

    for(int k = 0; true; k++)
    {
        Eigen::VectorXd d = cm->computeDistance(include_env, d_th);

        if(d.minCoeff() > min_d - err_th)
        {
            break;
        }

        Eigen::MatrixXd J = cm->getDistanceJacobian(include_env);

        EXPECT_EQ(J.rows(), d.size());

        int nn = 0;

        for(int i = 0; i < d.size(); i++)
        {
            if(d[i] >= min_d)
            {
                continue;
            }

            std::cout << k << " " << coll_pairs[i].first << " vs " << coll_pairs[i].second << ": " << d[i] << "\n";

            en[nn] = min_d - d[i];
            Jn.row(nn) = J.row(i);
            nn++;
        }

        auto Jn_block = Jn.topRows(nn);
        auto en_block = en.head(nn);

        auto svd = Jn_block.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV);
        svd.setThreshold(svd.singularValues()[0] / 10.0);

        Eigen::VectorXd dq = lam * svd.solve(en_block);

        // std::cout << "sv = " << svd.singularValues().transpose().format(2) << "\n";

        // std::cout << "J = \n" << Jn_block.format(2) << "\n";

        // std::cout << "en = " << en_block.transpose().format(2) << "\n";

        // std::cout << "dq = " << dq.transpose().format(2) << "\n";

        std::cout << "e = " << en_block.lpNorm<Eigen::Infinity>() << "\n";

        model->integrateJointPosition(dq);

        model->update();

        cm->update();

    }

    std::cout << "final q = " << model->getJointPosition().transpose() << "\n";

    EXPECT_FALSE(cm->checkCollision(include_env));

}


INSTANTIATE_TEST_SUITE_P(
    checkFixCollision,
    TestCollisionWithParam,
    testing::Values(true, false));

TEST_F(TestCollision, checkEnvironment)
{
    model->setJointPosition(model->getRobotState("home"));
    model->update();
    cm->update();

    // start configuration is collision free
    auto d = cm->computeDistance();
    std::cout << "min distance in homing = " << d.minCoeff() << "\n";
    ASSERT_GT(d.minCoeff(), 0);
    ASSERT_FALSE(cm->checkSelfCollision());
    ASSERT_FALSE(cm->checkCollision());

    // environment has no collisions
    ASSERT_EQ(cm->getNumCollisionPairs(true), cm->getNumCollisionPairs(false));

    // add one
    XBot::Collision::Shape::Sphere sp;
    sp.radius = 0.5;

    Eigen::Affine3d w_T_c;
    w_T_c.setIdentity();
    w_T_c.translation() << 3, 3, 3;  // very far away

    cm->addCollisionShape("mysphere", "world", sp, w_T_c);

    // environment has collisions
    ASSERT_GT(cm->getNumCollisionPairs(true), cm->getNumCollisionPairs(false));

    // still no self collisions
    d = cm->computeDistance(true);
    std::cout << "min distance in homing = " << d.minCoeff() << "\n";
    ASSERT_GT(d.minCoeff(), 0);
    ASSERT_FALSE(cm->checkSelfCollision());
    ASSERT_FALSE(cm->checkCollision());

    // move it to same position has the left hand
    w_T_c = model->getPose("ball1");
    cm->moveCollisionShape("mysphere", w_T_c);
    d = cm->computeDistance(true);
    std::cout << "min distance in homing = " << d.minCoeff() << "\n";
    ASSERT_LT(d.minCoeff(), 0);
    ASSERT_FALSE(cm->checkSelfCollision());
    std::vector<int> colliding_idx;
    ASSERT_TRUE(cm->checkCollision(colliding_idx));

    // check wp
    auto cpairs = cm->getCollisionPairs(true);
    auto wpv = cm->getWitnessPoints(true);
    EXPECT_GT(colliding_idx.size(), 0);

    for(auto i : colliding_idx)
    {
        // check all colliding pairs are robot-to-env
        EXPECT_EQ(cpairs[i].second, "world");

        // check env witness point lies on sphere surface
        Eigen::Vector3d wp_env = wpv[i].second;

        EXPECT_FLOAT_EQ((w_T_c.translation() - wp_env).norm(), sp.radius) <<
            "wp = " << wp_env.transpose() << "\n" <<
            "sp = " << w_T_c.translation().transpose() << "\n";

    }
}

TEST_F(TestCollision, checkPairSetters)
{
    // add env
    XBot::Collision::Shape::Sphere sp;
    sp.radius = 0.5;

    Eigen::Affine3d w_T_c;
    w_T_c.setIdentity();

    cm->addCollisionShape("mysphere", "world", sp, w_T_c);

    //
    EXPECT_THROW(cm->setLinkPairs({{"pelvis", "doesnotexist"}}), std::out_of_range);

    //
    cm->setLinkPairs({});

    EXPECT_EQ(cm->getNumCollisionPairs(false), 0);
    EXPECT_GT(cm->getNumCollisionPairs(true), 0);

    cm->setLinksVsEnvironment({"pelvis", "arm1_4"});

    EXPECT_EQ(cm->getNumCollisionPairs(false), 0);
    EXPECT_EQ(cm->getNumCollisionPairs(true), 2);

    cm->addCollisionShape("mysphere1", "world", sp, w_T_c);

    EXPECT_EQ(cm->getNumCollisionPairs(false), 0);
    EXPECT_EQ(cm->getNumCollisionPairs(true), 4);

    cm->setLinksVsEnvironment({"pelvis"});

    EXPECT_EQ(cm->getNumCollisionPairs(false), 0);
    EXPECT_EQ(cm->getNumCollisionPairs(true), 2);
}

TEST_F(TestCollision, checkCollision)
{
    // collision free pose
    model->setJointPosition(model->getRobotState("home"));
    model->update();
    cm->update();
    ASSERT_FALSE(cm->checkSelfCollision());

    // compute min distance
    double min_d = cm->computeDistance().minCoeff();

    // check consistency w.r.t. threshold
    EXPECT_FALSE(cm->checkSelfCollision(min_d * 0.1));
    EXPECT_FALSE(cm->checkSelfCollision(min_d * 0.5));
    EXPECT_FALSE(cm->checkSelfCollision(min_d * 0.9));
    EXPECT_FALSE(cm->checkSelfCollision(min_d * 0.99));
    EXPECT_TRUE(cm->checkSelfCollision(min_d * 1.0));
    EXPECT_TRUE(cm->checkSelfCollision(min_d * 1.1));
}

TEST_F(TestCollision, checkCollisionVsDistance)
{
    double dt_upd = 0, dt_dist = 0, dt_coll = 0;

    auto check_coll_vs_dist = [&]()
    {
        model->setJointPosition(model->generateRandomQ());
        model->update();

        TIC(upd);
        cm->update();
        dt_upd += TOC(upd);

        TIC(dist);
        double min_dist = cm->computeDistance(false, 0).minCoeff();
        dt_dist += TOC(dist);

        TIC(coll);
        bool collide = cm->checkSelfCollision();
        dt_coll += TOC(coll);

        EXPECT_EQ(collide, min_dist <= 0);
    };

    int count = 1000;

    for(int i = 0; i < count; i++)
    {
        check_coll_vs_dist();
    }


    std::cout << "CollisionModel::update requires " << dt_upd/count*1e6 << " us \n";
    std::cout << "CollisionModel::getDistance requires " << dt_dist/count*1e6 << " us \n";
    std::cout << "CollisionModel::checkSelfCollision requires " << dt_coll/count*1e6 << " us \n";

}

TEST_F(TestCollision, checkUserCollisionActivation)
{
    // collision free pose
    model->setJointPosition(model->getRobotState("home"));
    model->update();
    cm->update();
    ASSERT_FALSE(cm->checkCollision());
    ASSERT_FALSE(cm->computeDistance().minCoeff() <= 0);

    // add one
    XBot::Collision::Shape::Sphere sp;
    sp.radius = 0.2;
    Eigen::Affine3d w_T_c = model->getPose("ball1");
    cm->addCollisionShape("mysphere", "world", sp, w_T_c);
    ASSERT_TRUE(cm->checkCollision());
    ASSERT_TRUE(cm->computeDistance(true).minCoeff() <= 0);

    // deactivate
    cm->setCollisionShapeActive("mysphere", false);
    ASSERT_FALSE(cm->checkCollision());
    ASSERT_FALSE(cm->computeDistance(true).minCoeff() <= 0);

    // activate
    cm->setCollisionShapeActive("mysphere", true);
    ASSERT_TRUE(cm->checkCollision());
    ASSERT_TRUE(cm->computeDistance(true).minCoeff() <= 0);

}

TEST_F(TestCollision, checkUserCollisionTouchLinks)
{
    // collision free pose
    model->setJointPosition(model->getRobotState("home"));
    model->update();
    cm->update();
    ASSERT_FALSE(cm->checkCollision());
    ASSERT_FALSE(cm->computeDistance().minCoeff() <= 0);

    // add one
    XBot::Collision::Shape::Sphere sp;
    sp.radius = 0.2;
    Eigen::Affine3d w_T_c;
    w_T_c.setIdentity();
    std::vector<int> coll_idx;
    cm->addCollisionShape("mysphere",
                          "ball1",
                          sp,
                          w_T_c,
                          {"arm1_1", "arm1_5", "arm1_6", "arm1_7", "ball1"});
    EXPECT_FALSE(cm->checkCollision(coll_idx));
    EXPECT_FALSE(cm->computeDistance().minCoeff() <= 0);

    auto d = cm->computeDistance();
    auto pairs = cm->getCollisionPairs();

    for(int i : coll_idx)
    {
        fmt::print("{} vs {} -> d = {} \n",
                   pairs[i].first, pairs[i].second, d[i]);
    }

    // another but without dc
    int n1 = cm->getNumCollisionPairs();
    cm->addCollisionShape("mysphere1",
                          "ball1",
                          sp,
                          w_T_c,
                          {});
    int n2 = cm->getNumCollisionPairs();
    EXPECT_GT(n2, n1);
    EXPECT_TRUE(cm->checkCollision(coll_idx));
    EXPECT_TRUE(cm->computeDistance().minCoeff() <= 0);




}

TEST_F(TestCollision, checkWpNormals)
{

    auto check_wp_normals = [&]()
    {
        auto qrand = model->sum(model->getNeutralQ(), 3*Eigen::VectorXd::Random(model->getNv()));
        model->setJointPosition(qrand);
        model->update();

        cm->update();

        auto d = cm->computeDistance();
        auto wp = cm->getWitnessPoints();
        auto n = cm->getNormals();
        auto lp = cm->getCollisionPairs();

        for(int i = 0; i < cm->getNumCollisionPairs(); i++)
        {
            auto [l1, l2] = lp[i];
            auto [w1, w2] = wp[i];

            EXPECT_NEAR((w1 - w2).norm(), fabs(d[i]), 1e-3) <<
                l1 << " vs " << l2 << ": d = " << d[i] << "\n" <<
                "d0 = " << (w1 - w2).norm() << "\n" <<
                "d1 = " << fabs(d[i]) << "\n";

            Eigen::Vector3d n_from_wp = (w2 - w1).normalized() * (d[i] < 0 ? -1 : 1);

            EXPECT_LT((n[i] - n_from_wp).lpNorm<Eigen::Infinity>(), 1e-3) <<
                l1 << " vs " << l2 << ": d = " << d[i] << "\n" <<
                "n0 = " << n[i].transpose() << "\n" <<
                "n1 = " << n_from_wp.transpose() << "\n";
        }

    };

    int count = 10001;

    for(int i = 0; i < count; i++)
    {
        check_wp_normals();
    }
}


TEST_F(TestCollision, checkOrderedIndices)
{

    auto check_ordered_idx = [&]()
    {
        auto qrand = model->sum(model->getNeutralQ(), 3*Eigen::VectorXd::Random(model->getNv()));
        model->setJointPosition(qrand);
        model->update();

        cm->update();

        auto d = cm->computeDistance();

        auto ordered_idx = cm->getOrderedCollisionPairIndices();

        ASSERT_EQ(ordered_idx.size(), cm->getNumCollisionPairs());

        for(int i = 0; i < cm->getNumCollisionPairs() - 1; i++)
        {
            EXPECT_LE(d(ordered_idx[i]), d(ordered_idx[i+1]));
        }

    };

    int count = 1001;

    for(int i = 0; i < count; i++)
    {
        check_ordered_idx();
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
        cm->update();
        dt_upd += TOC(upd);

        TIC(d);
        auto d0 = cm->computeDistance(th);
        dt_dist += TOC(d);

        TIC(jac);
        auto J = cm->getDistanceJacobian();
        dt_jac += TOC(jac);

        Eigen::MatrixXd Jhat;
        Jhat.resizeLike(J);


        double h = 1e-4;

        for(int i = 0; i < model->getNv(); i++)
        {
            Eigen::VectorXd dq = Eigen::VectorXd::Unit(model->getNv(), i)*h;

            model->setJointPosition(model->sum(qrand, dq));
            // std::cout << i << " qplus  " << model->getJointPosition().transpose().format(5) << "\n";
            model->update();
            cm->update();
            auto dplus = cm->computeDistance(th);

            model->setJointPosition(model->sum(qrand, -dq));
            // std::cout << i << " qminus " << model->getJointPosition().transpose().format(5) << "\n";
            model->update();
            cm->update();
            auto dminus = cm->computeDistance(th);


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

    int count = 1001;

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
