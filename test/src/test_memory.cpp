#include "common.h"
#include "dlfcn.h"
#include "malloc.h"

#include <xbot2_interface/collision.h>

using TestMemory = TestWithModel;

int malloc_calls = 0;
int free_calls = 0;
bool malloc_banned = false;
bool tracing_enabled = false;
bool tracing_allow_free_nullptr = true;

void reset_counters()
{
    malloc_calls = 0;
    free_calls = 0;
}

void *malloc(size_t sz)
{
    static auto libc_malloc = reinterpret_cast<void *(*) (size_t)>(dlsym(RTLD_NEXT, "malloc"));
    if (tracing_enabled) {
        if (malloc_banned)
            std::abort();
        malloc_calls++;
    }
    return libc_malloc(sz);
}

void free(void *p)
{
    static auto libc_free = reinterpret_cast<void (*)(void *)>(dlsym(RTLD_NEXT, "free"));

    if (p == nullptr && tracing_allow_free_nullptr) {
        return;
    }

    if (tracing_enabled) {
        if (malloc_banned)
            std::abort();
        free_calls++;
    }

    libc_free(p);
}

struct MemoryTracer
{
    MemoryTracer(bool _malloc_banned = false)
    {
        reset_counters();
        malloc_banned = _malloc_banned;
        tracing_enabled = true;
    }

    ~MemoryTracer() { tracing_enabled = false; malloc_banned = false; }
};

TEST_F(TestMemory, checkMalloc)
{
    std::string name = "arm1_7";
    int id = model->getLinkId(name);

    Eigen::VectorXd q = model->generateRandomQ();

    {
        MemoryTracer mt;
        model->setJointPosition(q);
        model->update();
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        q = model->getJointPosition();
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        auto T = model->getPose(id);
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        auto v = model->getVelocityTwist(id);
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        auto a = model->getAccelerationTwist(id);
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        auto jd = model->getJdotTimesV(id);
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        auto jdrel = model->getRelativeJdotTimesV(id, 0);
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        auto vrel = model->getRelativeVelocityTwist(id, 0);
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        auto arel = model->getRelativeAccelerationTwist(id, 0);
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    Eigen::MatrixXd J;
    {
        MemoryTracer mt;
        J = model->getJacobian(name);
    }
    EXPECT_GT(malloc_calls, 0);

    {
        MemoryTracer mt;
        model->getJacobian(id, J);
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        model->getRelativeJacobian(id, id + 1, J);
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    Eigen::MatrixXd Jbig(40, model->getNv());
    {
        MemoryTracer mt;
        model->getJacobian(id, Jbig.middleRows(10, 6));
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        model->getRelativeJacobian(id, id + 1, Jbig.middleRows(20, 6));
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        model->getRelativeJacobian(id, id + 1, Jbig.middleRows(20, 6));
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        auto tau = model->computeInverseDynamics();
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        auto gc = model->computeGravityCompensation();
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    Eigen::VectorXd gc1(model->getNv());
    {
        MemoryTracer mt;
        gc1 = model->computeGravityCompensation();
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        model->computeGravityCompensation(gc1);
    }
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    {
        MemoryTracer mt;
        model->computeInverseDynamics(gc1);
    }
    EXPECT_EQ(malloc_calls, 0) << "computeInverseDynamics";
    EXPECT_EQ(free_calls, 0) << "computeInverseDynamics";

    {
        MemoryTracer mt;
        model->computeForwardDynamics(gc1);
    }
    EXPECT_EQ(malloc_calls, 0) << "computeForwardDynamics";
    EXPECT_EQ(free_calls, 0) << "computeForwardDynamics";

    {
        MemoryTracer mt;
        auto M = model->computeInertiaMatrix();
    }
    EXPECT_EQ(malloc_calls, 0) << "computeInertiaMatrix";
    EXPECT_EQ(free_calls, 0) << "computeInertiaMatrix";

    {
        MemoryTracer mt;
        auto Minv = model->computeInertiaInverse();
    }
    EXPECT_EQ(malloc_calls, 0) << "computeInertiaInverse";
    EXPECT_EQ(free_calls, 0) << "computeInertiaInverse";

    {
        MemoryTracer mt;
        auto Ag = model->computeCentroidalMomentumMatrix();
    }
    EXPECT_EQ(malloc_calls, 0) << "computeCentroidalMomentumMatrix";
    EXPECT_EQ(free_calls, 0) << "computeCentroidalMomentumMatrix";
}

TEST_F(TestMemory, checkCollisionModelMalloc)
{
    reset_counters();
    tracing_enabled = true;
    XBot::Collision::CollisionModel cm(model);
    tracing_enabled = false;
    Eigen::VectorXd d(cm.getNumCollisionPairs());
    Eigen::MatrixXd J(d.size(), model->getNv());
    EXPECT_GT(malloc_calls, 0) << "CollisionModel";
    EXPECT_GT(free_calls, 0) << "CollisionModel";

    {
        MemoryTracer mt;
        cm.update();
    }
    EXPECT_EQ(malloc_calls, 0) << "update";
    EXPECT_EQ(free_calls, 0) << "update";

    {
        MemoryTracer mt;
        cm.computeDistance(d);
    }
    EXPECT_EQ(malloc_calls, 0) << "computeDistance";
    EXPECT_EQ(free_calls, 0) << "computeDistance";

    {
        MemoryTracer mt;
        cm.getDistanceJacobian(J);
    }
    EXPECT_EQ(malloc_calls, 0) << "getDistanceJacobian";
    EXPECT_EQ(free_calls, 0) << "getDistanceJacobian";
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
