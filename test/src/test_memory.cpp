#include "common.h"
#include "malloc.h"
#include "dlfcn.h"

using TestMemory = TestWithModel;

int malloc_calls = 0;
int free_calls = 0;
bool malloc_banned = false;

void reset_counters()
{
    malloc_calls = 0;
    free_calls = 0;
}

void* malloc(size_t sz)
{
    static auto libc_malloc = reinterpret_cast<void *(*)(size_t)>(dlsym(RTLD_NEXT, "malloc"));
    if(malloc_banned) std::abort();
    malloc_calls++;
    return libc_malloc(sz);
}

void free(void *p)
{
    static auto libc_free = reinterpret_cast<void(*)(void*)>(dlsym(RTLD_NEXT, "free"));
    if(p == nullptr) return;
    if(malloc_banned) std::abort();
    free_calls++;
    libc_free(p);
}

TEST_F(TestMemory, checkMalloc)
{
    std::string name = "arm1_7";
    int id = model->getLinkId(name);

    Eigen::VectorXd q = model->generateRandomQ();

    reset_counters();
    model->setJointPosition(q);
    model->update();
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    q = model->getJointPosition();
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    auto T = model->getPose(id);
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    auto v = model->getVelocityTwist(id);
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    auto a = model->getAccelerationTwist(id);
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    auto jd = model->getJdotTimesV(id);
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    auto jdrel = model->getRelativeJdotTimesV(id, 0);
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    auto vrel = model->getRelativeVelocityTwist(id, 0);
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    auto arel = model->getRelativeAccelerationTwist(id, 0);
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    auto J = model->getJacobian(name);
    EXPECT_GT(malloc_calls, 0);

    reset_counters();
    model->getJacobian(id, J);
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    model->getRelativeJacobian(id, id+1, J);
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    Eigen::MatrixXd Jbig(40, model->getNv());
    reset_counters();
    model->getJacobian(id, Jbig.middleRows(10, 6));
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    model->getRelativeJacobian(id, id+1, Jbig.middleRows(20, 6));
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    model->getRelativeJacobian(id, id+1, Jbig.middleRows(20, 6));
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    auto tau = model->computeInverseDynamics();
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    auto gc = model->computeGravityCompensation();
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    Eigen::VectorXd gc1(model->getNv());
    reset_counters();
    gc1 = model->computeGravityCompensation();
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    model->computeGravityCompensation(gc1);
    EXPECT_EQ(malloc_calls, 0);
    EXPECT_EQ(free_calls, 0);

    reset_counters();
    model->computeInverseDynamics(gc1);
    EXPECT_EQ(malloc_calls, 0) << "computeInverseDynamics";
    EXPECT_EQ(free_calls, 0) << "computeInverseDynamics";

    reset_counters();
    auto M = model->computeInertiaMatrix();
    EXPECT_EQ(malloc_calls, 0) << "computeInertiaMatrix";
    EXPECT_EQ(free_calls, 0) << "computeInertiaMatrix";

    reset_counters();
    malloc_banned = true;
    auto Minv = model->computeInertiaInverse();
    malloc_banned = false;
    EXPECT_EQ(malloc_calls, 0) << "computeInertiaInverse";
    EXPECT_EQ(free_calls, 0) << "computeInertiaInverse";

}


int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
