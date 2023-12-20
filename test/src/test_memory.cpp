#include "common.h"
#include "malloc.h"
#include "dlfcn.h"

using TestMemory = TestWithModel;

int malloc_calls = 0;
int free_calls = 0;
void reset_counters()
{
    malloc_calls = 0;
    free_calls = 0;
}

void* malloc(size_t sz)
{
    static auto libc_malloc = reinterpret_cast<void *(*)(size_t)>(dlsym(RTLD_NEXT, "malloc"));
    malloc_calls++;
    return libc_malloc(sz);
}

void free(void *p)
{
    static auto libc_free = reinterpret_cast<void(*)(void*)>(dlsym(RTLD_NEXT, "free"));
    free_calls++;
    libc_free(p);
}

TEST_F(TestMemory, checkMalloc)
{
    std::string name = "arm1_7";
    int id = model->getLinkId(name);

    Eigen::VectorXd q(model->getNq());
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
    EXPECT_GT(free_calls, 0);

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


}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
