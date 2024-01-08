#include <Eigen/Dense>
#include <malloc.h>
#include <dlfcn.h>
#include <iostream>

int __malloc_calls = 0;
int __free_calls = 0;
bool __register_calls = false;

void reset_counters()
{
    __register_calls = true;
    __malloc_calls = 0;
    __free_calls = 0;
}

std::pair<int, int> get_counters()
{
    __register_calls = false;
    return std::make_pair(__malloc_calls, __free_calls);
}

void* malloc(size_t sz)
{
    static auto libc_malloc = reinterpret_cast<void *(*)(size_t)>(dlsym(RTLD_NEXT, "malloc"));

    if(!__register_calls) return libc_malloc(sz);

    __malloc_calls++;

    return libc_malloc(sz);
}

void free(void *p)
{
    static auto libc_free = reinterpret_cast<void(*)(void*)>(dlsym(RTLD_NEXT, "free"));

    if (!__register_calls)
        return libc_free(p);

    __free_calls++;

    fprintf(stderr, "free(%p)\n", p);

    libc_free(p);
}


int main()
{
    Eigen::VectorXd a(10);

    std::cerr << "const case: \n";

    reset_counters();
    {
        Eigen::Ref<const Eigen::VectorXd> aref = a;
    }
    auto ct = get_counters();

    std::cerr << "n malloc called " << ct.first << "\n";
    std::cerr << "n free   called " << ct.second << "\n";

    std::cerr << "non-const case: \n";
    reset_counters();
    {
        Eigen::Ref<Eigen::VectorXd> aref = a;
    }
    ct = get_counters();

    std::cerr << "n malloc called " << ct.first << "\n";
    std::cerr << "n free   called " << ct.second << "\n";
}
