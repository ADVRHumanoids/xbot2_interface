#include <coroutine>
#include <utility>
#include <optional>
#include <iostream>

struct R
{

    struct promise_type;

    using coro_handle_type = std::coroutine_handle<promise_type>;

    struct promise_type
    {
        promise_type()
        {
            std::cerr << __PRETTY_FUNCTION__ << std::endl;
        }

        std::optional<int> i;

        std::suspend_always initial_suspend() { return {}; }

        std::suspend_always final_suspend() noexcept { return {}; }

        R get_return_object()
        {
            R ret;
            ret._h = coro_handle_type::from_promise(*this);
            return std::move(ret);
        }

        void unhandled_exception()
        {

        }

        void return_value(int i)
        {
            std::cerr << __PRETTY_FUNCTION__ << std::endl;
            this->i = i;
        }

        std::suspend_always yield_value(int i)
        {
            std::cerr << __PRETTY_FUNCTION__ << std::endl;
            this->i = i;
            return {};
        }
    };

    std::optional<int> run()
    {
        _h.promise().i = std::nullopt;
        _h.resume();
        return _h.promise().i;
    }

    R() { std::cerr << __PRETTY_FUNCTION__ << std::endl; }
    R(R&&) = default;
    R(const R&) = delete;
    R operator=(const R&) = delete;

private:

    coro_handle_type _h;

};

R example_coro(int i0)
{
    co_yield 3+i0;
    co_return 1+i0;
}

int main()
{
    auto c = example_coro(5);
    std::cerr << c.run().value() << std::endl;
    std::cerr << c.run().value() << std::endl;
    std::cerr << c.run().value() << std::endl;
    std::cerr << c.run().value() << std::endl;
}
