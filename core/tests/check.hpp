// Tiny test harness so the core doesn't need gtest and builds with a bare
// clang. Each test file is its own binary.
#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace check {
inline int failures = 0;
inline std::vector<std::pair<std::string, std::function<void()>>>& tests() {
    static std::vector<std::pair<std::string, std::function<void()>>> t;
    return t;
}
struct Reg {
    Reg(const char* n, std::function<void()> f) { tests().emplace_back(n, std::move(f)); }
};
inline int run() {
    for (auto& [name, fn] : tests()) {
        const int before = failures;
        fn();
        std::printf("%s %s\n", failures == before ? "[ OK ]" : "[FAIL]", name.c_str());
    }
    std::printf("%zu tests, %d failures\n", tests().size(), failures);
    return failures ? 1 : 0;
}
}  // namespace check

#define TEST(name) static void name(); static check::Reg reg_##name(#name, name); static void name()

#define EXPECT_TRUE(c)                                                          \
    do {                                                                        \
        if (!(c)) {                                                             \
            ++check::failures;                                                  \
            std::printf("  %s:%d: expected %s\n", __FILE__, __LINE__, #c);      \
        }                                                                       \
    } while (0)

#define EXPECT_NEAR(a, b, tol)                                                  \
    do {                                                                        \
        const double _a = (a), _b = (b), _tol = static_cast<double>(tol);      \
        if (std::fabs(_a - _b) > _tol) {                                        \
            ++check::failures;                                                  \
            std::printf("  %s:%d: %s=%g vs %s=%g (tol %g)\n", __FILE__, __LINE__, \
                        #a, _a, #b, _b, _tol);                                  \
        }                                                                       \
    } while (0)

#define TEST_MAIN() int main() { return check::run(); }
