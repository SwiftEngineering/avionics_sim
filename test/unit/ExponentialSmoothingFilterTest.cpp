/**
 * @copyright   Copyright (c) 2021, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "Exponential_smoothing_filter.hpp"

#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace {
class sinewave {
  public:
    explicit sinewave(const double f) : m_omega_f(2.0 * M_PI * f) {
    }

    double operator()(const double t) {
        return sin(m_omega_f * t);
    }
  protected:
    // convert to angular frequency
    const double m_omega_f;
};

class squarewave {
  public:
    explicit squarewave(const double f) : m_f(f) {
    }

    double operator()(const double t) {
        double omega = 2.0 * M_PI * m_f * t;

        omega = fmod(omega, 2.0 * M_PI);

        if (omega < 0.0) {
            omega += 2.0 * M_PI;
        }

        if (omega >= M_PI) {
            return 1.0;
        }

        return -1.0;
    }
  protected:
    double m_f;
};

class step {
  public:
    explicit step(const double f) : m_f(f) {
    }

    double operator()(const double t) {
        double omega = 2.0 * M_PI * m_f * t;

        if (omega > M_PI / 2.0) {
            return 1;
        }

        return 0;
    }
  protected:
    double m_f;
};

template< class InputIt >
double calculate_rms(InputIt first, InputIt last) {
    const double count = std::distance(first, last);

    const double sum_sq = std::accumulate(first, last, 0.0, [](const double & a,
    const double & b) {
        return a + b * b;
    });

    return sqrt(sum_sq / count);
}

}  // namespace


TEST(Exponential_smoothing_filter_UnitTest, lpf_f0_500hz_3db_500hz) {
    // sample at 1MHz
    const double dT = 1e-6;

    // 0.002s per cycle, 2000 cycles per period, 4 periods
    const size_t nsamp = 8000;

    // 500 Hz 3db / 1e-6 dT
    avionics_sim::Exponential_smoothing_filter lpf(500.0, dT);

    // x0 / y0 is 500Hz sinewave
    const double f0 = 500.0;
    // filter input
    std::vector<double> x0(nsamp);
    // filter output
    std::vector<double> y0(nsamp);

    sinewave s0_gen(f0);

    for (size_t n = 0 ; n < nsamp; n++) {
        const double t = static_cast<double>(n) * dT;
        x0[n] = s0_gen(t);
        y0[n] = lpf.next_y_n(x0[n]);
    }

    // y0 tests
    // check gain
    EXPECT_DOUBLE_EQ(lpf.get_gain(f0), sqrt(2.0) / 2.0);
    // check phase
    EXPECT_DOUBLE_EQ(lpf.get_phase(f0), -45.0 * M_PI / 180.0);

    // skip first four periods
    auto x_start = x0.begin();
    std::advance(x_start, 4000);

    auto y_start = y0.begin();
    std::advance(y_start, 4000);

    double x0_rms = calculate_rms(x_start, x0.end());
    double y0_rms = calculate_rms(y_start, y0.end());
    double y0_gain = 20.0 * log10(y0_rms / x0_rms);

    EXPECT_DOUBLE_EQ(x0_rms, sqrt(2.0) / 2.0);
    EXPECT_NEAR(y0_rms, 0.5, 0.05);
    EXPECT_NEAR(y0_gain, -3.0, 0.05);
}

TEST(Exponential_smoothing_filter_UnitTest, lpf_f0_1000hz_3db_500hz) {
    // sample at 1MHz
    const double dT = 1e-6;

    // 0.001s per cycle, 1000 cycles per period, 8 periods
    const size_t nsamp = 8000;

    // 500 Hz 3db / 1e-6 dT
    avionics_sim::Exponential_smoothing_filter lpf(500.0, dT);

    // x0 / y1 is 1000kHz sinewave
    const double f0 = 1000.0;
    // filter input
    std::vector<double> x0(nsamp);
    // filter output
    std::vector<double> y0(nsamp);

    sinewave s0_gen(f0);

    for (size_t n = 0; n < nsamp; n++) {
        const double t = static_cast<double>(n) * dT;
        x0[n] = s0_gen(t);
        y0[n] = lpf.next_y_n(x0[n]);
    }

    // skip first 4 periods
    auto x_start = x0.begin();
    std::advance(x_start, 4000);

    auto y_start = y0.begin();
    std::advance(y_start, 4000);

    double x0_rms = calculate_rms(x_start, x0.end());
    double y0_rms = calculate_rms(y_start, y0.end());
    double y0_gain = 20.0 * log10(y0_rms / x0_rms);

    // y1 tets
    // check gain
    EXPECT_DOUBLE_EQ(lpf.get_gain(f0), 0.447213595499958);
    // check phase
    EXPECT_DOUBLE_EQ(lpf.get_phase(f0), -1.10714871779409);

    EXPECT_DOUBLE_EQ(x0_rms, sqrt(2.0) / 2.0);
    EXPECT_NEAR(y0_rms, 0.447213595499958 * sqrt(2.0) / 2.0, 0.01);
    EXPECT_NEAR(y0_gain, -6.98970004336019, 0.01);
}

TEST(Exponential_smoothing_filter_UnitTest, lpf_step_500hz_3db_500hz) {
    const size_t nsamp = 4000;

    // sample at 1MHz
    const double dT = 1e-6;

    // 500 Hz 3db / 1e-6 dT
    avionics_sim::Exponential_smoothing_filter lpf(500.0, dT);

    // x0 / y0 is 500Hz step
    const double f0 = 500.0;
    // filter input
    std::vector<double> x0(nsamp);
    // filter output
    std::vector<double> y0(nsamp);

    step step_gen(f0);

    for (size_t n = 0 ; n < nsamp; n++) {
        const double t = static_cast<double>(n) * dT;
        x0[n] = step_gen(t);
        y0[n] = lpf.next_y_n(x0[n]);
    }

    // the step starts at 0.0005s, so at 0.005s + tau it should be at ~63.2%
    double tau = 1.0 / (2.0 * M_PI * 500.0);
    double tau_0 = tau + 0.0005;
    size_t tau_0_n = round(tau_0 / dT);

    EXPECT_NEAR(y0[tau_0_n], 0.632, 0.001);
}

TEST(Exponential_smoothing_filter_UnitTest, lpf_square_500hz_3db_500hz) {
    const size_t nsamp = 8000;

    // sample at 1MHz
    const double dT = 1e-6;

    // 500 Hz 3db / 1e-6 dT
    avionics_sim::Exponential_smoothing_filter lpf(500.0, dT);

    // x0 / y0 is 500Hz squarewave
    const double f0 = 500.0;
    // filter input
    std::vector<double> x0(nsamp);
    // filter output
    std::vector<double> y0(nsamp);

    squarewave sq_gen(f0);

    for (size_t n = 0 ; n < nsamp; n++) {
        const double t = static_cast<double>(n) * dT;
        x0[n] = sq_gen(t);
        y0[n] = lpf.next_y_n(x0[n]);
    }

    // skip first 2 periods
    auto x_start = x0.begin();
    std::advance(x_start, 4000);

    auto y_start = y0.begin();
    std::advance(y_start, 4000);

    double x0_rms = calculate_rms(x_start, x0.end());
    double y0_rms = calculate_rms(y_start, y0.end());
    double y0_gain = 20.0 * log10(y0_rms / x0_rms);

    EXPECT_DOUBLE_EQ(x0_rms, 1.0);

    double tau = 1.0 / (2.0 * M_PI * 500.0);

    // rising
    double tau_0 = tau + 0.001;
    size_t tau_0_n = round(tau_0 / dT);
    EXPECT_NEAR(y0[tau_0_n], 0.632 * 2.0 - 1.0, 0.005);

    // falling
    double tau_1 = tau + 0.002;
    size_t tau_1_n = round(tau_0 / dT);
    EXPECT_NEAR(y0[tau_1_n], 1.0 - 0.368 * 2.0, 0.005);
}
