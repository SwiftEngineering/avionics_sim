#include "GaussianMarkov_noise.hpp"

// gtest
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <array>
#include <cmath>
#include <vector>

TEST(GaussianMarkov_noise_UnitTest, init) {
  avionics_sim::GaussianMarkov_noise gm(1, 1, 0);

  std::array<double, 4> vals_a;

  for (size_t i = 0; i < vals_a.size(); i++) {
    vals_a[i] = gm.update(0.001);
  }
}

/// test that a random seed gm can be reset
TEST(GaussianMarkov_noise_UnitTest, test_reset_default_seed) {
  avionics_sim::GaussianMarkov_noise gm(1, 1, 0);

  std::array<double, 4> vals_a;

  for (size_t i = 0; i < vals_a.size(); i++) {
    vals_a[i] = gm.update(0.001);
  }

  gm.reset();

  std::array<double, 4> vals_b;

  for (size_t i = 0; i < vals_b.size(); i++) {
    vals_b[i] = gm.update(0.001);
  }

  EXPECT_THAT(vals_a, ::testing::ContainerEq(vals_b));
}

/// test that a provided seed gm can be reset
TEST(GaussianMarkov_noise_UnitTest, test_reset_provided_seed) {
  std::array<uint32_t, 4> seed = {1, 2, 3, 4};
  avionics_sim::GaussianMarkov_noise gm(1, 1, 0, seed.data(), seed.size());

  std::array<double, 4> vals_a;

  for (size_t i = 0; i < vals_a.size(); i++) {
    vals_a[i] = gm.update(0.001);
  }

  gm.reset();

  std::array<double, 4> vals_b;

  for (size_t i = 0; i < vals_b.size(); i++) {
    vals_b[i] = gm.update(0.001);
  }

  EXPECT_THAT(vals_a, ::testing::ContainerEq(vals_b));
}


/// Test that same seeds make same sequences
TEST(GaussianMarkov_noise_UnitTest, test_seed_same_seq) {
  std::array<uint32_t, 4> seed = {1, 2, 3, 4};

  avionics_sim::GaussianMarkov_noise gm_a(1, 1, 0, seed.data(), seed.size());
  std::array<double, 4> vals_a;

  for (size_t i = 0; i < vals_a.size(); i++) {
    vals_a[i] = gm_a.update(0.001);
  }

  avionics_sim::GaussianMarkov_noise gm_b(1, 1, 0, seed.data(), seed.size());
  std::array<double, 4> vals_b;

  for (size_t i = 0; i < vals_b.size(); i++) {
    vals_b[i] = gm_b.update(0.001);
  }

  EXPECT_THAT(vals_a, ::testing::ContainerEq(vals_b));
}

/// Test that different seeds make different sequences
TEST(GaussianMarkov_noise_UnitTest, test_seed_diff_seq) {
  std::array<uint32_t, 4> seed_a = {1, 2, 3, 4};
  avionics_sim::GaussianMarkov_noise gm_a(1, 1, 0, seed_a.data(), seed_a.size());
  std::array<double, 4> vals_a;

  for (size_t i = 0; i < vals_a.size(); i++) {
    vals_a[i] = gm_a.update(0.001);
  }

  std::array<uint32_t, 4> seed_b = {5, 6, 7, 8};
  avionics_sim::GaussianMarkov_noise gm_b(1, 1, 0, seed_b.data(), seed_b.size());
  std::array<double, 4> vals_b;

  for (size_t i = 0; i < vals_b.size(); i++) {
    vals_b[i] = gm_b.update(0.001);
  }

  EXPECT_THAT(vals_a, ::testing::Not(::testing::ContainerEq(vals_b)));
}

/// Test that walk distance scales with sigma
TEST(GaussianMarkov_noise_UnitTest, test_sigma) {
  const double dt = 0.01;

  avionics_sim::GaussianMarkov_noise gm_a(1, 1, 0);
  double last_a = gm_a.update(dt);
  double distance_a = 0.0;

  for (size_t i = 0; i < 100000; i++) {
    const double next = gm_a.update(dt);
    const double dx = next - last_a;
    last_a = next;

    distance_a += dx * dx;
  }

  avionics_sim::GaussianMarkov_noise gm_b(1, 10, 0);
  double last_b = gm_b.update(dt);
  double distance_b = 0.0;

  for (size_t i = 0; i < 100000; i++) {
    const double next = gm_b.update(dt);
    const double dx = next - last_b;
    last_b = next;

    distance_b += dx * dx;
  }

  //over enough generations, distance should approach sigma^2 faster
  EXPECT_NEAR(distance_b / distance_a, 100.0, 2.5);
}

/// Test that distance scales with tau
TEST(GaussianMarkov_noise_UnitTest, test_tau) {
  const double dt = 0.01;

  avionics_sim::GaussianMarkov_noise gm_a(1, 1, 0);
  double len_a = 0.0;
  double last = gm_a.update(dt);

  for (size_t i = 0; i < 100000; i++) {
    double next = gm_a.update(dt);
    const double dx = next - last;
    len_a += dx * dx;
  }

  avionics_sim::GaussianMarkov_noise gm_b(0.1, 1, 0);
  double len_b = 0.0;
  last = gm_b.update(dt);

  for (size_t i = 0; i < 100000; i++) {
    double next = gm_b.update(0.01);
    len_b += std::abs(next - last);
  }

  //dist b should move faster than dist a
  EXPECT_GT(len_b, len_a);
}
