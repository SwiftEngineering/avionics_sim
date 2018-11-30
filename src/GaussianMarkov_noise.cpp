#include "swift_common/GaussianMarkov_noise.hpp"

#include <algorithm>
#include <functional>

GaussianMarkov_noise::GaussianMarkov_noise(const double tau, const double sigma, const double initial_output)
{
	//arbitrarily pick 16 as length of seed
	//no way to gaurntee mt19937 state is filled, so no point giving 624 elements
	std::vector<std::uint32_t> seed_data(16);

	std::random_device rd;
	std::generate_n(seed_data.data(), seed_data.size(), std::ref(rd));

	//delegate initialization
	initialize(tau, sigma, initial_output, seed_data.data(), seed_data.size());
}

GaussianMarkov_noise::GaussianMarkov_noise(const double tau, const double sigma, const double initial_output, const uint32_t seed[], const size_t seed_len)
{
	//delegate initialization
	initialize(tau, sigma, initial_output, seed, seed_len);
}

void GaussianMarkov_noise::initialize(const double tau, const double sigma, const double initial_output, const uint32_t seed[], const size_t seed_len)
{
	m_tau = tau;
	m_sigma = sigma;

	m_initial_output = initial_output;
	m_last_output = m_initial_output;

	m_normal_dist = std::normal_distribution<double>(0.0, 1.0);

	//init rng
	std::seed_seq seed_gen(seed, seed + seed_len);
	m_rand_gen.seed(seed_gen);

	//save initial state
	m_init_rng_state << m_rand_gen;	
}

void GaussianMarkov_noise::reset()
{
	m_last_output = m_initial_output;

	//reset the rng
	m_init_rng_state >> m_rand_gen;
	m_normal_dist.reset();
}

double GaussianMarkov_noise::update(const double dT)
{
	if(m_sigma == 0.0)
	{
		return 0.0;
	}
	
	const double alpha = exp(-dT / m_tau);

	const double y_n = alpha * m_last_output + (1.0 - alpha) * m_sigma * m_normal_dist(m_rand_gen);

	m_last_output = y_n;

	return y_n;
}