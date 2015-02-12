/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef _RANDOM_HPP
#define _RANDOM_HPP

/**
 * Generates a random number between a lower bound (inclusive) and an upper
 * bound (exclusive).
 */
template <typename T=double>
struct rng_uniform {
    typedef T result_type;

    T m_min;
    T m_max;
    T m_ratio;

    rng_uniform(T min=0, T max=1) : m_min(min), m_max(max) {
        m_ratio = (m_max - m_min) / static_cast<T>(RAND_MAX);
    }

    T operator()() const {
        return rand() * m_ratio + m_min;
    }
};

#endif       // _RANDOM_HPP
