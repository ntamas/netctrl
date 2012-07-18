/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef _WALKER_SAMPLING_HPP
#define _WALKER_SAMPLING_HPP

#include <cstdlib>
#include <cstring>
#include <numeric>
#include <vector>

/**
 * Efficient random sampling with replacement using Walker's alias method.
 *
 * Make sure to initialize the random number generator by calling \c srand
 * before using this class!
 */
class WalkerSampling {

private:
    /**
     * Vector specifying the index of the "other" element that is contained
     * in a given bin.
     */
    std::vector<long int> m_indexes;

    /**
     * Vector specifying the probability of drawing the "own" and not the "other"
     * element from a given bin.
     */
    std::vector<double> m_probs;

public:

    /**
     * Constructs a new, uninitialized sampler.
     *
     * You must initialize the sampler by calling \c initialize() before
     * using it.
     */
    WalkerSampling() : m_indexes(), m_probs() {}

    /**
     * Constructs a new sampler whose weights are initialized from the
     * given container.
     *
     * \param  begin  the beginning of the container
     * \param  end    the end of the container
     */
    template <typename InputIterator>
    WalkerSampling(InputIterator begin, InputIterator end) : m_indexes(), m_probs() {
        initialize(begin, end);
    }

    /**
     * Initializes the sampler from the weights in the given container.
     *
     * \param  begin  the beginning of the container
     * \param  end    the end of the container
     */
    template <typename InputIterator>
    void initialize(InputIterator begin, InputIterator end);

    /**
     * Draws a given number of samples from the sampler and writes them to
     * an iterator.
     *
     * \param  n   the number of samples to draw
     * \param  it  an output iterator to write the results to.
     */
    template <typename OutputIterator>
    void sample(OutputIterator it, size_t n=1) const;
};


template <typename InputIterator>
void WalkerSampling::initialize(InputIterator begin, InputIterator end) {
    InputIterator it;
    const double sum = std::accumulate(begin, end, 0.0);
    size_t n = end-begin, i;
    std::vector<long int> shortIndexes;
    std::vector<long int> longIndexes;
    std::vector<double>::iterator it2;

    // Initialize m_probs and m_indexes
    m_probs.resize(n);
    m_indexes.resize(n);
    std::fill(m_indexes.begin(), m_indexes.end(), -1);

    // Normalize the probabilities
    for (it = begin, it2 = m_probs.begin(); it != end; ++it, ++it2) {
        *it2 = (*it / sum) * n;
    }

    // Initialize shortIndexes and longIndexes
    for (i = 0, it2 = m_probs.begin(); it2 != m_probs.end(); ++i, ++it2) {
        if (*it2 < 1) {
            shortIndexes.push_back(i);
        } else if (*it2 > 1) {
            longIndexes.push_back(i);
        }
    }

    // Prepare the tables
    while (!shortIndexes.empty() && !longIndexes.empty()) {
        long int shortIndex = shortIndexes.pop_back();
        long int longIndex = longIndexes.back();
        m_indexes[shortIndex] = longIndex;
        m_probs[longIndex] -= (1 - m_probs[shortIndex]);
        if (m_probs[longIndex] < 1) {
            shortIndexes.push_back(longIndex);
            longIndexes.pop_back();
        }
    }
}

template <typename OutputIterator>
void WalkerSampling::sample(OutputIterator it, size_t n) const {
    double u;
    long int j;
    size_t m = m_probs.size();

    while (n > 0) {
        u = rand() / static_cast<double>(RAND_MAX);
        j = rand() % m;
        *it = (u < m_probs[j]) ? j : m_indexes[j];
        n--; ++it;
    }
}

#endif       // _WALKER_SAMPLING_HPP
