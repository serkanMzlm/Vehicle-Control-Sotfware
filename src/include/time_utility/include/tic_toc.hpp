#ifndef __TIC_TOC_HPP__
#define __TIC_TOC_HPP__

#include <ctime>
#include <cstdlib>
#include <chrono>

/**
 * @class TicToc
 * @brief A simple timer class for measuring time intervals.
 */
class TicToc
{
public:
    /**
     * @brief Constructor that initializes the timer.
     */
    TicToc();

    /**
     * @brief Starts or restarts the timer.
     */
    void tic();

    /**
     * @brief Stops the timer and returns the elapsed time in milliseconds.
     *
     * @return Elapsed time in milliseconds.
     */
    double toc();

    /**
     * @brief Returns the current time in milliseconds using the system clock.
     *
     * @return The current time in milliseconds.
     */
    uint64_t timeStep();

private:
    std::chrono::time_point<std::chrono::steady_clock> start;
    std::chrono::time_point<std::chrono::steady_clock> end;
};

#endif