#include "tic_toc.hpp"

TicToc::TicToc()
{
    tic();
}

void TicToc::tic()
{
    this->start = std::chrono::steady_clock::now();
}

double TicToc::toc()
{
    end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_sec = end - start;
    return elapsed_sec.count();
}

uint64_t TicToc::timeStep()
{
    struct timespec tms;

    if (!timespec_get(&tms, TIME_UTC))
    {
        return 0;
    }

    uint64_t microsecond = tms.tv_sec * 1000000 + tms.tv_nsec / 1000;
    if (tms.tv_nsec % 1000 >= 500)
    {
        microsecond++;
    }

    int64_t millisecond = microsecond / 1000;
    return millisecond;
}