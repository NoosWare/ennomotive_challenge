#ifndef LOGGER_HPP
#define LOGGER_HPP
#include <fstream>
#include <thread>
#include <mutex>
#include <exception>

/// @brief log data to a text file
class logger
{
public:

    logger(std::string fname)
    {
        file__.open(fname, std::ios::out | std::ios::app);
        if (!file__.is_open()) {
            throw std::runtime_error("failed to open logfile");
        }
    }

    template <typename T>
    void operator<<(T arg)
    {
        std::lock_guard<std::mutex> lock(mtx__);
        file__ << arg;
    }

private:
    std::ofstream file__;
    std::mutex mtx__;
};
#endif
