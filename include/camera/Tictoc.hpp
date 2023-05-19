#pragma once
#ifndef FPS_COUNT_HPP
#define FPD_COUNT_HPP

#include <iostream>
#include <chrono>
#include <fmt/core.h>
#include <thread>
class Tictok{
public:
    explicit Tictok(){
        start_time = std::chrono::steady_clock::now();
        fps = fps_count = 0;
    };
    double this_time(){
        auto t1 = std::chrono::steady_clock::now();
        double time_stamp = std::chrono::duration<double, std::milli>(t1 - start_time).count();//ms
        return time_stamp;
    };
    int fps_calculate(){
        fps++;
        auto time_ = this_time();
        if (time_ - fps_count >= 1000) {
//            fmt::print("thread process  FPS : {}",std::this_thread::get_id());
            std::cout<< std::this_thread::get_id()<<" "<< "process FPS : " <<fps<<std::endl;
            fps = 0;
            fps_count = time_;
        }

    }

private:

    int fps, fps_count;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
};

#endif