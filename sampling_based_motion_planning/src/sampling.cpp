#include <iostream>
#include <ctime>
#include <map>
#include <math.h>
#include "sampling.hpp"
#include "halton.hpp"
#include <pthread.h>

struct thread_args_t
{
    std::vector<cv::Point2d> points;
    int N;
    int start;
};

void* thread_body(void* _args){
    thread_args_t *args;
    args = (thread_args_t*)_args;
    Sampling s;
    args->points = s.generate_N_Halton_points(args->N, args->start);
    return 0;
}

/*!
  * @brief Generate one random cv::Point2d point
  * @param[in]  const int max_X     max value for x
  * @param[in]  const int max_Y     max value for y
  * @return[cv::Point2d] the random cv::Point2d point
  */
cv::Point2d Sampling::generate_rnd_point(const int max_X, const int max_Y) const{
    double x = double(double(rand()) * double(max_X/double(RAND_MAX)));
    double y = double(double(rand()) * double(max_Y/double(RAND_MAX)));
    return cv::Point2d(x,y);
}

/*!
  * @brief Generate N random cv::Point2d points
  * @param[in]  const int N         Number of points
  * @param[in]  const int max_X     max value for x
  * @param[in]  const int max_Y     max value for y
  * @return[std::vector<cv::Point2d>] A vector with N random points
  */
std::vector<cv::Point2d> Sampling::generate_N_rnd_points(const int N, const int max_X, const int max_Y) const{
    std::vector<cv::Point2d> rnd_points;
    for(int i = 0; i < N; i++){
        rnd_points.emplace_back(this->generate_rnd_point(max_X, max_Y));
    }
    return rnd_points;
}

/*!
  * @brief Generate N Halton cv::Point2d points
  * @param[in]  const int N         Number of points
  * @param[in]  const int start     Halton start value
  * @return[std::vector<cv::Point2d>] A vector with N Halton points
  */
std::vector<cv::Point2d> Sampling::generate_N_Halton_points(const int N, const int start) const{
    
    double *num_halton[N];
    std::vector<cv::Point2d> points;
    for (int i = 0; i < N; i++)
    {
        num_halton[i] = halton(start + i + 1, 2);
        points.push_back(cv::Point2d(num_halton[i][0], num_halton[i][1]));
    }
    for (int i = 0; i < N; i++)
        delete num_halton[i];

    return points;
}

/*!
  * @brief Generate N Halton cv::Point2d points, using multiple threads
  * @param[in]  const int N             Number of points
  * @param[in]  const int N_jobs = 1    Number of threads
  * @return[std::vector<cv::Point2d>] A vector with N Halton points
  */
std::vector<cv::Point2d> Sampling::generate_N_Halton_points_multithread(const int N, const int N_jobs = 1) const{
    
    std::vector<cv::Point2d> points;

    pthread_t* threads = new pthread_t[N_jobs];

    thread_args_t* thread_args = new thread_args_t[N_jobs];

    /*** For logarithmic decomposition **/
    // unsigned int last_start = 0;
    // for(int i = 0; i < N_jobs; i++){
    //     if(i == N_jobs-1){
    //         thread_args[i].N = N - last_start;
    //         thread_args[i].start = last_start;
    //     }else{
    //         thread_args[i].N = int(N * (1/(pow(3,i))-(1/pow(3,i+1))));
    //         thread_args[i].start = last_start;
    //         last_start = thread_args[i].start + thread_args[i].N;
    //     }
    /************************************/
    /*** For linear decomposition **/
    for(int i = 0; i < N_jobs; i++){
        thread_args[i].start = int(N/N_jobs)*i;
        if(i == N_jobs-1){
            thread_args[i].N = int(N/N_jobs) + N%N_jobs;
        }else{
            thread_args[i].N = int(N/N_jobs);
        }
    /*******************************/
        int ret = pthread_create(&threads[i], NULL, thread_body, &thread_args[i]);
        if(ret){
            std::cout<<"[ERROR] Sampling::generate_N_Halton_points_multithread unable to create thread, error :"<<ret<<std::endl;
            return points;
        }
    }
    
    for(int i = 0; i < N_jobs; i++){
        int ret = pthread_join(threads[i], NULL);
        if(ret){
            std::cout<<"[ERROR] Sampling::generate_N_Halton_points_multithread unable to join thread error :"<<ret<<std::endl;
            return points;
        }else{
            points.insert(points.end(), thread_args[i].points.begin(), thread_args[i].points.end());
        }
    }

    delete[] threads;
    delete[] thread_args;

    return points;
}

Sampling::Sampling(){
    srand((unsigned) time(0));
}
Sampling::~Sampling(){

}