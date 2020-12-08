#ifndef __SAMPLING_H__
#define __SAMPLING_H__

    #include <opencv2/opencv.hpp>
    #include <vector>

    class Sampling{
    private:
        /* data */
    public:
        cv::Point2d generate_rnd_point(const int max_X, const int max_Y);
        std::vector<cv::Point2d> generate_N_rnd_points(const int N, const int max_X, const int max_Y);
        std::vector<cv::Point2d> generate_N_Halton_points(const int N);
        Sampling(/* args */);
        ~Sampling();
    };


#endif
