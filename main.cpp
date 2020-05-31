#include <iostream>
#include <vector>
#include "Drawer.h"

/* 乱数初期化 */
cv::RNG rng(cv::getTickCount());


struct Point
{
    double x, y;
    double getX() {
        return x;
    };
    double getY() {
        return y;
    };
};

std::pair<double, double> LSM(std::vector<Point> &data)
{
    double sum_xy = 0.0;
    double sum_x  = 0.0;
    double sum_y  = 0.0;
    double sum_x2 = 0.0;
    int n = data.size();

    for (int i = 0; i < n; i++) {
        sum_xy += data[i].x * data[i].y;
        sum_x  += data[i].x;
        sum_y  += data[i].y;
        sum_x2 += data[i].x * data[i].x;
    }

    double z = n * sum_x2 - sum_x * sum_x;

    double est_a = (n * sum_xy - sum_x * sum_y)/z;
    double est_b = (sum_x2 * sum_y - sum_xy * sum_x)/z;

    return std::make_pair(est_a, est_b);
}

std::pair<double, double> RANSAC(std::vector<Point> &data)
{
    int iter = 1000;                // 繰り返し推定数
    int num_sample = 50;            // ランダムサンプリング数
    double ev_min = 999999999;      // 最小評価値
    std::pair<double, double> min;  // 最小評価値のパラメータ推定値

    for (int k = 0; k < iter; k++) {
        std::vector<Point> sample_data_init = data;
        std::cerr << k << std::endl;
        std::vector<Point> sample_data = sample_data_init;
        std::vector<Point> choise_data;

        // データからランダムサンプリング
        for (int i = 0; i < num_sample; i++) {
            int key = rng.uniform(0, sample_data.size());
            Point sample = sample_data[key];
            choise_data.emplace_back(sample);
            sample_data.erase(sample_data.begin() + key);
        }

        // サンプルデータから最小二乗法でパラメータを推定
        std::pair<double, double> est = LSM(choise_data);

        // 推定したパラメータ直線による誤差を求める
        double ev = 0.0;
        double a = est.first;
        double b = est.second;
        for (Point pt: sample_data_init) {
            ev += ((a * pt.x + b) - pt.y) *((a * pt.x + b) - pt.y);
        }
        ev /= sample_data_init.size();

        // ハズレ値を探し，sample_data_initから除去する
        std::vector<Point> tmp;
        for (Point pt: sample_data_init) {
            double eval = ((a * pt.x + b) - pt.y) * ((a * pt.x + b) - pt.y);
            if (eval < 2.0*ev) {
                tmp.emplace_back(pt);
            }
        }
        sample_data_init = tmp;

        // ハズレ値を除去したデータから評価値を求める
        for (Point pt: sample_data_init) {
            ev += ((a * pt.x + b) - pt.y) *((a * pt.x + b) - pt.y);
        }
        ev /= sample_data_init.size();

        if (ev < ev_min) {
            ev_min = ev;
            min = est;
        }
    }
    std::cerr << min.first << " " << min.second << " " << ev_min << std::endl;
    return min;
}

int main(int argc, char *argv[])
{
    double x_min = -4.0;
    double x_max =  4.0;
    double x_range = x_max - x_min;
    double y_range = x_range * 9/16;
    double y_min = -y_range/2;
    double y_max =  y_range/2;

    Drawer dr;
    dr.setImgWidth(x_range);
    dr.setImgHight(y_range);
    dr.setOriginXfromLeft(-x_min);
    dr.setOriginYfromBottom(-y_min);

    // MODEL
    // y = ax + b

    // データセットの作成
    double a = 1.0;
    double b = 0.0;
    std::vector<Point> data;
    Point pt;
    for (int k = 0; k < 10; k++) {
        for (double x = x_min; x < x_max; x += 0.001) {
            pt.x = x;
            pt.y = a * x + b + rng.gaussian(0.2);
            data.emplace_back(pt);
        }
    }

    // ハズレ値を含めたデータセットの作成
    std::vector<Point> data_addNoise;
    data_addNoise = data;   // データセットをコピー
    dr.setLineColor(cv::Scalar(0, 200, 200));
    double ratio = 0.1;     // データセットの内，ハズレ値に置き換える割合
    std::vector<bool> data_isNoise(data_addNoise.size(), false);
    for (int i = 0; i < data_addNoise.size()*ratio; i++) {
        int k = rng.uniform(0, data_addNoise.size());
        if (!data_isNoise[k]) {
            data_isNoise[k] = true;
            data_addNoise[k].y = y_max;
            dr.circle(data_addNoise[k].x, data_addNoise[k].y, 0.02);
        } else {
            i--;
        }
    }

    for (Point p: data) {
        dr.drawing<Point>(p);
    }

    dr.show();
    cv::waitKey(0);

    // 最小２乗法
    std::pair<double, double> est_normal = LSM(data);
    std::pair<double, double> est_noisy = LSM(data_addNoise);

    dr.setLineWidth(5);

    dr.setLineColor(cv::Scalar(200, 0, 0));
    dr.line(0.0, est_normal.second, atan2(est_normal.first, 1.0));

    dr.setLineColor(cv::Scalar(0, 0, 200));
    dr.line(0.0, est_noisy.second, atan2(est_noisy.first, 1.0));

    dr.show();
    cv::waitKey(0);

    // RANSAC
    std::pair<double, double> est_ransac = RANSAC(data_addNoise);
    dr.setLineColor(cv::Scalar(200, 0, 200));
    dr.line(0.0, est_ransac.second, atan2(est_ransac.first, 1.0));

    dr.show();
    cv::waitKey(0);

    dr.imgWrite();

    return 0;
}

