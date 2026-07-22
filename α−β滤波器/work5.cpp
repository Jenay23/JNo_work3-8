#include <vector>
#include <array>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

//位置 x^(n,n)=x^(n,n-1)+alpha(zn-x^(n,n-1))
//速度 v^(n,n)=v^(n,n-1)+beta(zn-x^(n,n-1))/dt

//zn-测量值 xnn-估计的当前位置 vnn-估计的当前速度 xn1n-预测的下一时刻位置 vn1n-预测的下一时刻速度

int main(){
    constexpr double alpha = 0.50908;
    constexpr double beta = 0.28501;
    constexpr double dt = 5.0;
    constexpr int n = 10;

    std::array<double, n> zn = {30171, 30353, 30756, 30799, 31018, 31278, 31276, 31379, 31748, 32175};
    std::array<double, n> truth = {30200, 30400, 30600, 30800, 31000, 31200, 31400, 31600, 31800, 32000};
    std::array<double, n> xnn{}, vnn{}, xn1n{}, vn1n{};

    xnn[0] = zn[0];
    vnn[0] = 0;
    xn1n[0] = xnn[0] + dt * vnn[0];
    vn1n[0] = vnn[0];

    for(int i = 1; i < n; i++){
        double residual = zn[i] - xn1n[i-1];
        xnn[i] = xn1n[i-1] + alpha * residual;
        vnn[i] = vn1n[i-1] + beta * residual / dt;
        xn1n[i] = xnn[i] + dt * vnn[i];
        vn1n[i] = vnn[i];
    }

    auto to_vec = [](const auto& arr){
        return std::vector<double>(arr.begin(), arr.end());
    };

    std::vector<double> idx = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    plt::backend("Agg");
    plt::figure_size(800, 600);
    plt::plot(idx, to_vec(truth), {{"label", "truth_value"}, {"color", "g"}, {"marker", "o"}});
    plt::plot(idx, to_vec(zn), {{"label", "measured_value"}, {"color", "r"}, {"marker", "o"}});
    plt::plot(idx, to_vec(xnn), {{"label", "current_estimate"}, {"color", "b"}, {"marker", "o"}});
    plt::title("Alpha-Beta Filter");
    plt::legend();
    plt::save("filter_result.png");

    return 0;
}
