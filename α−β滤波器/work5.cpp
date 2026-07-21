#include <iostream>
using namespace std;
#include <vector>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

//weizhi x^(n,n)=x^(n,n-1)+a(zn-x^(n,n-1))
//sudo x^`(n,n)=x^`(n,n-1)+b(zn-x^(n,n-1))/deta-t

int main(){
    double a = 0.50908;
    double b = 0.28501;
    int t = 5;
    int zn[10] = {30171,30353,30756,30799,31018,31278,31276,31379,31748,32175};
    int zhenzhi[10] = {30200,30400,30600,30800,31000,31200,31400,31600,31800,32000};

    double xnn[10], vnn[10];
    double xn1n[10], vn1n[10];

    for(int i = 0; i < 10; i++){
        if(i == 0){
            xnn[i] = zn[i];
            vnn[i] = 0;
        }else{
            xnn[i] = xn1n[i-1] + a * (zn[i] - xn1n[i-1]);
            vnn[i] = vn1n[i-1] + b * (zn[i] - xn1n[i-1]) / t;
        }
        xn1n[i] = xnn[i] + t * vnn[i];
        vn1n[i] = vnn[i];
    }
    for(int j = 0; j < 10; j++){
        cout << xnn[j] << " " ;
    }
    cout << endl;
    for(int j = 0; j < 10; j++){
        cout << vnn[j] << " " ;
    }
    cout << endl;
    for(int j = 0; j < 10; j++){
        cout << xn1n[j] << " ";
    }
    cout << endl;
    for(int j = 0; j < 10; j++){
        cout << vn1n[j] << " " ;
    }
    cout << endl;
 
    
    std::vector<double> idx = {1,2,3,4,5,6,7,8,9,10};
    std::vector<double> zhenzhi_vec(zhenzhi, zhenzhi+10);
    std::vector<double> zn_vec(zn, zn+10);
    std::vector<double> xnn_vec(xnn, xnn+10);

    plt::backend("Agg");
    plt::plot(idx, zhenzhi_vec, {{"label", "zhenzhi"}, {"color", "g"}, {"marker", "o"}});
    plt::plot(idx, zn_vec, {{"label", "zn"}, {"color", "r"}, {"marker", "o"}});
    plt::plot(idx, xnn_vec, {{"label", "xnn"}, {"color", "b"}, {"marker", "o"}});
    plt::legend();
    plt::save("filter_result.png");

    return 0;
}  