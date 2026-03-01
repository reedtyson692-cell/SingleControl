#include <math.h>
#include <vector>
#include <iostream>
//矩阵运算
using namespace std;
const double epsilon = 1e-12;  //小于该数判断为0

vector<vector<double>> creatmatrix(int h, int l);
vector<vector<double>> plus_mat(const vector<vector<double>>& A, const vector<vector<double>>& B);
vector<vector<double>> minus_mat(const vector<vector<double>>& A, const vector<vector<double>>& B);
vector<vector<double>> multiply(const vector<vector<double>>& A, const vector<vector<double>>& B);
vector<vector<double>> multiply_num(const vector<vector<double>>& A, double num);
vector<vector<double>> matrix_overlaying_below(const vector<vector<double>>& A, const vector<vector<double>>& B);
vector<vector<double>> matrix_overlaying_beside(const vector<vector<double>>& A, const vector<vector<double>>& B);
vector<vector<double>> trans(const vector<vector<double>>& A);
vector<vector<double>> inverse(const vector<vector<double>>& A);
void show_matrix(const vector<vector<double>>& A);