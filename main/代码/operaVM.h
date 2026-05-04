#pragma once
#ifndef MY_MATH_H
#define MY_MATH_H

#include <iostream>
#include <vector>
#include <cmath>
#include <stdexcept>

using namespace std;

// 向量运算 
void vector_add(const vector<double>& a, const vector<double>& b, vector<double>& result) {
    if (a.size() != b.size()) {
        cerr << "vector_add: size mismatch" << endl;
        return;
    }
    result.resize(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        result[i] = a[i] + b[i];
}

void vector_sub(const vector<double>& a, const vector<double>& b, vector<double>& result) {
    if (a.size() != b.size()) {
        cerr << "vector_sub: size mismatch" << endl;
        return;
    }
    result.resize(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        result[i] = a[i] - b[i];
}

double vector_dot(const vector<double>& a, const vector<double>& b) {
    if (a.size() != b.size()) {
        cerr << "vector_dot: size mismatch" << endl;
        return 0.0;
    }
    double res = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
        res += a[i] * b[i];
    return res;
}

void vector_cross(const vector<double>& a, const vector<double>& b, vector<double>& result) {
    if (a.size() != 3 || b.size() != 3) {
        cerr << "vector_cross: need 3D vectors" << endl;
        return;
    }
    result.resize(3);
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
}

void print_vector(const vector<double>& v) {
    cout << "[";
    for (size_t i = 0; i < v.size(); ++i) {
        cout << v[i];
        if (i != v.size() - 1) cout << ", ";
    }
    cout << "]" << endl;
}

// 矩阵运算 
using Matrix = vector<vector<double>>;

void matrix_add(const Matrix& a, const Matrix& b, Matrix& result) {
    if (a.empty() || b.empty() || a.size() != b.size() || a[0].size() != b[0].size()) {
        cerr << "matrix_add: size mismatch" << endl;
        return;
    }
    result.resize(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        vector_add(a[i], b[i], result[i]);
}

void matrix_sub(const Matrix& a, const Matrix& b, Matrix& result) {
    if (a.empty() || b.empty() || a.size() != b.size() || a[0].size() != b[0].size()) {
        cerr << "matrix_sub: size mismatch" << endl;
        return;
    }
    result.resize(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        vector_sub(a[i], b[i], result[i]);
}

void matrix_multiply(const Matrix& a, const Matrix& b, Matrix& c) {
    if (a.empty() || b.empty() || a[0].size() != b.size()) {
        cerr << "matrix_multiply: dimension mismatch (a_cols != b_rows)" << endl;
        return;
    }
    size_t m = a.size();
    size_t n = b[0].size();
    size_t p = b.size(); // = a[0].size()
    c.assign(m, vector<double>(n, 0.0));
    for (size_t i = 0; i < m; ++i)
        for (size_t k = 0; k < p; ++k)
            for (size_t j = 0; j < n; ++j)
                c[i][j] += a[i][k] * b[k][j];
}

void matrix_T(const Matrix& a, Matrix& aT) {
    if (a.empty()) return;
    size_t rows = a.size();
    size_t cols = a[0].size();
    aT.assign(cols, vector<double>(rows, 0.0));
    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            aT[j][i] = a[i][j];
}

bool matrix_inverse(const Matrix& A, Matrix& inv) {
    if (A.empty() || A.size() != A[0].size()) {
        cerr << "matrix_inverse: not square" << endl;
        return false;
    }
    size_t n = A.size();
    // 构造增广矩阵 [A | I]
    Matrix aug(n, vector<double>(2 * n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j)
            aug[i][j] = A[i][j];
        aug[i][n + i] = 1.0;
    }
    // 高斯-约旦消元
    for (size_t i = 0; i < n; ++i) {
        // 选主元
        size_t max_row = i;
        for (size_t k = i + 1; k < n; ++k) {
            if (fabs(aug[k][i]) > fabs(aug[max_row][i]))
                max_row = k;
        }
        if (fabs(aug[max_row][i]) < 1e-12)
            return false; // 奇异矩阵
        swap(aug[i], aug[max_row]);
        // 主元归一化
        double pivot = aug[i][i];
        for (size_t j = i; j < 2 * n; ++j)
            aug[i][j] /= pivot;
        // 消去其他行
        for (size_t k = 0; k < n; ++k) {
            if (k != i) {
                double factor = aug[k][i];
                for (size_t j = i; j < 2 * n; ++j)
                    aug[k][j] -= factor * aug[i][j];
            }
        }
    }
    // 提取逆矩阵
    inv.assign(n, vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j)
            inv[i][j] = aug[i][n + j];
    return true;
}

void print_matrix(const Matrix& M) {
    if (M.empty()) {
        cout << "[]" << endl;
        return;
    }
    cout << "[";
    for (size_t i = 0; i < M.size(); ++i) {
        cout << "[";
        for (size_t j = 0; j < M[i].size(); ++j) {
            cout << M[i][j];
            if (j != M[i].size() - 1) cout << ", ";
        }
        cout << "]";
        if (i != M.size() - 1) cout << "," << endl << " ";
    }
    cout << "]" << endl;
}

#endif