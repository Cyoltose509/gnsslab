//
// Created by shjzh on 2024/12/24.
//

/**
 * Copyright:
 *  This software is licensed under the Mulan Permissive Software License, Version 2 (MulanPSL-2.0).
 *  You may obtain a copy of the License at:http://license.coscl.org.cn/MulanPSL2
 *  As stipulated by the MulanPSL-2.0, you are granted the following freedoms:
 *      To copy, use, and modify the software;
 *      To use the software for commercial purposes;
 *      To redistribute the software.
 *
 * Author: shoujian zhang，shjzhang@sgg.whu.edu.cn， 2024-10-10
 *
 * References:
 * 1. Sanz Subirana, J., Juan Zornoza, J. M., & Hernández-Pajares, M. (2013).
 *    GNSS data processing: Volume I: Fundamentals and algorithms. ESA Communications.
 * 2. Eckel, Bruce. Thinking in C++. 2nd ed., Prentice Hall, 2000.
 */


#include <iostream>
#include <Eigen/Dense>    // Eigen头文件，<Eigen/Dense>包含Eigen库里面所有的函数和类

int main() {
    //2.1
    Eigen::MatrixXd m(2, 2); // MatrixXd 表示的是动态数组，初始化的时候指定数组的行数和列数

    m(0, 0) = 3; //m(i,j) 表示第i行第j列的值，这里对数组进行初始化
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    std::cout << "原矩阵：" << std::endl << m << std::endl;
    std::cout << "矩阵的二次方" << std::endl << m*m << std::endl;
    std::cout << "矩阵求逆" << std::endl << m.inverse() << std::endl;
}
