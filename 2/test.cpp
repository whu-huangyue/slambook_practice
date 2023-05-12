// #include <iostream>

// using namespace std;

// #include <ctime>
// #include <Eigen/Core>
// #include <Eigen/Dense>
// #include <iomanip>

// using namespace Eigen;

// #define MATRIX_SIZE 100

// // #include <iostream>

// // using namespace std;
// // using namespace Eigen;

// // #include <ctime>
// // #include <Eigen/Core>
// // #include <Eigen/Dense>

// // #define MATRIX_SIZE 100

// int main(int argc, char **argv) {
    
//     // //建立一个动态矩阵A
//     // MatrixXd A;
//     // //建立一个100*100的随机矩阵A                                      
//     // A = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE); 

//     // //保证A为正定矩阵
//     // A = A * A.transpose();    
                       
//     // //建立一个动态矩阵B
//     // Matrix<double, Dynamic, 1> B;   
//     // //建立一个100*1的随机矩阵B
//     // B = MatrixXd::Random(MATRIX_SIZE, 1); 
          
//     // //建立一个动态矩阵X
//     // Matrix<double, Dynamic, 1> X;
//     // //建立一个100*1的随机矩阵X                    
//     // X = MatrixXd::Random(MATRIX_SIZE, 1);            
    
//     // //QR分解
//     // X = A.colPivHouseholderQr().solve(B);            
//     // cout << "QR: X = " << X.transpose() << endl;

//     // //cholesky分解
//     // X = A.ldlt().solve(B);    
                       
//     // cout << "cholesky: X = " << X.transpose() << endl;
    
//     // return 0;

//     MatrixXd A;
//     A = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
//     A = A * A.transpose();

//     Matrix<double, Dynamic, 1> B;
//     B = MatrixXd::Random(MATRIX_SIZE, 1);

//     Matrix<double, Dynamic, 1> X;
//     X = MatrixXd::Random(MATRIX_SIZE, 1);

//     time_t start = 0, end = 0;

//     time(&start);
//     X = A.colPivHouseholderQr().solve(B);
//     time(&end);
//     cout << "QR: X = " << X.transpose()  << endl;
//     cout << "运行时间为： " << setprecision(10) << (end - start) << "秒！" << endl;

//     time(&start);
//     X = A.ldlt().solve(B);
//     time(&end);
//     cout << "cholesky: X = " << X.transpose() << endl;
//     cout << "运行时间为： " << setprecision(10) << (end - start) << "秒！" << endl;

//     return 0;
// }

#include <iostream>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

int main(int argc, char ** argv){

    //创建小萝卜1和2的位姿q1和q2
    Quaterniond q1(0.55, 0.3, 0.2, 0.2), q2(-0.1, 0.3, -0.7, 0.2);
    //四元数归一化
    q1.normalize();
    q2.normalize();
    
    //平移向量t1和t2
    Vector3d t1(0.7, 1.1, 0.2), t2(-0.1, 0.4, 0.8);
    //p1坐标
    Vector3d p1(0.5, -0.1, 0.2);  
    
    //变换矩阵Tc1w和Tc2w
    Isometry3d Tc1w(q1), Tc2w(q2);
    Tc1w.pretranslate(t1);
    Tc2w.pretranslate(t2);
    
    //计算p2
    Vector3d p2 = Tc2w*Tc1w.inverse()*p1;
    cout << p2.transpose() << endl;
    
    return 0;
}