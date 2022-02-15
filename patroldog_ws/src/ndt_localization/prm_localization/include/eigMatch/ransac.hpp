
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>

#include <algorithm>
#include <limits.h>


namespace wut_slam {

    using namespace std;
    using namespace Eigen;


    struct ransac {
    public:
        typedef pcl::PointXYZ PointT;

        ransac() {
            maxIter = 200;
        }

        std::vector<bool> compare(Eigen::MatrixXd &srcPts, Eigen::MatrixXd &tarPts, double threshold) {
            if (srcPts.rows() != 3 || tarPts.rows() != 3 || srcPts.cols() != tarPts.cols()) {
                std::cout << "error" << std::endl;
            }

            int N = srcPts.cols();
            int M = srcPts.rows();
            // Main loop
            //---------------------------------------------------------------
            // initializations
            int iter = 0; //number of iterations
            int bestSz = 3; //initial threshold for inlier size of a better model
            vector<int> randIdx(M, 0);
            //vector<double> x(3, 0), y_hat(3, 0), y(3, 0);
            vector<bool> thisCS(N, false);

            vector<bool> thisCS_2(N, false);
            Eigen::Vector3d y_hat, y;
            Eigen::VectorXd x;
            x.resize(4);

            srand((unsigned) time(NULL)); //set the seed to the current time
            //srand((unsigned)time(0)); //set the seed to 0

            while (iter <= maxIter) {
                randIdx = generate(N, M);
                //    std::cout<<"randIdx size : "<<randIdx.size()<<"  "<<std::endl;
                //      randIdx[0]=14;
                //      randIdx[1]=21;
                //     randIdx[2]=7;
                double eps;
                Eigen::Matrix4d T = estimateTform(srcPts, tarPts, randIdx, eps);

                // to get size of the consensus set
                int inlierSz = 0;
                /*      if(iter==0)
                      {
                          std::cout<<"T in ransac : "<<T<<std::endl;
                      }
             */         for (int i = 0; i < N; i++) {
                    x(0) = srcPts(0, i);
                    x(1) = srcPts(1, i);
                    x(2) = srcPts(2, i);
                    x(3) = 1;
                    y(0) = tarPts(0, i);
                    y(1) = tarPts(1, i);
                    y(2) = tarPts(2, i);

                    /*           y_hat[0] = T(0,0) * x[0] + T(1,0) * x[1] + T(2,0) * x[2] + T(3,0);
                               y_hat[1] = T(0,1) * x[0] + T(1,1) * x[1] + T(2,1) * x[2] + T(3,1);
                               y_hat[2] = T(0,2) * x[0] + T(1,2) * x[1] + T(2,2) * x[2] + T(3,2);
                      */
                    y_hat = T.block<3, 4>(0, 0) * x;

                    double thisErr = ((y - y_hat).norm()) * ((y - y_hat).norm());

                    thisCS[i] = 0;

                    if (thisErr < threshold) {
                        inlierSz++;
                        thisCS[i] = 1;

                    }
                }

                if (inlierSz > bestSz) {
                    // std::cout<<"bestsz "<<inlierSz<<std::endl;
                    bestSz = inlierSz; // update the best model size
                    for (int i = 0; i < N; i++) {
                        thisCS_2[i] = thisCS[i];
                    }
                }

                if (bestSz == N)
                    break;
                iter++;
            }

            //   std::cout<<"randidx "<<randIdx[0]<<" "<<randIdx[1]<<" "<<randIdx[2]<<std::endl;
            //--------------------------------------------------------------


            return thisCS_2;
        }

        vector<int> generate(int N, int M) {
            vector<int> index(N); //the whole indices
            for (int i = 0; i < N; i++) {
                index[i] = i;
            }

            vector<int> vektor(M);

            int in, im = 0;
            for (in = N; in > N - M; in--) {
                int temp = rand() / 65536;
                int r = temp % in; /* generate a random number 'r' */
                //     std::cout<<"rand "<<temp<<std::endl;
                vektor[im++] = index[r]; /* the range begins from 0 */
                index.erase(index.begin() + r);
            }

            return vektor;
        }

        Eigen::Matrix4d estimateTform(Eigen::MatrixXd &srcPts, Eigen::MatrixXd &tarPts,
                                      vector<int> &Idx, double &eps) {
            if (srcPts.rows() != 3 || tarPts.rows() != 3) {
                std::cout << "Input point clouds must be a 3xN matrix." << std::endl;
                return MatrixXd::Identity(4, 4);
            }
            if (srcPts.rows() < 3 || tarPts.rows() < 3) {
                std::cout << "At least 3 point matches are needed." << std::endl;
                return MatrixXd::Identity(4, 4);
            }
            if (srcPts.cols() != tarPts.cols()) {
                std::cout << "Input point clouds must be of the same size." << std::endl;
                return MatrixXd::Identity(4, 4);
            }
            unsigned int pointCount = Idx.size();
            Eigen::MatrixXd x(3, pointCount), y(3, pointCount);
            for (unsigned int i = 0; i < pointCount; i++) {
                x.col(i) = tarPts.col(Idx[i]);
                y.col(i) = srcPts.col(Idx[i]);
            }
            Eigen::Vector3d x_centroid, y_centroid;
            x_centroid.setZero();
            y_centroid.setZero();

            for (unsigned int i = 0; i < Idx.size(); i++) {
                x_centroid[0] += x(0, i);
                x_centroid[1] += x(1, i);
                x_centroid[2] += x(2, i);
                y_centroid[0] += y(0, i);
                y_centroid[1] += y(1, i);
                y_centroid[2] += y(2, i);
            }
            x_centroid /= pointCount;
            y_centroid /= pointCount;

            for (unsigned int i = 0; i < pointCount; i++) {
                x.col(i) = x.col(i) - x_centroid;
                y.col(i) = y.col(i) - y_centroid;
            }
            Eigen::MatrixXd R12(pointCount, 3), R21(3, pointCount), R22_1(3, pointCount);
            R12 = y.transpose() - x.transpose();
            R21 = x - y;
            R22_1 = x + y;
            Eigen::Matrix4d B;
            B.setZero();

            for (unsigned int i = 0; i < Idx.size(); i++) {
                Eigen::Matrix4d a_temp;
                a_temp.setZero();
                a_temp.block<1, 3>(0, 1) = R12.row(i);
                a_temp.block<3, 1>(1, 0) = R21.col(i);
                a_temp.block<3, 3>(1, 1) << 0, -1 * R22_1(2, i), R22_1(1, i),
                        R22_1(2, i), 0, -1 * R22_1(0, i),
                        -1 * R22_1(1, i), R22_1(0, i), 0;
                B = B + a_temp.transpose() * a_temp;

            }


            JacobiSVD<Eigen::MatrixXd> svd(B, ComputeThinU | ComputeThinV);
            Eigen::Matrix4d V = svd.matrixV(), U = svd.matrixU();
            Eigen::Matrix4d S = U.inverse() * B * V.transpose().inverse();

            Eigen::Matrix3d rot = quat2rot(V.col(3));
            Eigen::Matrix4d T1, T2, T3, T;
            T1.setIdentity(4, 4);
            T2.setIdentity(4, 4);
            T3.setIdentity(4, 4);
            T1.block<3, 1>(0, 3) = -1 * y_centroid;
            T2.block<3, 3>(0, 0) = rot;
            T3.block<3, 1>(0, 3) = x_centroid;

            T = T3 * T2 * T1;
            eps = S(3, 3);

            if (isnan(T(0, 0)) || isnan(-1 * T(0, 0))) {
                std::cout << "nnnnnnnnnaaaaaaaaaaaaaaa  " << B << std::endl;
                std::cout << x_centroid << std::endl;
                std::cout << y_centroid << std::endl;
            }
            return T;
        }

        Eigen::Matrix3d quat2rot(Eigen::VectorXd quat) {
            Eigen::Matrix3d R;
            R(0, 0) = quat(0) * quat(0) + quat(1) * quat(1) - quat(2) * quat(2) - quat(3) * quat(3);
            R(0, 1) = 2 * (quat(1) * quat(2) - quat(0) * quat(3));
            R(0, 2) = 2 * (quat(1) * quat(3) + quat(0) * quat(2));

            R(1, 0) = 2 * (quat(1) * quat(2) + quat(0) * quat(3));
            R(1, 1) = quat(0) * quat(0) - quat(1) * quat(1) + quat(2) * quat(2) - quat(3) * quat(3);
            R(1, 2) = 2 * (quat(2) * quat(3) - quat(0) * quat(1));

            R(2, 0) = 2 * (quat(1) * quat(3) - quat(0) * quat(2));
            R(2, 1) = 2 * (quat(2) * quat(3) + quat(0) * quat(1));
            R(2, 2) = quat(0) * quat(0) - quat(1) * quat(1) - quat(2) * quat(2) + quat(3) * quat(3);
            return R;

        }

        int maxIter;

    };
}
