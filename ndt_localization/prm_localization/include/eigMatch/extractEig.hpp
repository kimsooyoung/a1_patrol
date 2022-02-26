#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/bfgs.h>
#include <pcl/common/transforms.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/correspondence.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <utility>

using namespace std;
using namespace Eigen;
namespace wut_slam {

    struct extractEig {
    public:
        typedef pcl::PointXYZ PointT;

        struct S_N {
        public:
            S_N() = default;

            S_N(Eigen::Vector3d data1, Eigen::Vector3d data2) {
                s = std::move(data1);
                n = std::move(data2);
            }

            Eigen::Vector3d s;
            Eigen::Vector3d n;
        };

        extractEig() = default;

        void extract(pcl::PointCloud<PointT>::Ptr &srcData, float gridStep) {
            std::vector<pcl::PointXYZ> srcSeed;//´æ´¢Ë÷ÒýÊý´óÓÚ10µÄÖÐÐÄµã
            vector<float> radii;
            for (unsigned int i = 1; i <= 4; i++) {
                radii.push_back((float) i * gridStep / 2);
            }
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloudDown(new pcl::PointCloud<pcl::PointXYZ>);
            vg.setInputCloud(srcData);
            vg.setLeafSize((float) gridStep, (float) gridStep, (float) gridStep);//change leaf size into 0.5cm
            vg.filter(*srcCloudDown);
            int K = radii.size();
            vector<vector<int>> srcIdx;
            vector<int> idxSz;
            float radius = radii[0];
            pcl::KdTreeFLANN<PointT> kdtree;
            std::cout << "set input" << srcCloudDown->points.size() << " " << radius << std::endl;
            kdtree.setInputCloud(srcData);
            for (unsigned int i = 0; i < srcCloudDown->points.size(); i++) {
                vector<int> pointIdxRadiusSearch;
                vector<float> pointIdxRadiusSquaredDistance;
                kdtree.radiusSearch(srcCloudDown->points[i], radius, pointIdxRadiusSearch,
                                    pointIdxRadiusSquaredDistance);
                idxSz.push_back(pointIdxRadiusSearch.size());
                if (idxSz[i] > 10) {
                    SeedDesNum.push_back(pointIdxRadiusSearch.size());
                    srcIdx.push_back(pointIdxRadiusSearch);
                    srcSeed.push_back(srcCloudDown->points[i]);
                }
            }
            std::cout << "srcIdx size " << srcIdx.size() << std::endl;
            unsigned int M = SeedDesNum.size();
            std::vector<Eigen::Matrix<double, 3, 4>> s;
            std::vector<Eigen::Matrix<double, 3, 4>> n;
            s.resize(M);
            n.resize(M);
            std::cout << "M " << M << std::endl;
            for (unsigned int i = 0; i < M; i++) {
                S_N sn;
                sn = svdCov(srcIdx[i], i, srcData, srcSeed);
                s[i].col(0) = sn.s;
                n[i].col(0) = sn.n;
            }
            kdtree.setInputCloud(srcData);
            std::cout << "radii " << radii[0] << " " << radii[1] << " " << radii[2] << " " << radii[3] << std::endl;
            std::cout << K << std::endl;
            for (unsigned int i = 1; i < K; i++) {
                vector<vector<int>> srcIdx_2;
                vector<int> pointIdxRadiusSearch;
                vector<float> pointIdxRadiusSquaredDistance;
                for (auto seed_point : srcSeed) {
                    kdtree.radiusSearch(seed_point, radii[i], pointIdxRadiusSearch, pointIdxRadiusSquaredDistance);
                    srcIdx_2.push_back(pointIdxRadiusSearch);

                }
                for (int j = 0; j < srcSeed.size(); j++) {
                    S_N sn;
                    sn = svdCov(srcIdx_2[j], j, srcData, srcSeed);
                    s[j].col(i) = sn.s;
                    n[j].col(i) = sn.n;
                }
            }

            srcDesp.resize(9, M);
            for (unsigned int i = 0; i < M; i++) {
                srcDesp(0, i) = s[i].col(1)(0) - s[i].col(0)(0);
                srcDesp(1, i) = s[i].col(2)(0) - s[i].col(1)(0);
                srcDesp(2, i) = s[i].col(3)(0) - s[i].col(2)(0);
                srcDesp(3, i) = s[i].col(1)(1) - s[i].col(0)(1);
                srcDesp(4, i) = s[i].col(2)(1) - s[i].col(1)(1);
                srcDesp(5, i) = s[i].col(3)(1) - s[i].col(2)(1);
                srcDesp(6, i) = s[i].col(1)(2) - s[i].col(0)(2);
                srcDesp(7, i) = s[i].col(2)(2) - s[i].col(1)(2);
                srcDesp(8, i) = s[i].col(3)(2) - s[i].col(2)(2);
            }
            srcNorm.resize(12, M);
            for (unsigned int i = 0; i < M; i++) {
                srcNorm(0, i) = n[i].col(0)(0);
                srcNorm(1, i) = n[i].col(0)(1);
                srcNorm(2, i) = n[i].col(0)(2);
                srcNorm(3, i) = n[i].col(1)(0);
                srcNorm(4, i) = n[i].col(1)(1);
                srcNorm(5, i) = n[i].col(1)(2);
                srcNorm(6, i) = n[i].col(2)(0);
                srcNorm(7, i) = n[i].col(2)(1);
                srcNorm(8, i) = n[i].col(2)(2);
                srcNorm(9, i) = n[i].col(3)(0);
                srcNorm(10, i) = n[i].col(3)(1);
                srcNorm(11, i) = n[i].col(3)(2);
            }
            srcSeed_m.resize(3, srcSeed.size());
            for (unsigned int seed_i = 0; seed_i < srcSeed.size(); seed_i++) {
                srcSeed_m(0, seed_i) = srcSeed[seed_i].x;
                srcSeed_m(1, seed_i) = srcSeed[seed_i].y;
                srcSeed_m(2, seed_i) = srcSeed[seed_i].z;
            }

        }

        void extract_2(pcl::PointCloud<PointT>::Ptr &srcData, float gridStep) {
            std::vector<pcl::PointXYZ> srcSeed;//´æ´¢Ë÷ÒýÊý´óÓÚ10µÄÖÐÐÄµã
            vector<float> radii;
            for (unsigned int i = 1; i <= 4; i++) {
                radii.push_back((float) i * gridStep / 2);
            }
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            pcl::PointCloud<pcl::PointXYZ>::Ptr srcCloudDown(new pcl::PointCloud<pcl::PointXYZ>);
            vg.setInputCloud(srcData);
            vg.setLeafSize((float) gridStep, (float) gridStep, (float) gridStep);//change leaf size into 0.5cm
            vg.filter(*srcCloudDown);
            int K = radii.size();
            vector<vector<int>> srcIdx;
            vector<int> idxSz;
            float radius = radii[0];
            pcl::KdTreeFLANN<PointT> kdtree;
            std::cout << "set input" << srcCloudDown->points.size() << " " << radius << std::endl;
            kdtree.setInputCloud(srcData);
            for (unsigned int i = 0; i < srcCloudDown->points.size(); i++) {
                vector<int> pointIdxRadiusSearch;
                vector<float> pointIdxRadiusSquaredDistance;
                kdtree.radiusSearch(srcCloudDown->points[i], radius, pointIdxRadiusSearch,
                                    pointIdxRadiusSquaredDistance);
                idxSz.push_back(pointIdxRadiusSearch.size());
                if (idxSz[i] > 10) {
                    SeedDesNum.push_back(pointIdxRadiusSearch.size());
                    srcIdx.push_back(pointIdxRadiusSearch);
                    srcSeed.push_back(srcCloudDown->points[i]);
                }
            }
            std::cout << "srcIdx size " << srcIdx.size() << std::endl;
            unsigned int M = SeedDesNum.size();
            Eigen::MatrixXd s(4, M * 3);
            Eigen::MatrixXd n(4, M * 3);
            std::cout << "M " << M << std::endl;
            for (unsigned int i = 0; i < M; i++) {
                S_N sn;
                sn = svdCov(srcIdx[i], i, srcData, srcSeed);
                s(0, i * 3) = sn.s(0);
                s(0, i * 3 + 1) = sn.s(1);
                s(0, i * 3 + 2) = sn.s(2);
                n(0, i * 3) = sn.n(0);
                n(0, i * 3 + 1) = sn.n(1);
                n(0, i * 3 + 2) = sn.n(2);
            }
            pcl::KdTreeFLANN<PointT> kdtree_2;
            kdtree_2.setInputCloud(srcData);
            for (unsigned int i = 1; i < K; i++) {
                vector<vector<int>> srcIdx_2;
                vector<int> pointIdxRadiusSearch_2;
                vector<float> pointIdxRadiusSquaredDistance;
                float radis = radii[i];
                for (auto seed_point : srcSeed) {
                    kdtree_2.radiusSearch(seed_point, radis, pointIdxRadiusSearch_2, pointIdxRadiusSquaredDistance);
                    srcIdx_2.push_back(pointIdxRadiusSearch_2);
                }
                for (int j = 0; j < srcSeed.size(); j++) {
                    S_N sn;
                    sn = svdCov(srcIdx_2[j], j, srcData, srcSeed);
                    s(i, j * 3) = sn.s(0);
                    s(i, j * 3 + 1) = sn.s(1);
                    s(i, j * 3 + 2) = sn.s(2);
                    n(i, j * 3) = sn.n(0);
                    n(i, j * 3 + 1) = sn.n(1);
                    n(i, j * 3 + 2) = sn.n(2);

                }
            }
            srcDesp.resize(9, M);
            for (unsigned int i = 0; i < M; i++) {
                srcDesp(0, i) = s(1, i * 3) - s(0, i * 3);
                srcDesp(1, i) = s(2, i * 3) - s(1, i * 3);
                srcDesp(2, i) = s(3, i * 3) - s(2, i * 3);
                srcDesp(3, i) = s(1, i * 3 + 1) - s(0, i * 3 + 1);
                srcDesp(4, i) = s(2, i * 3 + 1) - s(1, i * 3 + 1);
                srcDesp(5, i) = s(3, i * 3 + 1) - s(2, i * 3 + 1);
                srcDesp(6, i) = s(1, i * 3 + 2) - s(0, i * 3 + 2);
                srcDesp(7, i) = s(2, i * 3 + 2) - s(1, i * 3 + 2);
                srcDesp(8, i) = s(3, i * 3 + 2) - s(2, i * 3 + 2);
            }
            srcNorm.resize(12, M);
            for (unsigned int i = 0; i < M; i++) {
                srcNorm(0, i) = n(0, 0 + i * 3);
                srcNorm(1, i) = n(0, 1 + i * 3);
                srcNorm(2, i) = n(0, 2 + i * 3);
                srcNorm(3, i) = n(1, 0 + i * 3);
                srcNorm(4, i) = n(1, 1 + i * 3);
                srcNorm(5, i) = n(1, 2 + i * 3);
                srcNorm(6, i) = n(2, 0 + i * 3);
                srcNorm(7, i) = n(2, 1 + i * 3);
                srcNorm(8, i) = n(2, 2 + i * 3);
                srcNorm(9, i) = n(3, 0 + i * 3);
                srcNorm(10, i) = n(3, 1 + i * 3);
                srcNorm(11, i) = n(3, 2 + i * 3);
            }
            srcSeed_m.resize(3, srcSeed.size());
            for (unsigned int seed_i = 0; seed_i < srcSeed.size(); seed_i++) {
                srcSeed_m(0, seed_i) = srcSeed[seed_i].x;
                srcSeed_m(1, seed_i) = srcSeed[seed_i].y;
                srcSeed_m(2, seed_i) = srcSeed[seed_i].z;
            }
        }


        S_N svdCov(vector<int> &nnIdx, int idx, pcl::PointCloud<PointT>::Ptr &srcData, vector<PointT> Seed) {
            MatrixXd nnPt;

            nnPt.resize(nnIdx.size(), 3);

            for (unsigned int i = 0; i < nnIdx.size(); i++) {
                nnPt(i, 0) = srcData->points[nnIdx[i]].x;
                nnPt(i, 1) = srcData->points[nnIdx[i]].y;
                nnPt(i, 2) = srcData->points[nnIdx[i]].z;
            }
            Eigen::Vector3d seed_;
            Eigen::Matrix3d C;
            seed_(0) = Seed[idx].x;
            seed_(1) = Seed[idx].y;
            seed_(2) = Seed[idx].z;
            C = matrixCompute(nnPt, seed_);
            JacobiSVD<Eigen::MatrixXd> svd(C, ComputeThinU | ComputeThinV);
            Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
            Eigen::Matrix3d S = U.inverse() * C * V.transpose().inverse();
            Eigen::Vector3d s = S.diagonal() / (S.diagonal()).sum();
            double nt1 = U.col(2).dot(seed_ * -1);
            int nt2;

            if (nt1 < 0) {
                nt2 = -1;
            } else if (nt1 > 0) {
                nt2 = 1;
            } else {
                nt2 = 0;
            }
            Eigen::Vector3d n = nt2 * U.col(2);
            S_N sn_(s, n);
            return sn_;
        }

        static Eigen::Matrix3d matrixCompute(Eigen::MatrixXd &A, Eigen::Vector3d &xc) {
            // construct the convariance matrix
            int K = A.rows();

            MatrixXd xct = xc.replicate(1, K);

            MatrixXd B = A.transpose() - xct;

            Eigen::Matrix3d mm = (1 / (float) K) * (B * B.transpose());

            return mm;
        }

        void clear(){
            //clear former desp data without let object out of scope
            srcNorm.resize(0,0);
            srcDesp.resize(0,0);
            srcSeed_m.resize(0,0);
            SeedDesNum.clear();
        }


        Eigen::MatrixXd srcNorm, srcDesp;//°ÑnµÄ3ÐÐ4ÁÐ±ä³ÉsrcNormµÄÒ»ÁÐ
        std::vector<int> SeedDesNum;
        Eigen::MatrixXd srcSeed_m;//´æ´¢Ë÷ÒýÊý´óÓÚ10µÄÖÐÐÄµã
    };
}
