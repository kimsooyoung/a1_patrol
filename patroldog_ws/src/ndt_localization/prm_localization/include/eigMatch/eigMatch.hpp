#define PCL_NO_PRECOMPILE

#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/bfgs.h>
#include <pcl/common/transforms.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/correspondence.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <climits>

#include "eigMatch/ransac.hpp"

struct Dimension_9 {
    float x_1;
    float y_1;
    float z_1;                 // preferred way of adding a XYZ+padding
    float x_2;
    float y_2;
    float z_2;
    float x_3;
    float y_3;
    float z_3;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (Dimension_9,           // here we assume a XYZ + "test" (as fields)
                                   (float, x_1, x_1)
                                           (float, y_1, y_1)
                                           (float, z_1, z_1)
                                           (float, x_2, x_2)
                                           (float, y_2, y_2)
                                           (float, z_2, z_2)
                                           (float, x_3, x_3)
                                           (float, y_3, y_3)
                                           (float, z_3, z_3)
)

struct Dimension_1 {
    float x;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (Dimension_1,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)

)

namespace wut_slam {

    using namespace std;
    using namespace Eigen;
    using namespace flann;


    struct eigMatch {
    public:
        typedef pcl::PointXYZ PointT;

        eigMatch() = default;

        void init_param(float overlap, float grid_step) {
            overlap_ = overlap;
            grid_step_ = grid_step;
        }

        void set_source(Eigen::MatrixXd &Desp, Eigen::MatrixXd &Seed, Eigen::MatrixXd &Norm) {
            srcDesp = Desp;
            srcSeed = Seed;
            srcNorm = Norm;
        }

        void set_target(Eigen::MatrixXd &Desp, Eigen::MatrixXd &Seed, Eigen::MatrixXd &Norm) {
            tarDesp = Desp;
            tarSeed = Seed;
            tarNorm = Norm;
        }

        Eigen::Matrix4d match() {
            std::vector<float> radii;
            for (unsigned int i = 1; i <= 4; i++) {
                radii.push_back((float) i * grid_step_ / 2);
            }

            Eigen::MatrixXd srcDesp_2 = srcDesp * 1000, tarDesp_2 = tarDesp * 1000;
            //   pcl::Correspondences src_to_tgt= knnSearch_mytype_9(srcDesp_2,tarDesp_2);
            pcl::Correspondences src_to_tgt = flann_search_9(srcDesp, tarDesp);
            sort(src_to_tgt.begin(), src_to_tgt.end(), compareCorrespondences);

            unsigned int M = srcSeed.cols();
            unsigned int N = tarSeed.cols();

            Eigen::MatrixXd Err(N, 1);//¶¨ÒåÒ»¸öN*1Êý¾Ý,ÆäÖµÎªinf
            Err.fill(ULONG_MAX);

            tform.resize(N);
            for (unsigned int tf_i = 0; tf_i < N; tf_i++) {
                tform[tf_i].setZero();
            }
            int ovNum = overlap_ * N + 1;
            //    std::cout<<"ovNum "<<ovNum<<" "<<overlap_<<" "<<N<<std::endl;
            double distThr = 0.2 / 4 * radii.size();
            int thetaThr = 10;
            int threshold = grid_step_ * grid_step_;
            unsigned int count_iterat = 0.3 * N + 1, n;

            std::cout << "start i " << N << " " << M << " " << count_iterat << " " << Err(0, 0) << std::endl;
            int count_ii = 0;
            for (unsigned int i = 0; i < count_iterat; i++) {
                count_ii++;
                n = src_to_tgt[i].index_query;
                Eigen::VectorXd seed(srcSeed.rows()), seedNorm(srcNorm.rows());
                seed = srcSeed.col(src_to_tgt[i].index_match);
                seedNorm = srcNorm.col(src_to_tgt[i].index_match);
                pcl::PointCloud<PointT>::Ptr d(new pcl::PointCloud<PointT>);
                d->width = srcSeed.cols();
                d->height = 1;
                d->points.resize(d->width * d->height);

                for (unsigned int j = 0; j < srcSeed.cols(); j++) {
                    d->points[j].x = (srcSeed.col(j) - seed).norm();
                    d->points[j].y = 0;
                    d->points[j].z = 0;
                }
                MatrixXd theta;
                MatrixXd inProd_2;
                theta.resize(srcNorm.rows() / 3, srcNorm.cols());
                inProd_2.resize(srcNorm.rows() / 3, srcNorm.cols());
                Eigen::VectorXd dd(srcNorm.rows());
                float inProd;
                for (unsigned int j = 0; j < srcNorm.cols(); j++) {
                    dd = srcNorm.col(j).cwiseProduct(seedNorm);
                    for (unsigned int k = 0; k < srcNorm.rows() / 3; k++) {
                        inProd = dd(k * 3) + dd(k * 3 + 1) + dd(k * 3 + 2);
                        inProd_2(k, j) = inProd;
                        float real_t = real(acos((inProd > 0.9999) ? 0.9999 : (inProd < -0.9999) ? -0.9999 : inProd));
                        theta(k, j) = real_t * 180 / 3.1415926;
                    }
                }

                pcl::PointCloud<PointT>::Ptr r(new pcl::PointCloud<PointT>);
                r->width = tarSeed.cols();
                r->height = 1;
                r->points.resize(r->width * r->height);

                for (unsigned int j = 0; j < tarSeed.cols(); j++) {
                    r->points[j].x = (tarSeed.col(j) - tarSeed.col(n)).norm();
                    r->points[j].y = 0;
                    r->points[j].z = 0;
                }
                MatrixXd alpha;
                alpha.resize(tarNorm.rows() / 3, tarNorm.cols());
                Eigen::VectorXd pp(tarNorm.rows());
                for (unsigned int j = 0; j < tarNorm.cols(); j++) {

                    pp = tarNorm.col(j).cwiseProduct(tarNorm.col(n));
                    for (unsigned int k = 0; k < tarNorm.rows() / 3; k++) {
                        inProd = pp(k * 3) + pp(k * 3 + 1) + pp(k * 3 + 2);
                        float real_t = real(acos((inProd > 0.9999) ? 0.9999 : (inProd < -0.9999) ? -0.9999 : inProd));

                        alpha(k, j) = real_t * 180 / 3.1415926;
                    }
                }
                pcl::KdTreeFLANN<PointT> kdtree_1;
                vector<vector<int>> IDX;
                kdtree_1.setInputCloud(r);
                for (auto point : d->points) {
                    vector<int> pointIdxRadiusSearch;
                    vector<float> pointIdxRadiusSquaredDistance;
                    kdtree_1.radiusSearch(point, (float) grid_step_ / 2, pointIdxRadiusSearch,
                                          pointIdxRadiusSquaredDistance);
                    IDX.push_back(pointIdxRadiusSearch);
                }
                std::vector<Eigen::Vector2i> matches;
                Eigen::Vector2i matches_1;
                matches_1 << src_to_tgt[i].index_match, src_to_tgt[i].index_query;
                matches.push_back(matches_1);
                std::vector<int> idx;

                for (unsigned int m = 0; m < M; m++) {
                    if (m == src_to_tgt[i].index_match) {
                        continue;
                    }
                    idx = IDX[m];

                    if (idx.empty()) {
                        continue;
                    }
                    Eigen::MatrixXd dTheta(alpha.rows(), idx.size());
                    Eigen::VectorXd Tab(idx.size());
                    Eigen::VectorXd sim(idx.size());
                    sim.setZero();
                    Tab.setZero();
                    bool count_tab_theta = 0;
                    for (unsigned int alpha_index = 0; alpha_index < idx.size(); alpha_index++) {
                        dTheta.col(alpha_index) = (alpha.col(idx[alpha_index]) - theta.col(m)).cwiseAbs();

                        for (unsigned int dTheta_row = 0; dTheta_row < dTheta.rows(); dTheta_row++) {
                            sim(alpha_index) = sim(alpha_index) + dTheta(dTheta_row, alpha_index);
                            //   std::cout<<"dTheta(dTheta_row,alpha_index) "<<dTheta(dTheta_row,alpha_index)<<std::endl;
                            if (dTheta(dTheta_row, alpha_index) < thetaThr) {
                                Tab(alpha_index)++;
                            }
                            if (Tab(alpha_index) >= theta.rows()) {
                                count_tab_theta = 1;
                            }
                        }
                        sim(alpha_index) /= dTheta.rows();

                        if (Tab(alpha_index) < theta.rows()) {
                            sim(alpha_index) = ULONG_MAX;
                        }
                    }
                    if (count_tab_theta == 0) {

                        continue;
                    }
                    Eigen::VectorXd::Index min_col;
                    double min_1 = sim.minCoeff(&min_col);
                    double R = (srcDesp.col(m) - tarDesp.col(idx[min_col])).norm();

                    if (min_1 < thetaThr && R < distThr) {
                        Eigen::Vector2i matches_2;
                        matches_2 << m, idx[min_col];
                        matches.push_back(matches_2);

                    }
                }

                if (matches.size() > 10) {
                    Eigen::MatrixXd match_srcSeed(srcSeed.rows(), matches.size());
                    Eigen::MatrixXd match_tarSeed(tarSeed.rows(), matches.size());
                    for (unsigned int k = 0; k < matches.size(); k++) {
                        match_srcSeed.col(k) = srcSeed.col(matches[k](0));
                        match_tarSeed.col(k) = tarSeed.col(matches[k][1]);
                    }
                    ransac ransac_c;
                    Eigen::MatrixXd match_srcSeed_2(srcSeed.rows(), matches.size());
                    Eigen::MatrixXd match_tarSeed_2(tarSeed.rows(), matches.size());
                    match_srcSeed_2 = match_srcSeed.cast<double>();
                    match_tarSeed_2 = match_tarSeed.cast<double>();

                    std::vector<bool> cs;
                    cs.resize(match_srcSeed_2.cols());
                    cs = ransac_c.compare(match_srcSeed_2, match_tarSeed_2, 1);
                    int count_cs = 0;
                    std::vector<int> match_index;
                    for (unsigned int c_cs = 0; c_cs < cs.size(); c_cs++) {
                        if (cs[c_cs] == 1) {
                            count_cs++;
                            match_index.push_back(c_cs);
                        }
                    }
                    if (count_cs < 3) {
                        continue;
                    }
                    //   std::cout<<matches.size()<<" "<<n<<" "<<count_cs<<std::endl;
                    ransac ransac_m;
                    double esp_1;
                    Eigen::Matrix4d T_i = ransac_m.estimateTform(match_srcSeed_2, match_tarSeed_2, match_index, esp_1);

                    Eigen::MatrixXd match_tarSeed_c(tarSeed.rows(), match_index.size());
                    Eigen::MatrixXd match_srcSeed_c(srcSeed.rows(), match_index.size());
                    std::vector<int> match_index_2;
                    for (unsigned int mc_i = 0; mc_i < match_index.size(); mc_i++) {
                        match_tarSeed_c.col(mc_i) = match_tarSeed_2.col(match_index[mc_i]);
                        match_srcSeed_c.col(mc_i) = match_srcSeed_2.col(match_index[mc_i]);
                        match_index_2.push_back(mc_i);
                    }
                    Eigen::MatrixXd tarEst(4, srcSeed.cols());
                    Eigen::MatrixXd srcSeed_2(4, srcSeed.cols());

                    srcSeed_2.topRows(3) = srcSeed.cast<double>();
                    srcSeed_2.row(3).fill(1);

                    tarEst = T_i * srcSeed_2;

                    tform[n] = T_i;

                    Eigen::MatrixXd tarEst_3 = tarEst.topRows(3);
                    pcl::Correspondences src_to_tgt_3 = knnSearch_mytype_3(tarEst_3, tarSeed);
                    sort(src_to_tgt_3.begin(), src_to_tgt_3.end(), compareCorrespondences);
                    double sum_est_seed = 0;
                    //   std::cout<<"est_seed_i "<<ovNum<<std::endl;
                    for (unsigned int est_seed_i = 0; est_seed_i < ovNum; est_seed_i++) {
                        Eigen::Vector3d est_seed;
                        est_seed = tarEst_3.col(src_to_tgt_3[est_seed_i].index_match) -
                                   tarSeed.col(src_to_tgt_3[est_seed_i].index_query);
                        sum_est_seed += est_seed.norm();
                    }
                    Err(n, 0) = sum_est_seed;
                }
                if (matches.size() > (0.65 * srcDesp.cols())) {
                    std::cout << "break " << matches.size() << " " << i << " " << srcDesp.cols() << std::endl;
                    break;
                }
            }

            std::cout << "end match in" << std::endl;
            Eigen::VectorXd::Index err_min_col, err_min_row;
            float min_err = Err.minCoeff(&err_min_row, &err_min_col);

            rawmse = min_err;
            return tform[err_min_row];

        }

        int output_once = 0;

        Eigen::Matrix4d match_err() {
            std::vector<float> radii;
            for (unsigned int i = 1; i <= 4; i++) {
                radii.push_back((float) i * grid_step_ / 2);
            }

            Eigen::MatrixXd srcDesp_2 = srcDesp * 1000, tarDesp_2 = tarDesp * 1000;
            //   pcl::Correspondences src_to_tgt= knnSearch_mytype_9(srcDesp,tarDesp);
            pcl::Correspondences src_to_tgt = flann_search_9(srcDesp, tarDesp);
            sort(src_to_tgt.begin(), src_to_tgt.end(), compareCorrespondences);

            unsigned int M = srcSeed.cols();
            unsigned int N = tarSeed.cols();

            Eigen::MatrixXd Err(N, 1);//¶¨ÒåÒ»¸öN*1Êý¾Ý,ÆäÖµÎªinf
            Err.fill(ULONG_MAX);

            tform.resize(N);
            for (unsigned int tf_i = 0; tf_i < N; tf_i++) {
                tform[tf_i].setZero();
            }
            int ovNum = overlap_ * N + 1;
            //    std::cout<<"ovNum "<<ovNum<<" "<<overlap_<<" "<<N<<std::endl;
            double distThr = 0.2 / 4 * radii.size();
            std::cout << "distThr " << distThr << " " << radii.size() << std::endl;
            int thetaThr = 10;
            int threshold = grid_step_ * grid_step_;
            unsigned int count_iterat = 0.3 * N + 1, n;

            std::cout << "start i " << N << " " << M << " " << count_iterat << " " << Err(0, 0) << std::endl;
            int count_ii = 0;
            for (unsigned int i = 0; i < count_iterat; i++) {
                n = src_to_tgt[i].index_query;
                Eigen::VectorXd seed(srcSeed.rows()), seedNorm(srcNorm.rows());
                seed = srcSeed.col(src_to_tgt[i].index_match);
                seedNorm = srcNorm.col(src_to_tgt[i].index_match);
                pcl::PointCloud<PointT>::Ptr d(new pcl::PointCloud<PointT>);
                d->width = srcSeed.cols();
                d->height = 1;
                d->points.resize(d->width * d->height);

                for (unsigned int j = 0; j < srcSeed.cols(); j++) {
                    d->points[j].x = (srcSeed.col(j) - seed).norm();
                    d->points[j].y = 0;
                    d->points[j].z = 0;
                }
                MatrixXd theta;
                MatrixXd inProd_2;
                theta.resize(srcNorm.rows() / 3, srcNorm.cols());
                inProd_2.resize(srcNorm.rows() / 3, srcNorm.cols());
                Eigen::VectorXd dd(srcNorm.rows());
                float inProd;
                for (unsigned int j = 0; j < srcNorm.cols(); j++) {
                    dd = srcNorm.col(j).cwiseProduct(seedNorm);
                    for (unsigned int k = 0; k < srcNorm.rows() / 3; k++) {
                        inProd = dd(k * 3) + dd(k * 3 + 1) + dd(k * 3 + 2);
                        inProd_2(k, j) = inProd;
                        float real_t = real(acos((inProd > 0.9999) ? 0.9999 : (inProd < -0.9999) ? -0.9999 : inProd));
                        theta(k, j) = real_t * 180 / 3.1415926;
                    }
                }

                pcl::PointCloud<PointT>::Ptr r(new pcl::PointCloud<PointT>);
                r->width = tarSeed.cols();
                r->height = 1;
                r->points.resize(r->width * r->height);

                for (unsigned int j = 0; j < tarSeed.cols(); j++) {
                    r->points[j].x = (tarSeed.col(j) - tarSeed.col(n)).norm();
                    r->points[j].y = 0;
                    r->points[j].z = 0;
                }
                MatrixXd alpha;
                alpha.resize(tarNorm.rows() / 3, tarNorm.cols());
                Eigen::VectorXd pp(tarNorm.rows());
                for (unsigned int j = 0; j < tarNorm.cols(); j++) {

                    pp = tarNorm.col(j).cwiseProduct(tarNorm.col(n));
                    for (unsigned int k = 0; k < tarNorm.rows() / 3; k++) {
                        inProd = pp(k * 3) + pp(k * 3 + 1) + pp(k * 3 + 2);
                        float real_t = real(acos((inProd > 0.9999) ? 0.9999 : (inProd < -0.9999) ? -0.9999 : inProd));

                        alpha(k, j) = real_t * 180 / 3.1415926;
                    }
                }
                pcl::KdTreeFLANN<PointT> kdtree_1;
                vector<vector<int>> IDX;
                kdtree_1.setInputCloud(r);
                for (auto point : d->points) {
                    vector<int> pointIdxRadiusSearch;
                    vector<float> pointIdxRadiusSquaredDistance;
                    kdtree_1.radiusSearch(point, (float) grid_step_ / 2, pointIdxRadiusSearch,
                                          pointIdxRadiusSquaredDistance);
                    IDX.push_back(pointIdxRadiusSearch);
                }
                std::vector<Eigen::Vector2i> matches;
                Eigen::Vector2i matches_1;
                matches_1 << src_to_tgt[i].index_match, src_to_tgt[i].index_query;
                matches.push_back(matches_1);
                std::vector<int> idx;
                int count_s = 0, count_r = 0, count_con_1 = 0, count_con_2 = 0;
                for (unsigned int m = 0; m < M; m++) {
                    if (m == src_to_tgt[i].index_match) {
                        continue;
                    }
                    idx = IDX[m];

                    if (idx.empty()) {
                        count_con_1 = count_con_1 + 1;
                        continue;
                    }
                    Eigen::MatrixXd dTheta(alpha.rows(), idx.size());
                    Eigen::VectorXd Tab(idx.size());
                    Eigen::VectorXd sim(idx.size());
                    sim.setZero();
                    Tab.setZero();
                    bool count_tab_theta = 0;
                    for (unsigned int alpha_index = 0; alpha_index < idx.size(); alpha_index++) {
                        dTheta.col(alpha_index) = (alpha.col(idx[alpha_index]) - theta.col(m)).cwiseAbs();

                        for (unsigned int dTheta_row = 0; dTheta_row < dTheta.rows(); dTheta_row++) {
                            sim(alpha_index) = sim(alpha_index) + dTheta(dTheta_row, alpha_index);
                            //   std::cout<<"dTheta(dTheta_row,alpha_index) "<<dTheta(dTheta_row,alpha_index)<<std::endl;
                            if (dTheta(dTheta_row, alpha_index) < thetaThr) {
                                Tab(alpha_index)++;
                            }
                            if (Tab(alpha_index) >= theta.rows()) {
                                count_tab_theta = 1;
                            }
                        }
                        sim(alpha_index) /= dTheta.rows();

                        if (Tab(alpha_index) < theta.rows()) {
                            sim(alpha_index) = ULONG_MAX;
                        }
                    }
                    if (count_tab_theta == 0) {
                        count_con_2 = count_con_2 + 1;
                        continue;
                    }
                    Eigen::VectorXd::Index min_col;
                    double min_1 = sim.minCoeff(&min_col);
                    double R = (srcDesp.col(m) - tarDesp.col(idx[min_col])).norm();

                    if (min_1 < thetaThr && R < distThr) {
                        Eigen::Vector2i matches_2;
                        matches_2 << m, idx[min_col];
                        matches.push_back(matches_2);
                    }


                }
                //    std::cout<<count_s<<" "<<count_r<<" "<<count_con_1<<" "<<count_con_2<<std::endl;
                if (matches.size() > 10) {
                    for (int mai = 0; mai < matches.size(); mai++) {
                        //			std::cout<<matches[mai](0)<<" "<<matches[mai](1)<<std::endl;
                    }
                    count_ii++;
                    Eigen::MatrixXd match_srcSeed(srcSeed.rows(), matches.size());
                    Eigen::MatrixXd match_tarSeed(tarSeed.rows(), matches.size());
                    for (unsigned int k = 0; k < matches.size(); k++) {
                        match_srcSeed.col(k) = srcSeed.col(matches[k](0));
                        match_tarSeed.col(k) = tarSeed.col(matches[k][1]);
                    }
                    ransac ransac_c;
                    Eigen::MatrixXd match_srcSeed_2(srcSeed.rows(), matches.size());
                    Eigen::MatrixXd match_tarSeed_2(tarSeed.rows(), matches.size());
                    match_srcSeed_2 = match_srcSeed.cast<double>();
                    match_tarSeed_2 = match_tarSeed.cast<double>();

                    std::vector<bool> cs;
                    cs.resize(match_srcSeed_2.cols());
                    cs = ransac_c.compare(match_srcSeed_2, match_tarSeed_2, 1);
                    int count_cs = 0;
                    std::vector<int> match_index;
                    for (unsigned int c_cs = 0; c_cs < cs.size(); c_cs++) {
                        if (cs[c_cs] == 1) {
                            count_cs++;
                            match_index.push_back(c_cs);
                        }
                    }
                    if (count_cs < 3) {
                        continue;
                    }

                    ransac ransac_m;
                    double esp_1;
                    Eigen::Matrix4d T_i = ransac_m.estimateTform(match_srcSeed_2, match_tarSeed_2, match_index, esp_1);
//		std::cout<<matches.size()<<" "<<n<<" "<<count_cs<<std::endl;                
                    Eigen::MatrixXd match_tarSeed_c(tarSeed.rows(), match_index.size());
                    Eigen::MatrixXd match_srcSeed_c(srcSeed.rows(), match_index.size());
                    std::vector<int> match_index_2;
                    for (unsigned int mc_i = 0; mc_i < match_index.size(); mc_i++) {
                        match_tarSeed_c.col(mc_i) = match_tarSeed_2.col(match_index[mc_i]);
                        match_srcSeed_c.col(mc_i) = match_srcSeed_2.col(match_index[mc_i]);
                        match_index_2.push_back(mc_i);
                    }
                    Eigen::MatrixXd tarEst(4, srcSeed.cols());
                    Eigen::MatrixXd srcSeed_2(4, srcSeed.cols());

                    srcSeed_2.topRows(3) = srcSeed.cast<double>();
                    srcSeed_2.row(3).fill(1);

                    tarEst = T_i * srcSeed_2;

                    tform[n] = T_i;

                    Eigen::MatrixXd tarEst_3 = tarEst.topRows(3);
                    //    pcl::Correspondences src_to_tgt_3= knnSearch_mytype_3(tarEst_3,tarSeed);
                    pcl::Correspondences src_to_tgt_3 = flann_search_9(tarEst_3, tarSeed);
                    sort(src_to_tgt_3.begin(), src_to_tgt_3.end(), compareCorrespondences);
                    double sum_est_seed = 0;
                    //     std::cout<<"est_seed_i "<<ovNum<<std::endl;
                    for (unsigned int est_seed_i = 0; est_seed_i < ovNum; est_seed_i++) {
                        Eigen::Vector3d est_seed;
                        est_seed = tarEst_3.col(src_to_tgt_3[est_seed_i].index_match) -
                                   tarSeed.col(src_to_tgt_3[est_seed_i].index_query);
                        sum_est_seed += est_seed.norm();
                    }
                    Err(n, 0) = sum_est_seed;
                }
                if (matches.size() > (0.65 * srcDesp.cols())) {
                    std::cout << "break " << matches.size() << " " << i << " " << srcDesp.cols() << std::endl;
                    break;
                }
            }
            int count_max_angle = 0;
            for (unsigned int err_i = 0; err_i < Err.rows(); err_i++) {
                if (Err(err_i, 0) == ULONG_MAX) {
                    continue;
                }
                Eigen::Matrix3d rotation = tform[err_i].block<3, 3>(0, 0);
//            Eigen::Vector3d eulerAngle=rotation.eulerAngles(0,1,2);
                Eigen::Vector3d eulerAngle = rotmat2euler(rotation);
                float max_angle = fabs(eulerAngle(0)) > fabs(eulerAngle(1)) ? fabs(eulerAngle(0)) : fabs(eulerAngle(1));
                if (max_angle > 1.5) {
                    Err(err_i, 0) = ULONG_MAX;
                    count_max_angle++;
//		std::cout<<err_i<<" "<<max_angle<<std::endl;
                }
            }
            std::cout << "count_max_angle " << count_max_angle << std::endl;
            Eigen::VectorXd::Index err_min_col, err_min_row;
            float min_err = Err.minCoeff(&err_min_row, &err_min_col);
            std::cout << "end match in" << err_min_row << " " << count_ii << std::endl;
            rawmse = min_err;
//	std::cout<<tform[err_min_row]<<std::endl;
            return tform[err_min_row];

        }

        static Eigen::Vector3d rotmat2euler(Eigen::Matrix3d coeff) {
            Eigen::Vector4i nextAxis;
            nextAxis << 1, 2, 0, 1;
            int i = 2, j = nextAxis(i + 1), k = nextAxis(i);
            double sy = sqrt(coeff(i, i) * coeff(i, i) + coeff(j, i) * coeff(j, i));
            //      std::cout<<"sy "<<sy<<" "<<-1*coeff(k,i)<<std::endl;
            Eigen::Vector3d eul;
            eul(2) = -1 * atan2(coeff(k, j), coeff(k, k));
            eul(1) = -1 * atan2(-1 * coeff(k, i), sy);
            eul(0) = -1 * atan2(coeff(j, i), coeff(i, i));

            return eul;
        }


        pcl::Correspondences knnSearch_mytype_9(Eigen::MatrixXd &data_1, Eigen::MatrixXd &data_2) {
            pcl::PointCloud<Dimension_9>::Ptr src_pc(new pcl::PointCloud<Dimension_9>);
            src_pc->width = data_1.cols();
            src_pc->height = 1;
            src_pc->points.resize(src_pc->width * src_pc->height);

            pcl::PointCloud<Dimension_9>::Ptr tar_pc(new pcl::PointCloud<Dimension_9>);
            tar_pc->width = data_2.cols();
            tar_pc->height = 1;
            tar_pc->points.resize(tar_pc->width * tar_pc->height);
            std::cout << "copy " << data_1.cols() << " " << data_2.cols() << " " << tar_pc->points.size() << std::endl;
            for (unsigned int i = 0; i < tar_pc->points.size(); ++i) {
                tar_pc->points[i].x_1 = data_2(0, i);
                tar_pc->points[i].y_1 = data_2(1, i);
                tar_pc->points[i].z_1 = data_2(2, i);
                tar_pc->points[i].x_2 = data_2(3, i);
                tar_pc->points[i].y_2 = data_2(4, i);
                tar_pc->points[i].z_2 = data_2(5, i);
                tar_pc->points[i].x_3 = data_2(6, i);
                tar_pc->points[i].y_3 = data_2(7, i);
                tar_pc->points[i].z_3 = data_2(8, i);
            }
            for (unsigned int i = 0; i < src_pc->points.size(); ++i) {
                src_pc->points[i].x_1 = data_1(0, i);
                src_pc->points[i].y_1 = data_1(1, i);
                src_pc->points[i].z_1 = data_1(2, i);
                src_pc->points[i].x_2 = data_1(3, i);
                src_pc->points[i].y_2 = data_1(4, i);
                src_pc->points[i].z_2 = data_1(5, i);
                src_pc->points[i].x_3 = data_1(6, i);
                src_pc->points[i].y_3 = data_1(7, i);
                src_pc->points[i].z_3 = data_1(8, i);
            }
            pcl::KdTreeFLANN<Dimension_9> my_kdtree_9;
            my_kdtree_9.setInputCloud(src_pc);
            int SourceCloudPoints = (int) (tar_pc->points.size());
            pcl::Correspondences src_to_tgt(SourceCloudPoints);
            std::vector<int> index_tgt(1);
            std::vector<float> dist_trans(1);
            std::cout << "start search " << tar_pc->points.size() << std::endl;
            for (unsigned int i = 0; i < tar_pc->points.size(); ++i) {

                my_kdtree_9.nearestKSearch(tar_pc->points[i], 1, index_tgt, dist_trans);

                src_to_tgt[i].index_query = i;
                src_to_tgt[i].index_match = index_tgt[0];
                src_to_tgt[i].distance = dist_trans[0];
            }

            return src_to_tgt;

        }

        static pcl::Correspondences flann_search_9(Eigen::MatrixXd &data_1, Eigen::MatrixXd &data_2) {
            flann::Matrix<float> dataset(new float[data_1.rows() * data_1.cols()], data_1.cols(), data_1.rows());
            flann::Matrix<float> query(new float[data_2.rows() * data_2.cols()], data_2.cols(), data_2.rows());

            for (int i = 0; i < data_1.cols(); i++) {
                for (int j = 0; j < data_1.rows(); j++) {
                    dataset[i][j] = data_1(j, i);
                }
            }
            for (int i = 0; i < data_2.cols(); i++) {
                for (int j = 0; j < data_2.rows(); j++) {
                    query[i][j] = data_2(j, i);
                }
            }
            flann::Matrix<int> indices(new int[query.rows], query.rows, 1);
            flann::Matrix<float> dists(new float[query.rows], query.rows, 1);
            flann::Index<L2<float> > index_(dataset, flann::KDTreeIndexParams(8));
            index_.buildIndex();
            index_.knnSearch(query, indices, dists, 1, flann::SearchParams(64));

            pcl::Correspondences src_to_tgt(query.rows);
            for (unsigned int i = 0; i < query.rows; ++i) {
                src_to_tgt[i].index_query = i;
                src_to_tgt[i].index_match = indices[i][0];
                src_to_tgt[i].distance = dists[i][0];
            }

            return src_to_tgt;

        }

        static pcl::Correspondences knnSearch_mytype_3(Eigen::MatrixXd &data_1, Eigen::MatrixXd &data_2) {
            pcl::PointCloud<PointT>::Ptr src_pc(new pcl::PointCloud<PointT>);
            src_pc->width = data_1.cols();
            src_pc->height = 1;
            src_pc->points.resize(src_pc->width * src_pc->height);

            pcl::PointCloud<PointT>::Ptr tar_pc(new pcl::PointCloud<PointT>);
            tar_pc->width = data_2.cols();
            tar_pc->height = 1;
            tar_pc->points.resize(tar_pc->width * tar_pc->height);
            for (unsigned int i = 0; i < tar_pc->points.size(); ++i) {
                tar_pc->points[i].x = data_2(0, i);
                tar_pc->points[i].y = data_2(1, i);
                tar_pc->points[i].z = data_2(2, i);
            }
            for (unsigned int i = 0; i < src_pc->points.size(); ++i) {
                src_pc->points[i].x = data_1(0, i);
                src_pc->points[i].y = data_1(1, i);
                src_pc->points[i].z = data_1(2, i);

            }
            pcl::KdTreeFLANN<PointT> my_kdtree_3;
            my_kdtree_3.setInputCloud(src_pc);
            int SourceCloudPoints = (int) (tar_pc->points.size());
            pcl::Correspondences src_to_tgt(SourceCloudPoints);
            std::vector<int> index_tgt(1);
            std::vector<float> dist_trans(1);
            for (unsigned int i = 0; i < tar_pc->points.size(); ++i) {
                my_kdtree_3.nearestKSearch(tar_pc->points[i], 1, index_tgt, dist_trans);
                src_to_tgt[i].index_query = i;
                src_to_tgt[i].index_match = index_tgt[0];
                src_to_tgt[i].distance = dist_trans[0];
            }
            return src_to_tgt;

        }

        static bool compareCorrespondences(const pcl::Correspondence &a, const pcl::Correspondence &b) {
            return a.distance < b.distance;
        }

        Eigen::MatrixXd srcDesp, srcSeed, srcNorm;
        Eigen::MatrixXd tarDesp, tarSeed, tarNorm;
        float overlap_;
        float grid_step_;
        std::vector<Eigen::Matrix4d> tform;

        float rawmse;
        bool output_1 = 0;
    };
}
