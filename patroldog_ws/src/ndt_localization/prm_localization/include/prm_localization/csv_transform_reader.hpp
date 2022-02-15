//
// Created by vickylzy on 20-2-21.
//

#ifndef SRC_CSV_TRANSFORM_READER_HPP
#define SRC_CSV_TRANSFORM_READER_HPP

#include <Eigen/Dense>
#include <fstream>

namespace prm_localization {
    using namespace std;

    class CSV_reader {
    public:
        CSV_reader(const string &filepath) : filepath(filepath) {
            std::cout << "filepath\n" << filepath << std::endl;

            ifstream f(filepath);
            istream &s = f;
            Eigen::Matrix3f trans2d = Eigen::MatrixXf::Zero(3, 3);
            string curr_line;
            size_t i = 0;
            if (!s.good()) {
                std::cerr << "error reading csv" << std::endl;
                return;
            }
            while (s.good() && i < 9) {
//                std::cout<<"read once"<<std::endl;

                getline(s, curr_line);
                stringstream ss(curr_line);
                string curr_s;
                while (getline(ss, curr_s, ',')) {
                    trans2d(i) = stof(curr_s);
                    ++i;
                }
            }
            f.close();
            trans2d.transposeInPlace();
            transform_mg.setIdentity();
            transform_mg.block(0, 0, 2, 2) = trans2d.block(0, 0, 2, 2);
            transform_mg.block(0, 3, 2, 1) = trans2d.block(0, 2, 2, 1);
//            std::cout<<"transform_mg\n"<<transform_mg<<std::endl;

        }

        const Eigen::Matrix4f &getTransformMg() const {
            return transform_mg;
        }

        const string &getFilepath() const {
            return filepath;
        }

    private:

        Eigen::Matrix4f transform_mg;
        string filepath;

    };
}

#endif //SRC_CSV_TRANSFORM_READER_HPP
