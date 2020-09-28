/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _ICP_CONTROL_UTILS_
#define _ICP_CONTROL_UTILS_

#include <pal_locomotion/biped_controller.h>
#include <armadillo>

namespace pal_locomotion
{
void control(BController *bc, const math_utils::HighPassRateLimiterVector2dPtr &rate_limiter,
             const eVector3 &targetDCM, const eVector3 &targetDCM_vel,
             const eVector2 &referenceCOP, const bool use_rate_limited_dcm,
             eVector2 &targetCOP_rate_limited_unclamped, eVector2 &targetCOP_unclamped);

void control(BController *bc, const math_utils::HighPassRateLimiterVector2dPtr &rate_limiter,
             const eVector3 &targetDCM, const eVector3 &targetDCM_vel,
             const eVector2 &referenceCOP, const bool use_rate_limited_dcm_,
             const eVector3 &actualCOM, const eVector2 &actualCOM_vel,
             eVector2 &targetCOP_rate_limited_unclamped, eVector2 &targetCOP_unclamped);


class CubicInterpolator
{
public:
  CubicInterpolator(const ros::Time &start_time, const ros::Time &end_time, const eVector3 &start_pos, const eVector3 &end_pos, 
                    const eVector3 &start_vel = eVector3::Zero(), const eVector3 &end_vel= eVector3::Zero(),
                    const eVector3 &start_acc = eVector3::Zero(), const eVector3 &end_acc= eVector3::Zero());
  ~CubicInterpolator(){};

public:
  void evaluate(const eVector3 &desired_pos, const eVector3 &desired_vel, const eVector3 &desired_acc);  

private:
  std::vector<Eigen::Matrix<double, 3, 1> >  p_;      
  ros::Time start_time_;
  ros::Time end_time_;      
};
}

class CSVReader
{
public:
  CSVReader(const std::string &filename, const std::string &delimeter = ",") 
  {
    arma_matrix = readCSV(filename, delimeter);
    eigen_matrix = arma2eigen(arma_matrix);
  }

  Eigen::MatrixXd row(int index)
  {
    return eigen_matrix.row(index);
  }

  arma::mat readCSV(const std::string &filename, const std::string &delimeter = ",")
{
    std::ifstream csv(filename);
    std::vector<std::vector<double>> datas;

    for(std::string line; std::getline(csv, line); ) {

        std::vector<double> data;

        // split string by delimeter
        auto start = 0U;
        auto end = line.find(delimeter);
        while (end != std::string::npos) {
            data.push_back(std::stod(line.substr(start, end - start)));
            start = end + delimeter.length();
            end = line.find(delimeter, start);
        }
        data.push_back(std::stod(line.substr(start, end)));
        datas.push_back(data);
    }

    arma::mat data_mat = arma::zeros<arma::mat>(datas.size(), datas[0].size());

    for (int i=0; i<datas.size(); i++) {
        arma::mat r(datas[i]);
        data_mat.row(i) = r.t();
    }

    return data_mat;
}

  Eigen::MatrixXd arma2eigen(arma::mat arma_M) {
    Eigen::MatrixXd eigen_M = Eigen::Map<Eigen::MatrixXd>(arma_M.memptr(), arma_M.n_rows, arma_M.n_cols);
    return eigen_M;
}

  arma::mat eigen2arma(Eigen::MatrixXd eigen_M) {
    arma::mat arma_M = arma::mat(eigen_M.data(), eigen_M.rows(), eigen_M.cols(),false, false);
    return arma_M;
}

private:

  arma::mat arma_matrix;
  Eigen::MatrixXd eigen_matrix;

};

#endif
