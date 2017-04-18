#pragma once
//
// Created by steve on 16-11-23.
//

//Create by steve in 16-11-23 at 下午8:02

#include "SettingPara.h"


#ifndef QUICKFUSING_EKF_HPP
#define QUICKFUSING_EKF_HPP


class Ekf {
public:
    Ekf(SettingPara para) {
//        MYCHECK(1);

        para_ = para;

        R_ = Eigen::Matrix3d::Zero();

        P_.resize(9, 9);
        P_.setZero();

        Q_.resize(6, 6);
        Q_.setZero();

        H_.resize(3, 9);
        H_.setZero();


        InitialFilter();

        x_h_.resize(9, 1);
        x_h_.setZero();

        K_.resize(9, 3);
        K_.setIdentity();

    }

    Ekf(const Ekf &orig) {
        para_ = orig.para_;
//
        R_ = orig.R_;

        P_ = orig.P_;

        Q_ = orig.Q_;

        H_ = orig.H_;

        x_h_ = orig.x_h_;

        K_ = orig.K_;

        F_ = orig.F_;

        G_ = orig.G_;

        quat_ = orig.quat_;

    }

    bool CopyMatrix(Eigen::MatrixXd in, Eigen::MatrixXd &out) {
        out.resize(in.rows(), in.cols());

        for (int i(0); i < in.rows(); ++i) {
            for (int j(0); j < in.cols(); ++j) {
                out(i, j) = in(i, j);
            }
        }

        return true;
    }


    bool InitNavEq(Eigen::MatrixXd u) {

        double f_u(0.0), f_v(0.0), f_w(0.0);

        f_u = u.col(0).mean();
        f_v = u.col(1).mean();
        f_w = u.col(2).mean();


        double roll(atan2(-f_v, -f_w)), pitch(atan2(f_u, sqrt(f_v * f_v + f_w * f_w)));

        Eigen::Vector3d attitude(roll, pitch, para_.init_heading1_);

        Eigen::Matrix3d Rb2t = Rt2b(attitude);
        Rb2t.transposeInPlace();

        quat_ = dcm2q(Rb2t);

        x_h_.block(0, 0, 3, 1) = para_.init_pos1_;
        x_h_.block(6, 0, 3, 1) = attitude;


        return true;
    }

    bool InitialFilter() {
//        MYCHECK("1");
        for (int i(0); i < 3; ++i) {
            P_(i, i) = para_.sigma_initial_pos1_(i) * para_.sigma_initial_pos1_(i);
            P_(i + 3, i + 3) = para_.sigma_initial_vel1_(i) * para_.sigma_initial_vel1_(i);
            P_(i + 6, i + 6) = para_.sigma_initial_att1_(i) * para_.sigma_initial_att1_(i);

            R_(i, i) = para_.sigma_vel_(i) * para_.sigma_vel_(i);

            Q_(i, i) = para_.sigma_acc_(i) * para_.sigma_acc_(i);
            Q_(i + 3, i + 3) = para_.sigma_gyro_(i) * para_.sigma_gyro_(i);

            H_(i, i + 3) = 1.0;
        }


        return true;
    }


//protected:

    /*
     * Euler to Rotation Matrix.
     */
    Eigen::MatrixXd Rt2b(Eigen::Vector3d ang) {
        double cr(cos(ang[0])), sr(sin(ang[0]));
        double cp(cos(ang[1])), sp(sin(ang[1]));
        double cy(cos(ang[2])), sy(sin(ang[2]));

        Eigen::Matrix3d R;

        R(0, 0) = cy * cp;
        R(0, 1) = sy * cp;
        R(0, 2) = -sp;

        R(1, 0) = -sy * cr + cy * sp * sr;
        R(1, 1) = cy * cr + sy * sp * sr;
        R(1, 2) = cp * sr;

        R(2, 0) = sy * sr + cy * sp * cr;
        R(2, 1) = -cy * sr + sy * sp * cr;
        R(2, 2) = cp * cr;

        return R;
    }


    /*
     * Rotation matrix to quanternions.
     */
    Eigen::Vector4d dcm2q(Eigen::Matrix3d R) {
//        MYCHECK(1);
        double T(1.0 + R(0, 0) + R(1, 1) + R(2, 2));

        double qw(0.0), qx(0.0), qy(0.0), qz(0.0);
        double S(0.0);


        try {
            // 1e-3  ==>>>  fabs(T) != 0
            if (fabs(T) > 1e-8) {
                S = 0.5 / sqrt(fabs(T));

                qw = 0.25 / S;
                qx = (R(2, 1) - R(1, 2)) * S;
                qy = (R(0, 2) - R(2, 0)) * S;
                qz = (R(1, 0) - R(0, 1)) * S;

            } else {
                if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
                    S = sqrt(1 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;

                    qw = (R(2, 1) - R(1, 2)) / S;
                    qx = 0.25 * S;
                    qy = (R(0, 1) + R(1, 0)) / S;
                    qz = (R(0, 2) + R(2, 0)) / S;
                } else if (R(1, 1) > R(2, 2)) {
                    S = sqrt(1 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;

                    qw = (R(0, 2) - R(2, 0)) / S;
                    qx = (R(0, 1) + R(1, 0)) / S;
                    qy = 0.25 * S;
                    qz = (R(1, 2) + R(2, 1)) / S;
                } else {

                    S = sqrt(1 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;

                    qw = (R(1, 0) - R(0, 1)) / S;
                    qx = (R(0, 2) + R(2, 0)) / S;
                    qy = (R(1, 2) + R(2, 1)) / S;
                    qz = 0.25 * S;

                }

            }

            Eigen::Vector4d quart(qx, qy, qz, qw);
//            std::cout << "quart:" << quart << "norm:" << quart.norm() << std::endl;
            quart /= quart.norm();

            return quart;

        } catch (...) {
            std::cout << "THERE ARE SOME ERROR!" << std::endl;
            return Eigen::Vector4d(0, 0, 0, 1.0);
        }
    }


    /*
     *Quanternions to rotation matrix.
     */
    Eigen::Matrix3d q2dcm(Eigen::Vector4d q) {
//        MYCHECK(1);

        Eigen::VectorXd p;
        p.resize(6);
        p.setZero();

        p.block(0, 0, 4, 1) = q.array().pow(2.0);

        p(4) = p(1) + p(2);

        if (fabs(p(0) + p(3) + p(4)) > 1e-18) {
            p(5) = 2.0 / (p(0) + p(3) + p(4));

        } else {
            p(5) = 0.0;
        }

        Eigen::Matrix3d R;
        R.setZero();

        R(0, 0) = 1 - p(5) * p(4);
        R(1, 1) = 1 - p(5) * (p(0) + p(2));
        R(2, 2) = 1 - p(5) * (p(0) + p(1));

        p(0) = p(5) * q(0);
        p(1) = p(5) * q(1);
        p(4) = p(5) * q(2) * q(3);
        p(5) = p(0) * q(1);

        R(0, 1) = p(5) - p(4);
        R(1, 0) = p(5) + p(4);

        p(4) = p(1) * q(3);
        p(5) = p(0) * q(2);

        R(0, 2) = p(5) + p(4);
        R(2, 0) = p(5) - p(4);

        p(4) = p(0) * q(3);
        p(5) = p(1) * q(2);

        R(1, 2) = p(5) - p(4);
        R(2, 1) = p(5) + p(4);

        return R;
    }

    /*
     * @brief : Navigation Equation .
     *
     *
     */
    Eigen::VectorXd NavigationEquation(Eigen::VectorXd x_h,
                                       Eigen::VectorXd u,
                                       Eigen::VectorXd q,
                                       double dt) {

//        MYCHECK(1);

        Eigen::VectorXd y;
        y.resize(9);

        Eigen::Vector3d w_tb(u.block(3, 0, 3, 1));

        double v(w_tb.norm() * dt);

        if (fabs(v) > 1e-8) {
            double P(w_tb(0) * dt * 0.5);
            double Q(w_tb(1) * dt * 0.5);
            double R(w_tb(2) * dt * 0.5);

            Eigen::Matrix4d OMEGA;

            OMEGA.setZero();

            OMEGA(0, 1) = R;
            OMEGA(0, 2) = -Q;
            OMEGA(0, 3) = P;

            OMEGA(1, 0) = -R;
            OMEGA(1, 2) = P;
            OMEGA(1, 3) = Q;

            OMEGA(2, 0) = Q;
            OMEGA(2, 1) = -P;
            OMEGA(2, 3) = R;

            OMEGA(3, 0) = -P;
            OMEGA(3, 1) = -Q;
            OMEGA(3, 2) = -R;

            if (std::isnan(OMEGA.sum())) {
                std::cout << "omega is nan " << std::endl;
            }
//            Eigen::Vector4d tq = quat_;

            quat_ = (cos(v / 2.0) * Eigen::Matrix4d::Identity() +
                     2.0 / v * sin(v / 2.0) * OMEGA) * (q);

            if (std::isnan(quat_.sum())) {
                std::cout << "quat_ error nann in state transition equation " << std::endl;
            }

//            quat_ /= quat_.norm();
//            for(int i(0);i<4;++i)
//            {
//                if(std::isnan(quat_(i)))
//                {
//                    quat_ = tq;
//                }
//            }
//            quat_ /= quat_(3);


        } else {
            /*
             * Need not do any thing.
             */
//            quat_ = q;
        }

//        MYCHECK(1);

        //---------------
        Eigen::Vector3d g_t(0, 0, 9.8173);
//        g_t = g_t.transpose();

        Eigen::Matrix3d Rb2t(q2dcm(quat_));
        Eigen::MatrixXd f_t(Rb2t * (u.block(0, 0, 3, 1)));

        Eigen::Vector3d acc_t(f_t + g_t);
        if (std::isnan(acc_t.sum())) {
            std::cout << "acc _ t  erro " << std::endl;
        }

        Eigen::MatrixXd A, B;

        A.resize(6, 6);
        A.setIdentity();
//        MYCHECK(1);
//        std::cout << A.rows() << " x " << A.cols() << std::endl;
//        std::cout << "dt :" << dt << std::endl;
        A(0, 3) = dt;
        A(1, 4) = dt;
        A(2, 5) = dt;

        B.resize(6, 3);
        B.setZero();
        Eigen::Matrix3d tmp;

//        std::cout << B.rows() << " x " << B.cols() << std::endl;
//        tmp.setZero();
//        B.block(0, 0, 3, 3) = tmp;
        B.block(3, 0, 3, 3) = Eigen::Matrix3d::Identity() * dt;

//        MYCHECK(1);
        y.block(0, 0, 6, 1) = A * (x_h.block(0, 0, 6, 1)) +
                              B * acc_t;
//        MYCHECK(1);


        if (std::isnan(x_h_.sum())) {
            std::cout << "new x_h_ is nana " << std::endl;
        }
        x_h_ = y;
//        MYCHECK(1);
        return y;
    }

    bool StateMatrix(Eigen::Vector4d q, Eigen::VectorXd u, double dt) {

//        MYCHECK(1);

        Eigen::Matrix3d Rb2t(q2dcm(q));

        Eigen::Vector3d f_t(Rb2t * (u.block(0, 0, 3, 1)));

        Eigen::Matrix3d St;
        St.setZero();

        St(0, 1) = -f_t(2);
        St(0, 2) = f_t(1);

        St(1, 0) = f_t(2);
        St(1, 2) = f_t(0);

        St(2, 0) = -f_t(1);
        St(2, 1) = f_t(0);

        Eigen::MatrixXd Fc;
        Fc.resize(9, 9);
        Fc.setZero();

        Fc.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
        Fc.block(3, 6, 3, 3) = St;

        Eigen::MatrixXd Gc;
        Gc.resize(9, 6);
        Gc.setZero();

        Gc.block(3, 0, 3, 3) = Rb2t;
        Gc.block(6, 3, 3, 3) = -Rb2t;


        Eigen::MatrixXd Id;
        Id.resize(9, 9);
        Id.setIdentity();

        F_ = Id + (Fc * dt);
        G_ = Gc * dt;

        return true;

    }

    Eigen::VectorXd ComputeInternalState(Eigen::VectorXd x_in,
                                         Eigen::VectorXd dx,
                                         Eigen::VectorXd q_in) {

//        MYCHECK(1);

        Eigen::MatrixXd R(q2dcm(q_in));

        Eigen::VectorXd x_out = x_in + dx;

        Eigen::Vector3d epsilon(dx.block(6, 0, 3, 1));

        Eigen::Matrix3d OMEGA;
        OMEGA.setZero();
        if (std::isnan(dx.sum())) {
            std::cout << "dx is nan " << std::endl;
        }

        if (std::isnan(epsilon(0) + epsilon(1) + epsilon(2))) {
            std::cout << "epsilon error " << std::endl;
        }

        OMEGA(0, 1) = -epsilon(2);
        OMEGA(0, 2) = epsilon(1);

        OMEGA(1, 0) = epsilon(2);
        OMEGA(1, 2) = -epsilon(0);

        OMEGA(2, 0) = -epsilon(1);
        OMEGA(2, 1) = epsilon(0);


        R = (Eigen::Matrix3d::Identity() - OMEGA) * (R);


        Eigen::Vector4d tq = quat_;

        quat_ = dcm2q(R);

        for (int i(0); i < 4; ++i) {
            if (std::isnan(quat_(i))) {
                quat_ = tq;
            }
        }


        return x_out;

    }

    Eigen::VectorXd GetPosition(Eigen::VectorXd u, double zupt1) {

//        MYCHECK(1);

        x_h_ = NavigationEquation(x_h_, u, quat_, para_.Ts_);
//        MYCHECK(1);
        StateMatrix(quat_, u, para_.Ts_);
//        MYCHECK(1);

        P_ = (F_ * (P_)) * (F_.transpose().eval()) +
             (G_ * Q_ * G_.transpose().eval());
        if (zupt1 > 0.5) {
            Eigen::Vector3d z(-x_h_.block(3, 0, 3, 1));

            if (std::isnan(z(0) + z(1) + z(2))) {
                std::cout << "z error . " << std::endl;
            }


            Eigen::MatrixXd K;
            K = P_ * H_.transpose().eval() * (H_ * P_ * H_.transpose().eval() + R_).inverse();
            if (std::isnan(K.sum())) {
                std::cout << " k in nan ... " << std::endl;
                K = last_K;
            } else {
                last_K = K;
            }
            Eigen::VectorXd dx = K * z;


            Eigen::MatrixXd Id;
            Id.resize(9, 9);
            Id.setIdentity();

            P_ = (Id - K * H_) * P_;

            x_h_ = ComputeInternalState(x_h_, dx, quat_);
        }
        P_ = (P_.eval() * 0.5 + P_.transpose().eval() * 0.5);

        return x_h_;
    }

    bool GetTransform(Eigen::MatrixXd &P, Eigen::Isometry3d &transform, Eigen::Isometry3d &abs_t) {
        P = P_;
        // build transform matrix
        Eigen::Isometry3d t2(Eigen::Isometry3d::Identity());

        Eigen::Quaterniond q;
        q.x() = quat_(0);
        q.y() = quat_(1);
        q.z() = quat_(2);
        q.w() = quat_(3);

        Eigen::Matrix3d rotation_matrix = q.normalized().toRotationMatrix();


//        Eigen::Matrix3d rotation_matrix = q2dcm(quat_);
        for (int i(0); i < 3; ++i) {
            for (int j(0); j < 3; ++j) {
                t2(i, j) = rotation_matrix(i, j);
                if (std::isnan(t2(i, j))) {
                    std::cout << t2(i, j) << std::endl;
                }
            }
        }
        for (int i(0); i < 3; ++i) {
            t2(i, 3) = x_h_(i, 0);
        }

        // return ...

        transform = latest_t.inverse() * t2;
        latest_t = t2;
        abs_t = t2;

    }

private:
    //Parameters in here.
    SettingPara para_;

    //P for single foot
    Eigen::Matrix<double, 9, 9> P_;

    Eigen::Matrix<double, 6, 6> Q_;

    Eigen::Matrix3d R_;

    Eigen::Matrix<double, 3, 9> H_;

    Eigen::Matrix<double, 9, 1> x_h_;

    Eigen::MatrixXd F_;
    Eigen::MatrixXd G_;

    Eigen::MatrixXd K_;

    Eigen::MatrixXd last_K;

    Eigen::Vector4d quat_;
    Eigen::Isometry3d latest_t = Eigen::Isometry3d::Identity();

};


#endif //QUICKFUSING_EKF_HPP
