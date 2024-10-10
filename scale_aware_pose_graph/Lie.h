#ifndef LIE
#define LIE

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/ceres.h>
#include <ceres/rotation.h>


using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector8d = Eigen::Matrix<double, 8, 1>;

/**
 * @brief calculate the skew symmetric matrix for a 3x1 vector
 * 
 * @tparam T 
 * @param v the 3x1 vector
 * @return the corresponding skew symmetric matrix 
 */
template<typename T>
Eigen::Matrix<T, 3, 3> skewSymmetric(Eigen::Matrix<T, 3, 1> v)
{
    Eigen::Matrix<T, 3, 3> m;
    m << T(0), -v(2), v(1),
         v(2), T(0), -v(0),
         -v(1), v(0), T(0);
    return m;
}


/**
 * @brief the exponential map of sim3 pose
 * 
 * @tparam T 
 * @param sim3 the minimum sim3 pose representation [omega, upsilon, sigma] (rotation, translation, scale)
 * @return the SIM3 pose representation [qw, qx, qy, qz, x, y, z, s] (quaternion, translation, scale)
 */
template <typename T>
Eigen::Matrix<T, 8, 1> Sim3Exp(const Eigen::Matrix<T, 7, 1> sim3)
{
    Eigen::Matrix<T, 3, 1> omega = sim3.segment(0, 3);
    Eigen::Matrix<T, 3, 1> upsilon = sim3.segment(3, 3);
    T sigma = sim3(6);

    // derive rotation (it's easy to derive so3 using quaternion)
    Eigen::Matrix<T, 4, 1> r;
    ceres::AngleAxisToQuaternion(omega.data(), r.data());
    T theta = omega.norm();

    // derive s
    T s = ceres::exp(sigma);

    // derive t
    Eigen::Matrix<T, 3, 3> omega0 = Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 3> omega1 = skewSymmetric(omega);
    Eigen::Matrix<T, 3, 3> omega2 = omega1 * omega1;
    T eps = T(0.00001);
    T block0 = T(0);
    T block1 = T(0);
    T block2 = T(0);
    if (ceres::abs(sigma) < eps)
    {
        block0 = T(1);
        if (ceres::abs(theta) < eps)
        {
            block1 = T(1.0 / 2.0);
            block2 = T(1.0 / 6.0);
        }
        else
        {
            block1 = (T(1) - ceres::cos(theta)) / (theta * theta);
            block2 = (theta - ceres::sin(theta)) / (theta * theta * theta);
        }
    }
    else
    {
        block0 = (s - T(1)) / sigma;
        if (ceres::abs(theta) < eps)
        {
            block1 = ((sigma - T(1)) * s + T(1)) / (sigma * sigma);
            block2 = (s * (sigma * sigma / T(2) - sigma + T(1)) - T(1)) / (sigma * sigma * sigma);
        }
        else
        {
            T A = s * ceres::sin(theta);
            T B = s * ceres::cos(theta);
            block1 = (A * sigma + (T(1) - B) * theta) / (theta*sigma*sigma + theta*theta*theta);
            block2 = ((s - T(1)) / sigma - ((B - T(1))*sigma + A*theta) / (sigma*sigma + theta*theta)) / (theta * theta);
        }
    }
    Eigen::Matrix<T, 3, 3> W = block0 * omega0 + block1 * omega1 + block2 * omega2;
    Eigen::Matrix<T, 3, 1> t = W * upsilon;

    Eigen::Matrix<T, 8, 1> SIM3 = Eigen::Matrix<T, 8, 1>::Zero();
    SIM3.segment(0, 4) = r;
    SIM3.segment(4, 3) = t;
    SIM3(7) = s;
    return SIM3;
}


/**
 * @brief the logarithm map of SIM3 pose
 * 
 * @tparam T 
 * @param SIM3 the SIM3 pose representation [qw, qx, qy, qz, x, y, z, s] (quaternion, translation, scale)
 * @return the minimum sim3 pose representation [omega, upsilon, sigma] (rotation, translation, scale)
 */
template <typename T>
Eigen::Matrix<T, 7, 1> Sim3Log(const Eigen::Matrix<T, 8, 1> SIM3)
{
    Eigen::Matrix<T, 4, 1> q = SIM3.segment(0, 4);
    Eigen::Matrix<T, 3, 1> t = SIM3.segment(4, 3);
    T s = SIM3(7);

    //derive omega
    Eigen::Matrix<T, 3, 1> omega;
    ceres::QuaternionToAngleAxis(q.data(), omega.data());


    // derive sigma
    T sigma = ceres::log(s);

    // derive upsilon
    Eigen::Matrix<T, 3, 3> omega0 = Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 3, 3> omega1 = skewSymmetric(omega);
    Eigen::Matrix<T, 3, 3> omega2 = omega1 * omega1;
    T theta = omega.norm();
    T eps = T(0.00001);
    T block0 = T(0);
    T block1 = T(0);
    T block2 = T(0);
    if (ceres::abs(sigma) < eps)
    {
        block0 = T(1);
        if (ceres::abs(theta) < eps) // not sure about this comparison when things become template
        {
            block1 = T(1.0 / 2.0);
            block2 = T(1.0 / 6.0);
        }
        else
        {
            block1 = (T(1) - ceres::cos(theta)) / (theta * theta);
            block2 = (theta - ceres::sin(theta)) / (theta * theta * theta);
        }
    }
    else
    {
        block0 = (s - T(1)) / sigma;
        if (ceres::abs(theta) < eps)
        {
            block1 = ((sigma - T(1)) * s + T(1)) / (sigma * sigma);
            block2 = (s * (sigma * sigma / T(2) - sigma + T(1)) - T(1)) / (sigma * sigma * sigma);
        }
        else
        {
            T A = s * ceres::sin(theta);
            T B = s * ceres::cos(theta);
            block1 = (A * sigma + (T(1) - B) * theta) / (theta*sigma*sigma + theta*theta*theta);
            block2 = ((s - T(1)) / sigma - ((B - T(1))*sigma + A*theta) / (sigma*sigma + theta*theta)) / (theta * theta);
        }
    }
    Eigen::Matrix<T, 3, 3> W = block0 * omega0 + block1 * omega1 + block2 * omega2;
    Eigen::Matrix<T, 3, 1> upsilon = W.lu().solve(t);

    Eigen::Matrix<T, 7, 1> sim3 = Eigen::Matrix<T, 7, 1>::Zero();
    sim3.segment(0, 3) = omega;
    sim3.segment(3, 3) = upsilon;
    sim3(6) = sigma;
    return sim3;
}

/**
 * @brief calculate the inverse of the SIM3 pose representation
 * 
 * @tparam T 
 * @param SIM3 the SIM3 pose representation [qw, qx, qy, qz, x, y, z, s] (quaternion, translation, scale)
 * @return the inverse of SIM3 pose representation
 */
template <typename T>
Eigen::Matrix<T, 8, 1> Sim3Inv(const Eigen::Matrix<T, 8, 1> SIM3)
{
    Eigen::Quaternion<T> q(SIM3(0), SIM3(1), SIM3(2), SIM3(3));
    Eigen::Matrix<T, 3, 1> t = SIM3.segment(4, 3);
    T s = SIM3(7);

    Eigen::Quaternion<T> q_inv = q.conjugate();
    T s_inv = T(1) / s;
    Eigen::Matrix<T, 3, 1> t_inv = -s_inv * q_inv.toRotationMatrix() * t;

    Eigen::Matrix<T, 8, 1> SIM3_inv;
    SIM3_inv << q_inv.w() , q_inv.x(), q_inv.y(), q_inv.z(), t_inv.x(), t_inv.y(), t_inv.z(), s_inv;

    return SIM3_inv;
}

/**
 * @brief This function transform the vector pose representation into a 4x4 matrix pose representation, then apply
          matrix multiplication, and transform the result back into vector representation
 * 
 * @tparam T 
 * @param T0 the pose vector on the left
 * @param T1 the pose vector on the right
 * @return the result pose vector
 */
template <typename T>
Eigen::Matrix<T, 8, 1> Tdot(const Eigen::Matrix<T, 8, 1> T0, const Eigen::Matrix<T, 8, 1> T1)
{
    Eigen::Quaternion<T> q0(T0(0), T0(1), T0(2), T0(3));
    Eigen::Matrix<T, 3, 1> t0 = T0.segment(4, 3);
    T s0 = T0(7);

    Eigen::Quaternion<T> q1(T1(0), T1(1), T1(2), T1(3));
    Eigen::Matrix<T, 3, 1> t1 = T1.segment(4, 3);
    T s1 = T1(7);

    Eigen::Quaternion<T> q_result = q0 * q1;
    Eigen::Matrix<T, 3, 1> t_result = s0 * q0.toRotationMatrix() * t1 + t0;
    T s_result = s0 * s1;

    Eigen::Matrix<T, 8, 1> T_result;
    T_result << q_result.w() , q_result.x(), q_result.y(), q_result.z(),
                t_result.x(), t_result.y(), t_result.z(), s_result;

    return T_result;
}


#endif /* LIE */
