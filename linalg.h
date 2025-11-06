#ifndef LINALG_H
#define LINALG_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <QString>

// ------------ Math ------------

using Frame = Eigen::Matrix<double, 6, 1>; // row 6x1

struct Plane {
  double A, B, C, D;   // A*x + B*y + C*z + D = 0
  double AA, BB, DD;   // z = AA*x + BB*y + DD
};

struct Cylinder {
  static Cylinder fromAxis(const Eigen::Vector3d& c1_,
                           const Eigen::Vector3d& c2_,
                           double R_);
  double R;
  Frame frame; // [X,Y,Z,A,B,C]
  Eigen::Matrix4d transform;
};

struct EulerSolution {
  double A1, A2; // yaw (Z)
  double B1, B2; // pitch (Y)
  double C1, C2; // roll (X)
};

Eigen::Matrix4d trMatrix4x4(const Eigen::Vector3d& delta);
Eigen::Matrix4d rotMatrix4x4(double angleDeg, char axis);
Plane pointsToPlane(const Eigen::Ref<const Eigen::VectorXd>& x,
                    const Eigen::Ref<const Eigen::VectorXd>& y,
                    const Eigen::Ref<const Eigen::VectorXd>& z);
Eigen::Vector3d poly(double x0, double x1, double x2,
                     double y0, double y1, double y2);

EulerSolution rot2euler(const Eigen::Matrix3d& R, bool is_deg = false);
Eigen::Matrix3d euler2rot(double A, double B, double C, bool is_deg = false);


// ------------ Frene ------------
struct Frene {
  Frene(const Eigen::Vector3d& t_, const Eigen::Vector3d& b_,
        const Eigen::Vector3d& n_, const Eigen::Vector3d& p_);

  Eigen::Vector3d t{Eigen::Vector3d::Zero()};
  Eigen::Vector3d b{Eigen::Vector3d::Zero()};
  Eigen::Vector3d n{Eigen::Vector3d::Zero()};
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Matrix4d transf{Eigen::Matrix4d::Identity()};
};

Frene getFreneByPoly(const Eigen::Vector3d& p0,
                     const Eigen::Vector3d& u1,
                     const Eigen::Vector3d& u2,
                     const Eigen::Vector3d& v1);

Frene getFreneByCirc(const Eigen::Vector3d& pt0,
                     const Eigen::Vector3d& ptc);

// ------------ Blade ------------
using MatN3 = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
struct Profile { MatN3 cx, cv, le, re; };
using Airfoil = std::vector<Profile>;

Frame getBeltFrame(const Eigen::Vector3d& o,
                   const Eigen::Ref<const Eigen::VectorXd>& x,
                   const Eigen::Ref<const Eigen::VectorXd>& y,
                   const Eigen::Ref<const Eigen::VectorXd>& z);

Airfoil loadBladeJson(const QString& filePath);


#endif // LINALG_H
