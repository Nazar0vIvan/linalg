#ifndef LINALG_H
#define LINALG_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <QString>

// ------------ Math ------------
struct Plane {
  double A, B, C, D;   // A*x + B*y + C*z + D = 0
  double AA, BB, DD;   // z = AA*x + BB*y + DD
};

struct Frame {
  Eigen::Matrix<double, 6, 1> frame;
  Eigen::Matrix4d transform;
};

Eigen::Matrix4d trMatrix4x4(const Eigen::Vector3d& delta);
Eigen::Matrix4d rotMatrix4x4(double angleDeg, char axis);
Plane pointsToPlane(const Eigen::Ref<const Eigen::VectorXd>& x,
                    const Eigen::Ref<const Eigen::VectorXd>& y,
                    const Eigen::Ref<const Eigen::VectorXd>& z);
Eigen::Vector3d poly(double x0, double x1, double x2,
                     double y0, double y1, double y2);

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
                            const Eigen::Vector3d& ptc)
{
  Eigen::Vector3d v = pt0 - ptc;
  Eigen::Vector3d n = v.normalized();    // unit normal (radial)
  Eigen::Vector3d t(-n.y(), n.x(), 0.0); // in-plane tangent
  if (t.x() < 0.0) t = -t;
  Eigen::Vector3d b = n.cross(t);        // binormal

  return Frene(t, b, n, pt0);
}

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
