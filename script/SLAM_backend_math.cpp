// Minimal 2D Pose Graph Optimization (Gauss-Newton) in C++
// - Measurement (dx,dy,dtheta) is expressed in frame of pose i.
// - This is just the math background. Need to combine this with ICP data input to complete SLAM.

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <cmath>
#include <iostream>
#include <vector>

static inline double wrapAngle(double a) {
  // Wrap to (-pi, pi]
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

struct Pose2 {
  double x = 0, y = 0, th = 0;
};

struct Edge2 {
  int i = 0, j = 0;
  Eigen::Vector3d z;      
  Eigen::Matrix3d Omega;  // information (inverse covariance)
};

static inline Eigen::Matrix2d R(double th) {
  double c = std::cos(th), s = std::sin(th);
  Eigen::Matrix2d M;
  M << c, -s,
       s,  c;
  return M;
}

// Error: e = [ R_i^T*(t_j - t_i) - [dx,dy] ; wrap((th_j - th_i) - dth) ]
static inline Eigen::Vector3d computeError(const Pose2& xi, const Pose2& xj, const Eigen::Vector3d& z) {
  Eigen::Vector2d ti(xi.x, xi.y), tj(xj.x, xj.y);
  Eigen::Vector2d dt = tj - ti;

  Eigen::Vector2d trans_err = R(xi.th).transpose() * dt - z.head<2>();
  double rot_err = wrapAngle((xj.th - xi.th) - z[2]);

  Eigen::Vector3d e;
  e << trans_err, rot_err;
  return e;
}

// Jacobians wrt xi and xj for the above error.
// e_t = R_i^T (t_j - t_i) - z_t
//   de_t/dt_i = -R_i^T
//   de_t/dt_j =  R_i^T
//   de_t/dth_i = d(R_i^T)/dth_i * (t_j - t_i)
// where d(R^T)/dth = [ -s  c; -c -s ] for R = [c -s; s c]
static inline void computeJacobians(
    const Pose2& xi, const Pose2& xj,
    Eigen::Matrix<double,3,3>& Ji, Eigen::Matrix<double,3,3>& Jj)
{
  Ji.setZero();
  Jj.setZero();

  Eigen::Matrix2d RiT = R(xi.th).transpose();

  // Translational parts
  Ji.block<2,2>(0,0) = -RiT;
  Jj.block<2,2>(0,0) =  RiT;

  // de_t/dth_i
  double c = std::cos(xi.th), s = std::sin(xi.th);
  Eigen::Matrix2d dRiT;
  // derivative of R^T:
  // R^T = [ c  s; -s c ]
  // d/dth = [ -s  c; -c -s ]
  dRiT << -s,  c,
          -c, -s;

  Eigen::Vector2d ti(xi.x, xi.y), tj(xj.x, xj.y);
  Eigen::Vector2d dt = tj - ti;

  Eigen::Vector2d d = dRiT * dt;
  Ji(0,2) = d[0];
  Ji(1,2) = d[1];

  // Rotation error: e_th = (th_j - th_i) - dth
  Ji(2,2) = -1.0;
  Jj(2,2) =  1.0;
}

static inline void applyIncrement(Pose2& x, const Eigen::Vector3d& dx) {
  x.x  += dx[0];
  x.y  += dx[1];
  x.th  = wrapAngle(x.th + dx[2]);
}

// Build and solve one Gauss-Newton step: H dx = -b
// We assemble H and b from edge contributions:
// H += J^T Omega J,  b += J^T Omega e
// and then solve for dx.
bool gaussNewtonStep(std::vector<Pose2>& X, const std::vector<Edge2>& edges, double damping = 0.0) {
  const int N = static_cast<int>(X.size());
  const int D = 3 * N;

  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(edges.size() * 36); // rough

  Eigen::VectorXd b = Eigen::VectorXd::Zero(D);

  auto addBlock = [&](int r0, int c0, const Eigen::Matrix3d& M) {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        if (M(r,c) != 0.0)
          trips.emplace_back(r0 + r, c0 + c, M(r,c));
  };

  double chi2 = 0.0;

  for (const auto& e : edges) {
    const Pose2& xi = X[e.i];
    const Pose2& xj = X[e.j];

    Eigen::Vector3d err = computeError(xi, xj, e.z);
    chi2 += err.transpose() * e.Omega * err;

    Eigen::Matrix<double,3,3> Ji, Jj;
    computeJacobians(xi, xj, Ji, Jj);

    Eigen::Matrix3d Hii = Ji.transpose() * e.Omega * Ji;
    Eigen::Matrix3d Hij = Ji.transpose() * e.Omega * Jj;
    Eigen::Matrix3d Hji = Jj.transpose() * e.Omega * Ji;
    Eigen::Matrix3d Hjj = Jj.transpose() * e.Omega * Jj;

    Eigen::Vector3d bi = Ji.transpose() * e.Omega * err;
    Eigen::Vector3d bj = Jj.transpose() * e.Omega * err;

    int ii = 3 * e.i;
    int jj = 3 * e.j;

    addBlock(ii, ii, Hii);
    addBlock(ii, jj, Hij);
    addBlock(jj, ii, Hji);
    addBlock(jj, jj, Hjj);

    b.segment<3>(ii) += bi;
    b.segment<3>(jj) += bj;
  }

  Eigen::SparseMatrix<double> H(D, D);
  H.setFromTriplets(trips.begin(), trips.end());

  if (damping > 0.0) {
    for (int k = 0; k < D; ++k) H.coeffRef(k,k) += damping;
  }

  for (int k = 0; k < 3; ++k) {
    int idx = k;
    H.coeffRef(idx, idx) += 1e12;
    b[idx] = 0.0;
  }

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  solver.compute(H);
  if (solver.info() != Eigen::Success) {
    std::cerr << "Decomposition failed\n";
    return false;
  }

  Eigen::VectorXd dx = solver.solve(-b);
  if (solver.info() != Eigen::Success) {
    std::cerr << "Solve failed\n";
    return false;
  }

  for (int i = 0; i < N; ++i) {
    Eigen::Vector3d dxi = dx.segment<3>(3*i);
    applyIncrement(X[i], dxi);
  }

  std::cout << "chi2: " << chi2 << "\n";
  return true;
}

int main() {
  std::vector<Pose2> X(4);
  X[0] = {0,0,0};
  X[1] = {1.1,0.0,0.02};
  X[2] = {2.1,0.1,0.01};
  X[3] = {3.2,0.0,-0.02};

  auto makeOmega = [](double sx, double sy, double sth) {
    Eigen::Matrix3d Cov = Eigen::Matrix3d::Zero();
    Cov(0,0) = sx*sx;
    Cov(1,1) = sy*sy;
    Cov(2,2) = sth*sth;
    return Cov.inverse();
  };

  std::vector<Edge2> edges;
  edges.push_back({0,1, Eigen::Vector3d(1.0, 0.0, 0.0), makeOmega(0.05,0.05,0.02)});
  edges.push_back({1,2, Eigen::Vector3d(1.0, 0.0, 0.0), makeOmega(0.05,0.05,0.02)});
  edges.push_back({2,3, Eigen::Vector3d(1.0, 0.0, 0.0), makeOmega(0.05,0.05,0.02)});
  edges.push_back({3,0, Eigen::Vector3d(-3.0, 0.0, 0.0), makeOmega(0.08,0.08,0.04)});

  for (int it = 0; it < 10; ++it) {
    if (!gaussNewtonStep(X, edges, /*damping=*/1e-6)) return 1;
  }

  std::cout << "\nOptimized poses:\n";
  for (int i = 0; i < (int)X.size(); ++i) {
    std::cout << i << ": " << X[i].x << " " << X[i].y << " " << X[i].th << "\n";
  }
  return 0;
}