#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <cmath>
#include <iostream>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

// --- Math Helpers ---

static inline double wrapAngle(double a) {
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
    Eigen::Matrix3d Omega;
};

static inline Eigen::Matrix2d R(double th) {
    double c = std::cos(th), s = std::sin(th);
    Eigen::Matrix2d M;
    M << c, -s, s, c;
    return M;
}

static inline Eigen::Vector3d computeError(const Pose2& xi, const Pose2& xj, const Eigen::Vector3d& z) {
    Eigen::Vector2d ti(xi.x, xi.y), tj(xj.x, xj.y);
    Eigen::Vector2d dt = tj - ti;
    Eigen::Vector2d trans_err = R(xi.th).transpose() * dt - z.head<2>();
    double rot_err = wrapAngle((xj.th - xi.th) - z[2]);
    Eigen::Vector3d e;
    e << trans_err, rot_err;
    return e;
}

static inline void computeJacobians(
    const Pose2& xi, const Pose2& xj,
    Eigen::Matrix<double,3,3>& Ji, Eigen::Matrix<double,3,3>& Jj)
{
    Ji.setZero(); Jj.setZero();
    Eigen::Matrix2d RiT = R(xi.th).transpose();
    Ji.block<2,2>(0,0) = -RiT;
    Jj.block<2,2>(0,0) =  RiT;
    double c = std::cos(xi.th), s = std::sin(xi.th);
    Eigen::Matrix2d dRiT;
    dRiT << -s, c, -c, -s;
    Eigen::Vector2d dt(xj.x - xi.x, xj.y - xi.y);
    Eigen::Vector2d d = dRiT * dt;
    Ji(0,2) = d[0]; Ji(1,2) = d[1];
    Ji(2,2) = -1.0; Jj(2,2) = 1.0;
}

static inline void applyIncrement(Pose2& x, const Eigen::Vector3d& dx) {
    x.x += dx[0];
    x.y += dx[1];
    x.th = wrapAngle(x.th + dx[2]);
}

bool gaussNewtonStep(std::vector<Pose2>& X, const std::vector<Edge2>& edges, double damping = 0.0) {
    const int N = static_cast<int>(X.size());
    const int D = 3 * N;
    std::vector<Eigen::Triplet<double>> trips;
    Eigen::VectorXd b = Eigen::VectorXd::Zero(D);

    auto addBlock = [&](int r0, int c0, const Eigen::Matrix3d& M) {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                if (std::abs(M(r,c)) > 1e-12)
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
        addBlock(3*e.i, 3*e.i, Ji.transpose() * e.Omega * Ji);
        addBlock(3*e.i, 3*e.j, Ji.transpose() * e.Omega * Jj);
        addBlock(3*e.j, 3*e.i, Jj.transpose() * e.Omega * Ji);
        addBlock(3*e.j, 3*e.j, Jj.transpose() * e.Omega * Jj);
        b.segment<3>(3*e.i) += Ji.transpose() * e.Omega * err;
        b.segment<3>(3*e.j) += Jj.transpose() * e.Omega * err;
    }

    Eigen::SparseMatrix<double> H(D, D);
    H.setFromTriplets(trips.begin(), trips.end());
    if (damping > 0.0) {
        for (int k = 0; k < D; ++k) H.coeffRef(k,k) += damping;
    }
    // Fix first pose
    for (int k = 0; k < 3; ++k) { H.coeffRef(k, k) += 1e12; b[k] = 0.0; }

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(H);
    if (solver.info() != Eigen::Success) return false;
    Eigen::VectorXd dx = solver.solve(-b);
    for (int i = 0; i < N; ++i) applyIncrement(X[i], dx.segment<3>(3*i));
    return true;
}

// --- ROS 2 Node ---

class PgoNode : public rclcpp::Node {
public:
    PgoNode() : Node("pgo_backend") {
        // STEP 1: Change subscription from "/odom" to "/odometry/filtered"
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&PgoNode::odom_callback, this, std::placeholders::_1));
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Initialize the first pose at the origin
        if (X.empty()) {
            X.push_back({0.0, 0.0, 0.0}); 
        }
        
        RCLCPP_INFO(this->get_logger(), "PGO Backend Initialized. Listening to Fused EKF Odometry.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extract fused position
        double cur_x = msg->pose.pose.position.x;
        double cur_y = msg->pose.pose.position.y;
        
        // Extract fused orientation (Yaw) from Quaternion
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double cur_th = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);

        Pose2 last = X.back();
        
        // Keyframe Trigger: Add a node if we've moved > 0.3m OR rotated > 15 degrees
        double travel_dist = std::hypot(cur_x - last.x, cur_y - last.y);
        double rotation_dist = std::abs(wrapAngle(cur_th - last.th));

        if (travel_dist > 0.3 || rotation_dist > 0.26) { // ~15 degrees
            // Calculate the relative transform (The "Edge")
            Eigen::Vector3d z(cur_x - last.x, cur_y - last.y, wrapAngle(cur_th - last.th));
            X.push_back({cur_x, cur_y, cur_th});
            
            // Information Matrix (Weights): We trust rotation (IMU) more than translation (LiDAR)
            Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
            info(0,0) = 500.0;  // X weight
            info(1,1) = 500.0;  // Y weight
            info(2,2) = 3000.0; // Yaw weight (Higher because of IMU stability)

            edges.push_back({(int)X.size()-2, (int)X.size()-1, z, info});
            
            // Run 5 iterations of Gauss-Newton to smooth the graph
            for (int it = 0; it < 5; ++it) {
                gaussNewtonStep(X, edges, 1e-6);
            }

            RCLCPP_INFO(this->get_logger(), "Added Keyframe %ld. Optimization complete.", X.size());

            // --- RADIUS SEARCH FOR LOOP CLOSURE ---
            // Ensure we have enough history to actually "loop"
            if (X.size() > 20) {
                int current_idx = (int)X.size() - 1;
                Pose2 current_pose = X[current_idx];

                // Search backwards, skipping the last 10 nodes (don't match with immediate past)
                for (int i = 0; i < current_idx - 10; ++i) {
                    double dist = std::hypot(current_pose.x - X[i].x, current_pose.y - X[i].y);
                    
                    // If an old node is within a 1.0 meter radius
                    if (dist < 1.0) {
                        RCLCPP_INFO(this->get_logger(), 
                            "\n====================================\n"
                            "LOOP CLOSURE CANDIDATE FOUND!\n"
                            "Current Node: %d is near Old Node: %d\n"
                            "Distance: %.2f meters\n"
                            "====================================\n", 
                            current_idx, i, dist);
                        
                        // NOTE: We only log it right now. To actually close the loop, 
                        // we must send a request to the ICP node to see if the current 
                        // LiDAR scan matches Node i's LiDAR scan.
                        
                        break; // Stop searching once we find one good candidate per keyframe
                    }
                }
            }
            // -------------------------------------------
        }

        // Always publish the map->odom transform to keep the TF tree linked
        publish_correction(cur_x, cur_y, cur_th);
    }

    void publish_correction(double fused_x, double fused_y, double fused_th) {
        // The correction is the difference between our Optimized map pose 
        // and the EKF's current fused odometry pose.
        Pose2 opt = X.back();
        
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "odom";

        // Math: T_map_to_odom = T_map_to_base * inv(T_odom_to_base)
        t.transform.translation.x = opt.x - fused_x;
        t.transform.translation.y = opt.y - fused_y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, wrapAngle(opt.th - fused_th));
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<Pose2> X;
    std::vector<Edge2> edges;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PgoNode>());
    rclcpp::shutdown();
    return 0;
}