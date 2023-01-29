/*
 * kinovalib.cpp
 *
 *  Created on: Mar 10, 2013
 *      Author: mdedonato
 */

#include <kinova_driver/kinova_arm_kinematics.h>
#include <kinova_driver/kinova_ros_types.h>
#include <boost/lexical_cast.hpp>


namespace kinova
{

std::string concatTfName(const std::string& prefix, const std::string name)
{
    std::stringstream ss;
    ss << prefix << name;
    return ss.str();
}
std::string concatTfName(const std::string& prefix, const std::string name, const int index)
{
    std::stringstream ss;
    ss << prefix << name << index;
    return ss.str();
}
std::vector<double> array2vector(double* array, int length)
{
    std::vector<double> result;
    for (int i=0; i<length; i++)
    {
        result.push_back(array[i]);
    }
    return result;
}

KinovaKinematics::KinovaKinematics(const std::shared_ptr<rclcpp::Node> node_handle, std::string& kinova_robotType)
    : kinova_robotType_(kinova_robotType), node_handle(node_handle)
{
    if (valid_kinovaRobotType(kinova_robotType_) == false)
    {
        RCLCPP_WARN(node_handle->get_logger(), "Invalid kinova_robotType error! Obtained: %s.", kinova_robotType_.c_str());
        return;
    }

//    tf_prefix_ = kinova_robotType_ + "_" + boost::lexical_cast<string>(same_type_index); // in case of multiple same_type robots
    tf_prefix_ = kinova_robotType_ + "_";

    // Maximum number of joints on Kinova-like robots:
    robot_category_ = kinova_robotType_[0];
    robot_category_version_ = kinova_robotType_[1]-'0';
    wrist_type_ = kinova_robotType_[2];
    arm_joint_number_ = kinova_robotType_[3]-'0';
    robot_mode_ = kinova_robotType_[4];
    finger_number_ = kinova_robotType_[5]-'0';
    int joint_total_number_ = arm_joint_number_ + finger_number_;

    if (kinova_robotType_.substr(0,4) == "j2n4" || kinova_robotType_.substr(0,4) == "m1n4")
        {
            // parameters stored in DSP chip
            double D1_ = 0.2755;
            if(robot_category_ == 'j') {
                double D2_ = 0.41;
                double D3_ = 0.2073;
                double e2_ = 0.0098;
            }
            else
            {
                double D2_ = 0.29;
                double D3_ = 0.1233;
                double e2_ = 0.0070;
            }
            double D4_ = 0.160;
            double wrist_deg_ = 60.0;

            if (!node_handle->has_parameter("D1_"))
                node_handle->declare_parameter("D1_", D1_);
            node_handle->get_parameter("D1_", D1_);
            if (!node_handle->has_parameter("D2_"))
                node_handle->declare_parameter("D2_", D2_);
            node_handle->get_parameter("D2_", D2_);
            if (!node_handle->has_parameter("D3_"))
                node_handle->declare_parameter("D3_", D3_);
            node_handle->get_parameter("D3_", D3_);
            if (!node_handle->has_parameter("e2_"))
                node_handle->declare_parameter("e2_", e2_);
            node_handle->get_parameter("e2_", e2_);
            if (!node_handle->has_parameter("D4_"))
                node_handle->declare_parameter("D4_", D4_);
            node_handle->get_parameter("D4_", D4_);
            if (!node_handle->has_parameter("wrist_deg_"))
                node_handle->declare_parameter("wrist_deg_", wrist_deg_);
            node_handle->get_parameter("wrist_deg_", wrist_deg_);

            double aa = wrist_deg_/2 * M_PI/180.0, sa = sin(aa), s2a = sin(2*aa); // temp parameters

            // avoid use dynamic array DH_a[arm_joint_number_], but vector for DH parameters.
            double DH_a[4] = {0,  D2_, 0, 0};
            double DH_d[4] = {D1_, 0, -e2_, -(D3_ + D4_)};
            double DH_alpha[4] = {M_PI/2, M_PI, M_PI/2, M_PI};
            // DH_theta = DH_theta_sign*Q + DH_theta_offset
            double DH_theta_sign[4] = {-1, 1, 1, 1};
            double DH_theta_offset[4] = {0, -M_PI/2, M_PI/2, 3/2*M_PI};

            // copy local array values to class-scope vector.
            DH_a_ = array2vector(DH_a, arm_joint_number_);
            DH_d_ = array2vector(DH_d, arm_joint_number_);
            DH_alpha_ = array2vector(DH_alpha, arm_joint_number_);
            DH_theta_sign_ = array2vector(DH_theta_sign, arm_joint_number_);
            DH_theta_offset_ = array2vector(DH_theta_offset, arm_joint_number_);
        }
    else if (kinova_robotType_.substr(0,4) == "j2n6" || kinova_robotType_.substr(0,4) == "m1n6" )
    {
        // parameters stored in DSP chip
        double D1_ = 0.2755;
        if(robot_category_ == 'j') {
            double D2_ = 0.41;
            double D3_ = 0.2073;
            double e2_ = 0.0098;
        }
        else
        {
            double D2_ = 0.29;
            double D3_ = 0.1233;
            double e2_ = 0.0070;
        }
        double D4_ = 0.0741;
        double D5_ = 0.0741;
        double D6_ = 0.160;
        double wrist_deg_ = 60.0;

        if (!node_handle->has_parameter("D1_"))
            node_handle->declare_parameter("D1_", D1_);
        node_handle->get_parameter("D1_", D1_);
        if (!node_handle->has_parameter("D2_"))
            node_handle->declare_parameter("D2_", D2_);
        node_handle->get_parameter("D2_", D2_);
        if (!node_handle->has_parameter("D3_"))
            node_handle->declare_parameter("D3_", D3_);
        node_handle->get_parameter("D3_", D3_);
        if (!node_handle->has_parameter("e2_"))
            node_handle->declare_parameter("e2_", e2_);
        node_handle->get_parameter("e2_", e2_);
        if (!node_handle->has_parameter("D4_"))
            node_handle->declare_parameter("D4_", D4_);
        node_handle->get_parameter("D4_", D4_);
        if (!node_handle->has_parameter("D5_"))
            node_handle->declare_parameter("D5_", D5_);
        node_handle->get_parameter("D5_", D5_);
        if (!node_handle->has_parameter("D6_"))
            node_handle->declare_parameter("D6_", D6_);
        node_handle->get_parameter("D6_", D6_);
        if (!node_handle->has_parameter("wrist_deg_"))
            node_handle->declare_parameter("wrist_deg_", wrist_deg_);
        node_handle->get_parameter("wrist_deg_", wrist_deg_);
        
        double aa = wrist_deg_/2 * M_PI/180.0, sa = sin(aa), s2a = sin(2*aa); // temp parameters

        // avoid use dynamic array DH_a[arm_joint_number_], but vector for DH parameters.
        double DH_a[6] = {0,  D2_, 0, 0, 0, 0};
        double DH_d[6] = {D1_, 0, -e2_, -(D3_ + sa/s2a * D4_), -(sa/s2a * D4_ + sa/s2a * D5_), -(sa/s2a * D5_ + D6_)};
        double DH_alpha[6] = {M_PI/2, M_PI, M_PI/2, 2*aa, 2*aa, M_PI};
        // DH_theta = DH_theta_sign*Q + DH_theta_offset
        double DH_theta_sign[6] = {-1, 1, 1, 1, 1, 1};
        double DH_theta_offset[6] = {0, -M_PI/2, +M_PI/2, 0, -M_PI, +M_PI/2};

        // copy local array values to class-scope vector.
        DH_a_ = array2vector(DH_a, arm_joint_number_);
        DH_d_ = array2vector(DH_d, arm_joint_number_);
        DH_alpha_ = array2vector(DH_alpha, arm_joint_number_);
        DH_theta_sign_ = array2vector(DH_theta_sign, arm_joint_number_);
        DH_theta_offset_ = array2vector(DH_theta_offset, arm_joint_number_);
    }
    else if (kinova_robotType_.substr(0,4) == "j2s6")
    {
        // parameters stored in DSP chip
        double D1_ = 0.2755;
        double D2_ = 0.41;
        double D3_ = 0.2073;
        double D4_ = 0.1038;
        double D5_ = 0.1038;
        double D6_ = 0.160;
        double e2_ = 0.0098;

        if (!node_handle->has_parameter("D1_"))
            node_handle->declare_parameter("D1_", D1_);
        node_handle->get_parameter("D1_", D1_);
        if (!node_handle->has_parameter("D2_"))
            node_handle->declare_parameter("D2_", D2_);
        node_handle->get_parameter("D2_", D2_);
        if (!node_handle->has_parameter("D3_"))
            node_handle->declare_parameter("D3_", D3_);
        node_handle->get_parameter("D3_", D3_);
        if (!node_handle->has_parameter("D4_"))
            node_handle->declare_parameter("D4_", D4_);
        node_handle->get_parameter("D4_", D4_);
        if (!node_handle->has_parameter("D5_"))
            node_handle->declare_parameter("D5_", D5_);
        node_handle->get_parameter("D5_", D5_);
        if (!node_handle->has_parameter("D6_"))
            node_handle->declare_parameter("D6_", D6_);
        node_handle->get_parameter("D6_", D6_);
        if (!node_handle->has_parameter("e2_"))
            node_handle->declare_parameter("e2_", e2_);
        node_handle->get_parameter("e2_", e2_);
        
        // avoid use dynamic array DH_a[arm_joint_number_], but vector for DH parameters.
        double DH_a[6] = {0,  D2_, 0, 0, 0, 0};
        double DH_d[6] = {-D1_, 0, -e2_, -(D3_ + D4_), 0, -(D5_ + D6_)};
        double DH_alpha[6] = {M_PI/2, M_PI, M_PI/2, M_PI/2, M_PI/2, M_PI};
        // DH_theta = DH_theta_sign*Q + DH_theta_offset
        double DH_theta_sign[6] = {1, 1, 1, 1, 1, 1};
        double DH_theta_offset[6] = {-M_PI, M_PI/2, M_PI/2, 0, 0, -M_PI/2};

        // copy local array values to class-scope vector.
        DH_a_ = array2vector(DH_a, arm_joint_number_);
        DH_d_ = array2vector(DH_d, arm_joint_number_);
        DH_alpha_ = array2vector(DH_alpha, arm_joint_number_);
        DH_theta_sign_ = array2vector(DH_theta_sign, arm_joint_number_);
        DH_theta_offset_ = array2vector(DH_theta_offset, arm_joint_number_);
    }
    else if (kinova_robotType_.substr(0,4) == "j2s7")
    {
        // parameters stored in DSP chip
        double D1_ = 0.2755;
        double D2_ = 0.205;
        double D3_ = 0.205;
        double D4_ = 0.2073;
        double D5_ = 0.1038;
        double D6_ = 0.1038;
        double D7_ = 0.160;
        double e2_ = 0.0098;

        if (!node_handle->has_parameter("D1_"))
            node_handle->declare_parameter("D1_", D1_);
        node_handle->get_parameter("D1_", D1_);
        if (!node_handle->has_parameter("D2_"))
            node_handle->declare_parameter("D2_", D2_);
        node_handle->get_parameter("D2_", D2_);
        if (!node_handle->has_parameter("D3_"))
            node_handle->declare_parameter("D3_", D3_);
        node_handle->get_parameter("D3_", D3_);
        if (!node_handle->has_parameter("D4_"))
            node_handle->declare_parameter("D4_", D4_);
        node_handle->get_parameter("D4_", D4_);
        if (!node_handle->has_parameter("D5_"))
            node_handle->declare_parameter("D5_", D5_);
        node_handle->get_parameter("D5_", D5_);
        if (!node_handle->has_parameter("D6_"))
            node_handle->declare_parameter("D6_", D6_);
        node_handle->get_parameter("D6_", D6_);
        if (!node_handle->has_parameter("D7_"))
            node_handle->declare_parameter("D7_", D7_);
        node_handle->get_parameter("D7_", D7_);
        if (!node_handle->has_parameter("e2_"))
            node_handle->declare_parameter("e2_", e2_);
        node_handle->get_parameter("e2_", e2_);

        // avoid use dynamic array DH_a[arm_joint_number_], but vector for DH parameters.
        double DH_a[7] = {0,  0, 0, 0, 0, 0, 0};
        double DH_d[7] = {-D1_, 0, -(D2_+ D3_), -e2_, -(D4_ + D5_), 0, -(D6_ + D7_)};
        double DH_alpha[7] = {M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI};
        // DH_theta = DH_theta_sign*Q + DH_theta_offset
        double DH_theta_sign[7] = {-1, 1, 1, 1, 1, 1, 1};
        double DH_theta_offset[7] = {0, 0, 0, 0, 0, 0, 0};

        // copy local array values to class-scope vector.
        DH_a_ = array2vector(DH_a, arm_joint_number_);
        DH_d_ = array2vector(DH_d, arm_joint_number_);
        DH_alpha_ = array2vector(DH_alpha, arm_joint_number_);
        DH_theta_sign_ = array2vector(DH_theta_sign, arm_joint_number_);
        DH_theta_offset_ = array2vector(DH_theta_offset, arm_joint_number_);
    }
    else
    {
        // special parameters for custom robot or other cases
        printf("Please specify the kinematic of robots other than jaco and mico!\n");
    }

    baseFrame = "root";
    if (!node_handle->has_parameter("base_frame"))
        node_handle->declare_parameter("base_frame", baseFrame);
    node_handle->get_parameter("base_frame", baseFrame);

    broadcaster_ = new tf2_ros::TransformBroadcaster(node_handle);
}

tf2::Transform KinovaKinematics::DHParam2Transform(float d, float theta, float a, float alpha)
{
    tf2::Transform transform;
    tf2::Quaternion rotation_q(0, 0, 0, 1);
    tf2::Matrix3x3 rot_matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
    tf2::Vector3 translation_v(0, 0, 0);

    rot_matrix.setValue(cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),
                        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),
                        0,           sin(alpha),             cos(alpha));
    rot_matrix.getRotation(rotation_q);
    transform.setRotation(rotation_q.normalize());

    translation_v.setValue(a*cos(theta), a*sin(theta), d);
    transform.setOrigin(translation_v);
    return transform;
}

/**************************************/
/*                                    */
/*   Compute and publish transform    */
/*                                    */
/**************************************/
// IMPORTANT!!! In the robot DSP chip, the classical D-H parameters are used to define the robot. Therefore, the frame definition is different comparing with the frames in URDF model.
void KinovaKinematics::updateForward(float* Q)
{
    tf2::Transform transform;
    // the orientation of frame0 is differently defined starting from Jaco 6 spherical robot.
    if(kinova_robotType_.substr(0,4) == "j2s6" || kinova_robotType_.substr(0,4) == "j2s7")
    {
        transform = DHParam2Transform(0, 0, 0, M_PI);
    }
    else
    {
        transform = DHParam2Transform(0, 0, 0, 0);
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node_handle->get_clock()->now();
    t.header.frame_id = baseFrame;
    t.child_frame_id = concatTfName(tf_prefix_, "link_base");
    t.transform.translation.x = transform.getOrigin()[0];
    t.transform.translation.y = transform.getOrigin()[1];
    t.transform.translation.z = transform.getOrigin()[2];
    t.transform.rotation.x = transform.getRotation()[0];
    t.transform.rotation.y = transform.getRotation()[1];
    t.transform.rotation.z = transform.getRotation()[2];
    t.transform.rotation.w = transform.getRotation()[3];
    broadcaster_->sendTransform(t);

    double DH_theta_i;
    for (int i = 0; i<arm_joint_number_; i++)
    {
        DH_theta_i = DH_theta_sign_[i]*Q[i] + DH_theta_offset_[i];
        transform = DHParam2Transform(DH_d_[i], DH_theta_i, DH_a_[i], DH_alpha_[i]);
        if(i==0)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = node_handle->get_clock()->now();
            t.header.frame_id = concatTfName(tf_prefix_, "link_base");
            t.child_frame_id = concatTfName(tf_prefix_, "link_1");
            t.transform.translation.x = transform.getOrigin()[0];
            t.transform.translation.y = transform.getOrigin()[1];
            t.transform.translation.z = transform.getOrigin()[2];
            t.transform.rotation.x = transform.getRotation()[0];
            t.transform.rotation.y = transform.getRotation()[1];
            t.transform.rotation.z = transform.getRotation()[2];
            t.transform.rotation.w = transform.getRotation()[3];
            broadcaster_->sendTransform(t);
        }
        else
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = node_handle->get_clock()->now();
            t.header.frame_id = concatTfName(tf_prefix_, "link_", i);
            t.child_frame_id = concatTfName(tf_prefix_, "link_", i+1);
            t.transform.translation.x = transform.getOrigin()[0];
            t.transform.translation.y = transform.getOrigin()[1];
            t.transform.translation.z = transform.getOrigin()[2];
            t.transform.rotation.x = transform.getRotation()[0];
            t.transform.rotation.y = transform.getRotation()[1];
            t.transform.rotation.z = transform.getRotation()[2];
            t.transform.rotation.w = transform.getRotation()[3];
            broadcaster_->sendTransform(t);
        }
    }

}

}  // namespace kinova
