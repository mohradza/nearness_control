#include <nearness_control/nearness_controller_3d.h>

namespace nearness_3d{
NearnessController3D::NearnessController3D(const ros::NodeHandle &node_handle,
                                       const ros::NodeHandle &private_node_handle)
      :nh_(node_handle),
       pnh_(private_node_handle) {
      this->init();
  }

void NearnessController3D::init() {
    // Set up dynamic reconfigure
    reconfigure_server_.reset(new ReconfigureServer(config_mutex_, pnh_));
    ReconfigureServer::CallbackType f = boost::bind(&NearnessController3D::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);

    // Set up subscribers and callbacks
    sub_pcl_ = nh_.subscribe("/OHRAD_X3/ouster_points", 1, &NearnessController3D::pclCb, this);

    // Set up publishers
    pub_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl_out",1);
    pub_mu_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("mu_pcl_out",1);
    pub_mu_pcl2_ = nh_.advertise<sensor_msgs::PointCloud2>("mu_pcl_test_out",1);
    pub_dist_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>("dist_test_out",1);
    pub_Y00_ = nh_.advertise<sensor_msgs::PointCloud2>("Y00",1);
    pub_Y0p1_ = nh_.advertise<sensor_msgs::PointCloud2>("Y0p1",1);
    pub_Yp1p1_ = nh_.advertise<sensor_msgs::PointCloud2>("Yp1p1",1);
    pub_Yn1p1_ = nh_.advertise<sensor_msgs::PointCloud2>("Yn1p1",1);
    pub_Y0p2_ = nh_.advertise<sensor_msgs::PointCloud2>("Y0p2",1);
    pub_Yp1p2_ = nh_.advertise<sensor_msgs::PointCloud2>("Yp1p2",1);
    pub_Yn1p2_ = nh_.advertise<sensor_msgs::PointCloud2>("Yn1p2",1);
    pub_Yp2p2_ = nh_.advertise<sensor_msgs::PointCloud2>("Yp2p2",1);
    pub_Yn2p2_ = nh_.advertise<sensor_msgs::PointCloud2>("Yn2p2",1);
    //pub_control_commands_ = nh_.advertise<geometry_msgs::Twist>("control_commands", 10);

    // Import parameters
    pnh_.param("pcl_height", pcl_height_, 64);
    pnh_.param("pcl_width",  pcl_width_, 360);
    pnh_.param("pcl_vertical_spread", pcl_vertical_spread_, 64);
    pnh_.param("ring", test_ring_, 1.0);
    // pnh_.param("pcl_", pcl_height_, 64);

    frame_id_ = "OHRAD_X3";

    enable_debug_ = true;
    phi_start_ = 3.14159;
    dphi_ = abs(2*phi_start_)/float(pcl_width_);
    float phi_val = 0.0;
    for(int i=0; i < pcl_width_; i++){
        phi_val = phi_start_ - float(i)*dphi_;
        if(phi_val < 0.0){
            phi_view_vec_.push_back(phi_val + 2*3.14159);
        } else {
        phi_view_vec_.push_back(phi_val);
      }
    }

    // for(int i=0; i < pcl_width_; i++){
    //     phi_view_vec_.push_back(float(i)*dphi_);
    // }

    theta_start_ = 3.14159;
    dtheta_ = abs(theta_start_)/float(pcl_height_);
    float theta_val = 0.0;
    for(int i=1; i <= pcl_height_; i++){
        theta_val = theta_start_ - float(i)*dtheta_;
        // if(theta_val < 0.0){
        //     theta_view_vec_.push_back(theta_val + 2*3.1459);
        // } else {
            theta_view_vec_.push_back(theta_val);
        // }
    }

    generateProjectionShapes();

} // End of init

void NearnessController3D::configCb(Config &config, uint32_t level)
{
    config_ = config;

    test_ring_ = config_.ring;
    // Controller gains
    //u_k_hb_1_ = config_.forward_speed_k_hb_1;
    //u_k_hb_2_ = config_.forward_speed_k_hb_2;

}

void NearnessController3D::pclCb(const sensor_msgs::PointCloud2ConstPtr& pcl_msg){
  new_pcl_ = true;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr dist_test_out (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg (*pcl_msg, *cloud_in);
  size_t pcl_size = cloud_in->points.size();

  int num_rings = cloud_in->height;
  int num_ring_points = cloud_in->width;

  int test_ring = test_ring_;

  // Convert from cartesian coordinates to spherical nearness
  std::vector<float> mu_sphere;
  cloud_out_.clear();
  mu_cloud_out_.clear();

  for(int i = 1; i <= num_rings-1; i++){
      for(int j = 0; j < num_ring_points; j++){
          //pcl::PointXYZ p = cloud_in->points[(i*num_ring_points+(num_ring_points-1)) - j];
          pcl::PointXYZ p = cloud_in->points[i*num_ring_points + j];
          cloud_out_.push_back(p);
          float dist = sqrt(pow(p.x,2) + pow(p.y,2) + pow(p.z,2));
          float mu = 1/dist;
          mu_sphere_.push_back(mu);

          // Convert back to cartesian for viewing
          if(enable_debug_){
              pcl::PointXYZ mu_p (mu*sin(theta_view_vec_[i])*cos(phi_view_vec_[j]), mu*sin(theta_view_vec_[i])*sin(phi_view_vec_[j]), mu*cos(theta_view_vec_[i]) );
              mu_cloud_out_.push_back(mu_p);
          }
      }
  }

  if(enable_debug_){
      sensor_msgs::PointCloud2 pcl_out_msg;
      pcl::toROSMsg(cloud_out_, pcl_out_msg);
      pcl_out_msg.header = pcl_msg->header;
      pub_pcl_.publish(pcl_out_msg);

      sensor_msgs::PointCloud2 mu_out_msg;
      pcl::toROSMsg(mu_cloud_out_, mu_out_msg);
      mu_out_msg.header = pcl_msg->header;
      pub_mu_pcl_.publish(mu_out_msg);

  }

}

void NearnessController3D::generateProjectionShapes(){
    int num_rings = 64;
    int num_ring_points = 360;

    // Inclination angle
    float theta_start = 0;
    float theta_end = M_PI;
    float dtheta = (theta_end - theta_start)/float(num_rings);
    std::vector<float> theta_view_vec;
    for(int i = 0; i <= num_rings; i++){
        theta_view_vec.push_back(theta_end - float(i)*dtheta);
    }

    //float phi_start = 0;
    //float phi_end = 2*M_PI;
    float phi_start = M_PI;
    float phi_end = -M_PI;
    float dphi = (phi_start - phi_end)/float(num_ring_points);
    std::vector<float> phi_view_vec;
    for(int i = 0; i < num_ring_points; i++){
      //phi_view_vec.push_back(float(i)*dphi);
      phi_view_vec.push_back(phi_start - float(i)*dphi);
    }

    // int m_lim = 2;
    // int l_lim = 2;
    // float d_mat[2*m_lim + 1][l_lim];
    // float lP[m_lim + 1][l_lim];
    // float Knorm[m_lim + 1][l_lim];
    // const std::complex<double> i(0, 1);
    // for(int m = 0; m <= m_lim; m++){
    //   for(int l = 0; l < 2; l++){
    //       Knorm[m][l] = sqrt(((2*l)/(4*M_PI))*(fact(l-m)/fact(l+m)));
    //       lP[m][l] = 0.0;
    //   }
    // }


    for(int i = 1; i <= num_rings-1; i++){
      for(int j = 0; j < num_ring_points; j++){

        float phi = phi_view_vec[j];
        float theta = theta_view_vec[i];
        float intensity_val = 0.0;
        float max_intensity = -1.0;
        float min_intensity = .6;



        // for(int m = -m_lim; m <= m_lim; m++){
        //   for(int l = 0; l < 2; l++){
        //     if(m >= 0){
        //       d_mat[m+m_lim][l] = lP[m][l]*cos(theta)*cos(m*phi);
        //     } else {
        //       d_mat[m+m_lim][l] = lP[abs(m)][l]*cos(theta)*sin(abs(m)*phi);
        //     }
        //   }
        // }

        // if(sgn(d) > 0){
        //   intensity_val = max_intensity;
        // } else {
        //   intensity_val = min_intensity;
        // }
        // float d_abs = abs(d);
        // pcl::PointXYZ Y00_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
        // pcl::PointXYZI Y00_pcli;
        // Y00_pcli.x = Y00_pcl.x; Y00_pcli.y = Y00_pcl.y; Y00_pcli.z = Y00_pcl.z;
        // Y00_pcli.intensity = intensity_val;
        // Y00_.push_back(Y00_pcli);


            // Y00 -- Good
            float d = sqrt(1/(4*M_PI));
            float d_abs = abs(d);
            Y00_vec_.push_back(d);
            pcl::PointXYZ Y00_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
            pcl::PointXYZI Y00_pcli;
            Y00_pcli.x = Y00_pcl.x; Y00_pcli.y = Y00_pcl.y; Y00_pcli.z = Y00_pcl.z;
            if(d >= 0){
              intensity_val = max_intensity;
            } else {
              intensity_val = min_intensity;
            }
            Y00_pcli.intensity = intensity_val;
            Y00_.push_back(Y00_pcli);


            //Y0p1 -- Good
            d = sqrt(3.0/(4.0*M_PI))*cos(theta);
            d_abs = abs(d);
            Y0p1_vec_.push_back(d);
            pcl::PointXYZ Y0p1_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
            pcl::PointXYZI Y0p1_pcli;
            Y0p1_pcli.x = Y0p1_pcl.x; Y0p1_pcli.y = Y0p1_pcl.y; Y0p1_pcli.z = Y0p1_pcl.z;
            if(d >= 0){
              intensity_val = max_intensity;
            } else {
              intensity_val = min_intensity;
            }
            Y0p1_pcli.intensity = intensity_val;
            Y0p1_.push_back(Y0p1_pcli);

            //Yp1p1 -- Good
            d = sqrt(3.0/(4.0*M_PI))*cos(phi)*sin(theta);
            d_abs = abs(d);
            Yp1p1_vec_.push_back(d);
            pcl::PointXYZ Yp1p1_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
            pcl::PointXYZI Yp1p1_pcli;
            Yp1p1_pcli.x = Yp1p1_pcl.x; Yp1p1_pcli.y = Yp1p1_pcl.y; Yp1p1_pcli.z = Yp1p1_pcl.z;
            if(d >= 0){
              intensity_val = max_intensity;
            } else {
              intensity_val = min_intensity;
            }
            Yp1p1_pcli.intensity = intensity_val;
            Yp1p1_.push_back(Yp1p1_pcli);

            //Yn1p1 -- Good
            d = sqrt(3.0/(4.0*M_PI))*sin(phi)*sin(theta);
            d_abs = abs(d);
            Yn1p1_vec_.push_back(d);
            pcl::PointXYZ Yn1p1_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
            pcl::PointXYZI Yn1p1_pcli;
            Yn1p1_pcli.x = Yn1p1_pcl.x; Yn1p1_pcli.y = Yn1p1_pcl.y; Yn1p1_pcli.z = Yn1p1_pcl.z;
            if(sgn(d) > 0){
              intensity_val = max_intensity;
            } else {
              intensity_val = min_intensity;
            }
            Yn1p1_pcli.intensity = intensity_val;
            Yn1p1_.push_back(Yn1p1_pcli);

            //Y0p2 -- Good
            d = sqrt(5.0/(16.0*M_PI))*(3.0*pow(cos(theta),2)-1);
            d_abs = abs(d);
            Y0p2_vec_.push_back(d);
            pcl::PointXYZ Y0p2_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
            pcl::PointXYZI Y0p2_pcli;
            Y0p2_pcli.x = Y0p2_pcl.x; Y0p2_pcli.y = Y0p2_pcl.y; Y0p2_pcli.z = Y0p2_pcl.z;
            if(d >= 0){
              intensity_val = max_intensity;
            } else {
              intensity_val = min_intensity;
            }
            Y0p2_pcli.intensity = intensity_val;
            Y0p2_.push_back(Y0p2_pcli);

            //Yp1p2 -- Good
            d = sqrt(15.0/(4.0*M_PI))*cos(phi)*sin(theta)*cos(theta);
            d_abs = abs(d);
            Yp1p2_vec_.push_back(d);
            pcl::PointXYZ Yp1p2_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
            pcl::PointXYZI Yp1p2_pcli;
            Yp1p2_pcli.x = Yp1p2_pcl.x; Yp1p2_pcli.y = Yp1p2_pcl.y; Yp1p2_pcli.z = Yp1p2_pcl.z;
            if(d >= 0){
              intensity_val = max_intensity;
            } else {
              intensity_val = min_intensity;
            }
            Yp1p2_pcli.intensity = intensity_val;
            Yp1p2_.push_back(Yp1p2_pcli);

            //Yn1p2 -- Good
            d = sqrt(15.0/(4.0*M_PI))*sin(phi)*sin(theta)*cos(theta);
            d_abs = abs(d);
            Yn1p2_vec_.push_back(d);
            pcl::PointXYZ Yn1p2_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
            pcl::PointXYZI Yn1p2_pcli;
            Yn1p2_pcli.x = Yn1p2_pcl.x; Yn1p2_pcli.y = Yn1p2_pcl.y; Yn1p2_pcli.z = Yn1p2_pcl.z;
            if(d >= 0){
              intensity_val = max_intensity;
            } else {
              intensity_val = min_intensity;
            }
            Yn1p2_pcli.intensity = intensity_val;
            Yn1p2_.push_back(Yn1p2_pcli);

            //Yp2p2 -- Good
            d = sqrt(15.0/(16.0*M_PI))*(pow(cos(phi),2)-pow(sin(phi),2))*pow(sin(theta),2);
            d_abs = abs(d);
            Yp2p2_vec_.push_back(d);
            pcl::PointXYZ Yp2p2_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
            pcl::PointXYZI Yp2p2_pcli;
            Yp2p2_pcli.x = Yp2p2_pcl.x; Yp2p2_pcli.y = Yp2p2_pcl.y; Yp2p2_pcli.z = Yp2p2_pcl.z;
            if(d >= 0){
              intensity_val = max_intensity;
            } else {
              intensity_val = min_intensity;
            }
            Yp2p2_pcli.intensity = intensity_val;
            Yp2p2_.push_back(Yp2p2_pcli);

            //Yn2p2 -- Good
            d = sqrt(15.0/(4.0*M_PI))*sin(phi)*cos(phi)*pow(sin(theta),2);
            d_abs = abs(d);
            Yn2p2_vec_.push_back(d);
            pcl::PointXYZ Yn2p2_pcl (d_abs*sin(theta)*cos(phi), d_abs*sin(theta)*sin(phi), d_abs*cos(theta) );
            pcl::PointXYZI Yn2p2_pcli;
            Yn2p2_pcli.x = Yn2p2_pcl.x; Yn2p2_pcli.y = Yn2p2_pcl.y; Yn2p2_pcli.z = Yn2p2_pcl.z;
            if(d >= 0){
              intensity_val = max_intensity;
            } else {
              intensity_val = min_intensity;
            }
            Yn2p2_pcli.intensity = intensity_val;
            Yn2p2_.push_back(Yn2p2_pcli);
        }

        // Create array of projection shape arrays
        // Indexing is i*ring_num + j
        vector<vector<float>> shape_mat;
        shape_mat.push_back(Y00_vec_);
        shape_mat.push_back(Y0p1_vec_);
        shape_mat.push_back(Yp1p1_vec_);
        shape_mat.push_back(Yn1p1_vec_);
        shape_mat.push_back(Y0p2_vec_);
        shape_mat.push_back(Yp1p2_vec_);
        shape_mat.push_back(Yn1p2_vec_);
        shape_mat.push_back(Yp2p2_vec_);
        shape_mat.push_back(Yn2p2_vec_);

        // This is stricly for making rviz look nice
        pcl::PointXYZI Yp_pcli;
        Yp_pcli.x = 0.0; Yp_pcli.y = 0.0; Yp_pcli.z = 0.0;
        Yp_pcli.intensity = 1.0;
        Y00_.push_back(Yp_pcli);
        Y0p1_.push_back(Yp_pcli);
        Yp1p1_.push_back(Yp_pcli);
        Yn1p1_.push_back(Yp_pcli);
        Y0p2_.push_back(Yp_pcli);
        Yp1p2_.push_back(Yp_pcli);
        Yn1p2_.push_back(Yp_pcli);
        Yp2p2_.push_back(Yp_pcli);
        Yn2p2_.push_back(Yp_pcli);
        pcl::PointXYZI Yn_pcli;
        Yn_pcli.x = 0.0; Yn_pcli.y = 0.0; Yn_pcli.z = 0.0;
        Yn_pcli.intensity = -1.0;
        Y00_.push_back(Yn_pcli);
        Y0p1_.push_back(Yn_pcli);
        Yp1p1_.push_back(Yn_pcli);
        Yn1p1_.push_back(Yn_pcli);
        Y0p2_.push_back(Yn_pcli);
        Yp1p2_.push_back(Yn_pcli);
        Yn1p2_.push_back(Yn_pcli);
        Yp2p2_.push_back(Yn_pcli);
        Yn2p2_.push_back(Yn_pcli);
    }

}

void NearnessController3D::publishProjectionShapes(){
    sensor_msgs::PointCloud2 Y00_msg;
    pcl::toROSMsg(Y00_, Y00_msg);
    Y00_msg.header.stamp = ros::Time::now();
    Y00_msg.header.frame_id = frame_id_;
    pub_Y00_.publish(Y00_msg);

    sensor_msgs::PointCloud2 Y0p1_msg;
    pcl::toROSMsg(Y0p1_, Y0p1_msg);
    Y0p1_msg.header.stamp = ros::Time::now();
    Y0p1_msg.header.frame_id = frame_id_;
    pub_Y0p1_.publish(Y0p1_msg);

    sensor_msgs::PointCloud2 Yp1p1_msg;
    pcl::toROSMsg(Yp1p1_, Yp1p1_msg);
    Yp1p1_msg.header.stamp = ros::Time::now();
    Yp1p1_msg.header.frame_id = frame_id_;
    pub_Yp1p1_.publish(Yp1p1_msg);

    sensor_msgs::PointCloud2 Yn1p1_msg;
    pcl::toROSMsg(Yn1p1_, Yn1p1_msg);
    Yn1p1_msg.header.stamp = ros::Time::now();
    Yn1p1_msg.header.frame_id = frame_id_;
    pub_Yn1p1_.publish(Yn1p1_msg);

    sensor_msgs::PointCloud2 Y0p2_msg;
    pcl::toROSMsg(Y0p2_, Y0p2_msg);
    Y0p2_msg.header.stamp = ros::Time::now();
    Y0p2_msg.header.frame_id = frame_id_;
    pub_Y0p2_.publish(Y0p2_msg);

    sensor_msgs::PointCloud2 Yp1p2_msg;
    pcl::toROSMsg(Yp1p2_, Yp1p2_msg);
    Yp1p2_msg.header.stamp = ros::Time::now();
    Yp1p2_msg.header.frame_id = frame_id_;
    pub_Yp1p2_.publish(Yp1p2_msg);

    sensor_msgs::PointCloud2 Yn1p2_msg;
    pcl::toROSMsg(Yn1p2_, Yn1p2_msg);
    Yn1p2_msg.header.stamp = ros::Time::now();
    Yn1p2_msg.header.frame_id = frame_id_;
    pub_Yn1p2_.publish(Yn1p2_msg);

    sensor_msgs::PointCloud2 Yp2p2_msg;
    pcl::toROSMsg(Yp2p2_, Yp2p2_msg);
    Yp2p2_msg.header.stamp = ros::Time::now();
    Yp2p2_msg.header.frame_id = frame_id_;
    pub_Yp2p2_.publish(Yp2p2_msg);

    sensor_msgs::PointCloud2 Yn2p2_msg;
    pcl::toROSMsg(Yn2p2_, Yn2p2_msg);
    Yn2p2_msg.header.stamp = ros::Time::now();
    Yn2p2_msg.header.frame_id = frame_id_;
    pub_Yn2p2_.publish(Yn2p2_msg);
}

void NearnessController3D::projectNearness(){
  // Project measured nearness onto different shapes


}

bool NearnessController3D::newPcl(){
    // if(new_pcl_){
    //     new_pcl_ = false;
    //     return true;
    // } else {
    //     return false;
    // }
    return true;
}

float NearnessController3D::sgn(double v) {
    return (v < 0.0) ? -1.0 : ((v > 0.0) ? 1.0 : 0.0);
}

float NearnessController3D::wrapAngle(float angle){
    if (angle > M_PI){
        angle -= 2*M_PI;
    } else if( angle < -M_PI){
        angle += 2*M_PI;
    }
    return angle;
}

float NearnessController3D::sat(float num, float min_val, float max_val){
    if (num >= max_val){
        return max_val;
    } else if( num <= min_val){
         return min_val;
    } else {
      return num;
    }
}

int NearnessController3D::fact(int n)
{
    if(n > 1)
        return n * fact(n - 1);
    else
        return 1;
}

 // end of class
} // End of namespace nearness
