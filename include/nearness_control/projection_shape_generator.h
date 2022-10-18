#ifndef PROJECTION_SHAPE_GENERATOR_H
#define PROJECTION_SHAPE_GENERATOR_H

#include <vector>

#include <ros/ros.h>

class ProjectionShapeGenerator {
public:
  ProjectionShapeGenerator(){};
  ~ProjectionShapeGenerator(){};

  class ShapeOptions {
  public:
    enum shape_types { spherical_harmonics, spherical_state, planar };
    int selected_type;
    std::vector<int> size;
  }; // ShapeOptions class

  std::vector<std::vector<float>> getProjectionShapes(ShapeOptions &options);

  std::vector<float> getThetaViewingVector() { return theta_view_vec_; }

  std::vector<float> getPhiViewingVector() { return phi_view_vec_; }

  std::vector<std::vector<float>> getViewingAngleMatrix() {
    return viewing_angle_mat_;
  }

private:
  void generateSphericalHarmonics(const std::vector<int> shape_size);

  void generateViewingAngleVectors(const int num_rings,
                                   const int num_ring_points);

  void generateStateProjectionShapes(const std::vector<int> shape_size);

  void generatePlanarShapes(std::vector<int> shape_size);

  std::vector<std::vector<float>> shapes_vec_;
  std::vector<float> theta_view_vec_;
  std::vector<float> phi_view_vec_;
  std::vector<std::vector<float>> viewing_angle_mat_;

  // Spherical harmonic shape vectors
  std::vector<std::vector<float>> spherical_harmonics_vec_;
  std::vector<float> Y00_vec_;
  std::vector<float> Y0p1_vec_;
  std::vector<float> Yp1p1_vec_;
  std::vector<float> Yn1p1_vec_;
  std::vector<float> Y0p2_vec_;
  std::vector<float> Yp1p2_vec_;
  std::vector<float> Yn1p2_vec_;
  std::vector<float> Yp2p2_vec_;
  std::vector<float> Yn2p2_vec_;

  // State sensitivity shape vectors
  std::vector<std::vector<float>> state_sensitivity_shapes_vec_;
  std::vector<float> y_projection_shape_vec_;
  std::vector<float> theta_projection_shape_vec_;
  std::vector<float> z_projection_shape_vec_;

}; // ProjectionShapeGenerator class

#endif