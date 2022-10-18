#include <nearness_control/projection_shape_generator.h>

std::vector<std::vector<float>>
ProjectionShapeGenerator::getProjectionShapes(ShapeOptions &options) {
  switch (options.selected_type) {
  case ShapeOptions::spherical_harmonics:
    generateSphericalHarmonics(options.size);
    return spherical_harmonics_vec_;
  case ShapeOptions::spherical_state:
    generateSphericalHarmonics(options.size);
    generateStateProjectionShapes(options.size);
    return state_sensitivity_shapes_vec_;
  case ShapeOptions::planar:
    // Not implemented currently
    generatePlanarShapes(options.size);
    return shapes_vec_;
  }
  return shapes_vec_;
}

void ProjectionShapeGenerator::generatePlanarShapes(
    std::vector<int> shape_size) {
  // Not implemented currently
}

void ProjectionShapeGenerator::generateSphericalHarmonics(
    const std::vector<int> shape_size) {

  const int num_rings = shape_size[0];
  const int num_ring_points = shape_size[1];
  generateViewingAngleVectors(num_rings, num_ring_points);

  float theta, phi, d;

  // Cycle through every point in the spherical coordinate system
  // and generate the Laplace Spherical harmonic shapes. Cycle
  // from bottom to top ring, and pi to -pi along a ring.
  for (int i = 1; i < num_rings - 1; i++) {
    // Get current theta value
    theta = theta_view_vec_[i];

    for (int j = 0; j < num_ring_points; j++) {
      // Get current phi value
      phi = phi_view_vec_[j];

      // Y00
      d = (1.0 / 2.0) * sqrt(1.0 / (M_PI));
      Y00_vec_.push_back(d);

      // Y0p1
      d = (1.0 / 2.0) * sqrt(3.0 / (M_PI)) * cos(theta);
      Y0p1_vec_.push_back(d);

      // Yp1p1
      d = (1.0 / 2.0) * sqrt(3.0 / M_PI) * sin(theta) * cos(phi);
      Yp1p1_vec_.push_back(d);

      // Yp1n1
      d = (1.0 / 2.0) * sqrt(3.0 / M_PI) * sin(theta) * sin(phi);
      Yn1p1_vec_.push_back(d);

      // Yp20
      d = (1.0 / 4.0) * sqrt(5.0 / M_PI) * (3.0 * pow(cos(theta), 2) - 1);
      Y0p2_vec_.push_back(d);

      // Yp2p1
      d = (3.0 / 2.0) * sqrt(5.0 / (3.0 * M_PI)) * sin(theta) * cos(theta) *
          cos(phi);
      Yp1p2_vec_.push_back(d);

      //  Yp2n1
      d = (3.0 / 2.0) * sqrt(5.0 / (3.0 * M_PI)) * sin(theta) * cos(theta) *
          sin(phi);
      Yn1p2_vec_.push_back(d);

      // Yp2p2
      d = (3.0 / 4.0) * sqrt(5.0 / (3.0 * M_PI)) * pow(sin(theta), 2) *
          cos(2 * phi);
      Yp2p2_vec_.push_back(d);

      // Yp2n2
      d = (3.0 / 4.0) * sqrt(5.0 / (3.0 * M_PI)) * pow(sin(theta), 2) *
          sin(2 * phi);
      Yn2p2_vec_.push_back(d);
    }
  }

  // Create array of projection shape arrays, Indexing is
  // [shape][i*ring_num + j] for i rings and j points per ring
  spherical_harmonics_vec_.push_back(Y00_vec_);
  spherical_harmonics_vec_.push_back(Y0p1_vec_);
  spherical_harmonics_vec_.push_back(Yp1p1_vec_);
  spherical_harmonics_vec_.push_back(Yn1p1_vec_);
  spherical_harmonics_vec_.push_back(Y0p2_vec_);
  spherical_harmonics_vec_.push_back(Yp1p2_vec_);
  spherical_harmonics_vec_.push_back(Yn1p2_vec_);
  spherical_harmonics_vec_.push_back(Yp2p2_vec_);
  spherical_harmonics_vec_.push_back(Yn2p2_vec_);
}

void ProjectionShapeGenerator::generateStateProjectionShapes(
    const std::vector<int> shape_size) {

  const int num_basis_shapes = spherical_harmonics_vec_.size();
  const std::vector<float> C_y = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<float> C_z = {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<float> C_theta = {0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 1.0};

  // Generate state projection shapes
  float d_y, d_theta, d_z;
  const int last_index = shape_size[0] * shape_size[1];
  for (int i = 0; i < last_index; i++) {
    d_y = 0.0;
    d_theta = 0.0;
    d_z = 0.0;
    for (int k = 0; k < num_basis_shapes; k++) {
      d_y += C_y[k] * spherical_harmonics_vec_[k][i];
      d_theta += C_theta[k] * spherical_harmonics_vec_[k][i];
      if (k < last_index / 2) {
        d_z += C_z[k] * spherical_harmonics_vec_[k][i];
      }
    }
    y_projection_shape_vec_.push_back(d_y);
    theta_projection_shape_vec_.push_back(d_theta);
    z_projection_shape_vec_.push_back(d_z);
  }
  state_sensitivity_shapes_vec_.push_back(y_projection_shape_vec_);
  state_sensitivity_shapes_vec_.push_back(theta_projection_shape_vec_);
  state_sensitivity_shapes_vec_.push_back(z_projection_shape_vec_);
}

void ProjectionShapeGenerator::generateViewingAngleVectors(
    const int num_rings, const int num_ring_points) {
  // Inclination angle
  constexpr float theta_start = M_PI;
  constexpr float theta_end = 0;
  const float dtheta = (theta_start - theta_end) / float(num_rings);
  for (int i = 0; i <= num_rings; i++) {
    theta_view_vec_.push_back(theta_start - float(i) * dtheta);
  }

  // Azimuthal angle
  constexpr float phi_start = M_PI;
  constexpr float phi_end = -M_PI;
  const float dphi = (phi_start - phi_end) / float(num_ring_points);
  for (int i = 0; i < num_ring_points; i++) {
    phi_view_vec_.push_back(phi_start - float(i) * dphi);
  }

  // Make a matrix for generating pointclouds from
  // vector representations of nearness
  for (int i = 0; i < num_rings; i++) {
    for (int j = 0; j < num_ring_points; j++) {
      viewing_angle_mat_.push_back({theta_view_vec_[i], phi_view_vec_[j]});
    }
  }
}
