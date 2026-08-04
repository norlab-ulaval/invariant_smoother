// pybind11 bindings for Yoon et al. (2024) InvariantSmoother.
//
// Drives third_party/invariant_smoother directly from Python instead of going
// through their CSV main(): we hand it numpy arrays per step and read the
// ROBOT_STATES back out. Their sources are used UNMODIFIED (the submodule is
// pinned upstream) — everything project-specific lives here.
//
// Why not their main.cpp: it reads an 82-column CSV, hardcodes the parameters,
// and only writes position/velocity/orientation. Binding Onestep() gives us
// per-step dt control, the internal Slip / Hard_Contact / d_v diagnostics, and
// avoids a ~150 MB CSV round-trip per bag.
//
// Reference for the call sequence: their src/main.cpp.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <memory>
#include <vector>
#include <stdexcept>

#include "estimator/InvariantSmoother.hpp"
#include "estimator/RobotState_Smoother.hpp"
#include "utility/EstimatorCommonStruct.hpp"

namespace py = pybind11;

namespace {

// One driver call per bag: build the estimator, replay every sample, and return
// the logged state trajectory. Keeping the loop in C++ (rather than calling
// Onestep from Python N times) avoids ~180k GIL round-trips per bag.
struct RunResult {
  Eigen::MatrixXd position;      // (N, 3) world
  Eigen::MatrixXd velocity;      // (N, 3) world
  Eigen::MatrixXd rotation;      // (N, 9) row-major R_wb
  Eigen::MatrixXd bias_gyro;     // (N, 3)
  Eigen::MatrixXd bias_acc;      // (N, 3)
  Eigen::MatrixXd foothold;      // (N, 12)
  Eigen::MatrixXd foot_velocity; // (N, 12)  their d_v — the SR trigger
  Eigen::MatrixXi slip;          // (N, 4)   their Slip flag
  Eigen::MatrixXi hard_contact;  // (N, 4)
};

RunResult run_batch(const Eigen::MatrixXd &sensor,       // (N, 30)
                    const Eigen::MatrixXi &contact,      // (N, 4)
                    const Eigen::MatrixXd &fk_position,  // (N, 12)
                    const Eigen::MatrixXd &fk_jacobian,  // (N, 36) row-major 3x3 per leg
                    double dt,
                    const Eigen::Matrix<double, 16, 1> &initial_condition,
                    const EstimatorCovariances &covariances,
                    bool slip_rejection_mode,
                    double slip_threshold,
                    bool variable_contact_cov_mode,
                    double cov_amplifier,
                    int max_iteration,
                    int max_backpropagate_num,
                    double backpropagate_rate,
                    double convergence_cond,
                    int num_trash_data) {
  const Eigen::Index N = sensor.rows();
  if (sensor.cols() != InvariantSmoother::num_z)
    throw std::invalid_argument("sensor must have num_z (30) columns");
  if (contact.rows() != N || fk_position.rows() != N || fk_jacobian.rows() != N)
    throw std::invalid_argument("all inputs must share the same row count");
  if (contact.cols() != 4 || fk_position.cols() != 12 || fk_jacobian.cols() != 36)
    throw std::invalid_argument("expected contact (N,4), fk_position (N,12), fk_jacobian (N,36)");

  // Heap, not stack: InvariantSmoother carries ~160 MB of fixed member arrays
  // (SAVE_BUFFER[115][140000] + SensorData[140000][30]), so a stack instance
  // segfaults instantly. Their main.cpp sidesteps this by making it a global.
  auto est_owned = std::make_unique<InvariantSmoother>();
  InvariantSmoother &est = *est_owned;
  est.estimator_common_struct_.leg_no = 4;
  est.Optimization_Epsilon = convergence_cond;
  est.Max_Iteration = max_iteration;
  est.Max_backpropagate_num = max_backpropagate_num;
  est.backppgn_rate = backpropagate_rate;
  est.NUM_OF_TRASH_DATA = num_trash_data;
  est.slip_rejection_mode = slip_rejection_mode;
  est.slip_threshold = slip_threshold;
  est.variable_contact_cov_mode = variable_contact_cov_mode;
  est.cov_amplifier = cov_amplifier;
  est.Retract_All_flag = false;

  EstimatorCovariances cov = covariances;         // Initialize takes a non-const ref
  Eigen::Matrix<double, 16, 1> ic = initial_condition;
  est.Initialize(dt, cov, ic);

  RunResult out;
  out.position.resize(N, 3);
  out.velocity.resize(N, 3);
  out.rotation.resize(N, 9);
  out.bias_gyro.resize(N, 3);
  out.bias_acc.resize(N, 3);
  out.foothold.resize(N, 12);
  out.foot_velocity.resize(N, 12);
  out.slip.resize(N, 4);
  out.hard_contact.resize(N, 4);

  MEAS_FORWARD_KINEMATICS forkin;
  forkin.forkin_position.resize(4);
  forkin.forkin_jacobian.resize(4);
  ROBOT_STATES state;

  for (Eigen::Index k = 0; k < N; ++k) {
    Eigen::Matrix<double, InvariantSmoother::num_z, 1> s = sensor.row(k).transpose();
    Eigen::Matrix<bool, 4, 1> c;
    for (int j = 0; j < 4; ++j) c(j) = contact(k, j) != 0;
    for (int j = 0; j < 4; ++j) {
      forkin.forkin_position[j] = fk_position.block(k, 3 * j, 1, 3).transpose();
      // row-major 3x3 per leg, matching their CallFile reader
      for (int r = 0; r < 3; ++r)
        for (int cc = 0; cc < 3; ++cc)
          forkin.forkin_jacobian[j](r, cc) = fk_jacobian(k, 9 * j + 3 * r + cc);
    }

    est.Onestep(s, c, forkin, state);

    out.position.row(k) = state.Position.transpose();
    out.velocity.row(k) = state.Velocity.transpose();
    for (int r = 0; r < 3; ++r)
      for (int cc = 0; cc < 3; ++cc)
        out.rotation(k, 3 * r + cc) = state.Rotation(r, cc);
    out.bias_gyro.row(k) = state.Bias_Gyro.transpose();
    out.bias_acc.row(k) = state.Bias_Acc.transpose();
    out.foothold.row(k) = state.d.transpose();
    out.foot_velocity.row(k) = state.d_v.transpose();
    for (int j = 0; j < 4; ++j) {
      out.slip(k, j) = state.Slip(j) ? 1 : 0;
      out.hard_contact(k, j) = state.Hard_Contact(j) ? 1 : 0;
    }
  }
  return out;
}

}  // namespace

PYBIND11_MODULE(invariant_smoother_py, m) {
  m.doc() = "Yoon et al. (2024) InvariantSmoother, driven per-step from Python";
  // int(...) forces an rvalue: InvariantSmoother::num_z is a `const static int`
  // declared in-class with no out-of-class definition, so binding it by
  // reference (what `attr =` does) would ODR-use it and fail to link.
  m.attr("NUM_Z") = int(InvariantSmoother::num_z);
  m.attr("WINDOW_SIZE") = int(WINDOW_SIZE);

  py::class_<EstimatorCovariances>(m, "EstimatorCovariances")
      .def(py::init<>())
#define COV_FIELD(name) .def_readwrite(#name, &EstimatorCovariances::name)
      COV_FIELD(cov_gyro_diagonal)
      COV_FIELD(cov_acc_diagonal)
      COV_FIELD(cov_slip_diagonal)
      COV_FIELD(cov_contact_diagonal)
      COV_FIELD(cov_enc_diagonal)
      COV_FIELD(cov_bias_gyro_diagonal)
      COV_FIELD(cov_bias_acc_diagonal)
      COV_FIELD(cov_prior_orientation_diagonal)
      COV_FIELD(cov_prior_velocity_diagonal)
      COV_FIELD(cov_prior_position_diagonal)
      COV_FIELD(cov_prior_bias_gyro_diagonal)
      COV_FIELD(cov_prior_bias_acc_diagonal)
#undef COV_FIELD
      ;

  py::class_<RunResult>(m, "RunResult")
      .def_readonly("position", &RunResult::position)
      .def_readonly("velocity", &RunResult::velocity)
      .def_readonly("rotation", &RunResult::rotation)
      .def_readonly("bias_gyro", &RunResult::bias_gyro)
      .def_readonly("bias_acc", &RunResult::bias_acc)
      .def_readonly("foothold", &RunResult::foothold)
      .def_readonly("foot_velocity", &RunResult::foot_velocity)
      .def_readonly("slip", &RunResult::slip)
      .def_readonly("hard_contact", &RunResult::hard_contact);

  m.def("run_batch", &run_batch,
        py::arg("sensor"), py::arg("contact"), py::arg("fk_position"),
        py::arg("fk_jacobian"), py::arg("dt"), py::arg("initial_condition"),
        // Defaults are the PAPER's (Table II / §VII), not the released
        // main.cpp's, which differ: slip_thr 0.48, slip_exp -1.3, WINDOW_SIZE 5.
        py::arg("covariances"), py::arg("slip_rejection_mode") = false,
        py::arg("slip_threshold") = 0.3, py::arg("variable_contact_cov_mode") = false,
        py::arg("cov_amplifier") = 1.0, py::arg("max_iteration") = 1,
        py::arg("max_backpropagate_num") = 1, py::arg("backpropagate_rate") = 0.5,
        py::arg("convergence_cond") = 1e-3, py::arg("num_trash_data") = 1,
        py::call_guard<py::gil_scoped_release>(),
        "Replay a whole run through InvariantSmoother::Onestep and return the state trajectory.");
}
