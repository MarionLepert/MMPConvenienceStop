/**
 * args.cc
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: December 20, 2018
 * Authors: Toki Migimatsu
 */

#include "../include/args.h"

#include <exception>  // std::runtime_error
#include <iostream>   // std::cout

#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

using std::cout;
using std::endl;

namespace mmp_driver {

std::array<double, 16> ParseTransform(const YAML::Node& yaml) {
  std::array<double, 16> arr;
  Eigen::Map<Eigen::Matrix4d> map(arr.data());

  // Get position
  for (size_t i = 0; i < 3; i++) {
    map(i,3) = yaml["pos"][i].as<double>();
  }

  // Get rotation
  if (yaml["ori"]["w"]) {
    Eigen::Quaterniond quat;
    quat.w() = yaml["ori"]["w"].as<double>();
    quat.x() = yaml["ori"]["x"].as<double>();
    quat.y() = yaml["ori"]["y"].as<double>();
    quat.z() = yaml["ori"]["z"].as<double>();
    map.topLeftCorner<3,3>() = quat.matrix();
  } else {
    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        map(i, j) = yaml["ori"][i][j].as<double>();
      }
    }
    Eigen::Affine3d T(map);
    map.topLeftCorner<3,3>() = T.rotation();
  }

  map.row(map.rows()-1) << 0., 0., 0., 1.;
  return arr;
}

template<unsigned long Dim>
std::array<double, Dim> ParseArray(const YAML::Node& yaml) {
  std::array<double, Dim> arr;
  if (yaml.IsSequence()) {
    arr = yaml.as<std::array<double, Dim>>();
  } else {
    double d = yaml.as<double>();
    for (size_t i = 0; i < arr.size(); i++) {
      arr[i] = d;
    }
  }
  return arr;
}

Args ParseYaml(const char* filename) {
  Args args;
  YAML::Node yaml = YAML::LoadFile(filename);
  std::cout << "Loading config: " << filename << std::endl << std::endl
            << yaml << std::endl << std::endl;

  try {
    // Redis parameters
    args.ip_master  = yaml["master"]["ip"].as<std::string>();
    args.ip_bot1    = yaml["bot1"]["ip"].as<std::string>();
    args.ip_bot2    = yaml["bot2"]["ip"].as<std::string>();
    args.ip_bot3    = yaml["bot3"]["ip"].as<std::string>();
    args.port_redis = yaml["redis"]["port"].as<size_t>();

    args.controller_running   = yaml["redis"]["keys"]["controller_running"].as<std::string>();
    args.emergency_shutdown   = yaml["redis"]["keys"]["emergency_shutdown"].as<std::string>();
    args.cstop                = yaml["redis"]["keys"]["cstop"].as<std::string>();
    args.cstop_running        = yaml["redis"]["keys"]["cstop_running"].as<std::string>();
    args.comm                 = yaml["redis"]["keys"]["communication"].as<std::string>();

    args.sim_prefix           = yaml["redis"]["keys"]["sim_prefix"].as<std::string>();
    args.mmp_prefix           = yaml["redis"]["keys"]["mmp_prefix"].as<std::string>();
    args.master_prefix        = yaml["redis"]["keys"]["master_prefix"].as<std::string>();
    args.bot1_prefix          = yaml["redis"]["keys"]["bot1_prefix"].as<std::string>();
    args.bot2_prefix          = yaml["redis"]["keys"]["bot2_prefix"].as<std::string>();
    args.bot3_prefix          = yaml["redis"]["keys"]["bot3_prefix"].as<std::string>();
    args.vehicle_prefix       = yaml["redis"]["keys"]["vehicle_prefix"].as<std::string>();
    args.arm_prefix           = yaml["redis"]["keys"]["arm_prefix"].as<std::string>();
    
    args.q                    = yaml["redis"]["keys"]["q"].as<std::string>();
    args.dq                   = yaml["redis"]["keys"]["dq"].as<std::string>();
    args.x                    = yaml["redis"]["keys"]["x"].as<std::string>(); 
    args.dx                   = yaml["redis"]["keys"]["dx"].as<std::string>(); 
    args.x_ori                = yaml["redis"]["keys"]["x_ori"].as<std::string>(); 
    args.tau                  = yaml["redis"]["keys"]["tau"].as<std::string>();
    args.current              = yaml["redis"]["keys"]["current"].as<std::string>();

    args.q_des                = yaml["redis"]["keys"]["q_des"].as<std::string>();
    args.dq_des               = yaml["redis"]["keys"]["dq_des"].as<std::string>();
    args.x_des                = yaml["redis"]["keys"]["x_des"].as<std::string>();
    args.xd_des               = yaml["redis"]["keys"]["xd_des"].as<std::string>(); 
    args.tau_des              = yaml["redis"]["keys"]["tau_des"].as<std::string>(); 
    args.f_des                = yaml["redis"]["keys"]["f_des"].as<std::string>(); 
    args.current_des          = yaml["redis"]["keys"]["current_des"].as<std::string>(); 

    args.joint_kp             = yaml["control"]["joint_space_controller"]["kp"].as<std::string>(); 
    args.joint_kv             = yaml["control"]["joint_space_controller"]["kv"].as<std::string>();

    args.pos_kp               = yaml["control"]["pos_controller"]["kp"].as<std::string>(); 
    args.pos_kv               = yaml["control"]["pos_controller"]["kv"].as<std::string>();

    args.ori_kp               = yaml["control"]["ori_controller"]["kp"].as<std::string>(); 
    args.ori_kv               = yaml["control"]["ori_controller"]["kv"].as<std::string>();
    
    args.tau_command_timeout  = yaml["control"]["torque_controller"]["tau_command_timeout"].as<int>();
    args.btn_read_timeout_s   = yaml["button"]["read_timeout_s"].as<int>();          
    args.btn_read_timeout_us  = yaml["button"]["read_timeout_us"].as<int>();          
  } catch (...) {
    throw std::runtime_error("kinova::ParseYaml(): Unable to parse YAML config.");
  }
  return args;
}

Args ParseArgs(int argc, char* argv[]) {
  // if (argc < 2) {
  //   std::cout << "Usage:" << std::endl
  //             << "\t./kinova_driver ../resources/default.yaml" << std::endl;
  //   exit(0);
  // }

  int i = 1;
  Args args = ParseYaml("../resources/default.yaml");

  // Args args = ParseYaml(argv[i]);

  // Parse remaining command line arguments
  std::string arg;
  for ( ; i < argc; i++) {
    arg = argv[i];
  }

  if (i != argc) throw std::invalid_argument("ParseArgs(): Invalid '" + arg + "' argument.");
  return args;
}

template<unsigned long Dim>
std::ostream& operator<<(std::ostream& os, const std::array<double, Dim>& arr) {
  constexpr unsigned long dim_sqrt = round(sqrt(Dim));
  constexpr bool is_square = dim_sqrt * dim_sqrt == Dim;

  // Print square matrix
  if (is_square) {
    std::string sep_outer;
    os << "[";
    for (size_t i = 0; i < dim_sqrt; i++) {
      os << sep_outer;
      std::string sep_inner;
      for (size_t j = 0; j < dim_sqrt; j++) {
        os << sep_inner << arr[dim_sqrt*j + i];
        if (sep_inner.empty()) sep_inner = ", ";
      }
      if (sep_outer.empty()) sep_outer = "; ";
    }
    os << "]";
    return os;
  }

  // Print flat vector
  std::string sep;
  os << "[";
  for (size_t i = 0; i < arr.size(); i++) {
    os << sep << arr[i];
    if (sep.empty()) sep = ", ";
  }
  os << "]";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Args& args) {
  os << "Args {" << std::endl
     << "  ip_master: " << args.ip_master << std::endl
     << "  ip_bot1: " << args.ip_bot1 << std::endl
     << "  ip_bot2: " << args.ip_bot2 << std::endl
     << "  ip_bot3: " << args.ip_bot3 << std::endl

     << "  port_redis: " << args.port_redis << std::endl
     << "  key_bot1_prefix: " << args.bot1_prefix << std::endl
     << "  key_q: " << args.q << std::endl
     << "}" << std::endl;
  return os;
}

}  // namespace mmp_driver