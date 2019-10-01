#ifndef MMP_MASTER_ARGS_H_
#define MMP_MASTER_ARGS_H_

#include <array> 
#include <string> 
#include <ostream> 

namespace mmp_driver {

	struct Args {
		// Redis keys
		std::string ip_master = "172.24.69.155"; 
		std::string ip_bot1   = "172.24.69.149"; 
		std::string ip_bot2   = "172.24.69.150";
		std::string ip_bot3   = "172.24.69.151"; 
		size_t port_redis = 6379; 

		std::string sim_prefix         = "sim::"; 
	    std::string mmp_prefix         = "mmp::"; 
	    std::string master_prefix      = "master::";
	    std::string bot1_prefix        = "bot1::";
	    std::string bot2_prefix        = "bot2::";
	    std::string bot3_prefix        = "bot3::";
	    std::string vehicle_prefix     = "veh::"; 
	    std::string arm_prefix         = "arm::";  

	    std::string controller_running = "controller_running"; 
	    std::string emergency_shutdown = "emergency_shutdown"; 
	    std::string cstop              = "cstop"; 
	    std::string comm               = "comm"; 

	    // Sensor keys 
	    std::string q       = "sensor::q";
	    std::string dq      = "sensor::dq";  
	    std::string x       = "sensor::x";
	    std::string dx      = "sensor::dx"; 
	    std::string x_ori   = "sensor::x_ori";
	    std::string tau     = "sensor::tau"; 
	    std::string current = "sensor::current";

	    // Control keys
	    std::string q_des       = "control::q"; 
	    std::string dq_des      = "control::dq";
	    std::string x_des       = "control::x"; 
	    std::string xd_des      = "control::xd"; 
	    std::string tau_des     = "control::tau"; 
	    std::string f_des       = "control::f";
	    std::string current_des = "control::current"; 

	    std::string joint_kp      = "control::joint::kp"; 
	    std::string joint_kv      = "control::joint::kv";
	    std::string pos_kp        = "control::pos::kp"; 
	    std::string pos_kv        = "control::pos::kv";  
	    std::string ori_kp        = "control::ori::kp"; 
	    std::string ori_kv        = "control::ori::kv";  

	    // Torque control parameters 
	    int tau_command_timeout = 100; // [ms]

	    // Timeout for blocking call to read button stream
	    int btn_read_timeout_s = 0; // [s]
	    int btn_read_timeout_us = 500000; // [us]


	}; 

	Args ParseArgs(int argc, char* argv[]);

	std::ostream& operator<<(std::ostream& os, const Args& args); 
}

#endif // MMP_MASTER_ARGS_H_