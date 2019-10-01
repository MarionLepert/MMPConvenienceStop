#include <iostream>   // std::cout, std:cerr
#include <fstream>
#include <signal.h>   // signal, sig_atomic_t, SIGINT
#include <cstdlib>    // std:exit
#include <exception>  // std::exception
#include <future>     // std::future
#include <memory>     // std::shared_ptr
#include <string>     // std::string
#include <sstream>    // std::stringstream
#include <fcntl.h>
#include <unistd.h>
// #include <thread.h>   //std::thread

#include <linux/joystick.h>
#include <linux/usbdevice_fs.h>

#include <ctrl_utils/redis_client.h>
#include <ctrl_utils/timer.h>

#include "../include/args.h"
#include "../include/RT_utils.h"

using std::cout; 
using std::endl; 
using std::cerr; 

/*------------------------------------defines------------------------------------*/

#define BUTTON_PRESSED        1 

/*-----------------------static function declarations----------------------------*/
static void sig_handler(int); 
static int readButtonEvent(int fd, struct js_event *event);
static Eigen::VectorXd getRedisKey(std::string key);
static double getRedisKeyDouble(std::string key);  
static int clearButtonStream(int fd, struct js_event *event);
static void initializeRedis(); 
static void initializeConvenienceStop(); 
/*-----------------------static variable declarations----------------------------*/
const char *device;
static struct js_event event;  

/* Args */
static mmp_driver::Args args;

/* Redis */
static ctrl_utils::RedisClient redis_client;

static std::string EMERGENCY_SHUTDOWN_KEY; 
static std::string C_STOP_KEY; 

/* Convenience stop */
static int js; 
static bool joystick_open = false; 

static bool runloop = true; 


/*------------------------------------public functions---------------------------------*/

int main(int argc, char **argv)
{
  
  // SIGINT handler
  signal(SIGINT, sig_handler);

  // Parse args 
  args = mmp_driver::ParseArgs(argc, argv); 

  /******************************* Redis initialization **********************************/
  initializeRedis(); 

  /***************************** Initialize convenience stop ******************************/
   
  initializeConvenienceStop(); 

  /******************************** Start safety loop *************************************/ 
  ctrl_utils::Timer timer(100); 
  double start_time = timer.time();  

  while (runloop) {
    timer.Sleep();

    if (readButtonEvent(js, &event) != 0) {
      cerr << "ERROR: Cannot read joystick" << endl; 
    }

    if (getRedisKeyDouble(EMERGENCY_SHUTDOWN_KEY)) {
      cerr << "EMERGENCY SHUTDOWN" << endl; 
      runloop = false; 
    }

  }

  double end_time = timer.time(); 
  double sim_time = end_time - start_time; 

  if (joystick_open) {
    close(js); 
    joystick_open = false; 
  }

  cout << "\n";
  cout << "Safety Loop run time  : " << sim_time << " seconds\n";
  cout << "Safety Loop updates   : " << timer.num_iters() << "\n";
  cout << "Safety Loop frequency : " << timer.num_iters()/sim_time << "Hz\n";
    
}



/**
* SIGINT handler
**/
static void sig_handler(int)
{
  runloop = false; 

  redis_client.set(EMERGENCY_SHUTDOWN_KEY, 0); 
  redis_client.set(C_STOP_KEY, 0);
  redis_client.commit(); 
  cout << "SIGINT received in master" << endl;

}


/* Redis initialization */
static void initializeRedis()
{
    // Connect to Redis
    redis_client.connect(args.ip_master, args.port_redis);

    // Redis server authentication 
    redis_client.auth("bohg", [](const cpp_redis::reply& reply) {
      if (reply.is_error()) {
        std::cerr << "Redis authentication failed: " << reply.as_string() << std::endl; 
      } else {
        cout << "Successful redis authentication" << endl; 
      }
    });

    // Define redis keys 
    EMERGENCY_SHUTDOWN_KEY = args.mmp_prefix + args.emergency_shutdown; 
    C_STOP_KEY             = args.mmp_prefix + args.cstop;

    redis_client.set(C_STOP_KEY, 0); 
    redis_client.set(EMERGENCY_SHUTDOWN_KEY, 0); 
    redis_client.commit();  
}



/* Initialize reading the convenience stop input stream */
static void initializeConvenienceStop()
{
    device = "/dev/input/js0";
    js = open(device, O_RDONLY); 
    // js = open(device, O_NONBLOCK); 
    joystick_open = true; 

    if (js == -1){
      cerr << "ERROR: Could not open joystick. Check to see if it is plugged in." << endl;
      exit(1);
    }

    if (clearButtonStream(js, &event)) {
      runloop = false; 
    }

    cout << "Convenience stop active" << endl;
}



/* Remove any existing data on the button stream by reading data from 
   the stream until the stream is empty */ 
static int clearButtonStream(int fd, struct js_event *event)
{
  fd_set rfds; 
  struct timeval tv; 
  ssize_t bytes;
  int retval = 0; 

  FD_ZERO(&rfds);    // clears set 
  FD_SET(fd, &rfds); // add rfds file descriptor to set 

  tv.tv_sec = args.btn_read_timeout_s; 
  tv.tv_usec = args.btn_read_timeout_us; 

  retval = select(fd+1, &rfds, NULL, NULL, &tv);
  
  while (retval) {
    if (retval == -1){
      cout << "Error: select failed" << endl; 
      return -1; 
    }
    bytes = read(fd, event, sizeof(*event));
    if (bytes == -1) {
      cout << "ERROR: Cannot read joystick" << endl;
      return -1; 
    }
    retval = select(fd+1, &rfds, NULL, NULL, &tv);
  } 

  return 0; 
}


/**
  * Reads a joystick event from the yellow button
  * Returns 0 on success, otherwise -1
  * BLOCKING
**/ 
static int readButtonEvent(int fd, struct js_event *event)
{
  fd_set rfds; 
  struct timeval tv; 
  ssize_t bytes;
  int retval = 0; 

  FD_ZERO(&rfds);    // clears set 
  FD_SET(fd, &rfds); // add rfds file descriptor to set 

  tv.tv_sec  = args.btn_read_timeout_s; 
  tv.tv_usec = args.btn_read_timeout_us; 

  retval = select(fd+1, &rfds, NULL, NULL, &tv); 
  
  if (retval) {
    bytes = read(fd, event, sizeof(*event));
    if (bytes != sizeof(*event)){
      return -1; 
    }
    cerr << "CONVENIENCE STOP HIT" << endl; 
    redis_client.set(C_STOP_KEY, 1);
    redis_client.commit(); 
    runloop = false; 
  } else if (retval == -1) {
    cout << "Error: select failed" << endl; 
    return -1; 
  } 

  return 0; 
}


/*
 * Return value of redis key corresponding to vector 
 */
static Eigen::VectorXd getRedisKey(std::string key)
{
  std::future<Eigen::VectorXd> future_value = redis_client.get<Eigen::VectorXd>(key);
  redis_client.commit();
  Eigen::VectorXd value = future_value.get(); 
  return value; 
}


/* Return value of redis key corresponding to double */ 
static double getRedisKeyDouble(std::string key)
{
  std::future<double> future_value = redis_client.get<double>(key);
  redis_client.commit();
  double value = future_value.get(); 
  return value; 
}


