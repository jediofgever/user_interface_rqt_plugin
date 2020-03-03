/**
 * @author [Fetullah Atas]
 * @email [fetulahatas1@gmail.com]
 * @create date 2020-02-18 17:08:39
 * @modify date 2020-02-18 17:08:39
 * @desc [description]
 */
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <jsoncpp/json/json.h>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/cURLpp.hpp>

// from kliotyps.kl
/**
 * @brief PORT types available in Fanuc Controller
 *
 */
class PortTypes {
   public:
    u_int DIN = 1;
    u_int DOUT = 2;
    u_int ANIN = 3;
    u_int ANOUT = 4;
    u_int TOOL = 5;
    u_int RDI = 8;
    u_int RDO = 9;
    u_int GPIN = 18;
    u_int GPOUT = 19;
};

/**
 * @brief curlpp based class to read/write IO of fanuc robot controller
 *
 */
class robot_IO {
   private:
    PortTypes port_type;
    std::string HTTPS = "http://";
    std::string BASE_URL = "KAREL";
    std::string PROG_NAME = "ros_cgio";
    std::string OP_READ = "read";
    std::string OP_WRITE = "write";
    std::string ARG_OP = "io_op";
    std::string ARG_TYPE = "io_type";
    std::string ARG_IDX = "io_idx";
    std::string ARG_VAL = "io_val";
    std::string JSON_RESULT = "result";
    std::string JSON_OP = "op";
    std::string JSON_TYPE = "type";
    std::string JSON_IDX = "idx";
    std::string JSON_VALUE = "value";
    std::string JSON_SUCCESS = "success";
    std::string JSON_ERROR = "error";
    std::string JSON_REASON = "reason";

    // curlpp Objects to request, receieve data
    curlpp::Cleanup cleaner;
    curlpp::Easy request;

    // set this to true to enable communication with real robot controller, if using simulator set it to false
    bool youAreRunningonRealRobot = false;

   public:
    /**
     * @brief Construct a new robot IO object
     *
     */
    robot_IO(/* args */);

    /**
     * @brief Destroy the robot IO object
     *
     */
    ~robot_IO();

    /**
     * @brief loops from 1 to given NUMBER_OF_PORTS,  reads all of them , puts their current values into a vector
     *
     * @param robot_ip
     * @param OPERATION_TYPE
     * @param PORT_TYPE
     * @param NUMBER_OF_PORTS
     * @return std::vector<std::string>
     */
    std::vector<std::string> readAllIOs(std::string robot_ip, std::string OPERATION_TYPE, int PORT_TYPE,
                                        int NUMBER_OF_PORTS);

    /**
     * @brief write a singlo IO port, attentation to PORT_ID and VALUE
     *
     * @param robot_ip
     * @param OPERATION_TYPE
     * @param PORT_TYPE
     * @param PORT_ID
     * @param VALUE
     */
    void writeSingleIO(std::string robot_ip, std::string OPERATION_TYPE, int PORT_TYPE, int PORT_ID, int VALUE);

    /**
     * @brief Read single IO , returns the value of requested IO
     *
     * @param robot_ip
     * @param OPERATION_TYPE
     * @param PORT_TYPE
     * @param PORT_ID
     * @return int
     */
    int readSingleIO(std::string robot_ip, std::string OPERATION_TYPE, int PORT_TYPE, int PORT_ID);
};
