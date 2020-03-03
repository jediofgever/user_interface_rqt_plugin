/**
 * @author [Fetullah Atas]
 * @email [fetulahatas1@gmail.com]
 * @create date 2020-02-18 17:09:28
 * @modify date 2020-02-18 17:09:28
 * @desc [description]
 */
#include <user_interface_rqt_plugin/robot_IO.h>
#include <regex>
robot_IO::robot_IO(/* args */) {}

robot_IO::~robot_IO() {}

std::vector<std::string> robot_IO::readAllIOs(std::string robot_ip, std::string OPERAT, int PORT_TYPE,
                                              int NUMBER_OF_PORTS) {
    std::vector<std::string> vector_of_io_values;
    if (youAreRunningonRealRobot) {
        Json::Value root;
        Json::Reader reader;
        std::string result;
        for (size_t i = 1; i < NUMBER_OF_PORTS; i++) {
            std::string url_of_this_IO = HTTPS + robot_ip + "/" + BASE_URL + "/" + PROG_NAME + "?" + ARG_OP + "=" +
                                         OPERAT + "&" + ARG_TYPE + "=" + std::to_string(PORT_TYPE) + "&" + ARG_IDX +
                                         "=" + std::to_string(i);
            std::ostringstream os;
            try {
                os << curlpp::options::Url(url_of_this_IO);

                result = os.str();
            } catch (curlpp::LogicError &e) {
                std::cout << e.what() << std::endl;
            }

            bool parsingSuccessful = reader.parse(result.c_str(), root);  // parse process
            if (!parsingSuccessful) {
                std::cout << "Failed to parse" << reader.getFormattedErrorMessages();
            }
            std::string value_of_this_port = root.get("value", "NONE").asString();

            vector_of_io_values.push_back(value_of_this_port);
        }
    }

    return vector_of_io_values;
}

int robot_IO::readSingleIO(std::string robot_ip, std::string OPERAT, int PORT_TYPE, int PORT_ID) {
    std::string url_of_this_IO = HTTPS + robot_ip + "/" + BASE_URL + "/" + PROG_NAME + "?" + ARG_OP + "=" + OPERAT +
                                 "&" + ARG_TYPE + "=" + std::to_string(PORT_TYPE) + "&" + ARG_IDX + "=" +
                                 std::to_string(PORT_ID);

    if (youAreRunningonRealRobot) {
        std::ostringstream os;
        std::string result;
        try {
            os << curlpp::options::Url(url_of_this_IO);
            result = os.str();
        } catch (curlpp::LogicError &e) {
            std::cout << e.what() << std::endl;
        }

        Json::Value root;
        Json::Reader reader;
        bool parsingSuccessful = reader.parse(result.c_str(), root);  // parse process
        if (!parsingSuccessful) {
            std::cout << "Failed to parse" << reader.getFormattedErrorMessages();
        }
    }
}

void robot_IO::writeSingleIO(std::string robot_ip, std::string OPERAT, int PORT_TYPE, int PORT_ID, int VALUE) {
    std::string url_of_this_IO = HTTPS + robot_ip + "/" + BASE_URL + "/" + PROG_NAME + "?" + ARG_OP + "=" + OPERAT +
                                 "&" + ARG_TYPE + "=" + std::to_string(PORT_TYPE) + "&" + ARG_IDX + "=" +
                                 std::to_string(PORT_ID) + "&" + ARG_VAL + "=" + std::to_string(VALUE);
    if (youAreRunningonRealRobot) {
        try {
            request.setOpt(curlpp::Options::Verbose(true));
            request.setOpt(curlpp::Options::Url(url_of_this_IO));
            request.perform();

        } catch (curlpp::LogicError &e) {
            std::cout << e.what() << std::endl;
        }
    }
}
