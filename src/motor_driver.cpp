#include "motor_driver.hpp"

#include "dm_motor_driver.hpp"
#include "evo_motor_driver.hpp"
#include "lro_motor_driver.hpp"
// #include "xyn_motor_driver.hpp"

MotorDriver::MotorDriver() {
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
    logger_ = setup_logger(sinks, "motors");
}
std::shared_ptr<MotorDriver> MotorDriver::create_motor(uint16_t motor_id, const std::string& interface_type, const std::string& interface,
                                                      const std::string& motor_type, int motor_model, uint16_t master_id_offset, double motor_zero_offset) {
    if (motor_type == "DM") {
        return std::make_shared<DmMotorDriver>(motor_id, interface_type, interface, master_id_offset,
                                               static_cast<DM_Motor_Model>(motor_model), motor_zero_offset);
    } else if (motor_type == "EVO") { 
        return std::make_shared<EvoMotorDriver>(motor_id, interface_type, interface,
                                                static_cast<EVO_Motor_Model>(motor_model), motor_zero_offset);
    } else if (motor_type == "LRO") {
    return std::make_shared<LroMotorDriver>(motor_id, interface_type, interface,
                                            static_cast<LRO_Motor_Model>(motor_model), motor_zero_offset);
    // } else if (motor_type == "XYN") {
    //     return std::make_shared<XynMotorDriver>(motor_id, interface_type, interface,
    //                                             static_cast<XYN_Motor_Model>(motor_model), motor_zero_offset);
    } else {
        throw std::runtime_error("Motor type not supported");
    }
}

uint32_t MotorDriver::get_group_can_id(const std::string& motor_type) {
    // 1. LeadRobot (LRO) Brand
    // Uses 29-bit Extended Frame Format (EFF), fixed broadcast ID is 0x8080
    if (motor_type == "LRO") {
        return 0x8080 | CAN_EFF_FLAG;
    }
    // 2. EVO Brand
    // Uses 11-bit Standard Frame Format, broadcast ID for MIT control is 0x20
    if (motor_type == "EVO") {
        return 0x20;
    }
    // // 3. Xynova (XYN) Brand
    // // Also uses Standard Frame broadcasting, ID is 0x20
    // if (motor_type == "XYN") {
    //     return 0x20;
    // }

    // 4. DM Brand or other types that do not support Multi-Drop (one-to-many)
    // Returns 0 to indicate that the bus does not require aggregated transmission; 
    // it should continue using point-to-point mode.
    return 0;
}