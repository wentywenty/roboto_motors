#include "lro_motor_driver.hpp"

LRO_Limit_Param lro_limit_param[LRO_Num_Of_Motor] = {
    {12.5, 45.0, 40.0, 500.0, 5.0},  // LRO_PJ3_55_5550
    {12.5, 45.0, 40.0, 500.0, 5.0},  // LRO_PJ3_60_6562
    {12.5, 30.0, 60.0, 500.0, 5.0},  // LRO_PJ3_75_8462
    {12.5, 25.0, 80.0, 500.0, 5.0}   // LRO_PJ3_97_10062
};

LroMotorDriver::LroMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface, 
                               LRO_Motor_Model motor_model, double motor_zero_offset)
    : MotorDriver(), motor_model_(motor_model) {
    if (interface_type != "canfd" && interface_type != "ethercanfd" && interface_type != "ethercat") {
        throw std::runtime_error("LRO driver only supports CAN-FD and Ethercat interfaces");
    }
    motor_id_ = motor_id;
    limit_param_ = lro_limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;

    if (interface_type == "canfd" || interface_type == "ethercanfd") {
        comm_type_ = CommType::CANFD;
        motor_index_ = 0;
        canfd_ = MotorsSocketCANFD::get(can_interface);

        CanFdCbkFunc canfd_callback = std::bind(&LroMotorDriver::canfd_rx_cbk, this, std::placeholders::_1);
        canfd_->add_canfd_callback(canfd_callback, motor_id_);
    } else if (interface_type == "ethercat") {
        comm_type_ = CommType::ETHERCAT;
        throw std::runtime_error("LRO driver does not support EtherCAT interface yet");
    }
}

LroMotorDriver::~LroMotorDriver() {
    if (comm_type_ == CommType::CANFD) {
        canfd_->remove_canfd_callback(motor_id_);
    } else if (comm_type_ == CommType::ETHERCAT) {
        spdlog::error("LRO driver does not support EtherCAT interface yet");
    }
}

void LroMotorDriver::lock_motor() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x7FF;
        tx_frame.len = 0x04;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
        tx_frame.data[1] = motor_id_ & 0xFF;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = LRO_CMD_ENABLE;
        
        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::ETHERCAT) {
        throw std::runtime_error("LRO driver does not support EtherCAT interface yet");
    }
    {
        response_count_++;
    }
}

void LroMotorDriver::unlock_motor() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x7FF;
        tx_frame.len = 0x04;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
        tx_frame.data[1] = motor_id_ & 0xFF;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = LRO_CMD_DISABLE;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::ETHERCAT) {
        throw std::runtime_error("LRO driver does not support EtherCAT interface yet");
    }
    {
        response_count_++;
    }
}

uint8_t LroMotorDriver::init_motor() {
    // send disable command to enter read mode
    LroMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
    LroMotorDriver::set_motor_control_mode(MIT);
    Timer::sleep_for(normal_sleep_time);
    // send enable command to enter contorl mode
    LroMotorDriver::lock_motor();
    Timer::sleep_for(normal_sleep_time);
    LroMotorDriver::refresh_motor_status();
    Timer::sleep_for(normal_sleep_time);
    switch (error_id_) {
        case LROError::LRO_MOTOR_OVERHEAT:
            return LROError::LRO_MOTOR_OVERHEAT;
        case LROError::LRO_OVER_CURRENT:
            return LROError::LRO_OVER_CURRENT;
        case LROError::LRO_UNDER_VOLTAGE:
            return LROError::LRO_UNDER_VOLTAGE;
        case LROError::LRO_ENCODER_ERROR:
            return LROError::LRO_ENCODER_ERROR;
        case LROError::LRO_BRAKE_OVERVOLT:
            return LROError::LRO_BRAKE_OVERVOLT;
        case LROError::LRO_DRV_ERROR:
            return LROError::LRO_DRV_ERROR;
        default:
            return error_id_;
    }
    return error_id_;
}

void LroMotorDriver::deinit_motor() {
    LroMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
}

bool LroMotorDriver::write_motor_flash() {
    return true;
}

bool LroMotorDriver::set_motor_zero() {
    // send set zero command
    LroMotorDriver::set_motor_zero_lro();
    Timer::sleep_for(setup_sleep_time);
    LroMotorDriver::refresh_motor_status();
    Timer::sleep_for(setup_sleep_time);
    logger_->info("motor_id: {0}\tposition: {1}", motor_id_, get_motor_pos());
    LroMotorDriver::unlock_motor();
    if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
        logger_->warn("set zero error");
        return false;
    } else {
        logger_->info("set zero success");
        return true;
    }
    // disable motor
}

void LroMotorDriver::canfd_rx_cbk(const canfd_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    if (rx_frame.len < 0x08) return;
    uint16_t pos_int = 0;
    uint16_t spd_int = 0;
    uint16_t t_int = 0;
    uint8_t fb_type = (rx_frame.data[0] >> 5) & 0x07;
    error_id_ = rx_frame.data[0] & 0x1F;
    if (error_id_ > 0) {
        if (logger_) {
            logger_->error("can_interface: {0}\tmotor_id: {1}\terror_id: 0x{2:x}", can_interface_, motor_id_, static_cast<uint32_t>(error_id_));
        }
    }
    pos_int = (static_cast<uint16_t>(rx_frame.data[1]) << 8) | rx_frame.data[2];
    spd_int= (static_cast<uint16_t>(rx_frame.data[3]) << 4) | ((rx_frame.data[4] >> 4) & 0x0F);
    t_int = (static_cast<uint16_t>(rx_frame.data[4] & 0x0F) << 8) | rx_frame.data[5];
    motor_pos_ = 
        range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), -limit_param_.PosMax, limit_param_.PosMax) + static_cast<float>(motor_zero_offset_);
    motor_spd_ = 
        range_map(spd_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.SpdMax, limit_param_.SpdMax);
    motor_current_ = 
        range_map(t_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.TauMax, limit_param_.TauMax);
    mos_temperature_ = rx_frame.data[7];
    motor_temperature_ = static_cast<float>(static_cast<int>(rx_frame.data[6]) - 25); // Temperature: value - 25 = actual temperature
}

// void LroMotorDriver::ethercat_rx_cbk(const ethercat_frame& rx_frame) {}

void LroMotorDriver::get_motor_param(uint8_t param_cmd) {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_; // ID: Corresponds to the individual motor ID
        tx_frame.len = 0x02; // Query commands typically require only 2 bytes
        tx_frame.flags = CANFD_BRS;

        // Byte 0: Mode bits [Mode(3 bits) | Reserved(5 bits)]
        // Query mode (Mode 0x07) shifted left by 5 bits results in 0xE0
        tx_frame.data[0] = (uint8_t)(LRO_MODE_QUERY << 5);
        tx_frame.data[1] = param_cmd; //// Query (mode 0x07) — request param/status

        canfd_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

// // ------------------------------------------------------------------
// // Position control (mode 0x01)
// // Protocol bit layout (8 bytes):
// //   motor_mode    uint3   = 0x01
// //   pos (degrees) float32
// //   spd (0~32767) uint15  -> 0~3276.7 rpm, scale 10
// //   cur_limit     uint12  -> 0~409.5A, scale 10
// //   ack           uint2   -> 0=no reply, 1=type1, 2=type2, 3=type3
// // ------------------------------------------------------------------
// void LroMotorDriver::motor_pos_cmd(float pos, float spd, bool ignore_limit) {
//     // Convert rad to degrees
//     float pos_deg = (pos - static_cast<float>(motor_zero_offset_)) * 180.0f / static_cast<float>(M_PI);
//     // Convert rad/s to rpm (0~3276.7 rpm)
//     float spd_rpm = std::abs(spd) * 60.0f / (2.0f * static_cast<float>(M_PI));
//     uint16_t spd_val = static_cast<uint16_t>(limit(spd_rpm * 10.0f, 0.0f, 32767.0f));
//     uint16_t cur_limit = 4095;  // Max current, 409.5A
//     uint8_t ack = 1;            // Request type-1 feedback

//     // Float to bytes (big-endian)
//     union32_t pos_union;
//     pos_union.f = pos_deg;

//     canfd_frame tx_frame{};
//     tx_frame.can_id = motor_id_;
//     tx_frame.len = 0x08;
//     tx_frame.flags = CANFD_BRS;

//     // Byte0: mode[2:0](3bit) | pos_float[31:27](5bit, top 5 bits of float)
//     // Actually the protocol says:
//     //   motor_mode uint3 | pos float32 | spd uint15 | cur uint12 | ack uint2
//     //   Total = 3 + 32 + 15 + 12 + 2 = 64 bits
//     uint64_t packed = 0;
//     packed |= (static_cast<uint64_t>(LRO_MODE_POS & 0x07)) << 61;
//     packed |= (static_cast<uint64_t>(pos_union.u)) << 29;
//     packed |= (static_cast<uint64_t>(spd_val & 0x7FFF)) << 14;
//     packed |= (static_cast<uint64_t>(cur_limit & 0x0FFF)) << 2;
//     packed |= (static_cast<uint64_t>(ack & 0x03));

//     tx_frame.data[0] = (packed >> 56) & 0xFF;
//     tx_frame.data[1] = (packed >> 48) & 0xFF;
//     tx_frame.data[2] = (packed >> 40) & 0xFF;
//     tx_frame.data[3] = (packed >> 32) & 0xFF;
//     tx_frame.data[4] = (packed >> 24) & 0xFF;
//     tx_frame.data[5] = (packed >> 16) & 0xFF;
//     tx_frame.data[6] = (packed >> 8) & 0xFF;
//     tx_frame.data[7] = packed & 0xFF;

//     canfd_->transmit(tx_frame);
//     {
//         response_count_++;
//     }
// }

// // ------------------------------------------------------------------
// // Speed control (mode 0x02)
// // Protocol bit layout (8 bytes):
// //   motor_mode       uint3  = 0x02
// //   reserved_ctrl    uint3  = 0
// //   ack              uint2
// //   spd (rpm)        float32
// //   cur_limit        uint16 -> 0~6553.6A, scale 10
// // ------------------------------------------------------------------
// void LroMotorDriver::motor_spd_cmd(float spd) {
//     // Convert rad/s to rpm
//     float spd_rpm = spd * 60.0f / (2.0f * static_cast<float>(M_PI));
//     uint16_t cur_limit = 65535;  // Max current limit
//     uint8_t ack = 1;             // Request type-1 feedback

//     union32_t spd_union;
//     spd_union.f = spd_rpm;

//     canfd_frame tx_frame{};
//     tx_frame.can_id = motor_id_;
//     tx_frame.len = 0x08;
//     tx_frame.flags = CANFD_BRS;

//     // Byte0: mode[2:0](3) | reserved(3) | ack(2)
//     tx_frame.data[0] = ((LRO_MODE_SPD & 0x07) << 5) | (ack & 0x03);
//     // Byte1~4: float32 speed (big-endian)
//     tx_frame.data[1] = spd_union.buf[3];
//     tx_frame.data[2] = spd_union.buf[2];
//     tx_frame.data[3] = spd_union.buf[1];
//     tx_frame.data[4] = spd_union.buf[0];
//     // Byte5~6: current limit (uint16, big-endian)
//     tx_frame.data[5] = (cur_limit >> 8) & 0xFF;
//     tx_frame.data[6] = cur_limit & 0xFF;
//     tx_frame.data[7] = 0x00;

//     canfd_->transmit(tx_frame);
//     {
//         response_count_++;
//     }
// }

// ------------------------------------------------------------------
// MIT impedance control (mode 0x00)
// Protocol bit layout (8 bytes, standard CAN, ID = motor_id):
//   motor_mode  uint3   (bits 63-61)
//   KP          uint12  (bits 60-49)
//   KD          uint9   (bits 48-40)
//   pos         uint16  (bits 39-24)
//   spd         uint12  (bits 23-12)
//   torque      uint12  (bits 11-0)
// ------------------------------------------------------------------
void LroMotorDriver::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    // 1. Check control mode: switch to MIT mode if necessary
    // Note: Mode switching is a low-frequency operation; independent packet transmission is safe.
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        // Allow buffer time for motor internal logic to switch
        Timer::sleep_for(normal_sleep_time);
    }
    if (comm_type_ == CommType::CANFD) { 
        target_pos_.store(f_p);
        target_spd_.store(f_v);
        target_kp_.store(f_kp);
        target_kd_.store(f_kd);
        target_trq_.store(f_t);
        // f_p -= static_cast<float>(motor_zero_offset_);
        // f_p = limit(f_p, -limit_param_.PosMax, limit_param_.PosMax);
        // f_v = limit(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
        // f_kp = limit(f_kp, 0.0f, limit_param_.KpMax);
        // f_kd = limit(f_kd, 0.0f, limit_param_.KdMax);
        // f_t = limit(f_t, -limit_param_.TorMax, limit_param_.TorMax);

        // uint16_t kp = range_map(f_kp, 0.0f, limit_param_.KpMax, uint16_t(0), bitmax<uint16_t>(12));
        // uint16_t kd = range_map(f_kd, 0.0f, limit_param_.KdMax, uint16_t(0), bitmax<uint16_t>(9));
        // uint16_t pos = range_map(f_p, -limit_param_.PosMax, limit_param_.PosMax,
        //                          uint16_t(0), bitmax<uint16_t>(16));
        // uint16_t spd = range_map(f_v, -limit_param_.SpdMax, limit_param_.SpdMax,
        //                          uint16_t(0), bitmax<uint16_t>(12));
        // uint16_t tor = range_map(f_t, -limit_param_.TorMax, limit_param_.TorMax,
        //                          uint16_t(0), bitmax<uint16_t>(12));
        // // Pack into 8 bytes (big-endian bit order):
        // // Byte0: mode[2:0] | KP[11:9]     -> (mode<<5) | (kp>>7)
        // // Byte1: KP[8:1]                   -> (kp>>1) & 0xFF  [actually kp bits 8-1 => shift by... ]
        // // Let's lay it out carefully:
        // //
        // // Bit63..61 = mode (3 bits)  = 0x00
        // // Bit60..49 = KP (12 bits)
        // // Bit48..40 = KD (9 bits)
        // // Bit39..24 = POS (16 bits)
        // // Bit23..12 = SPD (12 bits)
        // // Bit11..0  = TOR (12 bits)
        // //
        // // Total = 3+12+9+16+12+12 = 64 bits = 8 bytes
        // uint64_t packed = 0;
        // packed |= (static_cast<uint64_t>(LRO_MODE_MIT & 0x07)) << 61;
        // packed |= (static_cast<uint64_t>(kp & 0x0FFF)) << 49;
        // packed |= (static_cast<uint64_t>(kd & 0x01FF)) << 40;
        // packed |= (static_cast<uint64_t>(pos & 0xFFFF)) << 24;
        // packed |= (static_cast<uint64_t>(spd & 0x0FFF)) << 12;
        // packed |= (static_cast<uint64_t>(tor & 0x0FFF));

        // canfd_frame tx_frame{};
        // tx_frame.can_id = motor_id_;
        // tx_frame.len = 0x08;
        // tx_frame.flags = CANFD_BRS;
        // tx_frame.data[0] = (packed >> 56) & 0xFF;
        // tx_frame.data[1] = (packed >> 48) & 0xFF;
        // tx_frame.data[2] = (packed >> 40) & 0xFF;
        // tx_frame.data[3] = (packed >> 32) & 0xFF;
        // tx_frame.data[4] = (packed >> 24) & 0xFF;
        // tx_frame.data[5] = (packed >> 16) & 0xFF;
        // tx_frame.data[6] = (packed >> 8) & 0xFF;
        // tx_frame.data[7] = packed & 0xFF;

        // canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::ETHERCAT) {
        throw std::runtime_error("LRO driver does not support EtherCAT interface yet");
    }
    {
        response_count_++;
    }
}

void LroMotorDriver::reset_motor_id() {
    // Reset all motors on the bus to ID 0x01
    canfd_frame tx_frame{};
    tx_frame.can_id = 0x7FF;
    tx_frame.len = 0x06;
    tx_frame.flags = CANFD_BRS;

    tx_frame.data[0] = 0x7F;
    tx_frame.data[1] = 0x7F;
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = LRO_CMD_RESET_ID;
    tx_frame.data[4] = 0x7F;
    tx_frame.data[5] = 0x7F;

    canfd_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void LroMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    motor_control_mode_ = motor_control_mode;
    // LeadRobot doesn't need explicit mode switching;
    // the mode is embedded in each control frame.
}

void LroMotorDriver::set_motor_zero_lro() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x7FF;
        tx_frame.len = 0x04;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
        tx_frame.data[1] = motor_id_ & 0xFF;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = LRO_CMD_SET_ZERO;
        
        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::ETHERCAT) {
        throw std::runtime_error("LRO driver does not support EtherCAT interface yet");
    }
    {
        response_count_++;
    }
}


void LroMotorDriver::clear_motor_error_lro() {
    if (comm_type_ == CommType::CANFD) {
        {
            canfd_frame tx_frame{};
            tx_frame.can_id = 0x7FF;
            tx_frame.len = 0x04;
            tx_frame.flags = CANFD_BRS;

            tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
            tx_frame.data[1] = motor_id_ & 0xFF;
            tx_frame.data[2] = 0x00;
            tx_frame.data[3] = LRO_CMD_DISABLE;

            canfd_->transmit(tx_frame);
        }
        {
            response_count_++;
        }
        Timer::sleep_for(normal_sleep_time);
        {
            canfd_frame tx_frame{};
            tx_frame.can_id = 0x7FF;
            tx_frame.len = 0x04;
            tx_frame.flags = CANFD_BRS;

            tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;
            tx_frame.data[1] = motor_id_ & 0xFF;
            tx_frame.data[2] = 0x00;
            tx_frame.data[3] = LRO_CMD_ENABLE;

            canfd_->transmit(tx_frame);
        }
        {
            response_count_++;
        }
    } else if (comm_type_ == CommType::ETHERCAT) {
        throw std::runtime_error("LRO driver does not support EtherCAT interface yet");
    }
}

// void EvoMotorDriver::write_register_evo(uint8_t rid, float value){
//     // Evo motor doesn't have a generic register write command;
//     // parameters are set via specific control commands.
// }

void LroMotorDriver::write_register_lro(uint8_t index, int32_t value) {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x06;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = (LRO_MODE_CONFIG << 5);
        tx_frame.data[1] = index;
        tx_frame.data[2] = (value >> 24) & 0xFF;
        tx_frame.data[3] = (value >> 16) & 0xFF;
        tx_frame.data[4] = (value >> 8) & 0xFF;
        tx_frame.data[5] = value & 0xFF;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::ETHERCAT) {
        throw std::runtime_error("LRO driver does not support EtherCAT interface yet");
    }
    {
        response_count_++;
    }
}

void LroMotorDriver::save_register_lro() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x7FF; // ID: Uses the dedicated global configuration identifier 0x7FF
        tx_frame.len = 0x04;
        tx_frame.flags = CANFD_BRS;

        // Byte 0-1: Target Motor ID (Big-Endian)
        tx_frame.data[0] = (uint8_t)((motor_id_ >> 8) & 0xFF);
        tx_frame.data[1] = (uint8_t)(motor_id_ & 0xFF);
        tx_frame.data[2] = 0x00; // Byte 2: Communication Direction (0x00 represents Controller to Motor)
        tx_frame.data[3] = 0x04; // Byte 3: Command Code (LRO protocol defines 0x04 as "Save Parameters to Flash")

        canfd_->transmit(tx_frame);
    }
    response_count_++;
}

void LroMotorDriver::refresh_motor_status() {
    // Send a zero MIT command with type-1 feedback to get current status
    motor_mit_cmd(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void LroMotorDriver::clear_motor_error() {
    clear_motor_error_lro();
}

void LroMotorDriver::pack_cmd_data(uint8_t* buffer) {
    float f_p, f_v, f_kp, f_kd, f_t;
    uint16_t p, v, kp, kd, t;

    f_p = limit(target_pos_.load(), -limit_param_.PosMax, limit_param_.PosMax);
    f_v = limit(target_spd_.load(), -limit_param_.SpdMax, limit_param_.SpdMax);
    f_kp = limit(target_kp_.load(), 0.0f, limit_param_.OKpMax);
    f_kd = limit(target_kd_.load(), 0.0f, limit_param_.OKdMax);
    f_t = limit(target_trq_.load(), -limit_param_.TauMax, limit_param_.TauMax);

    p = range_map(f_p, -limit_param_.PosMax, limit_param_.PosMax, uint16_t(0), bitmax<uint16_t>(16));
    v = range_map(f_v, -limit_param_.SpdMax, limit_param_.SpdMax, uint16_t(0), bitmax<uint16_t>(12));
    kp  = range_map(f_kp, 0.0f, limit_param_.OKpMax, uint16_t(0), bitmax<uint16_t>(12));
    kd  = range_map(f_kd, 0.0f, limit_param_.OKdMax, uint16_t(0), bitmax<uint16_t>(9));
    t = range_map(f_t, -limit_param_.TauMax, limit_param_.TauMax, uint16_t(0), bitmax<uint16_t>(12));

    // 3. Fill the buffer according to LRO protocol bitfields (Big-Endian)
    // Layout: Mode[3] + ID[5] + KP[12] + KD[9] + POS[16] + SPD[12] + TOR[12] = 64 bits total
    // Byte 0: Mode(3 bits) + Motor_ID(5 bits)
    buffer[0] = (uint8_t)(((LRO_MODE_MIT & 0x07) << 5) | (motor_id_ & 0x1F));
    
    // Byte 1: KP[11:4](8 bits)
    buffer[1] = (uint8_t)((kp >> 4) & 0xFF);
    
    // Byte 2: KP[3:0](4 bits) + KD[8:5](4 bits)
    buffer[2] = (uint8_t)(((kp & 0x0F) << 4) | ((kd >> 5) & 0x0F));

    // Byte 3: KD[4:0](5 bits) + POS[15:13](3 bits)
    buffer[3] = (uint8_t)(((kd & 0x1F) << 3) | ((p >> 13) & 0x07));
    
    // Byte 4: POS[7:0](8 bits)
    buffer[4] = (uint8_t)(p & 0xFF);
    
    // Byte 5: SPD[11:4](8 bits)
    buffer[5] = (uint8_t)((v >> 4) & 0xFF);
    
    // Byte 6: SPD[3:0](4 bits) + TOR[11:8](4 bits)
    buffer[6] = (uint8_t)(((v & 0x0F) << 4) | ((t >> 8) & 0x0F));
    
    // Byte 7: TOR[7:0](8 bits)
    buffer[7] = (uint8_t)(t & 0xFF);
}

// // ------------------------------------------------------------------
// // 0x7FF setup commands
// // ------------------------------------------------------------------
// void LroMotorDriver::send_setup_cmd(uint8_t cmd_code) {
//     canfd_frame tx_frame{};
//     tx_frame.can_id = 0x7FF;
//     tx_frame.len = 0x04;
//     tx_frame.flags = CANFD_BRS;

//     tx_frame.data[0] = (motor_id_ >> 8) & 0xFF;   // motor ID high
//     tx_frame.data[1] = motor_id_ & 0xFF;           // motor ID low
//     tx_frame.data[2] = 0x00;                       // 0x00 = from controller
//     tx_frame.data[3] = cmd_code;

//     canfd_->transmit(tx_frame);
//     response_count_++;
// }

    // switch (fb_type) {
    //     case LRO_FB_TYPE1: {
    //         // Type 1: bit-packed feedback
    //         // Byte0[7:5]=type, Byte0[4:0]=error
    //         // Byte1~2: position (uint16)
    //         // Byte3[7:4]~Byte4: speed (uint12)
    //         // Byte4[3:0]~Byte5: iq current (uint12)
    //         // Byte6: motor temperature (uint8, actual = value - 25)
    //         // Byte7: MOS temperature (uint8, actual = value - 25)
    //         // Byte8~9: DC bus voltage (uint16)
    //         // Byte10~11: DC bus current (uint16)
    //         if (rx_frame.len >= 8) {
    //             uint16_t pos_raw = (static_cast<uint16_t>(rx_frame.data[1]) << 8) | rx_frame.data[2];
    //             uint16_t spd_raw = (static_cast<uint16_t>(rx_frame.data[3]) << 4) |
    //                                ((rx_frame.data[4] >> 4) & 0x0F);
    //             uint16_t iq_raw = (static_cast<uint16_t>(rx_frame.data[4] & 0x0F) << 8) |
    //                               rx_frame.data[5];
    //             // // DC bus voltage/current (bytes 8~11, present when DLC >= 12)
    //             // if (rx_frame.len >= 12) {
    //             //     uint16_t dc_vol_raw = (static_cast<uint16_t>(rx_frame.data[8]) << 8) | rx_frame.data[9];
    //             //     uint16_t dc_cur_raw = (static_cast<uint16_t>(rx_frame.data[10]) << 8) | rx_frame.data[11];
    //             //     dc_bus_voltage_ = range_map(dc_vol_raw, uint16_t(0), bitmax<uint16_t>(16),
    //             //                                 0.0f, 100.0f);
    //             //     dc_bus_current_ = range_map(dc_cur_raw, uint16_t(0), bitmax<uint16_t>(16),
    //             //                                 -100.0f, 100.0f);
    //             // }
    //             motor_pos_ = 
    //                 range_map(pos_raw, uint16_t(0), bitmax<uint16_t>(16), -limit_param_.PosMax, limit_param_.PosMax) + static_cast<float>(motor_zero_offset_);
    //             motor_spd_ = 
    //                 range_map(spd_raw, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.SpdMax, limit_param_.SpdMax);
    //             motor_current_ = 
    //                 range_map(iq_raw, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.CurMax, limit_param_.CurMax);
    //             mos_temperature_ = rx_frame.data[7];
    //             motor_temperature_ = static_cast<float>(static_cast<int>(rx_frame.data[6]) - 25); // Temperature: value - 25 = actual temperature
    //         }
    //         break;
    //     }
    //     // case LRO_FB_TYPE2: {
    //     //     // Type 2: float position + int16 current + uint8 temperature
    //     //     if (rx_frame.len >= 8) {
    //     //         union32_t pos_union;
    //     //         pos_union.buf[3] = rx_frame.data[1];
    //     //         pos_union.buf[2] = rx_frame.data[2];
    //     //         pos_union.buf[1] = rx_frame.data[3];
    //     //         pos_union.buf[0] = rx_frame.data[4];
    //     //         // Position in degrees, convert to rad
    //     //         motor_pos_ = pos_union.f * static_cast<float>(M_PI) / 180.0f +
    //     //                      static_cast<float>(motor_zero_offset_);

    //     //         int16_t cur_raw = (static_cast<int16_t>(rx_frame.data[5]) << 8) | rx_frame.data[6];
    //     //         motor_current_ = static_cast<float>(cur_raw) / 100.0f;  // -327.68~327.67A
    //     //         motor_temperature_ = static_cast<float>(static_cast<int>(rx_frame.data[7]) - 25);
    //     //     }
    //     //     break;
    //     // }
    //     // case LRO_FB_TYPE3: {
    //     //     // Type 3: float speed + int16 current + uint8 temperature
    //     //     if (rx_frame.len >= 8) {
    //     //         union32_t spd_union;
    //     //         spd_union.buf[3] = rx_frame.data[1];
    //     //         spd_union.buf[2] = rx_frame.data[2];
    //     //         spd_union.buf[1] = rx_frame.data[3];
    //     //         spd_union.buf[0] = rx_frame.data[4];
    //     //         // Speed in RPM, convert to rad/s
    //     //         motor_spd_ = spd_union.f * 2.0f * static_cast<float>(M_PI) / 60.0f;

    //     //         int16_t cur_raw = (static_cast<int16_t>(rx_frame.data[5]) << 8) | rx_frame.data[6];
    //     //         motor_current_ = static_cast<float>(cur_raw) / 100.0f;
    //     //         motor_temperature_ = static_cast<float>(static_cast<int>(rx_frame.data[7]) - 25);
    //     //     }
    //     //     break;
    //     // }
    //     // case LRO_FB_TYPE4: {
    //     //     // Type 4: config result
    //     //     // data[1] = config code, data[2] = 0(fail)/1(success)
    //     //     break;
    //     // }
    //     // case LRO_FB_TYPE5: {
    //     //     // Type 5: query result
    //     //     // data[1] = query code, data[2..] = query data
    //     //     break;
    //     // }
    //     default:
    //         break;
    // }