#include "evo_motor_driver.hpp"

EVO_Limit_Param evo_limit_param[EVO_Num_Of_Motor] = {
    {12.5, 20.0, 18.0, 500.0, 5.0},     // EVO431040
    {12.5, 10.0, 50.0, 250.0, 50.0},    // EVO811825
    {12.5, 10.0, 50.0, 250.0, 50.0},    // EVO811832
};

EvoMotorDriver::EvoMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                               EVO_Motor_Model motor_model, double motor_zero_offset)
    : MotorDriver(), motor_model_(motor_model) {
    if (interface_type != "can" && interface_type != "canfd" && interface_type != "ethercanfd") {
        throw std::runtime_error("EVO driver only supports CAN and CAN-FD interfaces");
    }
    motor_id_ = motor_id;
    limit_param_ = evo_limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;

    if (interface_type == "canfd" || interface_type == "ethercanfd") {
        comm_type_ = CommType::CANFD;
        motor_index_ = (motor_id_ > 0 && motor_id_ <= 8) ? (motor_id_ - 1) : 0;
        canfd_ = MotorsSocketCANFD::get(can_interface);
        
        CanFdCbkFunc canfd_callback = std::bind(&EvoMotorDriver::canfd_rx_cbk, this, std::placeholders::_1);
        canfd_->add_canfd_callback(canfd_callback, motor_id_);
    } else if (interface_type == "can") {
        comm_type_ = CommType::CAN;
        can_ = MotorsSocketCAN::get(can_interface);
        
        CanCbkFunc can_callback = std::bind(&EvoMotorDriver::can_rx_cbk, this, std::placeholders::_1);
        can_->add_can_callback(can_callback, motor_id_);
    } 
}

EvoMotorDriver::~EvoMotorDriver() { 
    if (comm_type_ == CommType::CANFD) {
        canfd_->remove_canfd_callback(motor_id_);
    } else if (comm_type_ == CommType::CAN) {
        can_->remove_can_callback(motor_id_);
    }
}

void EvoMotorDriver::lock_motor() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = 0xFC;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.can_dlc = 0x08;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = 0xFC;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::unlock_motor() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;
        
        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = 0xFD;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.can_dlc = 0x08;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = 0xFD;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

uint8_t EvoMotorDriver::init_motor() {
    // send disable command to enter read mode
    EvoMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
    set_motor_control_mode(MIT);
    Timer::sleep_for(normal_sleep_time);
    // send enable command to enter contorl mode
    EvoMotorDriver::lock_motor();
    Timer::sleep_for(normal_sleep_time);
    EvoMotorDriver::refresh_motor_status();
    Timer::sleep_for(normal_sleep_time);
    switch (error_id_) {
        case EVOError::EVO_OVER_VOLTAGE:
            return EVOError::EVO_OVER_VOLTAGE;
            break;
        case EVOError::EVO_UNDER_VOLTAGE:
            return EVOError::EVO_UNDER_VOLTAGE;
            break;
        case EVOError::EVO_OVER_CURRENT:
            return EVOError::EVO_OVER_CURRENT;
            break;
        case EVOError::EVO_MOS_OVER_TEMP:
            return EVOError::EVO_MOS_OVER_TEMP;
            break;
        case EVOError::EVO_COIL_OVER_TEMP:
            return EVOError::EVO_COIL_OVER_TEMP;
            break;
        case EVOError::EVO_COMM_LOST:
            return EVOError::EVO_COMM_LOST;
            break;
        case EVOError::EVO_OVERLOAD:
            return EVOError::EVO_OVERLOAD;
            break;
        case EVOError::EVO_ENCODER_ERROR:
            return EVOError::EVO_ENCODER_ERROR;
            break;
        default:
            return error_id_;
    }
    return error_id_;
}

void EvoMotorDriver::deinit_motor() {
    EvoMotorDriver::unlock_motor();
    Timer::sleep_for(normal_sleep_time);
}

bool EvoMotorDriver::write_motor_flash() { 
    return true;
}

bool EvoMotorDriver::set_motor_zero() {
    // send set zero command
    EvoMotorDriver::set_motor_zero_evo();
    Timer::sleep_for(setup_sleep_time);
    EvoMotorDriver::refresh_motor_status();
    Timer::sleep_for(setup_sleep_time);  // wait for motor to set zero
    logger_->info("motor_id: {0}\tposition: {1}\t", motor_id_, get_motor_pos());
    EvoMotorDriver::unlock_motor();
    if (get_motor_pos() > judgment_accuracy_threshold || get_motor_pos() < -judgment_accuracy_threshold) {
        logger_->warn("set zero error");
        return false;
    } else {
        logger_->info("set zero success");
        return true;
    }
    // disable motor
}

void EvoMotorDriver::can_rx_cbk(const can_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    uint16_t pos_int = 0;
    uint16_t spd_int = 0;
    uint16_t t_int = 0;
    pos_int = rx_frame.data[1] << 8 | rx_frame.data[2];
    spd_int = rx_frame.data[3] << 4 | (rx_frame.data[4] & 0xF0) >> 4;
    t_int = (rx_frame.data[4] & 0x0F) << 8 | rx_frame.data[5];
    error_id_ = rx_frame.data[6];
    if (error_id_ > 0) {
            if (logger_) {
            logger_->error("can_interface: {0}\tmotor_id: {1}\terror_id: 0x{2:x}", can_interface_, motor_id_, (uint32_t)error_id_);
        }
    }
    motor_pos_ = 
        range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), -limit_param_.PosMax, limit_param_.PosMax) + motor_zero_offset_;
    motor_spd_ = 
        range_map(spd_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.SpdMax, limit_param_.SpdMax);
    motor_current_ = 
        range_map(t_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.TauMax, limit_param_.TauMax);
    mos_temperature_ = rx_frame.data[7];
    motor_temperature_ = rx_frame.data[7];
}

void EvoMotorDriver::canfd_rx_cbk(const canfd_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    if (rx_frame.len < 8) return;

    if (!(rx_frame.flags & CANFD_FDF)) {
        can_frame frame{};
        frame.can_id = rx_frame.can_id;
        frame.can_dlc = rx_frame.len;
        memcpy(frame.data, rx_frame.data, 8);
        can_rx_cbk(frame);
        return;
    }

    uint16_t pos_int = 0;
    uint16_t spd_int = 0;
    uint16_t t_int = 0;
    pos_int = rx_frame.data[0] << 8 | rx_frame.data[1];
    spd_int = rx_frame.data[2] << 4 | (rx_frame.data[3] >> 4);
    t_int = ((rx_frame.data[3] & 0x0F) << 8) | rx_frame.data[4];
    error_id_ = ((rx_frame.data[6] << 8) | rx_frame.data[7]) >> 1 & 0x7F;
    if (error_id_ > 0) {
        if (logger_) {
            logger_->error("can_interface: {0}\tmotor_id: {1}\terror_id: 0x{2:x}", can_interface_, motor_id_, (uint32_t)error_id_);
        }
    }
    motor_pos_ = 
        range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), -limit_param_.PosMax, limit_param_.PosMax) + motor_zero_offset_;
    motor_spd_ = 
        range_map(spd_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.SpdMax, limit_param_.SpdMax);
    motor_current_ = 
        range_map(t_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.TauMax, limit_param_.TauMax);
    mos_temperature_ = static_cast<float>(rx_frame.data[5]) - 40.0f;
    motor_temperature_ = static_cast<float>(rx_frame.data[5]) - 40.0f;
}

void EvoMotorDriver::get_motor_param(uint8_t param_cmd) {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0x67;      // Frame Header
        tx_frame.data[1] = param_cmd; // Register Index
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = 0x00;
        tx_frame.data[5] = 0x00;
        tx_frame.data[6] = 0x04;      // 0x04 denotes a "Read" operation
        tx_frame.data[7] = 0x76;      // Frame Tail

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.can_dlc = 0x08;
        
        tx_frame.data[0] = 0x67;
        tx_frame.data[1] = param_cmd;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = 0x00;
        tx_frame.data[5] = 0x00;
        tx_frame.data[6] = 0x04;
        tx_frame.data[7] = 0x76;
        
        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

// void EvoMotorDriver::motor_pos_cmd(float pos, float spd, bool ignore_limit) {
//     //
// }

// void EvoMotorDriver::motor_spd_cmd(float spd) {
//     //
// }

// Transmit MIT-mDme control(hybrid) package. Called in canTask.
void EvoMotorDriver::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    if (motor_control_mode_ != MIT) {
        set_motor_control_mode(MIT);
        return;
    }
    uint16_t p, v, kp, kd, t;

    f_p -= motor_zero_offset_;
    f_p = limit(f_p, -limit_param_.PosMax, limit_param_.PosMax);
    f_v = limit(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
    f_kp = limit(f_kp, 0.0f, limit_param_.OKpMax);
    f_kd = limit(f_kd, 0.0f, limit_param_.OKdMax);
    f_t = limit(f_t, -limit_param_.TauMax, limit_param_.TauMax);
    
    p = range_map(f_p, -limit_param_.PosMax, limit_param_.PosMax, uint16_t(0), bitmax<uint16_t>(16));
    v = range_map(f_v, -limit_param_.SpdMax, limit_param_.SpdMax, uint16_t(0), bitmax<uint16_t>(12));
    kp = range_map(f_kp, 0.0f, limit_param_.OKpMax, uint16_t(0), bitmax<uint16_t>(12));
    kd = range_map(f_kd, 0.0f, limit_param_.OKdMax, uint16_t(0), bitmax<uint16_t>(12));
    t = range_map(f_t, -limit_param_.TauMax, limit_param_.TauMax, uint16_t(0), bitmax<uint16_t>(12));

    if (comm_type_ == CommType::CANFD) {
        target_pos_.store(f_p);
        target_spd_.store(f_v);
        target_kp_.store(f_kp);
        target_kd_.store(f_kd);
        target_trq_.store(f_t);
        // canfd_frame tx_frame{};
        // tx_frame.can_id = EVOFD_MIT_ID;
        // tx_frame.len = 0x40;
        // tx_frame.flags = CANFD_BRS;
        // int offset = motor_index_ * 8;
        // tx_frame.data[0] = p >> 8;
        // tx_frame.data[1] = p & 0xFF;
        // tx_frame.data[2] = v >> 4;
        // tx_frame.data[3] = ((v & 0x0F) << 4) | (kp >> 8);
        // tx_frame.data[4] = kp & 0xFF;
        // tx_frame.data[5] = kd >> 4;
        // tx_frame.data[6] = ((kd & 0x0F) << 4) | (t >> 8);
        // tx_frame.data[7] = t & 0xFF;

        // canfd_->transmit(tx_frame);

    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.can_dlc = 0x08;

        tx_frame.data[0] = p >> 8;
        tx_frame.data[1] = p & 0xFF;
        tx_frame.data[2] = v >> 4;
        tx_frame.data[3] = (v & 0x0F) << 4 | kp >> 8;
        tx_frame.data[4] = kp & 0xFF;
        tx_frame.data[5] = kd >> 4;
        tx_frame.data[6] = (kd & 0x0F) << 4 | t >> 8;
        tx_frame.data[7] = t & 0xFF;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    if (comm_type_ == CommType::CAN) write_register_evo(11, 0x02);
    motor_control_mode_ = motor_control_mode;
}

void EvoMotorDriver::set_motor_zero_evo() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = 0xFE;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.can_dlc = 0x08;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = 0xFE;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::clear_motor_error_evo() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = 0xFD;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.can_dlc = 0x08;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = 0xFD;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

// void EvoMotorDriver::write_register_evo(uint8_t rid, float value){
//     //
// }

void EvoMotorDriver::write_register_evo(uint8_t index, int32_t value) {
    uint8_t* vbuf;
    vbuf = (uint8_t*)&value;

    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;
        
        uint8_t* vbuf = (uint8_t*)&value;
        tx_frame.data[0] = 0x67;
        tx_frame.data[1] = index;
        tx_frame.data[2] = vbuf[0];
        tx_frame.data[3] = vbuf[1];
        tx_frame.data[4] = vbuf[2];
        tx_frame.data[5] = vbuf[3];
        tx_frame.data[6] = 0x15;
        tx_frame.data[7] = 0x76;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.can_dlc = 0x08;

        tx_frame.data[0] = 0x67;
        tx_frame.data[1] = index;
        tx_frame.data[2] = *vbuf;
        tx_frame.data[3] = *(vbuf + 1);
        tx_frame.data[4] = *(vbuf + 2);
        tx_frame.data[5] = *(vbuf + 3);
        tx_frame.data[6] = 0x15;
        tx_frame.data[7] = 0x76;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::save_register_evo() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = EVOFD_CMD_ID;
        tx_frame.len = 64;
        tx_frame.flags = CANFD_BRS;
        for (int i = 0; i < 64; i++) tx_frame.data[i] = 0xFF;
        
        int offset = motor_index_ * 8;
        tx_frame.data[0] = 0x67; // EVO Header/Feature Byte
        tx_frame.data[1] = 0x00; // 0x00 is typically the Save Instruction index
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = 0x00;
        tx_frame.data[5] = 0x00;
        tx_frame.data[6] = 0x00;
        tx_frame.data[7] = 0x76; // Tail/End Byte

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = 0x600 + motor_id_;
        tx_frame.can_dlc = 0x08;
        
        tx_frame.data[0] = 0x67;
        tx_frame.data[1] = 0x00;
        tx_frame.data[2] = 0x00;
        tx_frame.data[3] = 0x00;
        tx_frame.data[4] = 0x00;
        tx_frame.data[5] = 0x00;
        tx_frame.data[6] = 0x00;
        tx_frame.data[7] = 0x76;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::refresh_motor_status() {
    if (comm_type_ == CommType::CANFD) {
        canfd_frame tx_frame{};
        tx_frame.can_id = motor_id_;
        tx_frame.len = 0x08;
        tx_frame.flags = CANFD_BRS;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = 0xFC;

        canfd_->transmit(tx_frame);
    } else if (comm_type_ == CommType::CAN) {
        can_frame tx_frame;
        tx_frame.can_id = motor_id_;
        tx_frame.can_dlc = 0x08;

        tx_frame.data[0] = 0xFF;
        tx_frame.data[1] = 0xFF;
        tx_frame.data[2] = 0xFF;
        tx_frame.data[3] = 0xFF;
        tx_frame.data[4] = 0xFF;
        tx_frame.data[5] = 0xFF;
        tx_frame.data[6] = 0xFF;
        tx_frame.data[7] = 0xFC;

        can_->transmit(tx_frame);
    }
    {
        response_count_++;
    }
}

void EvoMotorDriver::clear_motor_error() {
    clear_motor_error_evo();
}

void EvoMotorDriver::pack_cmd_data(uint8_t* buffer) {
    float f_p, f_v, f_kp, f_kd, f_t;
    uint16_t p, v, kp, kd, t;

    f_p  = limit((float)(target_pos_.load() - motor_zero_offset_), -limit_param_.PosMax, limit_param_.PosMax);
    f_v  = limit(target_spd_.load(), -limit_param_.SpdMax, limit_param_.SpdMax);
    f_kp = limit(target_kp_.load(), 0.0f, limit_param_.OKpMax);
    f_kd = limit(target_kd_.load(), 0.0f, limit_param_.OKdMax);
    f_t  = limit(target_trq_.load(), -limit_param_.TauMax, limit_param_.TauMax);

    p  = range_map(f_p, -limit_param_.PosMax, limit_param_.PosMax, uint16_t(0), uint16_t(0xFFFF));
    v  = range_map(f_v, -limit_param_.SpdMax, limit_param_.SpdMax, uint16_t(0), uint16_t(0xFFF));
    kp = range_map(f_kp, 0.0f, limit_param_.OKpMax, uint16_t(0), uint16_t(0xFFF));
    kd = range_map(f_kd, 0.0f, limit_param_.OKdMax, uint16_t(0), uint16_t(0xFFF));
    t  = range_map(f_t, -limit_param_.TauMax, limit_param_.TauMax, uint16_t(0), uint16_t(0xFFF));
    // 3. Payload Construction: Byte-by-byte Big-Endian splicing
    // Layout: P[16] + V[12] + KP[12] + KD[12] + T[12] = 64bit total
    
    // Position (16 bits)
    buffer[0] = (uint8_t)(p >> 8);
    buffer[1] = (uint8_t)(p & 0xFF);
    
    // Velocity (12 bits) + KP (high 4 bits)
    buffer[2] = (uint8_t)(v >> 4);
    buffer[3] = (uint8_t)(((v & 0x0F) << 4) | (kp >> 8));
    
    // KP (low 8 bits)
    buffer[4] = (uint8_t)(kp & 0xFF);
    
    // KD (12 bits) + Torque (high 4 bits)
    buffer[5] = (uint8_t)(kd >> 4);
    buffer[6] = (uint8_t)(((kd & 0x0F) << 4) | (t >> 8));
    
    // Torque (low 8 bits)
    buffer[7] = (uint8_t)(t & 0xFF);
}
