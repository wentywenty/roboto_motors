#pragma once

#include <atomic>
#include <string>

#include "motor_driver.hpp"
#include "protocol/canfd/socket_canfd.hpp"

// LeadRobot error codes (from type-1 feedback, 5-bit error field)
enum LROError : uint8_t {
    LRO_NO_ERROR        = 0x00,
    LRO_MOTOR_OVERHEAT  = 0x01,
    LRO_OVER_CURRENT    = 0x02,
    LRO_UNDER_VOLTAGE   = 0x03,
    LRO_ENCODER_ERROR   = 0x04,
    LRO_BRAKE_OVERVOLT  = 0x06,
    LRO_DRV_ERROR       = 0x07,
};

enum LRO_Motor_Model {
    LRO_5550,
    LRO_6562,
    LRO_8462,
    LRO_10062,
    LRO_Num_Of_Motor
};

// LeadRobot motor mode byte (upper 3 bits of Byte0 in control frames)
enum LROMotorMode : uint8_t {
    LRO_MODE_MIT     = 0x00,  // MIT impedance control
    LRO_MODE_POS     = 0x01,  // Servo position control
    LRO_MODE_SPD     = 0x02,  // Servo speed control
    LRO_MODE_CUR     = 0x03,  // Current/torque control
    LRO_MODE_CONFIG  = 0x06,  // Parameter configuration
    LRO_MODE_QUERY   = 0x07,  // Parameter query
};

// LeadRobot 0x7FF setup command codes (Byte3)
enum LROSetupCmd : uint8_t {
    LRO_CMD_SET_ZERO   = 0x03,  // Set zero position
    LRO_CMD_SET_ID     = 0x04,  // Change motor ID
    LRO_CMD_RESET_ID   = 0x05,  // Reset ID to 0x01
    LRO_CMD_ENABLE     = 0x06,  // Enable motor
    LRO_CMD_DISABLE    = 0x07,  // Disable motor
    LRO_CMD_QUERY_MODE = 0x81,  // Query communication mode
    LRO_CMD_QUERY_ID   = 0x82,  // Query motor ID
};

// LeadRobot feedback message type (upper 3 bits of Byte0 in response)
enum LROFeedbackType : uint8_t {
    LRO_FB_TYPE1 = 0x01,  // pos(u16) + spd(u12) + iq(u12) + temp(u8) + mos_temp(u8) + dcvol(u16) + dccur(u16)
    LRO_FB_TYPE2 = 0x02,  // pos(float) + cur(i16) + temp(u8)
    LRO_FB_TYPE3 = 0x03,  // spd(float) + cur(i16) + temp(u8)
    LRO_FB_TYPE4 = 0x04,  // config result: code(u8) + status(u8)
    LRO_FB_TYPE5 = 0x05,  // query result: code(u8) + data(variable)
};

// LeadRobot config codes (used with mode 0x06)
enum LROConfigCode : uint8_t {
    LRO_CFG_ACCEL       = 0x01,  // Set acceleration (rad/s^2)
    LRO_CFG_DECEL       = 0x02,  // Set deceleration (rad/s^2)
    LRO_CFG_MAX_SPD     = 0x03,  // Set maximum operating speed (rad/s)
    LRO_CFG_TORQUE_SENS = 0x04,  // Set torque constant (Nm/A)
    LRO_CFG_KP_MAX      = 0x05,  // Set maximum Kp limit
    LRO_CFG_KD_MAX      = 0x06,  // Set maximum Kd limit
    LRO_CFG_POS_MAX     = 0x07,  // Set maximum position limit (rad)
    LRO_CFG_SPD_MAX     = 0x08,  // Set maximum velocity limit (rad/s)
    LRO_CFG_TOR_MAX     = 0x09,  // Set maximum torque limit (Nm)
    LRO_CFG_CUR_MAX     = 0x0A,  // Set maximum current limit (A)
    LRO_CFG_TIMEOUT     = 0x0B,  // Set CAN communication timeout threshold (ms)
    LRO_CFG_CUR_PI      = 0x0C,  // Set current loop PI parameters
    LRO_CFG_SPD_PI      = 0x0D,  // Set speed loop PI parameters
    LRO_CFG_POS_PD      = 0x0E,  // Set position loop PD parameters
    LRO_CFG_KT_CALIB    = 0x0F  // Set torque constant calibration
};

// Query codes for Mode 0x07.Used to retrieve real-time internal information from the motor.
enum LROQueryCode : uint8_t {
    LRO_QRY_FIRMWARE    = 0x0A,  // Query firmware version
    LRO_QRY_HARDWARE    = 0x0B,  // Query hardware version
    LRO_QRY_SERIAL_NUM  = 0x0C   // Query motor serial number
};

// Parameter ranges for MIT mode (configurable per motor)
typedef struct {
    float PosMax;   // Maximum position (rad), default 12.5
    float SpdMax;   // Maximum velocity (rad/s), default 45
    float TauMax;   // Maximum torque (Nm), default 40
    // float CurMax;   // Maximum current (A), default 70
    float OKpMax;    // Default 500
    float OKdMax;    // Default 5
} LRO_Limit_Param;

class LroMotorDriver : public MotorDriver {
   public:
    LroMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                    LRO_Motor_Model motor_model, double motor_zero_offset = 0.0);
    ~LroMotorDriver();

    virtual void lock_motor() override;
    virtual void unlock_motor() override;
    virtual uint8_t init_motor() override;
    virtual void deinit_motor() override;
    virtual bool set_motor_zero() override;
    virtual bool write_motor_flash() override;

    virtual void get_motor_param(uint8_t param_cmd) override;
    virtual void motor_pos_cmd(float pos, float spd, bool ignore_limit) override;
    virtual void motor_spd_cmd(float spd) override;
    virtual void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
    virtual void reset_motor_id() override;
    virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
    virtual int get_response_count() const override { 
        return response_count_; 
    }
    virtual void refresh_motor_status() override;
    virtual void clear_motor_error() override;

    virtual uint8_t get_command_size() override { return 8; }
    virtual void pack_cmd_data(uint8_t* buffer) override;

   private:
    uint8_t motor_index_{0};

    // std::atomic<float> dc_bus_voltage_{0.f};
    // std::atomic<float> dc_bus_current_{0.f};
    std::atomic<float> target_pos_{0.0f};
    std::atomic<float> target_spd_{0.0f};
    std::atomic<float> target_kp_{0.0f};
    std::atomic<float> target_kd_{0.0f};
    std::atomic<float> target_trq_{0.0f};
    
    std::atomic<int> response_count_{0};
    LRO_Motor_Model motor_model_;
    LRO_Limit_Param limit_param_;
    std::atomic<uint8_t> mos_temperature_{0};
    void set_motor_zero_lro();
    void clear_motor_error_lro();
    void write_register_lro(uint8_t rid, float value);
    void write_register_lro(uint8_t index, int32_t value);
    void save_register_lro();

    virtual void canfd_rx_cbk(const canfd_frame& rx_frame);
    std::shared_ptr<MotorsSocketCANFD> canfd_;
    std::string can_interface_;
};
