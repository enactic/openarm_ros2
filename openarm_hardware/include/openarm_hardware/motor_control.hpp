#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <iostream>
#include <vector>
#include <map>
#include <array>
#include <cstring>
#include <unistd.h>
#include "motor.hpp"
#include "canbus.hpp"
#include <atomic>
#include <functional>

class MotorControl {
public:
    explicit MotorControl(CANBus& canbus);

    void controlMIT(Motor& motor, double kp, double kd, double q, double dq, double tau);
    void controlMIT2(Motor& motor, double kp, double kd, double q, double dq, double tau);

    void enable(Motor& motor);
    void disable(Motor& motor);
    void set_zero_position(Motor& motor);
    void recv();
    void sendData(uint16_t motor_id, const std::array<uint8_t,8>& data);
    bool addMotor(Motor& motor);
    void recv_set_param_data(); 

private:
    CANBus& canbus_;  

    std::map<uint16_t, Motor*> motors_map; 

    static constexpr double Limit_Param[12][3] = {
        {12.5, 30, 10}, {12.5, 50, 10}, {12.5, 8, 28}, {12.5, 10, 28},
        {12.5, 45, 20}, {12.5, 45, 40}, {12.5, 45, 54}, {12.5, 25, 200},
        {12.5, 20, 200}, {12.5, 280, 1}, {12.5, 45, 10}, {12.5, 45, 10}
    };

    void processPacket(const can_frame& frame);
    void controlCmd(Motor &motor, uint8_t cmd );
    void readRIDParam(Motor& motor, DM_variable RID);
    void writeMotorParam(Motor& motor, DM_variable RID, double value);

};
#endif // MOTOR_CONTROL_H
