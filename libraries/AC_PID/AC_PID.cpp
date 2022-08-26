/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PID.h"

const AP_Param::GroupInfo AC_PID::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    // P增益，产生与当前误差值成比例的输出值的P增益
    AP_GROUPINFO("P", 0, AC_PID, _kp, 0),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    // I增益，产生的输出与误差的幅度和持续时间成正比
    AP_GROUPINFO("I", 1, AC_PID, _ki, 0),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    // D增益，产生的输出与误差的变化速率成正比
    AP_GROUPINFO("D", 2, AC_PID, _kd, 0),

    // 3 was for uint16 IMAX

    // @Param: FF
    // @DisplayName: FF FeedForward Gain
    // @Description: FF Gain which produces an output value that is proportional to the demanded input
    // FF增益，它产生的输出值与要求的输入成比例
    AP_GROUPINFO("FF", 4, AC_PID, _kff, 0),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    // I项可以输出的最大/最小值
    AP_GROUPINFO("IMAX", 5, AC_PID, _kimax, 0),

    // 6 was for float FILT

    // 7 is for float ILMI and FF

    // index 8 was for AFF

    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // 目标滤波频率
    // @Units: Hz
    AP_GROUPINFO("FLTT", 9, AC_PID, _filt_T_hz, AC_PID_TFILT_HZ_DEFAULT),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // 误差滤波频率
    // @Units: Hz
    AP_GROUPINFO("FLTE", 10, AC_PID, _filt_E_hz, AC_PID_EFILT_HZ_DEFAULT),

    // @Param: FLTD
    // @DisplayName: PID Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // 微分滤波频率
    // @Units: Hz
    AP_GROUPINFO("FLTD", 11, AC_PID, _filt_D_hz, AC_PID_DFILT_HZ_DEFAULT),

    // @Param: SMAX
    // @DisplayName: Slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // 设置P和D增益所产生的回转速率的上限。如果速率反馈产生的控制动作的振幅超过这个值，那么D+P增益降低以尊重限制。这限制了由过度增益引起的高频振荡的振幅。该限制应设置为不超过执行机构最大回转率的25%，以考虑负载影响。注:收益不会减少到低于标称价值的10%。值为0将禁用此特性。
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("SMAX", 12, AC_PID, _slew_rate_max, 0),

    AP_GROUPEND
};

// Constructor
AC_PID::AC_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz,
               float dt, float initial_srmax, float initial_srtau):
    _dt(dt)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp = initial_p;
    _ki = initial_i;
    _kd = initial_d;
    _kff = initial_ff;
    _kimax = fabsf(initial_imax);
    filt_T_hz(initial_filt_T_hz);
    filt_E_hz(initial_filt_E_hz);
    filt_D_hz(initial_filt_D_hz);
    _slew_rate_max.set(initial_srmax);
    _slew_rate_tau.set(initial_srtau);

    // reset input filter to first value received
    _flags._reset_filter = true;

    memset(&_pid_info, 0, sizeof(_pid_info));

    // slew limit scaler allows for plane to use degrees/sec slew
    // limit
    _slew_limit_scale = 1;
}

// set_dt - set time step in seconds
void AC_PID::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
}

// filt_T_hz - set target filter hz
void AC_PID::filt_T_hz(float hz)
{
    _filt_T_hz.set(fabsf(hz));
}

// filt_E_hz - set error filter hz
void AC_PID::filt_E_hz(float hz)
{
    _filt_E_hz.set(fabsf(hz));
}

// filt_D_hz - set derivative filter hz
void AC_PID::filt_D_hz(float hz)
{
    _filt_D_hz.set(fabsf(hz));
}

// slew_limit - set slew limit
void AC_PID::slew_limit(float smax)
{
    _slew_rate_max.set(fabsf(smax));
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
//  update_all -设置PID控制器的目标和测量输入，并计算输出目标和过滤误差
//  计算和过滤导数
//  然后根据限制标志的设置更新积分
float AC_PID::update_all(float target, float measurement, bool limit)
{
    // don't process inf or NaN
    // 检测目标值和测量值是否为有限值，如果是正无穷大或负无穷大，则返回0.0f
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // reset input filter to value received
    // 将输入过滤器重置为接收到的值
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _derivative = 0.0f;
    } else {
        // 存储误差，更新目标值和误差
        float error_last = _error;
        _target += get_filt_T_alpha() * (target - _target);
        _error += get_filt_E_alpha() * ((_target - measurement) - _error);

        // calculate and filter derivative
        // 计算和过滤导数-derivative(导数)
        if (_dt > 0.0f) {
            float derivative = (_error - error_last) / _dt;
            _derivative += get_filt_D_alpha() * (derivative - _derivative);
        }
    }

    // update I term
    // 更新Ix项
    update_i(limit);

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);

    // calculate slew limit modifier for P+D
    // 计算P+D的回转极限修正量
    _pid_info.Dmod = _slew_limiter.modifier((_pid_info.P + _pid_info.D) * _slew_limit_scale, _dt);
    _pid_info.slew_rate = _slew_limiter.get_slew_rate();

    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;

    // 日志记录：更新logger中的记录PID内部成员数据的结构体
    _pid_info.target = _target;
    _pid_info.actual = measurement;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;

    return P_out + _integrator + D_out;
}

//  update_error - set error input to PID controller and calculate outputs
//  target is set to zero and error is set and filtered
//  the derivative then is calculated and filtered
//  the integral is then updated based on the setting of the limit flag
//  Target and Measured must be set manually for logging purposes.
// todo: remove function when it is no longer used.
float AC_PID::update_error(float error, bool limit)
{
    // don't process inf or NaN
    // 不处理无穷数或者非数值
    if (!isfinite(error)) {
        return 0.0f;
    }

    _target = 0.0f;

    // reset input filter to value received
    // 将输入过滤器重置为接收到的值
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _error = error;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _error += get_filt_E_alpha() * (error - _error);

        // calculate and filter derivative
        // 计算和过滤积分
        if (_dt > 0.0f) {
            float derivative = (_error - error_last) / _dt;
            _derivative += get_filt_D_alpha() * (derivative - _derivative);
        }
    }

    // update I term
    // 更新I项
    update_i(limit);

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);

    // calculate slew limit modifier for P+D
    // 计算P+D的回转极限修正量
    _pid_info.Dmod = _slew_limiter.modifier((_pid_info.P + _pid_info.D) * _slew_limit_scale, _dt);
    _pid_info.slew_rate = _slew_limiter.get_slew_rate();

    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;
    
    // 日志记录：更新logger中的记录PID内部成员数据的结构体
    _pid_info.target = 0.0f;
    _pid_info.actual = 0.0f;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;

    return P_out + _integrator + D_out;
}

//  update_i - update the integral
//  If the limit flag is set the integral is only allowed to shrink
void AC_PID::update_i(bool limit)
{
    if (!is_zero(_ki) && is_positive(_dt)) {
        // Ensure that integrator can only be reduced if the output is saturated
        // 如果输出饱和，确认积分项只可以减少
        // 三种情况:一.当 limit=0 时,必定进入if;当 limit=1 时,取决于后面两项判断
        // 二.当积分值为正,误差为负(目标值小于测量值);积累误差为正数,同时目标值大于测量值,此时需要积分的介入来减少误差的积累
        // 三.当积分为负,误差为正(目标值大于测量值);同上
        if (!limit || ((is_positive(_integrator) && is_negative(_error)) || (is_negative(_integrator) && is_positive(_error)))) {
            _integrator += ((float)_error * _ki) * _dt;
            _integrator = constrain_float(_integrator, -_kimax, _kimax);
        }
    } else {
        // 积分项赋为0,此时不需要积分介入
        _integrator = 0.0f;
    }
    // 存储参数
    _pid_info.I = _integrator;
    _pid_info.limit = limit;
}

float AC_PID::get_p() const
{
    return _error * _kp;
}

float AC_PID::get_i() const
{
    return _integrator;
}

float AC_PID::get_d() const
{
    return _kd * _derivative;
}

float AC_PID::get_ff()
{
    _pid_info.FF = _target * _kff;
    return _target * _kff;
}

void AC_PID::reset_I()
{
    _integrator = 0.0;
}

void AC_PID::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _kff.load();
    _kimax.load();
    _kimax = fabsf(_kimax);
    _filt_T_hz.load();
    _filt_E_hz.load();
    _filt_D_hz.load();
}

// save_gains - save gains to eeprom
void AC_PID::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _kff.save();
    _kimax.save();
    _filt_T_hz.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
}

/// Overload the function call operator to permit easy initialisation
/// 重载函数调用操作符已方便初始化
void AC_PID::operator()(float p_val, float i_val, float d_val, float ff_val, float imax_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz, float dt)
{
    _kp = p_val;
    _ki = i_val;
    _kd = d_val;
    _kff = ff_val;
    _kimax = fabsf(imax_val);
    _filt_T_hz = input_filt_T_hz;
    _filt_E_hz = input_filt_E_hz;
    _filt_D_hz = input_filt_D_hz;
    _dt = dt;
}

// get_filt_T_alpha - get the target filter alpha
float AC_PID::get_filt_T_alpha() const
{
    return get_filt_alpha(_filt_T_hz);
}

// get_filt_E_alpha - get the error filter alpha
float AC_PID::get_filt_E_alpha() const
{
    return get_filt_alpha(_filt_E_hz);
}

// get_filt_D_alpha - get the derivative filter alpha
float AC_PID::get_filt_D_alpha() const
{
    return get_filt_alpha(_filt_D_hz);
}

// get_filt_alpha - calculate a filter alpha
float AC_PID::get_filt_alpha(float filt_hz) const
{
    return calc_lowpass_alpha_dt(_dt, filt_hz);
}

void AC_PID::set_integrator(float target, float measurement, float integrator)
{
    set_integrator(target - measurement, integrator);
}

void AC_PID::set_integrator(float error, float integrator)
{
    _integrator = constrain_float(integrator - error * _kp, -_kimax, _kimax);
    _pid_info.I = _integrator;
}

void AC_PID::set_integrator(float integrator)
{
    _integrator = constrain_float(integrator, -_kimax, _kimax);
    _pid_info.I = _integrator;
}

void AC_PID::relax_integrator(float integrator, float time_constant)
{
    integrator = constrain_float(integrator, -_kimax, _kimax);
    _integrator = _integrator + (integrator - _integrator) * (_dt / (_dt + time_constant));
    _pid_info.I = _integrator;
}
