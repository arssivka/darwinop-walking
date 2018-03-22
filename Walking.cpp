/*
 *   Walking.cpp
 *
 *   Author: ROBOTIS
 *
 */
#include <stdio.h>
#include <math.h>
#include "Vector.h"
#include "Matrix.h"
#include "MX28.h"
#include "MotionStatus.h"
#include "Kinematics.h"
#include "Walking.h"

using namespace Robot;


#define PI (3.14159265)

Walking* Walking::m_UniqueInstance = new Walking();

Walking::Walking()
{
	X_OFFSET = -10;
	Y_OFFSET = 5;
	Z_OFFSET = 20;
    R_OFFSET = 0;
	P_OFFSET = 0;
    A_OFFSET = 0;
    HIP_PITCH_OFFSET = 13.0;
	PERIOD_TIME = 600;
	DSP_RATIO = 0.1;
	STEP_FB_RATIO = 0.28;
	Z_MOVE_AMPLITUDE = 40;
    Y_SWAP_AMPLITUDE = 20.0;
    Z_SWAP_AMPLITUDE = 5;
    PELVIS_OFFSET = 3.0;
    ARM_SWING_GAIN = 1.5;
	BALANCE_KNEE_GAIN = 0.3;
	BALANCE_ANKLE_PITCH_GAIN = 0.9;
	BALANCE_HIP_ROLL_GAIN = 0.5;
	BALANCE_ANKLE_ROLL_GAIN = 1.0;

	P_GAIN = JointData::P_GAIN_DEFAULT;
    I_GAIN = JointData::I_GAIN_DEFAULT;
    D_GAIN = JointData::D_GAIN_DEFAULT;

	X_MOVE_AMPLITUDE = 0;
	Y_MOVE_AMPLITUDE = 0;
	A_MOVE_AMPLITUDE = 0;	
	A_MOVE_AIM_ON = false;
	BALANCE_ENABLE = true;

	m_Joint.SetAngle(JointData::ID_R_SHOULDER_PITCH, -48.345);
	m_Joint.SetAngle(JointData::ID_L_SHOULDER_PITCH, 41.313);
	m_Joint.SetAngle(JointData::ID_R_SHOULDER_ROLL, -17.873);
    m_Joint.SetAngle(JointData::ID_L_SHOULDER_ROLL, 17.580);
	m_Joint.SetAngle(JointData::ID_R_ELBOW, 29.300);
	m_Joint.SetAngle(JointData::ID_L_ELBOW, -29.593);

	m_Joint.SetAngle(JointData::ID_HEAD_TILT, Kinematics::EYE_TILT_OFFSET_ANGLE);

	m_Joint.SetSlope(JointData::ID_R_SHOULDER_PITCH, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
	m_Joint.SetSlope(JointData::ID_L_SHOULDER_PITCH, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
    m_Joint.SetSlope(JointData::ID_R_SHOULDER_ROLL, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
    m_Joint.SetSlope(JointData::ID_L_SHOULDER_ROLL, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
    m_Joint.SetSlope(JointData::ID_R_ELBOW, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
    m_Joint.SetSlope(JointData::ID_L_ELBOW, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);
	m_Joint.SetSlope(JointData::ID_HEAD_PAN, JointData::SLOPE_EXTRASOFT, JointData::SLOPE_EXTRASOFT);

    m_Joint.SetPGain(JointData::ID_R_SHOULDER_PITCH, 8);
    m_Joint.SetPGain(JointData::ID_L_SHOULDER_PITCH, 8);
    m_Joint.SetPGain(JointData::ID_R_SHOULDER_ROLL, 8);
    m_Joint.SetPGain(JointData::ID_L_SHOULDER_ROLL, 8);
    m_Joint.SetPGain(JointData::ID_R_ELBOW, 8);
    m_Joint.SetPGain(JointData::ID_L_ELBOW, 8);
}

Walking::~Walking()
{
}

void Walking::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, WALKING_SECTION);
}
void Walking::LoadINISettings(minIni* ini, const std::string &section)
{
    double value = INVALID_VALUE;

    if((value = ini->getd(section, "x_offset", INVALID_VALUE)) != INVALID_VALUE)                X_OFFSET = value;
    if((value = ini->getd(section, "y_offset", INVALID_VALUE)) != INVALID_VALUE)                Y_OFFSET = value;
    if((value = ini->getd(section, "z_offset", INVALID_VALUE)) != INVALID_VALUE)                Z_OFFSET = value;
    if((value = ini->getd(section, "roll_offset", INVALID_VALUE)) != INVALID_VALUE)             R_OFFSET = value;
    if((value = ini->getd(section, "pitch_offset", INVALID_VALUE)) != INVALID_VALUE)            P_OFFSET = value;
    if((value = ini->getd(section, "yaw_offset", INVALID_VALUE)) != INVALID_VALUE)              A_OFFSET = value;
    if((value = ini->getd(section, "hip_pitch_offset", INVALID_VALUE)) != INVALID_VALUE)        HIP_PITCH_OFFSET = value;
    if((value = ini->getd(section, "period_time", INVALID_VALUE)) != INVALID_VALUE)             PERIOD_TIME = value;
    if((value = ini->getd(section, "dsp_ratio", INVALID_VALUE)) != INVALID_VALUE)               DSP_RATIO = value;
    if((value = ini->getd(section, "step_forward_back_ratio", INVALID_VALUE)) != INVALID_VALUE) STEP_FB_RATIO = value;
    if((value = ini->getd(section, "foot_height", INVALID_VALUE)) != INVALID_VALUE)             Z_MOVE_AMPLITUDE = value;
    if((value = ini->getd(section, "swing_right_left", INVALID_VALUE)) != INVALID_VALUE)        Y_SWAP_AMPLITUDE = value;
    if((value = ini->getd(section, "swing_top_down", INVALID_VALUE)) != INVALID_VALUE)          Z_SWAP_AMPLITUDE = value;
    if((value = ini->getd(section, "pelvis_offset", INVALID_VALUE)) != INVALID_VALUE)           PELVIS_OFFSET = value;
    if((value = ini->getd(section, "arm_swing_gain", INVALID_VALUE)) != INVALID_VALUE)          ARM_SWING_GAIN = value;
    if((value = ini->getd(section, "balance_knee_gain", INVALID_VALUE)) != INVALID_VALUE)       BALANCE_KNEE_GAIN = value;
    if((value = ini->getd(section, "balance_ankle_pitch_gain", INVALID_VALUE)) != INVALID_VALUE)BALANCE_ANKLE_PITCH_GAIN = value;
    if((value = ini->getd(section, "balance_hip_roll_gain", INVALID_VALUE)) != INVALID_VALUE)   BALANCE_HIP_ROLL_GAIN = value;
    if((value = ini->getd(section, "balance_ankle_roll_gain", INVALID_VALUE)) != INVALID_VALUE) BALANCE_ANKLE_ROLL_GAIN = value;
    if((value = ini->getd(section, "x_odo_factor", INVALID_VALUE)) != INVALID_VALUE) X_ODO_FACTOR = value;
    if((value = ini->getd(section, "y_odo_factor", INVALID_VALUE)) != INVALID_VALUE) Y_ODO_FACTOR = value;
    if((value = ini->getd(section, "a_odo_factor", INVALID_VALUE)) != INVALID_VALUE) A_ODO_FACTOR = value;

    int ivalue = INVALID_VALUE;

    if((ivalue = ini->geti(section, "p_gain", INVALID_VALUE)) != INVALID_VALUE)                 P_GAIN = ivalue;
    if((ivalue = ini->geti(section, "i_gain", INVALID_VALUE)) != INVALID_VALUE)                 I_GAIN = ivalue;
    if((ivalue = ini->geti(section, "d_gain", INVALID_VALUE)) != INVALID_VALUE)                 D_GAIN = ivalue;
}
void Walking::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, WALKING_SECTION);
}
void Walking::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "x_offset",                 X_OFFSET);
    ini->put(section,   "y_offset",                 Y_OFFSET);
    ini->put(section,   "z_offset",                 Z_OFFSET);
    ini->put(section,   "roll_offset",              R_OFFSET);
    ini->put(section,   "pitch_offset",             P_OFFSET);
    ini->put(section,   "yaw_offset",               A_OFFSET);
    ini->put(section,   "hip_pitch_offset",         HIP_PITCH_OFFSET);
    ini->put(section,   "period_time",              PERIOD_TIME);
    ini->put(section,   "dsp_ratio",                DSP_RATIO);
    ini->put(section,   "step_forward_back_ratio",  STEP_FB_RATIO);
    ini->put(section,   "foot_height",              Z_MOVE_AMPLITUDE);
    ini->put(section,   "swing_right_left",         Y_SWAP_AMPLITUDE);
    ini->put(section,   "swing_top_down",           Z_SWAP_AMPLITUDE);
    ini->put(section,   "pelvis_offset",            PELVIS_OFFSET);
    ini->put(section,   "arm_swing_gain",           ARM_SWING_GAIN);
    ini->put(section,   "balance_knee_gain",        BALANCE_KNEE_GAIN);
    ini->put(section,   "balance_ankle_pitch_gain", BALANCE_ANKLE_PITCH_GAIN);
    ini->put(section,   "balance_hip_roll_gain",    BALANCE_HIP_ROLL_GAIN);
    ini->put(section,   "balance_ankle_roll_gain",  BALANCE_ANKLE_ROLL_GAIN);
    ini->put(section,   "x_odo_factor", X_ODO_FACTOR);
    ini->put(section,   "y_odo_factor", Y_ODO_FACTOR);
    ini->put(section,   "a_odo_factor", A_ODO_FACTOR);

    ini->put(section,   "p_gain",                   P_GAIN);
    ini->put(section,   "i_gain",                   I_GAIN);
    ini->put(section,   "d_gain",                   D_GAIN);
}

double Walking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
	return mag * sin(2 * 3.141592 / period * time - period_shift) + mag_shift;
}

bool Walking::computeIK(double *out, double x, double y, double z, double a, double b, double c)
{
    const double LEG_LENGTH = Kinematics::LEG_LENGTH;
	const double THIGH_LENGTH = Kinematics::THIGH_LENGTH;
	const double CALF_LENGTH = Kinematics::CALF_LENGTH;
	const double ANKLE_LENGTH = Kinematics::ANKLE_LENGTH;

    const double cyaw = cos(c);
    const double syaw = sin(c);
    const double cpitch = cos(b);
    const double spitch = sin(b);
    const double croll = cos(a);
    const double sroll = sin(a);

    // Apply roll
    // TODO Applying roll may cause wrong calculations. Check it.
    x += -sroll * spitch * ANKLE_LENGTH;
    y += sroll *          ANKLE_LENGTH;
    z += sroll * cpitch * ANKLE_LENGTH;
    // Apply pitch
    z += cpitch * ANKLE_LENGTH;
    x += -spitch * ANKLE_LENGTH;
    // Apply yaw
    const double oldx = x;
    const double oldy  = y;
    x = cyaw * oldx + syaw * oldy;
    y = cyaw * oldy - syaw * oldx;
    // Ankle offset
    z = LEG_LENGTH - z;
    const double offset_sqr = z * z + x * x;
    const double offset_dist = sqrt(offset_sqr);

    if (offset_dist > CALF_LENGTH + THIGH_LENGTH) {
        return false;
    }

    // Hip yaw
    out[0] = c;
    // Hip roll
    out[1] = atan2(y, z);
    // Hip pitch
    out[2] = -atan2(x, z) - acos((offset_sqr + THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2.0 * THIGH_LENGTH * offset_dist));
    // Knee pitch
    out[3] = PI - acos((THIGH_LENGTH * THIGH_LENGTH + CALF_LENGTH * CALF_LENGTH - offset_sqr) / (2.0 * THIGH_LENGTH * CALF_LENGTH));
    // Ankle pitch
    out[4] = -out[3] - out[2] - b;
    // Ankle roll
    out[5] = -out[1] + a;
    return true;
}

void Walking::update_param_time()
{
	m_PeriodTime = PERIOD_TIME;
    m_DSP_Ratio = DSP_RATIO;
    m_SSP_Ratio = 1 - DSP_RATIO;

    m_X_Swap_PeriodTime = m_PeriodTime / 2;
    m_X_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
    m_Y_Swap_PeriodTime = m_PeriodTime;
    m_Y_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
    m_Z_Swap_PeriodTime = m_PeriodTime / 2;
    m_Z_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio / 2;
    m_A_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;

    m_SSP_Time = m_PeriodTime * m_SSP_Ratio;
    m_SSP_Time_Start_L = (1 - m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_End_L = (1 + m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_Start_R = (3 - m_SSP_Ratio) * m_PeriodTime / 4;
    m_SSP_Time_End_R = (3 + m_SSP_Ratio) * m_PeriodTime / 4;

    m_Phase_Time1 = (m_SSP_Time_End_L + m_SSP_Time_Start_L) / 2;
    m_Phase_Time2 = (m_SSP_Time_Start_R + m_SSP_Time_End_L) / 2;
    m_Phase_Time3 = (m_SSP_Time_End_R + m_SSP_Time_Start_R) / 2;

    m_Pelvis_Offset = PELVIS_OFFSET*MX28::RATIO_ANGLE2VALUE;
    m_Pelvis_Swing = m_Pelvis_Offset * 0.35;
    m_Arm_Swing_Gain = ARM_SWING_GAIN;
}

void Walking::update_param_move()
{
	// Forward/Back
    m_X_Move_Amplitude = X_MOVE_AMPLITUDE;
    m_X_Swap_Amplitude = X_MOVE_AMPLITUDE * STEP_FB_RATIO;

    // Right/Left
    m_Y_Move_Amplitude = Y_MOVE_AMPLITUDE / 2;
    if(m_Y_Move_Amplitude > 0)
        m_Y_Move_Amplitude_Shift = m_Y_Move_Amplitude;
    else
        m_Y_Move_Amplitude_Shift = -m_Y_Move_Amplitude;
    m_Y_Swap_Amplitude = Y_SWAP_AMPLITUDE + m_Y_Move_Amplitude_Shift * 0.04;

    m_Z_Move_Amplitude = Z_MOVE_AMPLITUDE / 2;
    m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude / 2;
    m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE;
    m_Z_Swap_Amplitude_Shift = m_Z_Swap_Amplitude;

    // Direction
    if(A_MOVE_AIM_ON == false)
    {
        m_A_Move_Amplitude = A_MOVE_AMPLITUDE * PI / 180.0 / 2;
        if(m_A_Move_Amplitude > 0)
            m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
        else
            m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
    }
    else
    {
        m_A_Move_Amplitude = -A_MOVE_AMPLITUDE * PI / 180.0 / 2;
        if(m_A_Move_Amplitude > 0)
            m_A_Move_Amplitude_Shift = -m_A_Move_Amplitude;
        else
            m_A_Move_Amplitude_Shift = m_A_Move_Amplitude;
    }
}

void Walking::update_param_balance()
{
	m_X_Offset = X_OFFSET;
    m_Y_Offset = Y_OFFSET;
    m_Z_Offset = Z_OFFSET;
    m_R_Offset = R_OFFSET * PI / 180.0;
    m_P_Offset = P_OFFSET * PI / 180.0;
    m_A_Offset = A_OFFSET * PI / 180.0;
    m_Hip_Pitch_Offset = HIP_PITCH_OFFSET*MX28::RATIO_ANGLE2VALUE;
}

void Walking::Initialize()
{
	X_MOVE_AMPLITUDE   = 0;
    Y_MOVE_AMPLITUDE   = 0;
    A_MOVE_AMPLITUDE   = 0;

    m_X_Odo_Offset = 0.0;
    m_Y_Odo_Offset = 0.0;
    m_A_Odo_Offset = 0.0;

	m_Body_Swing_Y = 0;
    m_Body_Swing_Z = 0;

	m_X_Swap_Phase_Shift = PI;
    m_X_Swap_Amplitude_Shift = 0;
    m_X_Move_Phase_Shift = PI / 2;
    m_X_Move_Amplitude_Shift = 0;
    m_Y_Swap_Phase_Shift = 0;
    m_Y_Swap_Amplitude_Shift = 0;
    m_Y_Move_Phase_Shift = PI / 2;
    m_Z_Swap_Phase_Shift = PI * 3 / 2;
    m_Z_Move_Phase_Shift = PI / 2;
    m_A_Move_Phase_Shift = PI / 2;

    m_X_Left_Odo = 0.0;
    m_Y_Left_Odo = 0.0;
    m_A_Left_Odo = 0.0;

    m_X_Right_Odo= 0.0;
    m_Y_Right_Odo= 0.0;
    m_A_Right_Odo= 0.0;

    m_Right_Start = false;
    m_Right_End = false;
    m_Left_Start = false;
    m_Left_End = false;

    X_ODO_FACTOR = 1.0;
    Y_ODO_FACTOR = 1.0;
    A_ODO_FACTOR = 1.0;

    X_ODO = 0.0;
    Y_ODO = 0.0;
    A_ODO = 0.0;

	m_Ctrl_Running = false;
    m_Real_Running = false;
    m_Time = 0;
    update_param_time();
    update_param_move();

    Process();
}

void Walking::Start()
{
	m_Ctrl_Running = true;
    m_Real_Running = true;
}

void Walking::Stop()
{
	m_Ctrl_Running = false;
}
		
bool Walking::IsRunning()
{
	return m_Real_Running;
}

void Walking::Process()
{
	double x_swap, y_swap, z_swap, a_swap, b_swap, c_swap;
    double x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
    double x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;
    double pelvis_offset_r, pelvis_offset_l;
    double angle[14], ep[12];
	double offset;
	double TIME_UNIT = MotionModule::TIME_UNIT;
	//                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL, L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL, R_ARM_SWING, L_ARM_SWING
	int dir[14]          = {   -1,        -1,          1,         1,         -1,            1,          -1,        -1,         -1,         -1,         1,            1,           1,           -1      };
    double initAngle[14] = {   0.0,       0.0,        0.0,       0.0,        0.0,          0.0,         0.0,       0.0,        0.0,        0.0,       0.0,          0.0,       -48.345,       41.313    };
	int outValue[14];

    // Update walk parameters
    if(m_Time == 0)
    {
        update_param_time();
        m_Phase = PHASE0;
        if(m_Ctrl_Running == false)
        {
            if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
            {
                m_Real_Running = false;
            }
            else
            {
                X_MOVE_AMPLITUDE = 0;
                Y_MOVE_AMPLITUDE = 0;
                A_MOVE_AMPLITUDE = 0;
            }
        }
    }
    else if(m_Time >= (m_Phase_Time1 - TIME_UNIT/2) && m_Time < (m_Phase_Time1 + TIME_UNIT/2))
    {
        update_param_move();
        m_Phase = PHASE1;
    }
    else if(m_Time >= (m_Phase_Time2 - TIME_UNIT/2) && m_Time < (m_Phase_Time2 + TIME_UNIT/2))
    {
        update_param_time();
        m_Time = m_Phase_Time2;
        m_Phase = PHASE2;
        if(m_Ctrl_Running == false)
        {
            if(m_X_Move_Amplitude == 0 && m_Y_Move_Amplitude == 0 && m_A_Move_Amplitude == 0)
            {
                m_Real_Running = false;
            }
            else
            {
                X_MOVE_AMPLITUDE = 0;
                Y_MOVE_AMPLITUDE = 0;
                A_MOVE_AMPLITUDE = 0;
            }
        }
    }
    else if(m_Time >= (m_Phase_Time3 - TIME_UNIT/2) && m_Time < (m_Phase_Time3 + TIME_UNIT/2))
    {
        update_param_move();
        m_Phase = PHASE3;
    }
    update_param_balance();

    // Compute endpoints
    x_swap = wsin(m_Time, m_X_Swap_PeriodTime, m_X_Swap_Phase_Shift, m_X_Swap_Amplitude, m_X_Swap_Amplitude_Shift);
    y_swap = wsin(m_Time, m_Y_Swap_PeriodTime, m_Y_Swap_Phase_Shift, m_Y_Swap_Amplitude, m_Y_Swap_Amplitude_Shift);
    z_swap = wsin(m_Time, m_Z_Swap_PeriodTime, m_Z_Swap_Phase_Shift, m_Z_Swap_Amplitude, m_Z_Swap_Amplitude_Shift);
    a_swap = 0;
    b_swap = 0;
    c_swap = 0;

    if(m_Time <= m_SSP_Time_Start_L)
    {
        x_move_l = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_Start_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
        m_Left_Start = true;
    }
    else if(m_Time <= m_SSP_Time_End_L)
    {
        x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
        pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, -m_Pelvis_Offset / 2, -m_Pelvis_Offset / 2);
        m_Left_End = true;
    }
    else if(m_Time <= m_SSP_Time_Start_R)
    {
        x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
        m_Right_Start = true;
    }
    else if(m_Time <= m_SSP_Time_End_R)
    {
        x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Pelvis_Offset / 2, m_Pelvis_Offset / 2);
        pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
        m_Right_End = true;
    }
    else
    {
        x_move_l = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
        y_move_l = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
        z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_l = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
        x_move_r = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * PI / m_X_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
        y_move_r = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * PI / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
        z_move_r = wsin(m_SSP_Time_End_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * PI / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
        c_move_r = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * PI / m_A_Move_PeriodTime * m_SSP_Time_Start_R + PI, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
        pelvis_offset_l = 0;
        pelvis_offset_r = 0;
    }

    a_move_l = 0;
    b_move_l = 0;
    a_move_r = 0;
    b_move_r = 0;

    ep[0] = x_swap + x_move_r + m_X_Offset;
    ep[1] = y_swap + y_move_r - m_Y_Offset / 2;
    ep[2] = z_swap + z_move_r + m_Z_Offset;
    ep[3] = a_swap + a_move_r - m_R_Offset / 2;
    ep[4] = b_swap + b_move_r + m_P_Offset;
    ep[5] = c_swap + c_move_r - m_A_Offset / 2;
    ep[6] = x_swap + x_move_l + m_X_Offset;
    ep[7] = y_swap + y_move_l + m_Y_Offset / 2;
    ep[8] = z_swap + z_move_l + m_Z_Offset;
    ep[9] = a_swap + a_move_l + m_R_Offset / 2;
    ep[10] = b_swap + b_move_l + m_P_Offset;
    ep[11] = c_swap + c_move_l + m_A_Offset / 2;

    // Calculate odometry
    if(m_Left_End && m_Left_Start)
    {
        if(m_X_Move_Amplitude != 0 || m_Y_Move_Amplitude != 0)
        {
            m_X_Odo_Offset += m_X_Left_Odo;
            m_Y_Odo_Offset += m_Y_Left_Odo;
        }
        m_A_Odo_Offset += m_A_Left_Odo;
        m_Left_End = false;
        m_Left_Start = false;
        m_Right_End = false;
    }
    else if(m_Right_End && m_Right_Start)
    {
        if(m_X_Move_Amplitude != 0 || m_Y_Move_Amplitude != 0)
        {
            m_X_Odo_Offset += m_X_Right_Odo;
            m_Y_Odo_Offset += m_Y_Right_Odo;
        }
        m_A_Odo_Offset += m_A_Right_Odo;
        m_Right_End = false;
        m_Right_Start = false;
        m_Left_End = false;
    }
    else
    {
        if(m_X_Move_Amplitude > 0)
            m_X_Left_Odo = -X_ODO_FACTOR * ep[6];
        else
            m_X_Left_Odo = X_ODO_FACTOR * ep[6];
        m_Y_Left_Odo = -Y_ODO_FACTOR * ep[7];
        m_A_Left_Odo = -A_ODO_FACTOR * ep[11];

        if(m_X_Move_Amplitude > 0)
            m_X_Right_Odo = -X_ODO_FACTOR * ep[0];
        else
            m_X_Right_Odo = X_ODO_FACTOR * ep[0];
        m_Y_Right_Odo = -Y_ODO_FACTOR * ep[1];
        m_A_Right_Odo = -A_ODO_FACTOR * ep[5];
    }

    double odo_offset_dst = hypot(m_X_Odo_Offset, m_Y_Odo_Offset);
    double odo_offset_angle = atan2(m_Y_Odo_Offset, m_X_Odo_Offset);
    X_ODO = X_ODO + cos(A_ODO + odo_offset_angle) * odo_offset_dst;
    Y_ODO = Y_ODO + sin(A_ODO + odo_offset_angle + m_A_Odo_Offset) * odo_offset_dst;
    A_ODO = A_ODO + m_A_Odo_Offset * 180.0 / PI;
    A_ODO = A_ODO - 360.0 * floor((A_ODO + 180.0) / 360.0); // Normalize between -180 and 180

    // Compute body swing
    if(m_Time <= m_SSP_Time_End_L)
    {
        m_Body_Swing_Y = -ep[7];
        m_Body_Swing_Z = ep[8];
    }
    else
    {
        m_Body_Swing_Y = -ep[1];
        m_Body_Swing_Z = ep[2];
    }
	m_Body_Swing_Z -= Kinematics::LEG_LENGTH;

    // Compute arm swing
    if(m_X_Move_Amplitude == 0)
    {
        angle[12] = 0; // Right
        angle[13] = 0; // Left
    }
    else
    {
        angle[12] = wsin(m_Time, m_PeriodTime, PI * 1.5, -m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
        angle[13] = wsin(m_Time, m_PeriodTime, PI * 1.5, m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
    }

    if(m_Real_Running == true)
    {
        m_Time += TIME_UNIT;
        if(m_Time >= m_PeriodTime)
            m_Time = 0;
    }

    // Compute angles
    if((computeIK(&angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == 1)
        && (computeIK(&angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == 1))
    {
        for(int i=0; i<12; i++)
            angle[i] *= 180.0 / PI;
    }
	else
	{
		return; // Do not use angle;
	}


    // Compute motor value
    for (int i = 0; i < 14; i++)
    {
        offset = (double)dir[i] * angle[i] * MX28::RATIO_ANGLE2VALUE;
        if (i == 1) // R_HIP_ROLL
            offset += (double)dir[i] * (pelvis_offset_r * cos(ep[5]) - HIP_PITCH_OFFSET * sin(ep[5]) * MX28::RATIO_ANGLE2VALUE);
        else if (i == 7) // L_HIP_ROLL
            offset += (double)dir[i] * (pelvis_offset_l * cos(ep[11]) - HIP_PITCH_OFFSET * sin(ep[11]) * MX28::RATIO_ANGLE2VALUE);
        else if (i == 2) // R_HIP_PITCH
            offset -= (double)dir[i] * (pelvis_offset_r * sin(ep[5]) + HIP_PITCH_OFFSET * cos(ep[5])) * MX28::RATIO_ANGLE2VALUE;
        else if (i == 8) // L_HIP_PITCH
            offset -= (double)dir[i] * (pelvis_offset_l * sin(ep[11]) + HIP_PITCH_OFFSET * cos(ep[11])) * MX28::RATIO_ANGLE2VALUE;

        outValue[i] = MX28::Angle2Value(initAngle[i]) + (int)offset;
    }


    // adjust balance offset
    if(BALANCE_ENABLE == true)
    {
		double rlGyroErr = MotionStatus::RL_GYRO;
		double fbGyroErr = MotionStatus::FB_GYRO;
#ifdef MX28_1024
        outValue[1] += (int)(dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN); // R_HIP_ROLL
        outValue[7] += (int)(dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN); // L_HIP_ROLL

        outValue[3] -= (int)(dir[3] * fbGyroErr * BALANCE_KNEE_GAIN); // R_KNEE
        outValue[9] -= (int)(dir[9] * fbGyroErr * BALANCE_KNEE_GAIN); // L_KNEE
        
        outValue[4] -= (int)(dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN); // R_ANKLE_PITCH
        outValue[10] -= (int)(dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN); // L_ANKLE_PITCH        
        
        outValue[5] -= (int)(dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN); // R_ANKLE_ROLL
        outValue[11] -= (int)(dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN); // L_ANKLE_ROLL
#else
		outValue[1] += (int)(dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN*4); // R_HIP_ROLL
        outValue[7] += (int)(dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN*4); // L_HIP_ROLL

        outValue[3] -= (int)(dir[3] * fbGyroErr * BALANCE_KNEE_GAIN*4); // R_KNEE
        outValue[9] -= (int)(dir[9] * fbGyroErr * BALANCE_KNEE_GAIN*4); // L_KNEE

		outValue[4] -= (int)(dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN*4); // R_ANKLE_PITCH
        outValue[10] -= (int)(dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN*4); // L_ANKLE_PITCH

		outValue[5] -= (int)(dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN*4); // R_ANKLE_ROLL
        outValue[11] -= (int)(dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN*4); // L_ANKLE_ROLL
#endif
    }

	m_Joint.SetValue(JointData::ID_R_HIP_YAW,           outValue[0]);
	m_Joint.SetValue(JointData::ID_R_HIP_ROLL,          outValue[1]);
	m_Joint.SetValue(JointData::ID_R_HIP_PITCH,         outValue[2]);
	m_Joint.SetValue(JointData::ID_R_KNEE,              outValue[3]);
	m_Joint.SetValue(JointData::ID_R_ANKLE_PITCH,       outValue[4]);
	m_Joint.SetValue(JointData::ID_R_ANKLE_ROLL,        outValue[5]);
	m_Joint.SetValue(JointData::ID_L_HIP_YAW,           outValue[6]);
	m_Joint.SetValue(JointData::ID_L_HIP_ROLL,          outValue[7]);
	m_Joint.SetValue(JointData::ID_L_HIP_PITCH,         outValue[8]);
	m_Joint.SetValue(JointData::ID_L_KNEE,              outValue[9]);
	m_Joint.SetValue(JointData::ID_L_ANKLE_PITCH,       outValue[10]);
	m_Joint.SetValue(JointData::ID_L_ANKLE_ROLL,        outValue[11]);
	m_Joint.SetValue(JointData::ID_R_SHOULDER_PITCH,    outValue[12]);
	m_Joint.SetValue(JointData::ID_L_SHOULDER_PITCH,    outValue[13]);
	m_Joint.SetAngle(JointData::ID_HEAD_PAN, A_MOVE_AMPLITUDE);

	for(int id = JointData::ID_R_HIP_YAW; id <= JointData::ID_L_ANKLE_ROLL; id++)
	{
	    m_Joint.SetPGain(id, P_GAIN);
        m_Joint.SetIGain(id, I_GAIN);
        m_Joint.SetDGain(id, D_GAIN);
	}
}
