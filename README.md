# Darwin OP Walking

This repo contains patched walking algorithm for Darwin OP humanoid robot based
on [default gait](https://sourceforge.net/projects/darwinop/files/Software/Main%20Controller/Source%20Code/). You can use it instead of default one. Just replace apropriate source files.

*Note: USE THIS SOFTWARE AT YOUR OWN RISK  WITHOUT WARRANTY OF ANY KIND. I simplified our
last code release by removing some internal classes but I can't test this changes. If you
find some serious bugs, please tell me about it.* 

## Changes list

### 1. Improved walking stability

Darwin OP with default walking algorithm implementation constantly falls. This
bug is caused by wrong calculation of influence hip pitch and pelvis offset on
the hip joints. This angles should affect on the joints with taking into
account leg rotation arount z-axis.

```cpp
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
``` 

You can also try to use code above. I'm using this version in our robot but
I don't remember why :)

```cpp
// Compute motor value
for (int i = 0; i < 14; i++) {
    offset = (double)dir[i] * angle[i] * MX28::RATIO_ANGLE2VALUE;
    if (i == 1) // R_HIP_ROLL
        offset += (double) dir[i] * (pelvis_offset_r - HIP_PITCH_OFFSET * sin(ep[5]) * MX28::RATIO_ANGLE2VALUE);
    else if (i == 7) // L_HIP_ROLL
        offset += (double) dir[i] * (pelvis_offset_l - HIP_PITCH_OFFSET * sin(ep[11]) * MX28::RATIO_ANGLE2VALUE);
    else if (i == 2) // R_HIP_PITCH
        offset -= (double) dir[i] * (HIP_PITCH_OFFSET * cos(ep[5])) * MX28::RATIO_ANGLE2VALUE;
    else if (i == 8) // L_HIP_PITCH
        offset -= (double) dir[i] * (HIP_PITCH_OFFSET * cos(ep[11])) * MX28::RATIO_ANGLE2VALUE;

    outValue[i] = MX28::Angle2Value(initAngle[i]) + (int)offset;
}
```

### 2. Odometry

We have written simple odometry accumulator. This algorithm uses position of
endpoint to calculate shift of leg. Odometry recalculates only at DSP phase.
If your error accumulation process too fast you will be able to calibrate scale of
odometry collector by setting X_ODO_FACTOR, Y_ODO_FACTOR or A_ODO_FACTOR.
Results are saved to X_ODO, Y_ODO and A_ODO members.   

```cpp
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
```

### 3. New leg inverse kinematics

When we compared results of inverse kinematics and forward kinematics we have
found a big differences between original and calculated endpoint of leg along y-axis. Rewritten IK
function have slightly improved stability of gait and increaced accuracy of
odometry calculations.  

```cpp
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
```
