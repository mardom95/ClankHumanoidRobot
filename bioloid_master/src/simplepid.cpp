#include "simplepid.h"


SimplePid::SimplePid()
{
}


SimplePid::SimplePid(float proportionalGain, float integralGain, float derivativeGain,
                     float outputMax, float outputMin) :
    pGain(proportionalGain), iGain(integralGain), dGain(derivativeGain),
    outMax(outputMax), outMin(outputMin)
{
    iState = 0.0;
    dState = 0.0;
}


SimplePid::~SimplePid()
{
}


float SimplePid::update(float error, float position)
{
    float pTerm, iTerm, dTerm, out;

    pTerm = pGain*error;

    iState += error;
    iTerm = iGain*iState;

    //ROS_ERROR_STREAM("ISTATE : " << iState);
    //ROS_ERROR_STREAM("ERROR  : " << error);
    //ROS_ERROR_STREAM("ITERM : " << iTerm);
    //ROS_ERROR_STREAM("OUTMX : " << outMax);
    //ROS_ERROR_STREAM("OUTMIN : " << outMin);

    if (iTerm > outMax)
        iTerm = outMax;
    else if (iTerm < outMin)
        iTerm = outMin;

    dTerm = dGain*(position - dState);
    dState = position;

    out = pTerm + iTerm - dTerm;
    if (out > outMax)
        return outMax;
    else if (iTerm < outMin)
        return outMin;

    return out;
}
