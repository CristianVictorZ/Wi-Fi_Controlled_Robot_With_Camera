#ifndef PID_H_
#define PID_H_

class PID
{
    private:

    double timeStep = 0;
    double inverseTimeStep = 0;
    double errorSum = 0;
    double lastError = 0;
    bool firstRun = true;

    public:

    double pidKp = 0;
    double pidKi  = 0;
    double pidKd = 0;
    double value;

    PID(double _kp, double _ki, double _kd, double _timeStep)
    {
        pidKp = _kp;
        pidKi = _ki;
        pidKd = _kd;
        timeStep = _timeStep;
        inverseTimeStep = 1 / timeStep;
    }

    double Control(double error)
    {
        double errorDerivative = (error - lastError) * inverseTimeStep;

        if (firstRun)
        {
            errorDerivative = 0;
            firstRun = false;
        }

        errorSum = GetIntegral(error, errorSum, timeStep);

        lastError = error;

        value = pidKp * error + pidKi * errorSum + pidKd * errorDerivative;
        return value;
    }

    double Control(double error, double _timeStep)
    {
        if (_timeStep != timeStep)
        {
            timeStep = _timeStep;
            inverseTimeStep = 1 / timeStep;
        }
        return Control(error);
    }

    void Reset()
    {
        errorSum = 0;
        lastError = 0;
        firstRun = true;
    }

    protected:

    virtual double GetIntegral(double currentError, double errorSum, double timeStep)
    {
        return errorSum + currentError * timeStep;
    }
};

#endif