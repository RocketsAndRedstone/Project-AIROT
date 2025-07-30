class PID:

    def __init__(self , proportinalGain:float , intergralGain:float , derivativeGain:float , loopTime:float , targetAttitude:float):
        self.prevError:float = 0.0
        self.intergral:float = 0.0
        self.lastOutput:float = 0.0
        self.propGain:float = proportinalGain
        self.intGain:float = intergralGain
        self.derivGain:float = derivativeGain
        self.loopTime:float = loopTime
        self.target:float = targetAttitude

    def updateOutput(self, currentAttitude) -> float:
        error = self.target - currentAttitude
        proportinal = error
        intergral = (self.intergral + error) * self.loopTime
        derivative = (error - self.prevError) / self.loopTime
        
        self.prevError = error

        output = (self.propGain * proportinal) + (self.intGain * intergral) + (self.derivGain * derivative)
        self.lastOutput = output

        return output
    
    def applyDeadzone(self , deadzoneMargin) -> float:
        if ((self.lastOutput - deadzoneMargin) > self.target > (self.lastOutput + deadzoneMargin)):
            self.lastOutput = 0.0
            return 0.0
        else:
            return self.lastOutput
        
    def applyLimits(self , minOutput , maxOutput) -> float:
        if (self.lastOutput > maxOutput):
            self.lastOutput = maxOutput
            return self.lastOutput
        
        elif (self.lastOutput < minOutput):
            self.lastOutput = minOutput
            return self.lastOutput
        
        return self.lastOutput