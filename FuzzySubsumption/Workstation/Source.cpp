#include "Aria.h"
#include <string>
#include <iostream>
#include <algorithm>



using namespace std;

class FuzzyLogicControllerREF
{
protected:
	class TrapezoidFunction
	{
	protected:
		float leftPoint, lMiddlePoint, rMiddlePoint, rightPoint;

	public:
		float topValue;	int index;
		void SetPoints(float left, float leftMiddle, float rightMiddle, float right)
		{
			leftPoint = left; lMiddlePoint = leftMiddle; rMiddlePoint = rightMiddle; rightPoint = right;
			topValue = 1;
		}
		float GetMembershipValue(float x)
		{
			if (x >= rightPoint) x = rightPoint - 1; if (x <= 0) x = topValue;
			if (x <= leftPoint || x >= rightPoint) return 0;
			else if (x < lMiddlePoint) return (x - leftPoint) / (lMiddlePoint - leftPoint);
			else if (x > rMiddlePoint) return (rightPoint - x) / (rightPoint - rMiddlePoint);
			else return topValue;
		}
		TrapezoidFunction GetAlphaCut(float y)
		{
			float newMiddleL = y * (lMiddlePoint - leftPoint) + leftPoint;
			float newMiddleR = rightPoint - y * (rightPoint - rMiddlePoint);
			TrapezoidFunction alphaCut; alphaCut.index = index;
			alphaCut.SetPoints(leftPoint, newMiddleL, newMiddleR, rightPoint); alphaCut.topValue = y;
			return alphaCut;
		}
	};

	class Rule
	{ //IF [FSS] and [BSS] THEN [LMS] and [RMS]
	public:
		TrapezoidFunction *frontFunction, *backFunction; //Rule Inputs
		TrapezoidFunction *leftSpeedFunction, *rightSpeedFunction; //Rule Outputs
		Rule* next;

		void SetRule(TrapezoidFunction *frontInput, TrapezoidFunction *backInput, TrapezoidFunction *leftOutput, TrapezoidFunction *rightOutput)
		{
			frontFunction = frontInput; backFunction = backInput;
			leftSpeedFunction = leftOutput; rightSpeedFunction = rightOutput;
		}
		float GetFiringStrength(float frontReading, float backReading)
		{
			float FS = frontFunction->GetMembershipValue(frontReading) * backFunction->GetMembershipValue(backReading);
			return FS;
		}

	};

	struct Antecedent
	{
		TrapezoidFunction close;
		TrapezoidFunction average;
		TrapezoidFunction fAr;
	};

	struct Consequent
	{
		TrapezoidFunction slow;
		TrapezoidFunction medium;
		TrapezoidFunction fast;
	};

	struct RuleBase
	{
		Rule closeClose, closeAverage, closeFar;
		Rule averageClose, averageAverage, averageFar;
		Rule farClose, farAverage, farFar;
		Rule *firstRule = &closeClose;
	};

	void SetConditionalVariables(Antecedent* frontInput, Antecedent* backInput, Consequent* motorSpeed)
	{
		frontInput->close.SetPoints(0, 0, 652.5, 978.75);				frontInput->close.index = 0;
		frontInput->average.SetPoints(652.5, 978.75, 978.75, 1305);	frontInput->average.index = 1;
		frontInput->fAr.SetPoints(978.75, 1305, 1631.25, 1631.25);			frontInput->fAr.index = 2;

		backInput->close.SetPoints(0, 0, 500, 750);			backInput->close.index = 0;
		backInput->average.SetPoints(500, 750, 750, 1000);	backInput->average.index = 1;
		backInput->fAr.SetPoints(750, 1000, 1250, 1250);		backInput->fAr.index = 2;

		motorSpeed->slow.SetPoints(0, 0, 50, 100);			motorSpeed->slow.index = 0;
		motorSpeed->medium.SetPoints(50, 100, 100, 150);	motorSpeed->medium.index = 1;
		motorSpeed->fast.SetPoints(100, 150, 200, 200);	motorSpeed->fast.index = 2;
	}

	void SetRuleBase(RuleBase* rules, Antecedent* frontInput, Antecedent* backInput, Consequent* motorSpeed)
	{
		rules->closeClose.SetRule(&frontInput->close, &backInput->close, &motorSpeed->medium, &motorSpeed->fast);
		rules->closeAverage.SetRule(&frontInput->close, &backInput->average, &motorSpeed->slow, &motorSpeed->medium);
		rules->closeFar.SetRule(&frontInput->close, &backInput->fAr, &motorSpeed->slow, &motorSpeed->fast);

		rules->averageClose.SetRule(&frontInput->average, &backInput->close, &motorSpeed->fast, &motorSpeed->slow);
		rules->averageAverage.SetRule(&frontInput->average, &backInput->average, &motorSpeed->medium, &motorSpeed->medium);
		rules->averageFar.SetRule(&frontInput->average, &backInput->fAr, &motorSpeed->medium, &motorSpeed->fast);

		rules->farClose.SetRule(&frontInput->fAr, &backInput->close, &motorSpeed->fast, &motorSpeed->slow);
		rules->farAverage.SetRule(&frontInput->fAr, &backInput->average, &motorSpeed->medium, &motorSpeed->slow);
		rules->farFar.SetRule(&frontInput->fAr, &backInput->fAr, &motorSpeed->fast, &motorSpeed->medium);

		rules->closeClose.next = &rules->closeAverage;
		rules->closeAverage.next = &rules->closeFar;
		rules->closeFar.next = &rules->averageClose;
		rules->averageClose.next = &rules->averageAverage;
		rules->averageAverage.next = &rules->averageFar;
		rules->averageFar.next = &rules->farClose;
		rules->farClose.next = &rules->farAverage;
		rules->farAverage.next = &rules->farFar;
		rules->farFar.next = &rules->closeClose;
	}

	float Defuzzify(TrapezoidFunction maxCuts[])
	{
		float numerator = 0; float denominator = 0;
		for (int i = 1; i < 10; i++)
		{
			float x = i * 100;
			float ySlow = maxCuts[0].GetMembershipValue(x);
			float yMedium = maxCuts[1].GetMembershipValue(x);
			float yFast = maxCuts[2].GetMembershipValue(x);
			float max = ySlow, y;
			if (yMedium > max) max = yMedium;
			if (yFast > max) max = yFast;
			y = max;
			numerator += (x * y); denominator += y;
		}
		float output = numerator / denominator;
		return output;
	}
public:
	float GetOutput(float frontSonarReading, float backSonarReading, int index)
	{
		Antecedent FSS, BSS; Consequent MotorSpeed;
		RuleBase ruleBase;
		SetConditionalVariables(&FSS, &BSS, &MotorSpeed);
		SetRuleBase(&ruleBase, &FSS, &BSS, &MotorSpeed);

		//1. Fuzzification and 2. Application of the AND[product] opperator to get FS[9]
		float fs[9];
		Rule* iRule = ruleBase.firstRule;
		for (int i = 0; i < 9; i++)
		{
			fs[i] = iRule->GetFiringStrength(frontSonarReading, backSonarReading);
			iRule = iRule->next;
		}

		//for LeftMotorSpeed
		//3. Implication Method and 4. Aggregates all outputs
		if (index == 0)
		{
			TrapezoidFunction maxCutsLMS[3];
			maxCutsLMS[0].topValue = 0; maxCutsLMS[1].topValue = 0; maxCutsLMS[2].topValue = 0;
			for (int i = 0; i < 9; i++)
			{//Finds max alpha cut of each speed membership function [s,m,f]
				if (fs[i] > 0)
				{
					TrapezoidFunction cut = iRule->leftSpeedFunction->GetAlphaCut(fs[i]);
					if (cut.topValue > maxCutsLMS[cut.index].topValue) maxCutsLMS[cut.index] = cut;
				}
				iRule = iRule->next;
			}

			//5. Defuzzification
			return Defuzzify(maxCutsLMS);
		}

		//for RightMotorSpeed
		//3. Implication Method and 4. Aggregates all outputs
		else if (index == 1)
		{
			TrapezoidFunction maxCutsRMS[3];
			maxCutsRMS[0].topValue = 0; maxCutsRMS[1].topValue = 0; maxCutsRMS[2].topValue = 0;
			for (int i = 0; i < 9; i++)
			{//Finds max alpha cut of each speed membership function [s,m,f]
				if (fs[i] > 0)
				{
					TrapezoidFunction cut = iRule->rightSpeedFunction->GetAlphaCut(fs[i]);
					if (cut.topValue > maxCutsRMS[cut.index].topValue) maxCutsRMS[cut.index] = cut;
				}
				iRule = iRule->next;
			}

			//5. Defuzzification
			return Defuzzify(maxCutsRMS);
		}
		else return 0;
	}
};

class FuzzyLogicControllerOA
{
protected:
	class TrapezoidFunction
	{
	protected:
		float leftPoint, lMiddlePoint, rMiddlePoint, rightPoint;

	public:
		float topValue;	int index;
		void SetPoints(float left, float leftMiddle, float rightMiddle, float right)
		{
			leftPoint = left; lMiddlePoint = leftMiddle; rMiddlePoint = rightMiddle; rightPoint = right;
			topValue = 1;
		}
		float GetMembershipValue(float x)
		{
			if (x >= rightPoint) x = rightPoint - 1; if (x <= 0) x = 1;
			if (x <= leftPoint || x >= rightPoint) return 0;
			else if (x < lMiddlePoint) return (x - leftPoint) / (lMiddlePoint - leftPoint);
			else if (x > rMiddlePoint) return (rightPoint - x) / (rightPoint - rMiddlePoint);
			else return topValue;
		}
		TrapezoidFunction GetAlphaCut(float y)
		{
			float newMiddleL = y * (lMiddlePoint - leftPoint) + leftPoint;
			float newMiddleR = rightPoint - y * (rightPoint - rMiddlePoint);
			TrapezoidFunction alphaCut; alphaCut.index = index;
			alphaCut.SetPoints(leftPoint, newMiddleL, newMiddleR, rightPoint); alphaCut.topValue = y;
			return alphaCut;
		}
	};

	class Rule
	{ //IF [FSS] and [BSS] THEN [LMS] and [RMS]
	public:
		TrapezoidFunction *frontFunction, *backFunction; //Rule Inputs
		TrapezoidFunction *leftSpeedFunction, *rightSpeedFunction; //Rule Outputs
		Rule* next;

		void SetRule(TrapezoidFunction *frontInput, TrapezoidFunction *backInput, TrapezoidFunction *leftOutput, TrapezoidFunction *rightOutput)
		{
			frontFunction = frontInput; backFunction = backInput;
			leftSpeedFunction = leftOutput; rightSpeedFunction = rightOutput;
		}
		float GetFiringStrength(float frontReading, float backReading)
		{
			float FS = frontFunction->GetMembershipValue(frontReading) * backFunction->GetMembershipValue(backReading);
			return FS;
		}

	};

	struct Antecedent
	{
		TrapezoidFunction close;
		TrapezoidFunction average;
		TrapezoidFunction fAr;
	};

	struct Consequent
	{
		TrapezoidFunction slow;
		TrapezoidFunction medium;
		TrapezoidFunction fast;
	};

	struct RuleBase
	{
		Rule closeClose, closeAverage, closeFar;
		Rule averageClose, averageAverage, averageFar;
		Rule farClose, farAverage, farFar;
		Rule *firstRule = &closeClose;
	};

	void SetConditionalVariables(Antecedent* frontInput, Antecedent* backInput, Consequent* motorSpeed)
	{
		frontInput->close.SetPoints(float(0), float(1), float(750), float(1000));				frontInput->close.index = 0;
		frontInput->average.SetPoints(float(750), float(1000), float(1001), float(1250));	frontInput->average.index = 1;
		frontInput->fAr.SetPoints(float(1000), float(1250), float(1499), float(1500));			frontInput->fAr.index = 2;

		backInput->close.SetPoints(float(0), float(1), float(750), float(1000));			backInput->close.index = 0;
		backInput->average.SetPoints(float(750), float(1000), float(1001), float(1250));	backInput->average.index = 1;
		backInput->fAr.SetPoints(float(1000), float(1250), float(1499), float(1500));		backInput->fAr.index = 2;

		motorSpeed->slow.SetPoints(float(0), float(1), float(25), float(50));			motorSpeed->slow.index = 0;
		motorSpeed->medium.SetPoints(float(25), float(50), float(51), float(75));	motorSpeed->medium.index = 1;
		motorSpeed->fast.SetPoints(float(50), float(75), float(99), float(100));	motorSpeed->fast.index = 2;
	}

	void SetRuleBase(RuleBase* rules, Antecedent* frontInput, Antecedent* backInput, Consequent* motorSpeed)
	{
		rules->closeClose.SetRule(&frontInput->close, &backInput->close, &motorSpeed->slow, &motorSpeed->fast);
		rules->closeAverage.SetRule(&frontInput->close, &backInput->average, &motorSpeed->slow, &motorSpeed->fast);
		rules->closeFar.SetRule(&frontInput->close, &backInput->fAr, &motorSpeed->slow, &motorSpeed->fast);

		rules->averageClose.SetRule(&frontInput->average, &backInput->close, &motorSpeed->medium, &motorSpeed->fast);
		rules->averageAverage.SetRule(&frontInput->average, &backInput->average, &motorSpeed->medium, &motorSpeed->medium);
		rules->averageFar.SetRule(&frontInput->average, &backInput->fAr, &motorSpeed->medium, &motorSpeed->medium);

		rules->farClose.SetRule(&frontInput->fAr, &backInput->close, &motorSpeed->slow, &motorSpeed->fast);
		rules->farAverage.SetRule(&frontInput->fAr, &backInput->average, &motorSpeed->medium, &motorSpeed->medium);
		rules->farFar.SetRule(&frontInput->fAr, &backInput->fAr, &motorSpeed->medium, &motorSpeed->medium);

		rules->closeClose.next = &rules->closeAverage;
		rules->closeAverage.next = &rules->closeFar;
		rules->closeFar.next = &rules->averageClose;
		rules->averageClose.next = &rules->averageAverage;
		rules->averageAverage.next = &rules->averageFar;
		rules->averageFar.next = &rules->farClose;
		rules->farClose.next = &rules->farAverage;
		rules->farAverage.next = &rules->farFar;
		rules->farFar.next = &rules->closeClose;
	}

	float Defuzzify(TrapezoidFunction maxCuts[])
	{
		float numerator = 0; float denominator = 0;
		for (int i = 1; i < 11; i++)
		{
			float x = i * 10;
			float ySlow = maxCuts[0].GetMembershipValue(x);
			float yMedium = maxCuts[1].GetMembershipValue(x);
			float yFast = maxCuts[2].GetMembershipValue(x);
			float y = ySlow;
			if (yMedium > y) y = yMedium;
			if (yFast > y) y = yFast;
			numerator += (x * y); denominator += y;
		}
		float output = numerator / denominator;
		return output;
	}
public:
	float GetOutput(float frontSonarReading, float backSonarReading, int index)
	{
		Antecedent FSS, BSS; Consequent MotorSpeed;
		RuleBase ruleBase;
		SetConditionalVariables(&FSS, &BSS, &MotorSpeed);
		SetRuleBase(&ruleBase, &FSS, &BSS, &MotorSpeed);

		//1. Fuzzification and 2. Application of the AND[product] opperator to get FS[9]
		float fs[9];
		Rule* iRule = ruleBase.firstRule;
		for (int i = 0; i < 9; i++)
		{
			fs[i] = iRule->GetFiringStrength(frontSonarReading, backSonarReading);
			iRule = iRule->next;
		}

		//for LeftMotorSpeed
		//3. Implication Method and 4. Aggregates all outputs
		if (index == 0)
		{
			TrapezoidFunction maxCutsLMS[3];
			maxCutsLMS[0].topValue = 0; maxCutsLMS[1].topValue = 0; maxCutsLMS[2].topValue = 0;
			for (int i = 0; i < 9; i++)
			{//Finds max alpha cut of each speed membership function [s,m,f]
				if (fs[i] > 0)
				{
					TrapezoidFunction cut = iRule->leftSpeedFunction->GetAlphaCut(fs[i]);
					if (cut.topValue > maxCutsLMS[cut.index].topValue) maxCutsLMS[cut.index] = cut;
				}
				iRule = iRule->next;
			}

			//5. Defuzzification
			return Defuzzify(maxCutsLMS);
		}

		//for RightMotorSpeed
		//3. Implication Method and 4. Aggregates all outputs
		else if (index == 1)
		{
			TrapezoidFunction maxCutsRMS[3];
			maxCutsRMS[0].topValue = 0; maxCutsRMS[1].topValue = 0; maxCutsRMS[2].topValue = 0;
			for (int i = 0; i < 9; i++)
			{//Finds max alpha cut of each speed membership function [s,m,f]
				if (fs[i] > 0)
				{
					TrapezoidFunction cut = iRule->rightSpeedFunction->GetAlphaCut(fs[i]);
					if (cut.topValue > maxCutsRMS[cut.index].topValue) maxCutsRMS[cut.index] = cut;
				}
				iRule = iRule->next;
			}

			//5. Defuzzification
			return Defuzzify(maxCutsRMS);
		}
		else return 0;
	}
};


void main(int argc, char** argv)
{
	Aria::init();
	ArRobot robot;
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot())
		cout << "Robot Connected!" << endl;
	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();
	//Connect to Sonars		
	ArSensorReading *sonarSensor[8];
	for (int i = 0; i < 8; i++)
	{
		sonarSensor[i] = robot.getSonarReading(i);
	}
	cout << "Sonars Connected!" << endl;
	//***********************RUN*******************************

	FuzzyLogicControllerREF refController;
	FuzzyLogicControllerOA oaController;
	while (true)
	{
		float sonarRange[8];
		for (int i = 0; i < 8; i++)
		{
			sonarRange[i] = sonarSensor[i]->getRange();
		}
		//Sonars 3 & 4 scale to sonar 3.5 = 1.305
		float frontDistance = sonarRange[3] / 1.305;
		if (frontDistance > sonarRange[4] / 1.305) frontDistance = sonarRange[4] / 1.305;
		float LMS; float RMS;

		if (frontDistance <= 750)
		{
			cout << "OA Cycle" << endl;
			LMS = oaController.GetOutput(sonarRange[3], sonarRange[4], 0);
			RMS = oaController.GetOutput(sonarRange[3], sonarRange[4], 1);
		}
		else
		{
			LMS = refController.GetOutput(sonarRange[6], sonarRange[7], 0);
			RMS = refController.GetOutput(sonarRange[6], sonarRange[7], 1);
		}		
		
		robot.setVel2(LMS, RMS);		
	}
	
	//*********************RUN END*********************************
	robot.lock();
	robot.stop();
	robot.unlock();
	Aria::exit();
}
