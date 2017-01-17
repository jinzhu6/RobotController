/*#include <string>
#include <iostream>
#include <algorithm>

using namespace std;

class TrapezoidFunction
{
protected:	
	float leftPoint, lMiddlePoint, rMiddlePoint, rightPoint;	
	
public:	
	float topValue;
	int index;
	void SetPoints(float left, float leftMiddle, float rightMiddle, float right)
	{
		leftPoint = left; lMiddlePoint = leftMiddle; rMiddlePoint = rightMiddle; rightPoint = right;
		topValue = 1;
	}
	void SetTopValue(float y)
	{
		topValue = y;
	}
	float GetMembershipValue(float x)
	{
		if (x >= 1000) x = 999; if (x <= 0) x = topValue;
		if (x <= leftPoint || x >= rightPoint) return 0;
		else if (x < lMiddlePoint) return (x - leftPoint) / (lMiddlePoint - leftPoint);
		else if (x > rMiddlePoint) return (rightPoint - x) / (rightPoint - rMiddlePoint);
		else return topValue;
	}
	TrapezoidFunction GetAlphaCut(float y)
	{
		float newMiddleL = y * (lMiddlePoint - leftPoint) - leftPoint;		
		float newMiddleR = -rightPoint - y * (rightPoint - rMiddlePoint);
		TrapezoidFunction alphaCut; 
		alphaCut.SetPoints(leftPoint, newMiddleL, newMiddleR, rightPoint);
		alphaCut.SetTopValue(y);
		return alphaCut;
	}	
};

class FuzzySet
{
public:
	string name;
	int setCount;
	TrapezoidFunction functions[]; 	

	FuzzySet(string Name)
	{
		name = Name;
		setCount = 0;
	}
	void AddFunction(float left, float middleLeft, float middleRight, float right)
	{
		setCount++;
		functions[setCount] = TrapezoidFunction();		
		functions[setCount].SetPoints(left, middleLeft, middleRight, right);		
	}
};

class Rule
{//[IF frontSensor AND backSensor THEN leftMotor and rightMotor];
public:	
	TrapezoidFunction *frontSensor, *backSensor;
	TrapezoidFunction *leftMotor, *rightMotor;
	Rule* next;
	TrapezoidFunction leftCut, rightCut;

	void SetRule(TrapezoidFunction *frontInput, TrapezoidFunction *backInput, TrapezoidFunction *leftOutput, TrapezoidFunction *rightOutput, Rule *nextRule)
	{
		frontSensor = frontInput; backSensor = backInput;
		leftMotor = leftOutput; rightMotor = rightOutput;
		next = nextRule;
	}
	float GetFiringStrength(float frontReading, float backReading)
	{
		float FS = frontSensor->GetMembershipValue(frontReading) * backSensor->GetMembershipValue(backReading);
		leftCut = leftMotor->GetAlphaCut(FS); rightCut = rightMotor->GetAlphaCut(FS);
		return FS;
	}
};

class RuleBase
{
public:
	Rule closeClose, closeAverage, closeFar;
	Rule averageClose, averageAverage, averageFar;
	Rule farClose, farAverage, farFar;
	Rule *firstRule;

	RuleBase()
	{
		firstRule = &closeClose;
	}	
};

struct FuzzyOutput
{
	float str; TrapezoidFunction cut;	
};

float GetCentroid(FunctionsUnion function)
{	
	float numerator = 0, denominator = 0;
	for (int i = 1; i < 11; i++)
	{
		float x = float(i * 100);
		float y = max(function.cutSlow.GetMembershipValue(x), max(function.cutMedium.GetMembershipValue(x), function.cutFast.GetMembershipValue(x)));
		numerator += (x * y);
		denominator += y;		
		cout << function.cutSlow.GetMembershipValue(x) << endl;
	}
	float output = numerator / denominator;	
	return output;
}


int main(void)
{
	FuzzySet frontSonar("Front Sonar Reading");
	frontSonar.AddFunction(0, 0, 250, 500); 				//[0] close
	frontSonar.AddFunction(250, 500, 500, 750);				//[1] average
	frontSonar.AddFunction(500, 750, 1000, 1000);			//[2] far
	for(int i = 0; i < 3; i++) frontSonar.functions[i].index = i;

	FuzzySet backSonar("Back Sonar Reading");
	backSonar.AddFunction(0, 0, 326.25, 652.5);				//[0] close
	backSonar.AddFunction(326.25, 652.5, 652.5, 978.75);	//[1] average
	backSonar.AddFunction(652.5, 978.75, 1305, 1305);		//[2] far
	for (int i = 0; i < 3; i++) backSonar.functions[i].index = i;

	FuzzySet motorSpeed("Motor Speed");
	motorSpeed.AddFunction(0, 0, 250, 500);			//[0] slow
	motorSpeed.AddFunction(250, 500, 500, 750);		//[1] medium
	motorSpeed.AddFunction(500, 750, 1000, 1000);	//[2] fast
	motorSpeed.functions[0].index = 0; motorSpeed.functions[1].index = 1; motorSpeed.functions[1].index = 1;

	RuleBase rules;
	rules.closeClose.SetRule(&frontSonar.functions[0], &backSonar.functions[0], &motorSpeed.functions[1], &motorSpeed.functions[2], &rules.closeAverage);
	rules.closeAverage.SetRule(&frontSonar.functions[0], &backSonar.functions[1], &motorSpeed.functions[0], &motorSpeed.functions[1], &rules.closeFar);
	rules.closeFar.SetRule(&frontSonar.functions[0], &backSonar.functions[2], &motorSpeed.functions[0], &motorSpeed.functions[2], &rules.averageClose);

	rules.averageClose.SetRule(&frontSonar.functions[1], &backSonar.functions[0], &motorSpeed.functions[2], &motorSpeed.functions[1], &rules.averageAverage);
	rules.averageAverage.SetRule(&frontSonar.functions[1], &backSonar.functions[1], &motorSpeed.functions[1], &motorSpeed.functions[1], &rules.averageFar);
	rules.averageFar.SetRule(&frontSonar.functions[1], &backSonar.functions[2], &motorSpeed.functions[1], &motorSpeed.functions[2], &rules.farClose);

	rules.farClose.SetRule(&frontSonar.functions[2], &backSonar.functions[0], &motorSpeed.functions[2], &motorSpeed.functions[0], &rules.farAverage);
	rules.farAverage.SetRule(&frontSonar.functions[2], &backSonar.functions[1], &motorSpeed.functions[1], &motorSpeed.functions[0], &rules.farFar);
	rules.farFar.SetRule(&frontSonar.functions[2], &backSonar.functions[2], &motorSpeed.functions[2], &motorSpeed.functions[1], &rules.closeClose);

	float fInput, bInput;
	while (true)
	{
		cout << "Insert Front Reading: "; cin >> fInput;
		cout << "Insert Back Reading: "; cin >> bInput;
		Rule *currentRule = rules.firstRule;
		int i = 0; 
		FuzzyOutput lSlow, lMedium, lFast;
		lSlow.str = 0; lMedium.str = 0, lFast.str = 0;
		for (int i = 0; i < 9; i++)
		{
			float FS = currentRule->GetFiringStrength(fInput, bInput);
			cout << currentRule->leftCut.index << endl;
			if (currentRule->leftCut.index == 0)
			{
				if (currentRule->leftCut.topValue > FS)
				{
					lSlow.str = currentRule->leftCut.topValue;
					lSlow.cut = currentRule->leftCut;
					cout << currentRule->leftCut.topValue << endl;
				}
			}
			currentRule = currentRule->next;
		}
		
		
		

				
	}	
} */