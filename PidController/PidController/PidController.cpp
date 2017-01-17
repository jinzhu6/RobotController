#include <Aria.h>;
#include <iostream>;
#include <array>;

using namespace std;

class PidController
{
private:
	float targetRange;
	float kP = 0.7;
	float kI = 0.025;
	float kD = 0.85;
	float sonarScale[8];

	std::array<float, 50> eHistory = {};
	float outMin, outMax;
	float eLast;

	float UpdateErrorHistory(float error)
	{
		float sum = 0;
		for (int i = 1; i < eHistory.max_size(); i++)	eHistory[i] = eHistory[i - 1];
		eHistory[0] = error;
		for (int i = 0; i < eHistory.max_size(); i++) sum += eHistory[i];
		return sum;
	}

public:
	PidController(int speed)
	{
		outMin = -speed; outMax = speed;
	}

	void SetTargetRange(float range)
	{
		targetRange = range;
	}

	void SetSonarScale(int sonar, float scale)
	{
		sonarScale[sonar] = scale;
	}

	int GetOutput(float sonarReading[])
	{
		//Find eP
		float eP = 0;
		for (int i = 4; i < 8; i++)
		{
			if (sonarReading[i] > 2 * targetRange) sonarReading[i] = 2 * targetRange;
			eP += (targetRange - (sonarReading[i] / sonarScale[i]));
		}
		eP = eP / 4;

		//Error Update
		float eI = UpdateErrorHistory(eP);
		float eD = eP - eLast;

		eLast = eP;
		//Get Output
		int Output = eP*kP + eI*kI + eD*kD;
		if (Output > outMax) Output = outMax;
		else if (Output < outMin) Output = outMin;
		return Output;
	}
};

int speed = 500;
float targetRange = 500;

int main(int argc, char** argv)
{
	Aria::init();
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobot robot;
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

	//-----Run Cycle-----

	PidController controller(speed);
	controller.SetTargetRange(targetRange);
	controller.SetSonarScale(4, 5.759);
	controller.SetSonarScale(5, 2.000);
	controller.SetSonarScale(6, 1.305);
	controller.SetSonarScale(7, 1.000);

	float sonarRange[8];
	while (true)
	{
		int output = 0;
		for (int i = 4; i < 8; i++)	sonarRange[i] = sonarSensor[i]->getRange();
		output = controller.GetOutput(sonarRange);
		
		int lVel = speed - output;
		int rVel = speed + output;
		robot.setVel2(lVel, rVel);
	}

	//-----End of Cycle-----
	robot.lock();
	robot.stop();
	robot.unlock();
	Aria::exit();
}