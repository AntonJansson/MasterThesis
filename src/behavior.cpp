/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h> 
#include "behavior.hpp"

Behavior::Behavior() noexcept:  
	m_leftWheelRequest{},  //Tom så betyder det att man sätter värdet till 0.
	m_rightWheelRequest{},
  m_frontUltrasonicReading{},
  m_rearUltrasonicReading{},
  m_leftIrReading{},
  m_rightIrReading{},
  m_groundSteeringAngleRequest{},
  m_pedalPositionRequest{},
	m_leftWheelRequestMutex{},
	m_rightWheelRequestMutex{},
  m_frontUltrasonicReadingMutex{},
  m_rearUltrasonicReadingMutex{},
  m_leftIrReadingMutex{},
  m_rightIrReadingMutex{},
  m_groundSteeringAngleRequestMutex{},
  m_pedalPositionRequestMutex{},
  startTime{cluon::time::toMicroseconds(cluon::time::now())}
{
}

opendlv::proxy::WheelSpeedRequest Behavior::getLeftWheelSpeed() noexcept  //:: Used to access static variables in of a class. Since it is static we can change it in every request
{
	std::lock_guard<std::mutex> lock(m_leftWheelRequestMutex);
	return m_leftWheelRequest;
}

opendlv::proxy::WheelSpeedRequest Behavior::getRightWheelSpeed() noexcept
{
	std::lock_guard<std::mutex> lock(m_rightWheelRequestMutex);
	return m_rightWheelRequest;
}

opendlv::proxy::GroundSteeringRequest Behavior::getGroundSteeringAngle() noexcept
{
  std::lock_guard<std::mutex> lock(m_groundSteeringAngleRequestMutex);
  return m_groundSteeringAngleRequest;
}

opendlv::proxy::PedalPositionRequest Behavior::getPedalPositionRequest() noexcept
{
  std::lock_guard<std::mutex> lock(m_pedalPositionRequestMutex);
  return m_pedalPositionRequest;
}

int64_t Behavior::getStartTime() noexcept  // Den här behövs nog inte
{
	return startTime;
}

void Behavior::setFrontUltrasonic(opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
  m_frontUltrasonicReading = frontUltrasonicReading;
}

void Behavior::setRearUltrasonic(opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
  m_rearUltrasonicReading = rearUltrasonicReading;
}

void Behavior::setLeftIr(opendlv::proxy::VoltageReading const &leftIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
  m_leftIrReading = leftIrReading;
}

void Behavior::setRightIr(opendlv::proxy::VoltageReading const &rightIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
  m_rightIrReading = rightIrReading;
}

bool randomBool() {  // i nuläget inte med
  return rand() > (RAND_MAX / 2);
}

void Behavior::step() noexcept
{
	opendlv::proxy::WheelSpeedRequest leftWheelRequest;
	opendlv::proxy::WheelSpeedRequest rightWheelRequest;
	opendlv::proxy::DistanceReading frontUltrasonicReading;  
	opendlv::proxy::DistanceReading rearUltrasonicReading;
	opendlv::proxy::VoltageReading leftIrReading;
	opendlv::proxy::VoltageReading rightIrReading;
  //opendlv::proxy::GroundSteeringRequest groundSteeringAngleRequest; // Måste ha dessa annars så sätts hastigheten till 0 hela tiden
	//opendlv::proxy::PedalPositionRequest pedalPositionRequest;
  {
		std::lock_guard<std::mutex> lock1(m_leftWheelRequestMutex);  //Dessa left och right wheel ska tas bort nu när jag har robotten.
		std::lock_guard<std::mutex> lock2(m_rightWheelRequestMutex);
    std::lock_guard<std::mutex> lock3(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock4(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock5(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock6(m_rightIrReadingMutex);
		//std::lock_guard<std::mutex> lock7(m_groundSteeringAngleRequestMutex);  //Måste ha dessa för funktionerna ovan returnerar mutex variables
		//std::lock_guard<std::mutex> lock8(m_pedalPositionRequestMutex);

		leftWheelRequest = m_leftWheelRequest;
		rightWheelRequest = m_rightWheelRequest;
    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
		//groundSteeringAngleRequest = m_groundSteeringAngleRequest; //Ty vill inte hålla värdena bara ändra dem
		//pedalPostionRequest = m_pedalPositionRequest;
  }

  float frontDistance = frontUltrasonicReading.distance();
  float rearDistance = rearUltrasonicReading.distance();
  double tmpleftDistance = convertIrVoltageToDistance(leftIrReading.voltage());
  double tmprightDistance = convertIrVoltageToDistance(rightIrReading.voltage());
	double leftDistance =  tmpleftDistance;
	double rightDistance = tmprightDistance; //Varför hade jag detta extra steg?

	float pedalPosition = 0;  //Ty float in message
	float steering = 0;

	//double vRigth = 0;
  //double vLeft = 0;

	// Beahvior (logic)
	// Case 7, Do not reverse, turn instead
	if (rearDistance < 0.22){ 
			steering = 0;
	    pedalPosition = 0;
	}
	// Case 2 Get out from right corner
	else if(frontDistance < 0.3f && rightDistance < 0.25f && rightDistance > 0.04f) { 
			steering = 0;
			pedalPosition = 0;
	}
	// Case 3 Get out from left corner

  else if (frontDistance < 0.3f  && leftDistance < 0.25f && leftDistance > 0.04f) { 

	}	
/*
	// Case 4 Avoid front obstacle
	else if(frontDistance < 0.4f ) { 
  	bool randomNumber = randomBool();
  	if (frontDistance < 0.2f ){
			vRigth = -0.22;
  		vLeft = -0.22;
		}
		else if(randomNumber) {
  		vRigth = 0.15;
  		vLeft = 0;
  	}
  	else {
  		vRigth = 0;
  		vLeft = 0.1;
  	}
	}
	// Case 5 Avoid left side
	else if (leftDistance < 0.25f && leftDistance > 0.04f) {
   	if (leftDistance < 0.2f){
  		vRigth = -0.05;
 			vLeft = 0.25;
		}
		else {
   		vRigth = 0.0;
  		vLeft = 0.15;
   	}
	}
	// Case 6 Avoid right side
 	else if (rightDistance < 0.25f && rightDistance > 0.04f) {
     	if (rightDistance < 0.2f){
   	   	vRigth = 0.25;
  			vLeft = -0.05;
			}
			else {
      	vRigth = 0.0;
  			vLeft = 0.15;
   		}
	}
	// Case 1 (default), keep moving forward
 	else { 
		int randomDirection = rand() % 100;
		if (randomDirection > 94){
			vRigth = 0.02;
  		vLeft = -0.02;
		}
		else if (randomDirection < 4){
			vRigth = -0.02;
  		vLeft = 0.02;
		}
		else{
  		vRigth = 0.1;
  		vLeft = 0.1;
		}
	}

	float vLeftFloat = (float) vLeft;
	float vRigthFloat = (float) vRigth;
*/

  {
    std::lock_guard<std::mutex> lock1(m_pedalPositionRequestMutex);
    std::lock_guard<std::mutex> lock2(m_groundSteeringAngleRequestMutex);


		opendlv::proxy::PedalPositionRequest pedalPositionRequest;
		pedalPositionRequest.position(pedalPosition);
		m_pedalPositionRequest = pedalPositionRequest;

		opendlv::proxy::GroundSteeringRequest groundSteeringAngleRequest;
		groundSteeringAngleRequest.groundSteering(steering);
   	m_groundSteeringAngleRequest = groundSteeringAngleRequest;

  }
}

double Behavior::convertIrVoltageToDistance(float voltage) const noexcept
{
  double voltageDividerR1 = 1000.0;
  double voltageDividerR2 = 1000.0;

  double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;
  double distance = (2.5 - sensorVoltage) / 0.07;
  return distance;
}
