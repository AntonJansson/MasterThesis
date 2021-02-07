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
#include <chrono>
#include <iostream>
#include <mutex>
#include <cmath>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"


//Motsvarande klassvariabler
opendlv::proxy::DistanceReading m_frontUltrasonicReading{};  //m_  = notation för kopia
opendlv::proxy::DistanceReading m_rearUltrasonicReading{};
opendlv::proxy::VoltageReading m_leftIrReading{};
opendlv::proxy::VoltageReading m_rightIrReading{};
opendlv::proxy::PedalPositionRequest m_pedalPositionRequest{};
opendlv::proxy::GroundSteeringRequest m_groundSteeringRequest{};
std::mutex m_frontUltrasonicReadingMutex{};
std::mutex m_rearUltrasonicReadingMutex{};
std::mutex m_leftIrReadingMutex{};
std::mutex m_rightIrReadingMutex{};
std::mutex m_pedalPositionRequestMutex{};
std::mutex m_groundSteeringRequestMutex{};



void setFrontUltrasonic(opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept
{	
	std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
 	m_frontUltrasonicReading = frontUltrasonicReading;
}	

void setRearUltrasonic(opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept
{
	std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
	m_rearUltrasonicReading = rearUltrasonicReading;
}

void setLeftIr(opendlv::proxy::VoltageReading const &leftIrReading) noexcept
{
	std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
	m_leftIrReading = leftIrReading;
}

void setRightIr(opendlv::proxy::VoltageReading const &rightIrReading) noexcept
{
	std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
	m_rightIrReading = rightIrReading;
}

opendlv::proxy::PedalPositionRequest getPedalPosition() noexcept
{
	std::lock_guard<std::mutex> lock(m_pedalPositionRequestMutex);
	return  m_pedalPositionRequest;
}
 
opendlv::proxy::GroundSteeringRequest getGroundSteeringAngle() noexcept
{
	std::lock_guard<std::mutex> lock(m_groundSteeringRequestMutex); 
	return m_groundSteeringRequest;
}


double convertIrVoltageToDistance(float voltage) noexcept
{
	double voltageDividerR1 = 1000.0;
	double voltageDividerR2 = 1000.0;

	double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;
	double distance = (2.5 - sensorVoltage) / 0.07;
	return distance;
}


auto onDistanceReading{[](cluon::data::Envelope &&envelope) 
{
	auto distanceReading = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
	uint32_t const senderStamp = envelope.senderStamp();
	if (senderStamp == 0) {
		  setFrontUltrasonic(distanceReading);  
	} 
	else if (senderStamp == 1) {
		  setRearUltrasonic(distanceReading);
	}
}};
		    
auto onVoltageReading{[](cluon::data::Envelope &&envelope) 
{
	auto voltageReading = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(envelope));
	uint32_t const senderStamp = envelope.senderStamp();
	if (senderStamp == 0){
		  setLeftIr(voltageReading);
	} 
	else if (senderStamp == 1) {
		  setRightIr(voltageReading);
	}
}};


int32_t main(int32_t argc, char **argv) {

		int32_t retCode{0};
		auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
		if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("freq")) {
		  std::cerr << argv[0] << " tests the Kiwi platform by sending actuation commands and reacting to sensor input." << std::endl;
		  std::cerr << "Usage:   " << argv[0] << " --freq=<Integration frequency> --cid=<OpenDaVINCI session> [--verbose]" << std::endl;
		  std::cerr << "Example: " << argv[0] << " --freq=10 --cid=111" << std::endl;
		  retCode = 1;
		} 
		else {
		  bool const VERBOSE{commandlineArguments.count("verbose") != 0};
		  uint16_t const CID = std::stoi(commandlineArguments["cid"]);
		  float const FREQ = std::stof(commandlineArguments["freq"]);		    

		  cluon::OD4Session od4{CID};
		  od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
		  od4.dataTrigger(opendlv::proxy::VoltageReading::ID(), onVoltageReading);

			auto atFrequency{[&VERBOSE, &od4]() -> bool{ //FLytta ut den här oxå, går inte ty tar in VERBOSE här och vill inte deklarera VERBOSE utanför  
			opendlv::proxy::DistanceReading frontUltrasonicReadingMessage{};
			opendlv::proxy::DistanceReading rearUltrasonicReadingMessage{};
			opendlv::proxy::VoltageReading leftIrReadingMessage{};
			opendlv::proxy::VoltageReading rightIrReadingMessage{};
			{
				std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);   //Mutex in the end only to look the m_frontUltrasonicReading variable.
				std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
				std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
				std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);	

			  frontUltrasonicReadingMessage = m_frontUltrasonicReading;
				rearUltrasonicReadingMessage = m_rearUltrasonicReading;
				leftIrReadingMessage = m_leftIrReading;
				rightIrReadingMessage = m_rightIrReading;
			}

			float distanceFront = frontUltrasonicReadingMessage.distance();
			float distanceRear = rearUltrasonicReadingMessage.distance(); //Error ty not used
			float leftIrReading = leftIrReadingMessage.voltage();  //Error ty not used
			float rightIrReading = rightIrReadingMessage.voltage();  //Error ty not used

			float pedalPosition = 0.0f;
			float steeringAngle = 0.0f;

			if(distanceFront > 0.25f){
					pedalPosition = 0.0f;	
			}
			if(distanceRear  < 0.0f and leftIrReading < 0.0f and rightIrReading < 0.0f ){  //Just for not get error for "not used variable"

			}		
			else if(distanceFront < 0.25f){
					pedalPosition = 0.2f;
			}
			else{
					pedalPosition = 0.05f;
			}

			{
					std::lock_guard<std::mutex> lock1(m_pedalPositionRequestMutex);
					std::lock_guard<std::mutex> lock2(m_groundSteeringRequestMutex);
							
					opendlv::proxy::PedalPositionRequest pedalPositionRequest;	  //Skulle kunna använda funktioner till detta oxå.
					pedalPositionRequest.position(pedalPosition);
					m_pedalPositionRequest = pedalPositionRequest;  //Lägger över det senaste värdet i m_kopian.

					opendlv::proxy::GroundSteeringRequest groundSteeringRequest;	
					groundSteeringRequest.groundSteering(steeringAngle);
					m_groundSteeringRequest = groundSteeringRequest;
			}
					
			auto groundSteeringMessage = getGroundSteeringAngle();
			auto pedalPositionMessage = getPedalPosition();

			cluon::data::TimeStamp sampleTime = cluon::time::now();
			od4.send(pedalPositionMessage, sampleTime, 0);
			od4.send(groundSteeringMessage, sampleTime, 1);

			if (VERBOSE) {
					std::cout << "The speed is " << getPedalPosition().position() << std::endl;
					std::cout << "The  groundsteering is " << getGroundSteeringAngle().groundSteering() << std::endl;
			}

				return true;
		}};

		od4.timeTrigger(FREQ, atFrequency);  //Hur ofta den ska kalla på funktionen finns i andra argumentet.
	}
	return retCode;
}

