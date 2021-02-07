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

#ifndef BEHAVIOR
#define BEHAVIOR

#include <mutex>
#include <chrono>
#include <iostream>
#include <cmath>


#include "opendlv-standard-message-set.hpp"
#include "cluon-complete.hpp"

class Behavior {
	private:
  	Behavior(Behavior const &) = delete;
  	Behavior(Behavior &&) = delete;
  	Behavior &operator=(Behavior const &) = delete;
  	Behavior &operator=(Behavior &&) = delete;

	public:
  	Behavior() noexcept;
  	~Behavior() = default;

 	public:
		opendlv::proxy::WheelSpeedRequest getLeftWheelSpeed() noexcept;  //Behöver skrivas som kod BEHÖVER NOG INTE DESSA
		opendlv::proxy::WheelSpeedRequest getRightWheelSpeed() noexcept;	//Behöver skrivas som kod
		opendlv::proxy::GroundSteeringRequest getGroundSteeringAngle() noexcept;
  	opendlv::proxy::PedalPositionRequest getPedalPositionRequest() noexcept;
		int64_t getStartTime() noexcept;
 		void setFrontUltrasonic(opendlv::proxy::DistanceReading const &) noexcept;
  	void setRearUltrasonic(opendlv::proxy::DistanceReading const &) noexcept;
 		void setLeftIr(opendlv::proxy::VoltageReading const &) noexcept;
  	void setRightIr(opendlv::proxy::VoltageReading const &) noexcept;
  	void setLeftWheelSpeed(opendlv::proxy::WheelSpeedRequest const &) noexcept; 
		void setRightWheelSpeed(opendlv::proxy::WheelSpeedRequest const &) noexcept;
  	void step() noexcept;

	private:
  	double convertIrVoltageToDistance(float) const noexcept;

 	private:
		opendlv::proxy::WheelSpeedRequest m_leftWheelRequest;  //Kom ihåg: Ordningen var väldigt viktig för dessa.
		opendlv::proxy::WheelSpeedRequest m_rightWheelRequest;
  	opendlv::proxy::DistanceReading m_frontUltrasonicReading;
  	opendlv::proxy::DistanceReading m_rearUltrasonicReading;
  	opendlv::proxy::VoltageReading m_leftIrReading;
  	opendlv::proxy::VoltageReading m_rightIrReading;
  	opendlv::proxy::GroundSteeringRequest m_groundSteeringAngleRequest;
  	opendlv::proxy::PedalPositionRequest m_pedalPositionRequest;
  	std::mutex m_leftWheelRequestMutex;
  	std::mutex m_rightWheelRequestMutex;
  	std::mutex m_frontUltrasonicReadingMutex;
  	std::mutex m_rearUltrasonicReadingMutex;
  	std::mutex m_leftIrReadingMutex;
  	std::mutex m_rightIrReadingMutex;
  	std::mutex m_groundSteeringAngleRequestMutex;
  	std::mutex m_pedalPositionRequestMutex;   
  	int64_t startTime;
};

#endif
