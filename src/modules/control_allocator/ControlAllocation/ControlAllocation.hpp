/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ControlAllocation.hpp
 *
 * Interface for Control Allocation Algorithms
 *
 * Implementers of this interface are expected to update the members
 * of this base class in the `allocate` method.
 *
 * Example usage:
 * ```
 * [...]
 * // Initialization
 * ControlAllocationMethodImpl alloc();
 * alloc.setEffectivenessMatrix(effectiveness, actuator_trim);
 * alloc.setActuatorMin(actuator_min);
 * alloc.setActuatorMin(actuator_max);
 *
 * while (1) {
 * 	[...]
 *
 * 	// Set control setpoint, allocate actuator setpoint, retrieve actuator setpoint
 * 	alloc.setControlSetpoint(control_sp);
 * 	alloc.allocate();
 * 	actuator_sp = alloc.getActuatorSetpoint();
 *
 * 	// Check if the control setpoint was fully allocated
 *	unallocated_control = control_sp - alloc.getAllocatedControl()
 *
 *	[...]
 * }
 * ```
 *
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_actuator_setpoint.h>

class ControlAllocation
{
public:
	ControlAllocation() = default;
	virtual ~ControlAllocation() = default;

	static constexpr uint8_t NUM_ACTUATORS = 16;
	static constexpr uint8_t NUM_AXES = 6;

	typedef matrix::Vector<float, NUM_ACTUATORS> ActuatorVector;

	enum ControlAxis {
		ROLL = 0,
		PITCH,
		YAW,
		THRUST_X,
		THRUST_Y,
		THRUST_Z
	};

	union saturation_status_u {
		struct {
			uint16_t valid		: 1; // 0 - true when the saturation status is used
			uint16_t act_pos	: 1; // 1 - true when any actuator has saturated in the positive direction
			uint16_t act_neg	: 1; // 2 - true when any actuator has saturated in the negative direction
			uint16_t roll_pos	: 1; // 3 - true when a positive roll demand change will increase saturation
			uint16_t roll_neg	: 1; // 4 - true when a negative roll demand change will increase saturation
			uint16_t pitch_pos	: 1; // 5 - true when a positive pitch demand change will increase saturation
			uint16_t pitch_neg	: 1; // 6 - true when a negative pitch demand change will increase saturation
			uint16_t yaw_pos	: 1; // 7 - true when a positive yaw demand change will increase saturation
			uint16_t yaw_neg	: 1; // 8 - true when a negative yaw demand change will increase saturation
			uint16_t thrust_x_pos	: 1; // 9 - true when a forward thrust demand change will increase saturation
			uint16_t thrust_x_neg	: 1; //10 - true when a backward thrust demand change will increase saturation
			uint16_t thrust_y_pos	: 1; //11 - true when a right thrust demand change will increase saturation
			uint16_t thrust_y_neg	: 1; //12 - true when a left thrust demand change will increase saturation
			uint16_t thrust_z_pos	: 1; //13 - true when a downward thrust demand change will increase saturation
			uint16_t thrust_z_neg	: 1; //14 - true when a upward thrust demand change will increase saturation
		} flags;
		uint16_t value;
	};

	/**
	 * Allocate control setpoint to actuators
	 *
	 * @param control_setpoint  Desired control setpoint vector (input)
	 */
	virtual void allocate() = 0;

	/**
	 * Set the control effectiveness matrix
	 *
	 * @param B Effectiveness matrix
	 */
	virtual void setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
					    const matrix::Vector<float, NUM_ACTUATORS> &actuator_trim, int num_actuators);

	/**
	 * Get the allocated actuator vector
	 *
	 * @return Actuator vector
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorSetpoint() const { return _actuator_sp; }

	/**
	 * Set the desired control vector
	 *
	 * @param Control vector
	 */
	void setControlSetpoint(const matrix::Vector<float, NUM_AXES> &control) { _control_sp = control; }

	/**
	 * Set the desired control vector
	 *
	 * @param Control vector
	 */
	const matrix::Vector<float, NUM_AXES> &getControlSetpoint() const { return _control_sp; }

	/**
	 * Get the allocated control vector
	 *
	 * @return Control vector
	 */
	const matrix::Vector<float, NUM_AXES> &getAllocatedControl() const { return _control_allocated; }

	/**
	 * Get the control effectiveness matrix
	 *
	 * @return Effectiveness matrix
	 */
	const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &getEffectivenessMatrix() const { return _effectiveness; }

	/**
	 * Set the minimum actuator values
	 *
	 * @param actuator_min Minimum actuator values
	 */
	void setActuatorMin(const matrix::Vector<float, NUM_ACTUATORS> &actuator_min) { _actuator_min = actuator_min; }

	/**
	 * Get the minimum actuator values
	 *
	 * @return Minimum actuator values
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorMin() const { return _actuator_min; }

	/**
	 * Set the maximum actuator values
	 *
	 * @param actuator_max Maximum actuator values
	 */
	void setActuatorMax(const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) { _actuator_max = actuator_max; }

	/**
	 * Get the maximum actuator values
	 *
	 * @return Maximum actuator values
	 */
	const matrix::Vector<float, NUM_ACTUATORS> &getActuatorMax() const { return _actuator_max; }

	/**
	 * Get the saturation status union
	 *
	 * @return saturation_status union
	 */
	const saturation_status_u &getSaturationStatus() const { return _saturation_status; }

	/**
	 * Get the saturation status flags
	 *
	 * @return saturation_status flags (bitmask)
	 */
	const decltype(saturation_status_u::flags) &getSaturationStatusFlags() const { return _saturation_status.flags; }

	/**
	 * Set the current actuator setpoint.
	 *
	 * Use this when a new allocation method is started to initialize it properly.
	 * In most cases, it is not needed to call this method before `allocate()`.
	 * Indeed the previous actuator setpoint is expected to be stored during calls to `allocate()`.
	 *
	 * @param actuator_sp Actuator setpoint
	 */
	void setActuatorSetpoint(const matrix::Vector<float, NUM_ACTUATORS> &actuator_sp);

	/**
	 * Clip the actuator setpoint between minimum and maximum values.
	 *
	 * The output is in the range [min; max]
	 *
	 * @param actuator Actuator vector to clip
	 */
	void clipActuatorSetpoint(matrix::Vector<float, NUM_ACTUATORS> &actuator);

	/**
	 * Set the saturation status flags given the desired
	 * and allocated torques and forces.
	 */
	void updateSaturationStatus();

	/**
	 * Normalize the actuator setpoint between minimum and maximum values.
	 *
	 * The output is in the range [-1; +1]
	 *
	 * @param actuator Actuator vector to normalize
	 *
	 * @return Clipped actuator setpoint
	 */
	matrix::Vector<float, NUM_ACTUATORS> normalizeActuatorSetpoint(const matrix::Vector<float, NUM_ACTUATORS> &actuator)
	const;

	virtual void updateParameters() {}

	int numConfiguredActuators() const { return _num_actuators; }

protected:
	matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> _effectiveness;  //< Effectiveness matrix
	matrix::Vector<float, NUM_ACTUATORS> _actuator_trim; 	//< Neutral actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_min; 	//< Minimum actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_max; 	//< Maximum actuator values
	matrix::Vector<float, NUM_ACTUATORS> _actuator_sp;  	//< Actuator setpoint
	matrix::Vector<float, NUM_AXES> _control_sp;   		//< Control setpoint
	matrix::Vector<float, NUM_AXES> _control_allocated;  	//< Allocated control
	matrix::Vector<float, NUM_AXES> _control_trim;  	//< Control at trim actuator values
	int _num_actuators{0};

	saturation_status_u _saturation_status{};
};
