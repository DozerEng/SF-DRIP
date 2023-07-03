/*
Copyright (c) 2019 STARFISH PRODUCT ENGINEERING INC.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


#include "brushlessMotor.h"
#include <stdbool.h>
#include <math.h>
#include "fastcodeUtil.h"

/*************************************************
 * Notes
 *
 * This implements a brushless motor drive strategy given 3 hall sensor inputs
 * It is assumed that there are three sensors: U, V, W
 * These correspond to the sign of the 3 winding phases: A, B, C
 * The three windings are actually the difference of groups of two motor terminals
 * Phase A = red - black
 * Phase B = white - red
 * Phase C = black - white
 * It is assumed that the phase is measured in degrees,
 * where 0 degrees is located at the transition between sensor state UVW = 010 to UVW = 110
 * Arbitrarily we'll call the direction of positively increasing phase clockwise (CW)
 * All phase angles are tracked as values >= 0 and < 360
 * if intermediate calculations fall outside of this range they will be corrected
 *************************************************/



//*************************************************
//Defines
//*************************************************

#define PHASE_DEBOUNCE_COUNT 3

//#define PHASE_ADVANCE 0
//#define UPDATE_TIME_US 50

//#define MAX_SLOW_SLEW 0.005
#define MAX_FAST_SLEW 0.25

//#define MAX_SLOW_SLEW 1
//#define MAX_FAST_SLEW 1

//#define SLEW_CONST 1.3e-6
//#define SLEW_CONST 0.7e-6

//#define WINDING_OFFSET 150
#define DEFAULT_WINDING_OFFSET (150.0f)
#define DEFAULT_WINDING_PHASE_ADVANCE (30.0f)

#define WINDING_A_PHASE_OFFSET (0.0f)
#define WINDING_B_PHASE_OFFSET (120.0f)
#define WINDING_C_PHASE_OFFSET (240.0f)

#define MIN_PHASE_TIME_FOR_DEAD_RECKONING (0.05f)


//*************************************************
//Variables
//*************************************************
static PortPin m_sensorA = 0;
static PortPin m_sensorB = 0;
static PortPin m_sensorC = 0;

static float m_phaseTime = 0;
static float m_lastPhaseTime = 0;
static float m_angle = 0;
static float m_phaseDrive[3] = {0, 0, 0};
static float m_drive = 0.0;
static float m_rotorPhase = 0;
static float m_direction = 0;
static float m_coursePosition = 0;
static float m_windingPhaseOffset = 150;
static float m_windingPhaseAdvance = 3;
/** the position of the motor in degrees of phase */
static float m_interpPosition = 0;
static SinkSource m_outputPhaseSink = NULL_SINK_SOURCE;
static SinkSource m_sensorPhaseSink = NULL_SINK_SOURCE;
static SinkSource m_positionSink = NULL_SINK_SOURCE;
static SinkSource m_velocitySink = NULL_SINK_SOURCE;

//static float m_lastPhaseA = 0;
//static float m_lastPhaseB = 0;
//static float m_lastPhaseC = 0;

static uint32_t m_phaseIndex = 0;
static uint32_t m_phaseDebounce = 0;
static float m_phaseCyclesPerRev = 0;


static float m_defaultPhaseLookup[6] = {
		30,
		90,
		150,
		210,
		270,
		330


};
static float m_phaseIndexFromHall[8] = {
		NAN,//Sensors = ABC = 000 Invalid
		3,  //Sensors = ABC = 001
		5,  //Sensors = ABC = 010
		4,  //Sensors = ABC = 011
		1,  //Sensors = ABC = 100
		2,  //Sensors = ABC = 101
		0,  //Sensors = ABC = 110
		NAN,//Sensors = ABC = 111 Invalid
};

const static float m_sinLookup[360] = {
//this is a pure sine lookup. It does not maximize the winding voltage
//		0.500, 0.509, 0.517, 0.526, 0.535, 0.544, 0.552, 0.561, 0.570, 0.578,
//		0.587, 0.595, 0.604, 0.612, 0.621, 0.629, 0.638, 0.646, 0.655, 0.663,
//		0.671, 0.679, 0.687, 0.695, 0.703, 0.711, 0.719, 0.727, 0.735, 0.742,
//		0.750, 0.758, 0.765, 0.772, 0.780, 0.787, 0.794, 0.801, 0.808, 0.815,
//		0.821, 0.828, 0.835, 0.841, 0.847, 0.854, 0.860, 0.866, 0.872, 0.877,
//		0.883, 0.889, 0.894, 0.899, 0.905, 0.910, 0.915, 0.919, 0.924, 0.929,
//		0.933, 0.937, 0.941, 0.946, 0.949, 0.953, 0.957, 0.960, 0.964, 0.967,
//		0.970, 0.973, 0.976, 0.978, 0.981, 0.983, 0.985, 0.987, 0.989, 0.991,
//		0.992, 0.994, 0.995, 0.996, 0.997, 0.998, 0.999, 0.999, 1.000, 1.000,
//		1.000, 1.000, 1.000, 0.999, 0.999, 0.998, 0.997, 0.996, 0.995, 0.994,
//		0.992, 0.991, 0.989, 0.987, 0.985, 0.983, 0.981, 0.978, 0.976, 0.973,
//		0.970, 0.967, 0.964, 0.960, 0.957, 0.953, 0.949, 0.946, 0.941, 0.937,
//		0.933, 0.929, 0.924, 0.919, 0.915, 0.910, 0.905, 0.899, 0.894, 0.889,
//		0.883, 0.877, 0.872, 0.866, 0.860, 0.854, 0.847, 0.841, 0.835, 0.828,
//		0.821, 0.815, 0.808, 0.801, 0.794, 0.787, 0.780, 0.772, 0.765, 0.758,
//		0.750, 0.742, 0.735, 0.727, 0.719, 0.711, 0.703, 0.695, 0.687, 0.679,
//		0.671, 0.663, 0.655, 0.646, 0.638, 0.629, 0.621, 0.612, 0.604, 0.595,
//		0.587, 0.578, 0.570, 0.561, 0.552, 0.544, 0.535, 0.526, 0.517, 0.509,
//		0.500, 0.491, 0.483, 0.474, 0.465, 0.456, 0.448, 0.439, 0.430, 0.422,
//		0.413, 0.405, 0.396, 0.388, 0.379, 0.371, 0.362, 0.354, 0.345, 0.337,
//		0.329, 0.321, 0.313, 0.305, 0.297, 0.289, 0.281, 0.273, 0.265, 0.258,
//		0.250, 0.242, 0.235, 0.228, 0.220, 0.213, 0.206, 0.199, 0.192, 0.185,
//		0.179, 0.172, 0.165, 0.159, 0.153, 0.146, 0.140, 0.134, 0.128, 0.123,
//		0.117, 0.111, 0.106, 0.101, 0.095, 0.090, 0.085, 0.081, 0.076, 0.071,
//		0.067, 0.063, 0.059, 0.054, 0.051, 0.047, 0.043, 0.040, 0.036, 0.033,
//		0.030, 0.027, 0.024, 0.022, 0.019, 0.017, 0.015, 0.013, 0.011, 0.009,
//		0.008, 0.006, 0.005, 0.004, 0.003, 0.002, 0.001, 0.001, 0.000, 0.000,
//		0.000, 0.000, 0.000, 0.001, 0.001, 0.002, 0.003, 0.004, 0.005, 0.006,
//		0.008, 0.009, 0.011, 0.013, 0.015, 0.017, 0.019, 0.022, 0.024, 0.027,
//		0.030, 0.033, 0.036, 0.040, 0.043, 0.047, 0.051, 0.054, 0.059, 0.063,
//		0.067, 0.071, 0.076, 0.081, 0.085, 0.090, 0.095, 0.101, 0.106, 0.111,
//		0.117, 0.123, 0.128, 0.134, 0.140, 0.146, 0.153, 0.159, 0.165, 0.172,
//		0.179, 0.185, 0.192, 0.199, 0.206, 0.213, 0.220, 0.228, 0.235, 0.242,
//		0.250, 0.258, 0.265, 0.273, 0.281, 0.289, 0.297, 0.305, 0.313, 0.321,
//		0.329, 0.337, 0.345, 0.354, 0.362, 0.371, 0.379, 0.388, 0.396, 0.405,
//		0.413, 0.422, 0.430, 0.439, 0.448, 0.456, 0.465, 0.474, 0.483, 0.491,

//the following a lookup that maximizes the winding voltage while still being pretty smooth
		0.500, 0.517, 0.534, 0.551, 0.568, 0.585, 0.602, 0.619, 0.635, 0.651,
		0.668, 0.683, 0.699, 0.715, 0.730, 0.744, 0.759, 0.773, 0.787, 0.800,
		0.813, 0.826, 0.838, 0.850, 0.862, 0.873, 0.883, 0.893, 0.903, 0.912,
		0.921, 0.929, 0.937, 0.944, 0.951, 0.957, 0.963, 0.969, 0.974, 0.978,
		0.982, 0.986, 0.989, 0.992, 0.994, 0.996, 0.998, 0.999, 0.999, 1.000,
		1.000, 1.000, 0.999, 0.998, 0.997, 0.996, 0.994, 0.993, 0.991, 0.988,
		0.986, 0.983, 0.981, 0.978, 0.975, 0.972, 0.969, 0.966, 0.963, 0.960,
		0.957, 0.954, 0.951, 0.948, 0.946, 0.943, 0.940, 0.938, 0.935, 0.933,
		0.931, 0.929, 0.928, 0.926, 0.925, 0.923, 0.923, 0.922, 0.921, 0.921,
		0.921, 0.921, 0.921, 0.922, 0.923, 0.923, 0.925, 0.926, 0.928, 0.929,
		0.931, 0.933, 0.935, 0.938, 0.940, 0.943, 0.946, 0.948, 0.951, 0.954,
		0.957, 0.960, 0.963, 0.966, 0.969, 0.972, 0.975, 0.978, 0.981, 0.983,
		0.986, 0.988, 0.991, 0.993, 0.994, 0.996, 0.997, 0.998, 0.999, 1.000,
		1.000, 1.000, 0.999, 0.999, 0.998, 0.996, 0.994, 0.992, 0.989, 0.986,
		0.982, 0.978, 0.974, 0.969, 0.963, 0.957, 0.951, 0.944, 0.937, 0.929,
		0.921, 0.912, 0.903, 0.893, 0.883, 0.873, 0.862, 0.850, 0.838, 0.826,
		0.813, 0.800, 0.787, 0.773, 0.759, 0.744, 0.730, 0.715, 0.699, 0.683,
		0.668, 0.651, 0.635, 0.619, 0.602, 0.585, 0.568, 0.551, 0.534, 0.517,
		0.500, 0.483, 0.466, 0.449, 0.432, 0.415, 0.398, 0.381, 0.365, 0.349,
		0.332, 0.317, 0.301, 0.285, 0.270, 0.256, 0.241, 0.227, 0.213, 0.200,
		0.187, 0.174, 0.162, 0.150, 0.138, 0.127, 0.117, 0.107, 0.097, 0.088,
		0.079, 0.071, 0.063, 0.056, 0.049, 0.043, 0.037, 0.031, 0.026, 0.022,
		0.018, 0.014, 0.011, 0.008, 0.006, 0.004, 0.002, 0.001, 0.001, 0.000,
		0.000, 0.000, 0.001, 0.002, 0.003, 0.004, 0.006, 0.007, 0.009, 0.012,
		0.014, 0.017, 0.019, 0.022, 0.025, 0.028, 0.031, 0.034, 0.037, 0.040,
		0.043, 0.046, 0.049, 0.052, 0.054, 0.057, 0.060, 0.062, 0.065, 0.067,
		0.069, 0.071, 0.072, 0.074, 0.075, 0.077, 0.077, 0.078, 0.079, 0.079,
		0.079, 0.079, 0.079, 0.078, 0.077, 0.077, 0.075, 0.074, 0.072, 0.071,
		0.069, 0.067, 0.065, 0.062, 0.060, 0.057, 0.054, 0.052, 0.049, 0.046,
		0.043, 0.040, 0.037, 0.034, 0.031, 0.028, 0.025, 0.022, 0.019, 0.017,
		0.014, 0.012, 0.009, 0.007, 0.006, 0.004, 0.003, 0.002, 0.001, 0.000,
		0.000, 0.000, 0.001, 0.001, 0.002, 0.004, 0.006, 0.008, 0.011, 0.014,
		0.018, 0.022, 0.026, 0.031, 0.037, 0.043, 0.049, 0.056, 0.063, 0.071,
		0.079, 0.088, 0.097, 0.107, 0.117, 0.127, 0.138, 0.150, 0.162, 0.174,
		0.187, 0.200, 0.213, 0.227, 0.241, 0.256, 0.270, 0.285, 0.301, 0.317,
		0.332, 0.349, 0.365, 0.381, 0.398, 0.415, 0.432, 0.449, 0.466, 0.483,

};


//*************************************************
//function prototypes
//*************************************************

static float slewRate(float currentVal, float targetVal, float slewRate);
//static float calcSlew(void);
static bool isEnabled(void);
/**
 * checks the state of the hall sensors and debounces them.
 * @return an index for the presently debounced hall sensor state
 */
static uint32_t debounceHallSensors(void);
/**
 * looks at the phase index. Determines if we've just crossed a sensor boundary.
 * @return If just crossed a phase boundary then returns the angle in degrees, else NAN
 */
static float checkForPhaseTransition(uint32_t phaseIndex);
/**
 * returns the angular difference between two phases
 * Correctly copes with the 360 to 0 discontinuity
 * Always assumes the phase difference magnitude is less than 180
 * The sign of the result indicates CW (positive) or CCW (negative).
 */
static float phaseCompare(float p1, float p2);
/**
 * make sure the phase is >= 0 and < 360
 */
static float correctPhase(float p);
/**
 * Computes the direction of the latest phase change
 * @return the direction of the new phase angle compared to the last one
 */
static float calcDirection(float angle);
/**
 * assuming this function is called at a phase transition, this computes the most recent phase time
 * this may include some filtering
 * @param timeUs the present system time in microseconds
 */
static float calcPhaseTime(uint32_t timeUs, bool phaseTransition);
/**
 * computes the step that phase should be incremented for the next period of dead wreckoning
 */
static float calcPhaseStep(float direction, float time);

/**
 * computes the new drive value based on phase the offset of the winding and the desired motor drive amplitude
 */
static float calcPwmDrive(float rotorPhase, float offset, float amplitude);
/**
 * if we haven't seen a phase transition for a long time then estimate the phase based on the absolute sensor state
 * Otherwise assume the phase is correct
 * @timeSinceLast the time since the last phase transition
 * @phase the phase we think we want
 * @phaseIndex the current hall sensor state
 */
static float backstopPhase(float timeDelta, float  phase, uint32_t phaseIndex);
/**
 * a handy function to put a break but only if we're in the right fb channel
 */
static void testForBreak();

//*************************************************
//Code
//*************************************************




/**
 * inits this module. Must be run at start
 */
void initBrushlessMotor(void){

}

/**
 * setups up a motor. This will use high power output channels 0, 1, 2
 * @param phaseCyclesPerRev indicates how many times the windings see a full phase cycle, per revolution of the motor shaft. If zero then the motor is considered disabled.
 */
void setupBrushlessMotor(PortPin sensorA, PortPin sensorB, PortPin sensorC, float phaseCyclesPerRev){
	m_sensorA = sensorA;
	m_sensorB = sensorB;
	m_sensorC = sensorC;
	m_phaseCyclesPerRev = phaseCyclesPerRev;

	if(m_phaseCyclesPerRev == 0){
		m_drive = 0;
	}

}
/**
 * @param winding phase the phase we're interested in. Valid values: 0, 1, 2.
 * @return the drive value for the specified phase
 */
float getBrushlessPhaseValue(uint8_t index){
	float result = 0;
	if(index < 3){
		result = m_phaseDrive[index];
	}

	return result;
}

/**
 * This is the brushless motor fast code
 * computes the present phase of the motor based on the sensor inputs
 */
void brushlessFastCode(void){
	static float phaseStep = 0;
	static float deltaPhase = 0;

	if(isEnabled()){

		uint32_t t = getTimeInMicroSeconds();
		uint32_t phaseIndexHall = debounceHallSensors();
		m_phaseIndex = phaseIndexHall;
		setSinkSource(m_sensorPhaseSink, 60.0f*(float)m_phaseIndex);
		float angle = checkForPhaseTransition(phaseIndexHall);

		bool phaseTransition = finitef(angle);
		m_phaseTime = calcPhaseTime(t, phaseTransition);

		if(phaseTransition){


//			togglePin(GPIO_A11_PIN);
			//there has been a phase transition
			float d = calcDirection(angle);
			//should measure the phase transition time

			m_direction = d;
			m_coursePosition += d;

			phaseStep = calcPhaseStep(d, m_phaseTime);
			m_rotorPhase = angle;
			m_interpPosition = m_coursePosition;
			deltaPhase = 0;
		} else {
			//there has not been a phase transition so dead wreckoning
			deltaPhase += phaseStep;
			testForBreak();
			//don't let it estimate beyond one hall state
			if(deltaPhase > 60){
				deltaPhase = 60;
			} else if(deltaPhase < -60){
				deltaPhase = -60;
			}



		}


		m_interpPosition = m_coursePosition + deltaPhase;
		setSinkSource(m_positionSink, getBrushlessPosition());
		setSinkSource(m_velocitySink, getBrushlessRevPerSec());
		float pf = backstopPhase(m_phaseTime, m_rotorPhase + deltaPhase, phaseIndexHall);
		pf = correctPhase(pf);
		m_angle = pf;
		pf += m_windingPhaseOffset;
		setSinkSource(m_outputPhaseSink, pf);
//		setPin(GPIO_A11_PIN, m_angle > 180);
		float pA = calcPwmDrive(pf, WINDING_A_PHASE_OFFSET, m_drive);
		float pB = calcPwmDrive(pf, WINDING_B_PHASE_OFFSET, m_drive);
		float pC = calcPwmDrive(pf, WINDING_C_PHASE_OFFSET, m_drive);

//		p0 = (p0 == 0.5) ? 0 : p0;
//		p1 = (p1 == 0.5) ? 0 : p1;
//		p2 = (p2 == 0.5) ? 0 : p2;

		m_phaseDrive[0] = slewRate(m_phaseDrive[0], pA, MAX_FAST_SLEW);
		m_phaseDrive[1] = slewRate(m_phaseDrive[1], pB, MAX_FAST_SLEW);
		m_phaseDrive[2] = slewRate(m_phaseDrive[2], pC, MAX_FAST_SLEW);

//		m_phaseDrive[0] = slewRate(m_phaseDrive[0], ps[0]*d, s);
//		m_phaseDrive[1] = slewRate(m_phaseDrive[1], ps[1]*d, s);
//		m_phaseDrive[2] = slewRate(m_phaseDrive[2], ps[2]*d, s);

//		setPin(HB_EN_1_PIN, ps[0] != 0.5);
//		setPin(HB_EN_2_PIN, ps[1] != 0.5);
//		setPin(HB_EN_3_PIN, ps[2] != 0.5);



	}
}

/**
 * sets the present drive. Negative values reverse direction.
 */
void setBrushlessDrive(float drive){
	m_drive = drive;
}
float getBrushlessDrive(void){
	return isEnabled() ? m_drive : NAN;
}
float getBrushlessAngle(void){
	return m_angle;
}

static float slewRate(float currentVal, float targetVal, float slewRate){
	float result = currentVal;
	if(!finitef(targetVal)){
		result = currentVal;
	} else {
		float diff = targetVal - currentVal;

		if(diff > slewRate){
			diff = slewRate;
		} else if(diff < -slewRate){
			diff = -slewRate;
		}
		result = currentVal + diff;
	}
	if(!finitef(result)){
		result = 0;
	}





	return result;
}
float getBrushlessRevPerSec(void){

	float result = 0;
	if(!isEnabled()){
		result = NAN;
	} else if(m_phaseTime != 0){
		result = m_direction/60.0f/(m_phaseTime*((float)m_phaseCyclesPerRev*6.0f));//12 phases per rev
	}
	return result;
}
float getBrushlessPosition(void){
	float result = 0;
		if(!isEnabled()){
			result = NAN;
		} else {
			result = m_interpPosition*(1.0f/60.0f)/(m_phaseCyclesPerRev*6.0f);
		}
	return result;
}

/**
 * indicates that the brushless motor module is being used
 */
static bool isEnabled(void){
	return m_phaseCyclesPerRev != 0;
}
/**
 * checks the state of the hall sensors and debounces them.
 * @return an index for the presently debounced hall sensor state
 */
static uint32_t debounceHallSensors(void){
	static uint32_t lastPhaseIndexSample = 0;
	static uint32_t debounced = 0;
	static uint32_t debounceCount = 0;

	//first sample sensors

	uint8_t temp = 0;

	if(isPinInputSet(m_sensorA)){
		temp |= 0b100;
	}
	if(isPinInputSet(m_sensorB)){
		temp |= 0b10;
	}
	if(isPinInputSet(m_sensorC)){
		temp |= 0b1;
	}
//	if((temp & 0b100) != 0){
//		setPin(GPIO_A11_PIN, true);
//	} else {
//		setPin(GPIO_A11_PIN, false);
//	}
//	setPin(GPIO_A11_PIN, (temp & 0b100) != 0);

	//convert to phase index
	uint32_t p = 0;
	float pf = m_phaseIndexFromHall[temp];
	//if this is non-finite then it's a bad hall state so ignore
	if(finitef(pf)){
		p = (uint32_t)pf;



		if(p != lastPhaseIndexSample){
			debounceCount = 0;
		} else if(debounceCount == PHASE_DEBOUNCE_COUNT){

			//			togglePin(GPIO_A11_PIN);


			debounced = p;
			++debounceCount;

		} else if(m_phaseDebounce > PHASE_DEBOUNCE_COUNT){
			//don't do anything here
			//this just stops the rising edge from being detected twice in a row

		} else {
			//the last phase and the new phase are equal so keep track of how long this has lasted
			++debounceCount;
		}
		lastPhaseIndexSample = p;

	}
	return debounced;

}
/**
 * looks at the phase index. Determines if we've just crossed a sensor boundary.
 * @return the angle in degrees if just crossed a phase boundary, else NAN
 */
static float checkForPhaseTransition(uint32_t phaseIndex){
	float result = NAN;
	static uint32_t lpi = 0;//last phase index

	uint32_t pi = phaseIndex;

	if((pi == 0 && lpi == 5) || (pi == 5 && lpi == 0)){
		result = 0.0f;
	} else if((pi == 1 && lpi == 0) || (pi == 0 && lpi == 1)){
		result = 60.0f;
	} else if((pi == 2 && lpi == 1) || (pi == 1 && lpi == 2)){
			result = 120.0f;
	} else if((pi == 3 && lpi == 2) || (pi == 2 && lpi == 3)){
			result = 180.0f;
	} else if((pi == 4 && lpi == 3) || (pi == 3 && lpi == 4)){
			result = 240.0f;
	} else if((pi == 5 && lpi == 4) || (pi == 4 && lpi == 5)){
			result = 300.0f;
	}
	//now remember the present phase for next time

	if(finitef(result)){
		lpi = pi;
//		setPin(GPIO_A11_PIN, ((uint32_t)result) % 120 == 0);
	}
	return result;


}
/**
 * returns the angular difference between two phases
 * Correctly copes with the 360 to 0 discontinuity
 * Always assumes the phase difference magnitude is less than 180
 * assume that p1 is the newest phase and p2 is the previous phase
 * Then the sign of the result indicates CW (positive) or CCW (negative).
 * so when p1 is greater than p2 it's CW.
 */
static float phaseCompare(float p1, float p2){
	float result = p1 - p2;//this should be positive if p1 greater than p2
	//if the result is
	result = correctPhase(result);
	//now offset down by 180 to achieve the desired sine
	if(result >= 180){
		result -= 360;
	}
	return result;
}

/**
 * make sure the phase is >= 0 and < 360
 * this should correct for lots of revolutions away from the desired angle range
 */
static float correctPhase(float p){
	float result = p;

	result *= (1.0f/360.0f);

	float pi = floorf(result);
	result -= pi;
	result *= 360.0f;
	return result;

//	if(p >= 720){
//		p -= 720;
//	} else if(p >= 360){
//		p -= 360;
//	} else if(p < -360){
//		p += 720;
//	} else if(p < 0){
//		p += 360;
//	}
}
/**
 * Computes the direction of the latest phase change
 * @return the direction of the new phase angle compared to the last one
 */
static float calcDirection(float angle){
	static float oldAngle = 0;
	float result = phaseCompare(angle, oldAngle);
	oldAngle = angle;
	return result;
}
/**
 * assuming this function is called at a phase transition, this computes the most recent phase time
 * this may include some filtering
 * @param timeUs the present system time in microseconds
 */
static float calcPhaseTime(uint32_t timeUs, bool phaseTransition){
	static float pt = 0;
	static float lastTimeInUs = NAN;
	float dt = compareTimeMicroSec(timeUs, lastTimeInUs);//calc time since last transition

	if(phaseTransition){//if this is a transition then record the time
		m_lastPhaseTime = dt;
		lastTimeInUs = timeUs;
	}
	if(!finitef(dt)){
		dt = 0;
	}
	dt = dt > m_lastPhaseTime ? dt : m_lastPhaseTime;
	pt += (dt - m_phaseTime)*0.1;

	if(!finitef(pt)){
		pt = dt;
	}

	return pt;
}
/**
 * computes the step that phase should be incremented for the next period of dead wreckoning
 */
static float calcPhaseStep(float direction, float time){
	float result = direction/time*100.0e-6f;
	return result;
}
/**
 * computes the new drive value based on phase the offset of the winding and the desired motor drive amplitude
 */
static float calcPwmDrive(float rotorPhase, float offset, float amplitude){
	float a = amplitude;
//	float reversePhase = 0;
	bool reverse = false;

	//if motor drive reversal is required, then do it
	if(m_phaseCyclesPerRev < 0){
		a = -a;
	};

	if(a < 0){
		a = -a;
//		reversePhase = 180.0f;
		reverse = true;
	}

//	float advance = reverse ? -m_windingPhaseAdvance : m_windingPhaseAdvance;
	float advance = getBrushlessRevPerSec();
	if(finitef(advance)){
		advance *= 0.1*m_windingPhaseAdvance;
	} else {
		advance = 0;
	}
	advance = getBrushlessRevPerSec()*m_windingPhaseAdvance;
	//pin it at +/- 60
	if(advance > 60){
		advance = 60;
	} else if(advance < -60){
		advance = -60;
	}

	float p = correctPhase(rotorPhase + offset + advance);// + reversePhase);
	uint32_t pi = (uint32_t)p;
	if(p >= 360){
		p = 0;
	}

	float result = m_sinLookup[pi];
	if(reverse){
		result = 1.0f - result;
	}
	result *= a;
	//if we're not driving full amplitude then shift up just a smidge so we don't pass the half bridge through zero.
	//Going through zero causes a glitch due to the half bridge's minimum on time.
	if(a < 0.9){
		result += 0.05;
	}
	return result;
}
/**
 * if we haven't seen a phase transition for a long time then estimate the phase based on the absolute sensor state
 * Otherwise assume the phase is correct
 * @timeDelta the length of the last phase transition
 * @phase the phase we think we want
 * @phaseIndex the current hall sensor state
 */
static float backstopPhase(float timeDelta, float  phase, uint32_t phaseIndex){

	float result = phase;

//	float dt = compareTimeMicroSec(time, lastTime);
	//if the time since the last phase change is really big then default to position based solely on current hall state
	if(!finitef(timeDelta) || timeDelta > MIN_PHASE_TIME_FOR_DEAD_RECKONING){//dt > 0.07f){
		if(phaseIndex >= 0 && phaseIndex < 6){
			result = m_defaultPhaseLookup[phaseIndex];
		} else {
			//result = 0;
			asm("nop");
		}

	}
	return result;
}
/**
 * a handy function to put a break but only if we're in the right fb channel
 */
static void testForBreak(){

	asm("nop");
}
/**
 * sets the phase offset and phase advance coefficient for the motor
 * @param offset the phase adjustment in degrees to offset the phase from the hall sensors
 * @param advance the amount to advance the phase per speed in degrees per Hz.
 */
void setBrushlessPhaseOffset(float offset, float advance){
	if(finitef(offset)){
		m_windingPhaseOffset = offset;
	} else {
		m_windingPhaseOffset = DEFAULT_WINDING_OFFSET;
	}
	if(finitef(advance)){
		m_windingPhaseAdvance = advance;
	} else {
		m_windingPhaseAdvance = DEFAULT_WINDING_PHASE_ADVANCE;
	}
}



/**
 * Sets some sinks for useful brushless motor info
 * @param index 0,1,2,3 = position, velocity, output phase, sensor phase.
 * @param sink the sink
 *
 */
void setBrushlessSink(uint32_t index, SinkSource sink){
	switch(index){
	case 0:
		m_positionSink = sink;
		break;
	case 1:
		m_velocitySink = sink;
		break;
	case 2:
		m_outputPhaseSink = sink;
		break;
	case 3:
		m_sensorPhaseSink = sink;
		break;
	default:
		break;
	}

}

/**
 * gets some sinks for useful brushless motor info
 * @param index 0,1,2,3 = position, velocity, output phase, sensor phase.
 * @return sink the sink
 *
 */
SinkSource getBrushlessSink(uint32_t index){
	SinkSource result = NULL_SINK_SOURCE;
	switch(index){
	case 0:
		result = m_positionSink;
		break;
	case 1:
		result = m_velocitySink;
		break;
	case 2:
		result = m_outputPhaseSink;
		break;
	case 3:
		result = m_sensorPhaseSink;
		break;
	default:
		break;
	}
	return result;
}

