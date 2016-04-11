#ifndef MagneticJammingFilter_h
#define MagneticJammingFilter_h
#include <Arduino.h>
#include <EEPROM.h>
#include <util/crc16.h>

#define NXP_MOTION_CAL_EEADDR  60
#define NXP_MOTION_CAL_SIZE    68

//--------------------------------------------------------------------------------------------
// Variable declaration
class MagneticJammingFilter{
private:
	float sampleFreq;

  float fieldIntensity = NAN;
  float fieldIntensityFiltered = 42.0f;
  float fieldIntensityFilteredMax = 62.0f;
  float fieldIntensityFilteredMin = 23.0f;
  
  float jammingThreshold = 5.0f;
  bool jammingActive = false;
  int jammingCounter = 0;
  
  elapsedMillis stateTime;

  float cal[16]; // 0-8=offsets, 9=field strength, 10-15=soft iron map
//-------------------------------------------------------------------------------------------
// Function declarations
public:
  MagneticJammingFilter(void){}
  
  void begin(float sampleFrequency) {    
    unsigned char buf[NXP_MOTION_CAL_SIZE];
    uint8_t i;
    uint16_t crc;
    
    sampleFreq = sampleFrequency; 
  
    for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
      buf[i] = EEPROM.read(NXP_MOTION_CAL_EEADDR + i);
    }
    crc = 0xFFFF;
    for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
      crc = _crc16_update(crc, buf[i]);
    }
    if (crc == 0 && buf[0] == 117 && buf[1] == 84) {
      //If a valid calibration exists, use it to tighten up the magnetic rejection
      memcpy(cal, buf+2, sizeof(cal));
      fieldIntensityFiltered = cal[9];
      fieldIntensityFilteredMax = cal[9] + 10;
      fieldIntensityFilteredMin = cal[9] - 10;
    }  
  }
  
	void update(float mx, float my, float mz) {
		fieldIntensity = sqrtf(mx*mx + my*my + mz*mz);

		if((fieldIntensity < (fieldIntensityFiltered + (jammingThreshold * 3))) && (fieldIntensity > (fieldIntensityFiltered - (jammingThreshold * 3)))) {
			fieldIntensityFiltered += ((fieldIntensity - fieldIntensityFiltered) * 0.001);
		}
		if(fieldIntensityFiltered > fieldIntensityFilteredMax)
			fieldIntensityFiltered = fieldIntensityFilteredMax;
		if(fieldIntensityFiltered < fieldIntensityFilteredMin)
			fieldIntensityFiltered = fieldIntensityFilteredMin;
		
		// Use IMU algorithm if magnetic jamming is occurring
		if(jammingActive) {
			//Maintain jammingActive for 1 second
			if(jammingCounter++ < sampleFreq) {
				return;
			}
		}
		if((fieldIntensity > (fieldIntensityFiltered + jammingThreshold)) || (fieldIntensity < (fieldIntensityFiltered - jammingThreshold))) {
			if(jammingActive == false)
				stateTime = 0;	//Only reset elapsedMillis if jamming state is new
			jammingCounter = 0;
			jammingActive = true;			
			return;
		}
		else {
			if(jammingActive == true)
				stateTime = 0;	//Only reset elapsedMillis if jamming state is new
			jammingCounter = 0;
			jammingActive = false;
		}
	}

  //Returns the latest calculated field intensity
  float getFieldIntensity() {
      return fieldIntensity;
  }
  
  //Returns the value that is being used for the window centre
  float getFieldIntensityFiltered(){
    return fieldIntensityFiltered;
  }
  
  //Returns the jamming status, i.e. true = jamming is active, magnetometer readings are not being used in AHRS calculation.
  bool getJammingStatus(){
    return jammingActive;
  } 
  
  //Returns the duration, in milliseconds, of the current jamming state
  long getTime(){
	return stateTime;
  }
  
  //Sets the allowable window for field intensity. i.e. window size of 10 means an allowable range of fieldIntensityFiltered +/- 5
  void setFieldIntensityWindow(float fieldIntensityWindow){
    if(fieldIntensityWindow < 1.0f)
      return;
    if(fieldIntensityWindow > 1000.0f)
      return;		
    jammingThreshold = fieldIntensityWindow/2;
  }    
};
#endif