#ifndef AI_ROBOT_NAVIGATION_NAVDATA_H_
#define AI_ROBOT_NAVIGATION_NAVDATA_H_

#include <cstdint>
#include <cstring>

// typedef
using float32_t = float;

// -------- constant definitions begin --------

// Accelerometer constants
enum DefAcc {
  kAccX = 0,
  kAccY,
  kAccZ,
  kNbAccS,
};

// gyrometer constants
enum DefGyro {
  kGyroX = 0,
  kGyroY,
  kGyroZ,
  kNbGyroS,
};

// tag index for each option
enum NavdataTag {
  kNavdataDemoTag = 0,
  kNavdataTimeTag,
  kNavdataRawMeasuresTag,
  kNavdataPhysMeasuresTag,
  kNavdataGyrosOffsetsTag,
  kNavdataEulerAnglesTag,
  kNavdataReferencesTag,
  kNavdataTrimsTag,
  kNavdataRcReferencesTag,
  kNavdataPwmTag,
  kNavdataAltitudeTag,
  kNavdataVisionRawTag,
  kNavdataVisionOfTag,
  kNavdataVisionTag,
  kNavdataVisionPerfTag,
  kNavdataTrackersSendTag,
  kNavdataVisionDetectTag,
  kNavdataWatchdogTag,
  kNavdataAdcDataFrameTag,
  kNavdataVideoStreamTag,
  kNavdataCksTag = 0xFFFF
};

// -------- constant definitions end --------

// navdata header
struct NavdataHeader {
  uint32_t header;  // header:55667788
  uint32_t state;   // the state of the drone
  uint32_t seq;     // sequence number
  uint32_t vision;  // vision flag
};

//	template for each option
struct NavdataOption {
  uint16_t tag;
  uint16_t size;
  uint8_t data[];
};

// navdata option: nav_demo mode
struct NavdataDemo {
  uint16_t tag;   // Navdata block ('option') identifier
  uint16_t size;  // set this to the size of this structure

  uint32_t ctrl_state;  // Flying state (landed, flying, hovering, etc.) defined
                        // in CTRL_STATES enum.
  uint32_t vbat_flying_percentage;  // battery voltage filtered (mV)

  float32_t theta;  // pitch angle in milli-degrees
  float32_t phi;    // roll  angle
  float32_t psi;    // yaw   angle

  int32_t altitude;  // altitude in centimeters[??] / should be milimeter?

  float32_t vx;  // estimated linear velocity
  float32_t vy;  // estimated linear velocity
  float32_t vz;  // estimated linear velocity

  uint32_t num_frames;  // streamed frame index  // Not used  To integrate in
                        // video stage.
};

struct NavTimestamp {
  unsigned int microsecond : 21;
  unsigned int second : 11;
};

struct NavdataTime {
  uint16_t tag;
  uint16_t size;
  uint32_t tm_stamp;
};

struct NavdataRawMeasures {
  uint16_t tag;
  uint16_t size;

  // +12 bytes
  uint16_t raw_accs[kNbAccS];   // filtered accelerometers
  uint16_t raw_gyros[kNbGyroS]; // filtered gyrometers
  uint16_t raw_gyros_110[2];    // gyrometers  x/y 110 deg/s
  uint32_t vbat_raw;            // battery voltage raw (mV)
  uint16_t us_debut_echo;
  uint16_t us_fin_echo;
  uint16_t us_association_echo;
  uint16_t us_distance_echo;
  uint16_t us_courbe_temps;
  uint16_t us_courbe_valeur;
  uint16_t us_courbe_ref;
};

struct NavdataPhyMseasures {
  uint16_t tag;
  uint16_t size;

  float32_t accs_temp;
  uint16_t gyro_temp;
  float32_t phys_accs[kNbAccS];
  float32_t phys_gyros[kNbGyroS];
  uint32_t alim3V3; // 3.3volt alim [LSB]
  uint32_t vrefEpson; // ref volt Epson gyro [LSB]
  uint32_t vrefIDG; // ref volt IDG gyro [LSB]
};

class NavData {
public:
  NavData() {};
  ~NavData() { this->curr_pos_ = NULL; }
  void setOptions();

public:
  char raw_data[2048];
  NavdataHeader nav_header;
  NavdataDemo nav_demo;
  NavdataTime nav_time;
  NavdataRawMeasures nav_raw_measures;
  NavdataPhyMseasures nav_phys_measures;

private:
  void setDemoInfo();
  void setHeader(); 
  void setPhysMeasurement();
  void setRawMeasurement();
  void setTimeInfo();

private:
  char* curr_pos_;
};

#endif  // AI_ROBOT_NAVIGATION_NAVDATA_H_
