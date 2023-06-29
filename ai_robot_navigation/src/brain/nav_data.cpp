#include "nav_data.h"

void NavData::setOptions() {
  setHeader();
  setDemoInfo();
  setTimeInfo();
  setRawMeasurement();
  setPhysMeasurement();
}

void NavData::setDemoInfo() {
  memcpy(&(this->nav_demo.tag), this->curr_pos_, sizeof(this->nav_demo.tag));
  if (this->nav_demo.tag == kNavdataDemoTag) {
    memcpy(&(this->nav_demo.size), this->curr_pos_ + 2,
           sizeof(this->nav_demo.size));
    memcpy(&(this->nav_demo.ctrl_state), this->curr_pos_ + 4,
           sizeof(this->nav_demo.ctrl_state));
    memcpy(&(this->nav_demo.vbat_flying_percentage), this->curr_pos_ + 8,
           sizeof(this->nav_demo.vbat_flying_percentage));
    memcpy(&(this->nav_demo.theta), this->curr_pos_ + 12,
           sizeof(this->nav_demo.theta));
    memcpy(&(this->nav_demo.phi), this->curr_pos_ + 16,
           sizeof(this->nav_demo.phi));
    memcpy(&(this->nav_demo.psi), this->curr_pos_ + 20,
           sizeof(this->nav_demo.psi));
    memcpy(&(this->nav_demo.altitude), this->curr_pos_ + 24,
           sizeof(this->nav_demo.altitude));
    memcpy(&(this->nav_demo.vx), this->curr_pos_ + 28,
           sizeof(this->nav_demo.vx));
    memcpy(&(this->nav_demo.vy), this->curr_pos_ + 32,
           sizeof(this->nav_demo.vy));
    memcpy(&(this->nav_demo.vz), this->curr_pos_ + 36,
           sizeof(this->nav_demo.vz));
    this->curr_pos_ = this->curr_pos_ + this->nav_demo.size;
  }
}

void NavData::setHeader() {
  memcpy(&this->nav_header, this->raw_data, 16);
  this->curr_pos_ = this->raw_data + sizeof(this->nav_header);  // read until nav_demo
}

void NavData::setPhysMeasurement() {
  memcpy(&(this->nav_phys_measures.tag), this->curr_pos_, sizeof(this->nav_phys_measures.tag));

  if (this->nav_phys_measures.tag == kNavdataPhysMeasuresTag) {
    memcpy(&(this->nav_phys_measures.size), this->curr_pos_ + 2,
           sizeof(this->nav_phys_measures.size));
    memcpy(&(this->nav_phys_measures.accs_temp), this->curr_pos_ + 4,
           sizeof(this->nav_phys_measures.accs_temp));
    memcpy(&(this->nav_phys_measures.gyro_temp), this->curr_pos_ + 8,
           sizeof(this->nav_phys_measures.gyro_temp));
    memcpy(&(this->nav_phys_measures.phys_accs[0]), this->curr_pos_ + 10,
           sizeof(this->nav_phys_measures.phys_accs[0]) * kNbAccS);
    memcpy(&(this->nav_phys_measures.phys_gyros[0]), this->curr_pos_ + 22,
           sizeof(this->nav_phys_measures.phys_gyros[0]) * kNbGyroS);
    this->curr_pos_ = this->curr_pos_ + this->nav_phys_measures.size;
  }
}

void NavData::setRawMeasurement() {
  memcpy(&(this->nav_raw_measures.tag), this->curr_pos_, sizeof(this->nav_raw_measures.tag));

  if (this->nav_raw_measures.tag == kNavdataRawMeasuresTag) {
    memcpy(&(this->nav_raw_measures.size), this->curr_pos_ + 2,
           sizeof(this->nav_raw_measures.size));
    memcpy(&(this->nav_raw_measures.raw_accs[0]), this->curr_pos_ + 4,
           sizeof(this->nav_raw_measures.raw_accs) * kNbAccS);
    memcpy(&(this->nav_raw_measures.raw_gyros[0]), this->curr_pos_ + 10,
           sizeof(this->nav_raw_measures.raw_gyros) * kNbGyroS);
    memcpy(&(this->nav_raw_measures.raw_gyros_110[0]), this->curr_pos_ + 16,
           sizeof(this->nav_raw_measures.raw_gyros_110) * 2);
    memcpy(&(this->nav_raw_measures.vbat_raw), this->curr_pos_ + 20,
           sizeof(this->nav_raw_measures.vbat_raw));
    this->curr_pos_ = this->curr_pos_ + this->nav_raw_measures.size;
  }
}

void NavData::setTimeInfo() {
  memcpy(&(this->nav_time.tag), this->curr_pos_, sizeof(this->nav_time.tag));
  if (this->nav_time.tag == kNavdataTimeTag) {
    memcpy(&(this->nav_time.size), this->curr_pos_ + 2,
           sizeof(this->nav_time.size));
    memcpy(&(this->nav_time.tm_stamp), this->curr_pos_ + 4,
           sizeof(this->nav_time.tm_stamp));
    this->curr_pos_ = this->curr_pos_ + this->nav_time.size;
  }
}
