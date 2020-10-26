#ifndef QMC5883L_H
#define QMC5883L_H

class QMC5883L {
public:
  void init();
  void reset();
  int  ready();
  void reconfig();
  
  float readHeading();
  int readRaw( int16_t *x, int16_t *y, int16_t *z, int16_t *t );
  int readCal( int16_t *xmax, int16_t *xmin, int16_t *ymax, int16_t *ymin);

  void resetCalibration();
  void setCalibration(int16_t xmax,int16_t xmin,int16_t ymax,int16_t ymin);

  void setSamplingRate( int rate );
  void setRange( int range );
  void setOversampling( int ovl );
  
private:
  int16_t xhigh, xlow;
  int16_t yhigh, ylow;
  uint8_t addr;
  uint8_t mode;
  uint8_t rate;
  uint8_t range;
  uint8_t oversampling;
  bool config;
  bool autocal;
};

#endif
