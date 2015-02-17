/*
  GNSS.cpp - C++ file of library for NavSpark GNSS
  Copyright (c) 2014 NavSpark.  All right reserved.

	This library is free software; you can redistribute it under the terms
  of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any
  later version.

  This library is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

	Created 24 Feb. 2014 by Ming-Jen Chen

	$Id$
*/

#include <stdio.h>
#include <math.h>
#include "stdint.h"
#include "sti_gnss_lib.h"
#include "Arduino.h"

#define _FEET_PER_METER 3.2808399
#define _MILES_PER_METER 0.00062137112
#define _KPH_PER_KNOT 1.852
#define _MPH_PER_KNOT 1.15077945

#ifdef __cplusplus
extern "C" {
#endif
void isrTimerFunc(void); // defined @ "Timer.cpp"
#ifdef __cplusplus
}
#endif

// **********************************************************************
// Description: Instances of GNSS and GNSSParam
// **********************************************************************
GNSSParam GnssConf = GNSSParam();

#if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1)
GNSS GnssInfo = GNSS();
#endif

// **********************************************************************
// Description: declaration of functions provided in "wiring_intr.c"
// **********************************************************************
#ifdef __cplusplus
extern "C" {
#endif
void detachInterrupt(uint8_t pin);
#ifdef __cplusplus
}
#endif

// **********************************************************************
// Description: Member functions for GNSSParam
// **********************************************************************
GNSSParam::GNSSParam(void)
{
	setDefault();
}

void GNSSParam::setDefault(void)
{
	_init_done = false;
	init_mode.power_mode = STGNSS_POWER_SAVE_ON;
	init_mode.acquisition_mode = STGNSS_PSE_MID_POWER_ACQUISITION;
	init_mode.position_update_rate=STGNSS_POSITION_UPDATE_RATE_1HZ;
#if defined(ST_CONST_SEL) && (ST_CONST_SEL==1)
	init_mode.gnss_select = STGNSS_BIT_CONSTELLATION_GPS;
#endif
#if defined(ST_CONST_SEL) && (ST_CONST_SEL==2)
	init_mode.gnss_select = (STGNSS_BIT_CONSTELLATION_GPS | STGNSS_BIT_CONSTELLATION_BEIDOU2);
#endif
#if defined(ST_CONST_SEL) && (ST_CONST_SEL==3)
	init_mode.gnss_select = (STGNSS_BIT_CONSTELLATION_GPS | STGNSS_BIT_CONSTELLATION_GLONASS);
#endif
	init_mode.datalog_mode = STGNSS_DATALOG_MODE_OFF;
	init_mode.nav_mode = STGNSS_NAV_MODE_AUTO;
  init_mode.pinning_mode = STGNSS_PINNING_MODE_OFF;
  init_mode.saee_mode = STGNSS_SAEE_MODE_OFF;
  init_mode.talker_id = STGNSS_TALKER_ID_MODE_GP;

  init_mode.uart1.baud = BAUDRATE;
  init_mode.uart1.word_length = STGNSS_UART_8BITS_WORD_LENGTH;
  init_mode.uart1.stop_bits = STGNSS_UART_1STOP_BITS;
  init_mode.uart1.parity = STGNSS_UART_NOPARITY;
  init_mode.uart2.baud = BAUDRATE;
  init_mode.uart2.word_length = STGNSS_UART_8BITS_WORD_LENGTH;
  init_mode.uart2.stop_bits = STGNSS_UART_1STOP_BITS;
  init_mode.uart2.parity = STGNSS_UART_NOPARITY;

  init_mode.nmea_interval.gga_t = 1;
  init_mode.nmea_interval.gll_t = 1;
  init_mode.nmea_interval.gsa_t = 1;
  init_mode.nmea_interval.gsv_t = 3;
  init_mode.nmea_interval.rmc_t = 1;
  init_mode.nmea_interval.vtg_t = 1;
  init_mode.nmea_interval.zda_t = 1;

  init_mode.sbas_para.sbas_enable_disable = STGNSS_SBAS_MODE_ON;
  init_mode.sbas_para.ranging_enable_disable = STGNSS_SBAS_RANGING_MODE;
  init_mode.sbas_para.ranging_ura_mask = STGNSS_SBAS_RANGING_URA_MASK;
  init_mode.sbas_para.correction_enable_disable = STGNSS_SBAS_CORRECTION_MODE;
  init_mode.sbas_para.num_of_tracking_channels = STGNSS_SBAS_NUMS_OF_TM;
  init_mode.sbas_para.subsystem_mask = STGNSS_SBAS_SUBSYSTEM_BITS;

	init_mode.qzss_para.qzss_enable_disable = STGNSS_QZSS_MODE_ON;
  init_mode.qzss_para.qzss_max_num = STGNSS_QZSS_MAX_NUM;

  init_mode.elv_cnr_mask.elv_cnr_select_mode = STGNSS_ELEVATION_CNR_MASK_MODE;
  init_mode.elv_cnr_mask.elv_mask = STGNSS_ELEVATION_MASK_DEGREE;
  init_mode.elv_cnr_mask.cnr_mask = STGNSS_CNR_MASK;

	init_mode.dop_mask.dop_mask_mode = STGNSS_DOP_MASK_AUTO;
  init_mode.dop_mask.pdop_mask = STGNSS_PDOP_MASK;
  init_mode.dop_mask.hdop_mask = STGNSS_HDOP_MASK;
  init_mode.dop_mask.gdop_mask = STGNSS_GDOP_MASK;
}

void GNSSParam::setNavMode(uint8_t mode)
{
	switch (mode) {
		case STGNSS_NAV_MODE_AUTO:
		case STGNSS_NAV_MODE_PEDESTRIAN:
		case STGNSS_NAV_MODE_CAR:
		case STGNSS_NAV_MODE_MARINE:
		case STGNSS_NAV_MODE_BALLOON:
		case STGNSS_NAV_MODE_AIRBORNE:
			init_mode.nav_mode = mode;
		break;
	}
}

void GNSSParam::setUpdateRate(uint8_t rate)
{
	switch (rate) {
		case STGNSS_POSITION_UPDATE_RATE_1HZ:
		case STGNSS_POSITION_UPDATE_RATE_2HZ:
		case STGNSS_POSITION_UPDATE_RATE_4HZ:
		case STGNSS_POSITION_UPDATE_RATE_5HZ:
		case STGNSS_POSITION_UPDATE_RATE_8HZ:
		case STGNSS_POSITION_UPDATE_RATE_10HZ:
			init_mode.position_update_rate = rate;
		break;
	}
}

void GNSSParam::setDopMaskMode(uint8_t mode)
{
	switch (mode) {
		case STGNSS_DOP_MASK_DISABLE:
		case STGNSS_DOP_MASK_AUTO:
		case STGNSS_DOP_MASK_PDOP:
		case STGNSS_DOP_MASK_HDOP:
		case STGNSS_DOP_MASK_GDOP:
			init_mode.dop_mask.dop_mask_mode = mode;
		break;
	}
}

void GNSSParam::setPdopMask(float pdop)
{
	uint16_t pdop_int;

	if (pdop >= 0.5 && pdop <= 30.0) {
		pdop_int = (uint16_t) (pdop * 10);
		init_mode.dop_mask.pdop_mask = pdop_int / 10.0;
	}
}

void GNSSParam::setHdopMask(float hdop)
{
	uint16_t hdop_int;

	if (hdop >= 0.5 && hdop <= 30.0) {
		hdop_int = (uint16_t) (hdop * 10);
		init_mode.dop_mask.hdop_mask = hdop_int / 10.0;
	}
}

void GNSSParam::setGdopMask(float gdop)
{
	uint16_t gdop_int;

	if (gdop >= 0.5 && gdop <= 30.0) {
		gdop_int = (uint16_t) (gdop * 10);
		init_mode.dop_mask.gdop_mask = gdop_int / 10.0;
	}
}

void GNSSParam::init(void)
{
	// system-wide initialization
#if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1)
	gnss_init(&(this->init_mode));
#else
	gnss_init();
#endif

	// setup ISR for Timer /* must be called after gnss_init() */
	gnss_disable_irq(TIMER_ISR_NUMBER);
	gnss_setup_isr(TIMER_ISR_NUMBER, (void*)isrTimerFunc);
  gnss_start_irq(TIMER_ISR_NUMBER);

	// mark init has completed
	this->_init_done = true;
}

bool GNSSParam::init_done(void)
{
	return this->_init_done;
}

#if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1)

// **********************************************************************
// Description: Member functions for GNSSDate
// **********************************************************************
void GNSSDate::update(uint16_t year, uint8_t month, uint8_t day)
{
	if ((year >= 1983) &&
		  (month > 0) && (month <= 12) &&
		  (day > 0) && (day <= 31))
	{
		gnssYear = year;
		gnssMonth = month;
		gnssDay = day;
	}
}

uint16_t GNSSDate::year(void)
{
	return gnssYear;
}

uint8_t GNSSDate::month(void)
{
	return gnssMonth;
}

uint8_t GNSSDate::day(void)
{
	return gnssDay;
}

uint16_t GNSSDate::formatString(char* str)
{
	// convert date to string in format "yyyy-mm-dd"
#ifdef USE_GNSS_STI_SPRINTF
	return gnss_sti_sprintf(str, "%04d-%02d-%02d", gnssYear, gnssMonth, gnssDay);
#else
	return sprintf(str, "%04d-%02d-%02d", gnssYear, gnssMonth, gnssDay);
#endif
}

// **********************************************************************
// Description: Member functions for GNSSTime
// **********************************************************************
void GNSSTime::update(uint8_t hour, uint8_t min, float second)
{
	if ((hour <= 23) && (min <= 59) && (second < 60.0))
	{
		gnssHour = hour;
		gnssMinute = min;
		gnssSecond = second;
	}
}

uint8_t GNSSTime::hour(void)
{
	return gnssHour;
}

uint8_t GNSSTime::minute(void)
{
	return gnssMinute;
}

uint8_t GNSSTime::second(void)
{
	return ((uint8_t)gnssSecond);
}

uint8_t GNSSTime::centisecond(void)
{
	float centi;

	centi = gnssSecond - second();
	centi = centi * 100.0;
	return ((uint8_t)centi);
}

uint8_t GNSSTime::decisecond(void)
{
        float deci;
        deci = gnssSecond - second();
        deci = deci * 10.0;
        return ((uint16_t)deci);
}

uint16_t GNSSTime::formatString(char* str)
{
	char A_P = 'A';
	uint8_t hour = gnssHour;

	if (gnssHour >= 12) {
		A_P = 'P';
		hour = gnssHour - 12;
	}

	// convert date to string in format "11:35:23 AM"
#ifdef USE_GNSS_STI_SPRINTF
	return gnss_sti_sprintf(str, "%02d:%02d:%02d %cM", hour, gnssMinute, second(), A_P);
#else
	return sprintf(str, "%02d:%02d:%02d %cM", hour, gnssMinute, second(), A_P);
#endif
}

// **********************************************************************
// Description: Member functions for GNSSLocation
// **********************************************************************
void GNSSLocation::update(double lat, double lng)
{
	if ((lat >= -90) && (lat <= 90) &&
		  (lng >= -180) && (lng <= 180))
	{
		latitude_in_deg = lat;
		longitude_in_deg = lng;
	}
}

double GNSSLocation::latitude(void)
{
	return latitude_in_deg;
}

double GNSSLocation::longitude(void)
{
	return longitude_in_deg;
}

uint16_t GNSSLocation::latitude_formatString(char* str)
{
	char N_S = 'N';
	double absLatitude = latitude_in_deg;
	uint16_t deg;
	uint8_t min;
#ifdef USE_GNSS_STI_SPRINTF
	float tmpFloat;
#endif

	if (latitude_in_deg < 0) {
		N_S = 'S';
		absLatitude = fabs(latitude_in_deg);
	}

	// convert the latitude to the format 120¢X36'55.22"
	deg = (uint16_t) absLatitude;
	absLatitude = (absLatitude - deg) * 60;
	min = (uint8_t) absLatitude;
	absLatitude = (absLatitude - min) * 60;

#ifdef USE_GNSS_STI_SPRINTF
	tmpFloat = (float)absLatitude;
	return gnss_sti_sprintf(str, "%03d¢X%02d\'%.2f\" %c", deg, min, tmpFloat, N_S);
#else
	return sprintf(str, "%03d¢X%02d\'%.2f\" %c", deg, min, absLatitude, N_S);
#endif
}

uint16_t GNSSLocation::longitude_formatString(char* str)
{
	char E_W = 'E';
	double absLongitude = longitude_in_deg;
	uint16_t deg;
	uint8_t min;
#ifdef USE_GNSS_STI_SPRINTF
	float tmpFloat;
#endif

	if (longitude_in_deg < 0) {
		E_W = 'W';
		absLongitude = fabs(longitude_in_deg);
	}

	// convert the latitude to the format 120¢X36'55.22"N
	deg = (uint16_t) absLongitude;
	absLongitude = (absLongitude - deg) * 60;
	min = (uint8_t) absLongitude;
	absLongitude = (absLongitude - min) * 60;

#ifdef USE_GNSS_STI_SPRINTF
	tmpFloat = (float)absLongitude;
	return gnss_sti_sprintf(str, "%03d¢X%02d\'%.2f\" %c", deg, min, tmpFloat, E_W);
#else
	return sprintf(str, "%03d¢X%02d\'%.2f\" %c", deg, min, absLongitude, E_W);
#endif
}

// **********************************************************************
// Description: Member functions for GNSSAltitude
// **********************************************************************
void GNSSAltitude::update(float altitude)
{
	altitude_in_meters = altitude;
}

float GNSSAltitude::meters(void)
{
	return altitude_in_meters;
}

float GNSSAltitude::miles(void)
{
	return (altitude_in_meters*_MILES_PER_METER);
}

float GNSSAltitude::kilometers(void)
{
	return (altitude_in_meters/1000);
}

float GNSSAltitude::feet(void)
{
	return (altitude_in_meters*_FEET_PER_METER);
}

// **********************************************************************
// Description: Member functions for GNSSGeoSeparation
// **********************************************************************
void GNSSGeoSeparation::update(float meters)
{
	geo_separation_in_meters = meters;
}

float GNSSGeoSeparation::meters(void)
{
	return geo_separation_in_meters;
}

float GNSSGeoSeparation::miles(void)
{
	return (geo_separation_in_meters*_MILES_PER_METER);
}

float GNSSGeoSeparation::kilometers(void)
{
	return (geo_separation_in_meters/1000);
}

float GNSSGeoSeparation::feet(void)
{
	return (geo_separation_in_meters*_FEET_PER_METER);
}

// **********************************************************************
// Description: Member functions for GNSSSpeed
// **********************************************************************
void GNSSSpeed::update(float speed)
{
	speed_in_knots = speed;
}

float GNSSSpeed::kph(void)
{
	return (speed_in_knots*_KPH_PER_KNOT);
}

float GNSSSpeed::knots(void)
{
	return speed_in_knots;
}

float GNSSSpeed::mph(void)
{
	return (speed_in_knots*_MPH_PER_KNOT);
}

void GNSSSpeed::updateVS(void)
{
	static VEL_ENU_T velEnU;

	if (gnss_get_enu_speed(&velEnU)) {
		vertical_speed = velEnU.vu;
	}
}

float GNSSSpeed::vertical_mph(void)
{
	return (vertical_speed * _MILES_PER_METER);
}

float GNSSSpeed::vertical_kph(void)
{
	return (vertical_speed / 1000.0);
}

// **********************************************************************
// Description: Member functions for GNSSCourse
// **********************************************************************
void GNSSCourse::update(float course)
{
	if ((course >= 0) && (course < 360)) {
		course_in_deg = course;
	}
}

float GNSSCourse::deg(void)
{
	return course_in_deg;
}

// **********************************************************************
// Description: Member functions for GNSSDOP
// **********************************************************************
float GNSSDOP::pdop(void)
{
	return gnss_get_pdop();
}

float GNSSDOP::hdop(void)
{
	return gnss_get_hdop();
}

float GNSSDOP::vdop(void)
{
	return gnss_get_vdop();
}

// **********************************************************************
// Description: Member functions for GNSSSatellites
// **********************************************************************
void GNSSSatellites::update(PVT_DATA_T* pPvtData, SV_INFO_T* pSvInfo)
{
	memcpy(&gnssPvtData, pPvtData, sizeof(PVT_DATA_T));
	memcpy(&gnssSvInfo, pSvInfo, sizeof(SV_INFO_T));
}

uint16_t GNSSSatellites::numGPSInView(uint16_t *prn)
{
	if (prn != NULL) {
		for (uint16_t k = 0; k < gnssSvInfo.n_gps_sv_in_view; k ++) {
			prn[k] = gnssSvInfo.gps_sv_data[k].sv;
		}
	}
	return gnssSvInfo.n_gps_sv_in_view;
}

uint16_t GNSSSatellites::numBD2InView(uint16_t *prn)
{
	if (prn != NULL) {
		for (uint16_t k = 0; k < gnssSvInfo.n_beidou2_sv_in_view; k ++) {
			prn[k] = gnssSvInfo.beidou2_sv_data[k].sv;
		}
	}
	return gnssSvInfo.n_beidou2_sv_in_view;
}

uint16_t GNSSSatellites::numGLNInView(uint16_t *prn)
{
	if (prn != NULL) {
		for (uint16_t k = 0; k < gnssSvInfo.n_glonass_sv_in_view; k ++) {
			prn[k] = gnssSvInfo.glonass_sv_data[k].sv;
		}
	}
	return gnssSvInfo.n_glonass_sv_in_view;
}

uint16_t GNSSSatellites::elevation(uint8_t constellation, uint16_t prn)
{
	uint16_t k;

	if (constellation == CONSTELLATION_GPS) {
		for (k = 0; k < numGPSInView(NULL); k++) {
			if (prn == gnssSvInfo.gps_sv_data[k].sv) {
				return gnssSvInfo.gps_sv_data[k].elv;
			}
		}
	}
	else if (constellation == CONSTELLATION_BD2) {
		for (k = 0; k < numBD2InView(NULL); k++) {
			if (prn == gnssSvInfo.beidou2_sv_data[k].sv) {
				return gnssSvInfo.beidou2_sv_data[k].elv;
			}
		}
	}
	else if (constellation == CONSTELLATION_GLONASS) {
		for (k = 0; k < numGLNInView(NULL); k++) {
			if (prn == gnssSvInfo.glonass_sv_data[k].sv) {
				return gnssSvInfo.glonass_sv_data[k].elv;
			}
		}
	}
	return 0xffff;
}

uint16_t GNSSSatellites::azimuth(uint8_t constellation, uint16_t prn)
{
	uint16_t k;

	if (constellation == CONSTELLATION_GPS) {
		for (k = 0; k < numGPSInView(NULL); k++) {
			if (prn == gnssSvInfo.gps_sv_data[k].sv) {
				return gnssSvInfo.gps_sv_data[k].azm;
			}
		}
	}
	else if (constellation == CONSTELLATION_BD2) {
		for (k = 0; k < numBD2InView(NULL); k++) {
			if (prn == gnssSvInfo.beidou2_sv_data[k].sv) {
				return gnssSvInfo.beidou2_sv_data[k].azm;
			}
		}
	}
	else if (constellation == CONSTELLATION_GLONASS) {
		for (k = 0; k < numGLNInView(NULL); k++) {
			if (prn == gnssSvInfo.glonass_sv_data[k].sv) {
				return gnssSvInfo.glonass_sv_data[k].azm;
			}
		}
	}
	return 0xffff;
}

uint16_t GNSSSatellites::CNR(uint8_t constellation, uint16_t prn)
{
	uint16_t k;

	if (constellation == CONSTELLATION_GPS) {
		for (k = 0; k < numGPSInView(NULL); k++) {
			if (prn == gnssSvInfo.gps_sv_data[k].sv) {
				return gnssSvInfo.gps_sv_data[k].cn0;
			}
		}
	}
	else if (constellation == CONSTELLATION_BD2) {
		for (k = 0; k < numBD2InView(NULL); k++) {
			if (prn == gnssSvInfo.beidou2_sv_data[k].sv) {
				return gnssSvInfo.beidou2_sv_data[k].cn0;
			}
		}
	}
	else if (constellation == CONSTELLATION_GLONASS) {
		for (k = 0; k < numGLNInView(NULL); k++) {
			if (prn == gnssSvInfo.glonass_sv_data[k].sv) {
				return gnssSvInfo.glonass_sv_data[k].cn0;
			}
		}
	}
	return 0xffff;
}

uint16_t GNSSSatellites::numGPSInUse(uint16_t *prn)
{
	if (prn != NULL) {
		for (uint16_t k = 0; k < gnssPvtData.nsv_used_gps; k ++) {
			prn[k] = gnssPvtData.id_nums_gps[k];
		}
	}
	return gnssPvtData.nsv_used_gps;
}

uint16_t GNSSSatellites::numBD2InUse(uint16_t *prn)
{
	if (prn != NULL) {
		for (uint16_t k = 0; k < gnssPvtData.nsv_used_beidou2; k ++) {
			prn[k] = gnssPvtData.id_nums_beidou2[k];
		}
	}
	return gnssPvtData.nsv_used_beidou2;
}

uint16_t GNSSSatellites::numGLNInUse(uint16_t *prn)
{
	if (prn != NULL) {
		for (uint16_t k = 0; k < gnssPvtData.nsv_used_glonass; k ++) {
			prn[k] = gnssPvtData.id_nums_glonass[k];
		}
	}
	return gnssPvtData.nsv_used_glonass;
}

// **********************************************************************
// Description: Member functions for GNSSTimeStamp
// **********************************************************************
GNSSTimeStamp::GNSSTimeStamp(void)
{
	_callBackEntry = NULL;
	_trigMode = 0;
	wptr = 0;
	rptr = 0;
	numTimeStampRecord = 0;
}

void GNSSTimeStamp::setTrigCapture(bool enable, uint8_t trigMode, void (*callback))
{
	if (enable == true) {
		// setup GPIO10 to be input pin mode
		pinMode(GPIO10_TRIG, INPUT);
		// remove interrupt attached to GPIO10
		detachInterrupt(GPIO10_TRIG);
		_callBackEntry = callback;
		_trigMode = trigMode;
		enableTrigCapture();
	}
	else if (enable == false) {
		_callBackEntry = NULL;
		_trigMode = 0;
		desableTrigCapture();
	}
}

void GNSSTimeStamp::enableTrigCapture(void)
{
	if (_callBackEntry) {
		gnss_trig_setup(1, _trigMode, (void*)_callBackEntry);
	}
}

void GNSSTimeStamp::desableTrigCapture(void)
{
	gnss_trig_setup(0, 0, 0);
}

uint16_t GNSSTimeStamp::numRecord(void)
{
	return numTimeStampRecord;
}

uint16_t GNSSTimeStamp::idxRecord(void)
{
	return currTS.trigger_count;
}

bool GNSSTimeStamp::push(TIME_STAMPING_STATUS_T ts)
{
	if (numTimeStampRecord != MAX_NUM_OF_TIMESTAMP_INFO) {
		timeStampRecord[wptr].trigger_count = ts.trigger_count;
		timeStampRecord[wptr].wn_of_last_trigger_edge = ts.wn_of_last_trigger_edge;
		timeStampRecord[wptr].tow_of_last_trigger_edge_unit_is_ms = ts.tow_of_last_trigger_edge_unit_is_ms;
		timeStampRecord[wptr].ms_fraction_of_tow_of_last_trigger_edge_unit_is_ns = ts.ms_fraction_of_tow_of_last_trigger_edge_unit_is_ns;
		desableTrigCapture();
		wptr = (wptr + 1) % MAX_NUM_OF_TIMESTAMP_INFO;
		numTimeStampRecord++;
		enableTrigCapture();
		return true;
	}
	else return false;
}

bool GNSSTimeStamp::pop(void)
{
	if (numTimeStampRecord != 0) {
		currTS.trigger_count = timeStampRecord[rptr].trigger_count;
		currTS.wn_of_last_trigger_edge = timeStampRecord[rptr].wn_of_last_trigger_edge;
		currTS.tow_of_last_trigger_edge_unit_is_ms = timeStampRecord[rptr].tow_of_last_trigger_edge_unit_is_ms;
		currTS.ms_fraction_of_tow_of_last_trigger_edge_unit_is_ns = timeStampRecord[rptr].ms_fraction_of_tow_of_last_trigger_edge_unit_is_ns;
		desableTrigCapture();
		rptr = (rptr + 1) % MAX_NUM_OF_TIMESTAMP_INFO;
		numTimeStampRecord--;
		enableTrigCapture();
		return true;
	}
	else return false;
}

void GNSSTimeStamp::convertTimeStampToUTC(void)
{
	double tow;

	tow = currTS.tow_of_last_trigger_edge_unit_is_ms / 1000.0
	    + currTS.ms_fraction_of_tow_of_last_trigger_edge_unit_is_ns / 1000000000.0;

	gnss_convert_gps_to_utc_time( currTS.wn_of_last_trigger_edge, tow, &currUTC, 0 );
}

uint16_t GNSSTimeStamp::year(void) { return currUTC.year; }
uint8_t GNSSTimeStamp::month(void) { return currUTC.month; }
uint8_t GNSSTimeStamp::day(void) { return currUTC.day; }
uint8_t GNSSTimeStamp::hour(void) { return currUTC.hour; }
uint8_t GNSSTimeStamp::minute(void) { return currUTC.minute; }
uint8_t GNSSTimeStamp::second(void)
{
	return ((uint8_t) currUTC.sec);
}
double GNSSTimeStamp::fractional_sec(void)
{
	return (currUTC.sec - second());
}

uint16_t GNSSTimeStamp::formatUTCString(char* str)
{
	char A_P = 'A';
	uint8_t hour = currUTC.hour;
#ifdef USE_GNSS_STI_SPRINTF
	float tmpFloat;
#endif

	if (currUTC.hour >= 12) {
		A_P = 'P';
		hour -= 12;
	}

#ifdef USE_GNSS_STI_SPRINTF
	tmpFloat = (float)fractional_sec();
	return gnss_sti_sprintf(str,
		"%04d-%02d-%02d @ %02d:%02d:%02d(+%.7f) %cM",
		currUTC.year, currUTC.month, currUTC.day,
		hour, currUTC.minute, second(), tmpFloat, A_P);
#else
	return sprintf(str,
		"%04d-%02d-%02d @ %02d:%02d:%02d(+%.7f) %cM",
		currUTC.year, currUTC.month, currUTC.day,
		hour, currUTC.minute, second(), fractional_sec(), A_P);
#endif
}

uint16_t GNSSTimeStamp::formatGPSString(char* str)
{
	double tow_ms;
#ifdef USE_GNSS_STI_SPRINTF
	float tmpFloat;
#endif

	tow_ms = currTS.tow_of_last_trigger_edge_unit_is_ms * 1.0
	       + currTS.ms_fraction_of_tow_of_last_trigger_edge_unit_is_ns / 1000000.0;

#ifdef USE_GNSS_STI_SPRINTF
	tmpFloat = (float) tow_ms;
	return gnss_sti_sprintf(str,
		"%.4f ms, week #%4d since 1980",
		tmpFloat,
		currTS.wn_of_last_trigger_edge);
#else
	return sprintf(str,
		"%.4f ms, week #%4d since 1980",
		tow_ms,
		currTS.wn_of_last_trigger_edge);
#endif
}

// **********************************************************************
// Description: Member functions for GNSS
// **********************************************************************
GNSS::GNSS(void)
{
	fix_mode = 0;
	// add any necessary code for GNSS initialization here
}

void GNSS::update(void)
{
	static RTC_INFO_T rtcInfo;
	static PVT_DATA_T pvtData;
	static SV_INFO_T svInfo;

	updated = false;

	if (gnss_get_rtc(&rtcInfo)) {
		lastUpdateTime = millis();
		date.update(rtcInfo.year, rtcInfo.month, rtcInfo.day);
		time.update(rtcInfo.hour, rtcInfo.minute, rtcInfo.sec);
	}
	else {
		date.update(1994, 1, 1);
		time.update(0, 0, 0);
	}

	if (gnss_get_pvt_data(&pvtData)) {
	#if defined(ST_CONST_SEL) && (ST_CONST_SEL==1)
		if (pvtData.nsv_used_gps)
	#endif
	#if defined(ST_CONST_SEL) && (ST_CONST_SEL==2)
		if (pvtData.nsv_used_gps || pvtData.nsv_used_beidou2)
	#endif
	#if defined(ST_CONST_SEL) && (ST_CONST_SEL==3)
		if (pvtData.nsv_used_gps || pvtData.nsv_used_glonass)
	#endif
		{
			updated = true;
		}
		location.update(pvtData.lat_deg, pvtData.lon_deg);
		altitude.update(pvtData.alt - gnss_get_geo_separation());
		geoseperation.update(gnss_get_geo_separation());
		speed.update(pvtData.speed);
		speed.updateVS();
		course.update(pvtData.course);
	}
	else {
		location.update(0.0, 0.0);
		altitude.update(0.0);
		geoseperation.update(0.0);
		speed.update(0.0);
		course.update(0.0);
	}

	if (gnss_get_sv_info(&svInfo)) {
	#if defined(ST_CONST_SEL) && (ST_CONST_SEL==1)
		if (svInfo.n_gps_sv_in_view)
	#endif
	#if defined(ST_CONST_SEL) && (ST_CONST_SEL==2)
		if (svInfo.n_gps_sv_in_view || svInfo.n_beidou2_sv_in_view)
	#endif
	#if defined(ST_CONST_SEL) && (ST_CONST_SEL==3)
		if (svInfo.n_gps_sv_in_view || svInfo.n_glonass_sv_in_view)
	#endif
		{

		}
		satellites.update(&pvtData, &svInfo);
	}
	fix_mode = (uint8_t) gnss_get_fixmode();
}

bool GNSS::isUpdated(void)
{
	return updated;
}

uint8_t GNSS::fixMode(void)
{
	return fix_mode;
}

double GNSS::distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

double GNSS::courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

#endif /* #if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1) */
