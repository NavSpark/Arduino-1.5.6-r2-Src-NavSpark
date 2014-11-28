/*
  GNSS.h - Header file of library for NavSpark GNSS
  Copyright (c) 2014 NavSpark.  All right reserved.

	This library is free software; you can redistribute it under the terms
  of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any
  later version.

  This library is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

	Created 28 Feb. 2014 by Ming-Jen Chen

	$Id$
*/

#ifndef _GNSS_H_
#define _GNSS_H_

#include "stdint.h"
#include "sti_gnss_lib.h"

#define ULONG_MAX	0xFFFFFFFF

#define	TS_TRIG_ON	true
#define	TS_TRIG_OFF	false

#define	TS_TRIG_RISING	0x0
#define	TS_TRIG_FALLING	0x1

#define MAX_NUM_OF_TIMESTAMP_INFO	10

#define CONSTELLATION_GPS 0
#define CONSTELLATION_BD2 1
#define CONSTELLATION_GLONASS 2

class GNSSParam
{
	private:
		bool _init_done;
		GNSS_INIT_MODE_T init_mode;

	public:
		GNSSParam(void);

		void setDefault(void);
		void setNavMode(uint8_t mode);
		void setUpdateRate(uint8_t rate);
		void setDopMaskMode(uint8_t mode);
		void setPdopMask(float pdop);
		void setHdopMask(float hdop);
		void setGdopMask(float gdop);
		void init(void);
		bool init_done(void);
};

extern GNSSParam GnssConf;

#if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1)

struct GNSSDate
{
	private:
		uint16_t gnssYear;
		uint8_t gnssMonth;
		uint8_t gnssDay;
	public:
		void update(uint16_t year, uint8_t month, uint8_t day);
		uint16_t year(void);
		uint8_t month(void);
		uint8_t day(void);
		uint16_t formatString(char* str);
};

struct GNSSTime
{
	private:
		uint8_t gnssHour;
		uint8_t gnssMinute;
		float gnssSecond;
	public:
		void update(uint8_t hour, uint8_t min, float second);
		uint8_t hour(void);
		uint8_t minute(void);
		uint8_t second(void);
		uint8_t centisecond(void);
		uint16_t formatString(char* str);
};

struct GNSSLocation
{
	private:
		double latitude_in_deg;
		double longitude_in_deg;
	public:
		void update(double lat, double lng);
		double latitude(void);
		double longitude(void);
		uint16_t latitude_formatString(char* str);
		uint16_t longitude_formatString(char* str);
};

struct GNSSAltitude
{
	private:
		float altitude_in_meters;
	public:
		void update(float altitude);
		float meters(void);
		float miles(void);
		float kilometers(void);
		float feet(void);
};

struct GNSSGeoSeparation
{
	private:
		float geo_separation_in_meters;
	public:
		void update(float meters);
		float meters(void);
		float miles(void);
		float kilometers(void);
		float feet(void);
};

struct GNSSSpeed
{
	private:
		float speed_in_knots; /* ground speed */
		float vertical_speed; /* vertical speed, meter per second */
	public:
		void update(float speed);
		float kph(void); // Kilo-meters per hour
		float knots(void); // Knots per hour
		float mph(void); // Miles per hour
		void updateVS(void);
		float vertical_mph(void);
		float vertical_kph(void);
};

struct GNSSCourse
{
	private:
		float course_in_deg;
	public:
		void update(float course);
		float deg(void);
};

struct GNSSDOP
{
	public:
		float pdop(void);
		float hdop(void);
		float vdop(void);
};

struct GNSSSatellites
{
	private:
		PVT_DATA_T gnssPvtData;
		SV_INFO_T gnssSvInfo;

	public:
		// function to get GNSS info. from H/W and update to necessary objects
		void update(PVT_DATA_T* pPvtData, SV_INFO_T* pSvInfo);

		// functions for how many satellites are in view
		uint16_t numGPSInView(uint16_t *prn);
		uint16_t numBD2InView(uint16_t *prn);
		uint16_t numGLNInView(uint16_t *prn);

		// functions for get info. of individual satellite
		uint16_t elevation(uint8_t constellation, uint16_t prn); // 0 ~ 90
		uint16_t azimuth(uint8_t constellation, uint16_t prn); // 0 ~ 359
		uint16_t CNR(uint8_t constellation, uint16_t prn);

		// functions for how many satellites are used for position fix
		uint16_t numGPSInUse(uint16_t *prn);
		uint16_t numBD2InUse(uint16_t *prn);
		uint16_t numGLNInUse(uint16_t *prn);
};

struct GNSSTimeStamp
{
	private:
		void *_callBackEntry;
		uint8_t _trigMode;

		UTC_TIME_T currUTC;
		TIME_STAMPING_STATUS_T currTS;
		TIME_STAMPING_STATUS_T timeStampRecord[MAX_NUM_OF_TIMESTAMP_INFO];
		uint16_t wptr;
		uint16_t rptr;
		uint16_t numTimeStampRecord;

		void enableTrigCapture(void);
		void desableTrigCapture(void);

	public:
		GNSSTimeStamp(void);

		// function to register the callback function for timestamp
		void setTrigCapture(bool enable, uint8_t trigMode, void (*callback));

		// functions for timestamp
		uint16_t numRecord(void);
		uint16_t idxRecord(void);
		bool push(TIME_STAMPING_STATUS_T ts);
		bool pop(void);
		void convertTimeStampToUTC(void);

		// functions to output the values of timestamp
		uint16_t year(void);
		uint8_t month(void);
		uint8_t day(void);
		uint8_t hour(void);
		uint8_t minute(void);
		uint8_t second(void);
		double fractional_sec(void);
		uint16_t formatUTCString(char* str);
		uint16_t formatGPSString(char* str);
};

class GNSS
{
	private:
		bool updated;
		uint8_t fix_mode;
		uint32_t lastUpdateTime;

	public:
		GNSS(void);

		// member functions
		void update(void);
		bool isUpdated(void);
		uint8_t fixMode(void);
		static double distanceBetween(double lat1, double long1, double lat2, double long2);
		static double courseTo(double lat1, double long1, double lat2, double long2);

		// info. of from GNSS
		GNSSDate date;
		GNSSTime time;
		GNSSLocation location;
		GNSSAltitude altitude;
		GNSSGeoSeparation geoseperation;
		GNSSSpeed speed;
		GNSSCourse course;
		GNSSSatellites satellites;
		GNSSDOP dop;
		GNSSTimeStamp timestamp;
};

extern GNSS GnssInfo;

#endif /* #if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1) */

#endif // _GNSS_H_
