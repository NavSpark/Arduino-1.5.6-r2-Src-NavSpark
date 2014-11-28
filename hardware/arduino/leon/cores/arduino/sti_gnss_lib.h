/*
  sti_gnss_lib.h - NavSpark GPS/BD2/GNSS library header file
  Copyright (c) 2013 NavSpark.  All right reserved.

	This library is free software; you can redistribute it (modification
	is NOT recommended) under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either version
	2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this library; if not, write to the Free Software Foundation,
  Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

	Created 25 December 2013 by Ming-Jen Chen

	$Id$
*/

#ifndef STI_GNSS_LIB_H_
#define	STI_GNSS_LIB_H_

#include "st_type.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------
// defines for constants
// ---------------------

#ifndef CFG_BASE
	#define	CFG_BASE	0x50000000UL
#endif
#define	REG_BASE_ADDR	0x80000000

#define	SWCFG_CUSTOMER_CMD_INPUT_EXAMPLE	0

#ifndef STGNSS_GPS_NCHAN
	#define	STGNSS_GPS_NCHAN	ST_MAX_GPS_NCHAN
#endif

#ifndef	STGNSS_BD2_NCHAN
	#define	STGNSS_BD2_NCHAN	ST_MAX_BD2_NCHAN
#endif

#ifndef	STGNSS_GLONASS_NCHAN
	#define	STGNSS_GLONASS_NCHAN	ST_MAX_GLO_NCHAN
#endif

#define	STGNSS_HOT_START	1
#define	STGNSS_WARM_START	2
#define	STGNSS_COLD_START	3

// for SBAS constellation selection
#define	STGNSS_BIT_CONSTELLATION_WAAS		(0x01)
#define	STGNSS_BIT_CONSTELLATION_EGNOS	(0x02)
#define	STGNSS_BIT_CONSTELLATION_MSAS		(0x04)
#define	STGNSS_SBAS_MODE_ON		1
#define	STGNSS_SBAS_MODE_OFF	0
#define STGNSS_SBAS_RANGING_MODE_DISABLE     0
#define STGNSS_SBAS_RANGING_MODE_ENABLE      1
#define STGNSS_SBAS_RANGING_MODE_AUTO        2
#define	STGNSS_SBAS_RANGING_MODE			STGNSS_SBAS_RANGING_MODE_AUTO
#define	STGNSS_SBAS_RANGING_URA_MASK	8 // why ? bcz ublock use 7 to position fix
#define	STGNSS_SBAS_CORRECTION_MODE		1
#define	STGNSS_SBAS_NUMS_OF_TM				2 // 3
#define STGNSS_BIT_CONSTELLATION_ALL	(0x80)
#define	STGNSS_SBAS_SUBSYSTEM_BITS		(STGNSS_BIT_CONSTELLATION_ALL)

#define	STGNSS_QZSS_MODE_ON		1
#define	STGNSS_QZSS_MODE_OFF	0
#define	STGNSS_QZSS_MAX_NUM		1 // 1~3

#define	STGNSS_POWER_SAVE_ON	1
#define	STGNSS_POWER_SAVE_OFF	0

#define	STGNSS_ENHANCE_ACQUISITION		1
#define	STGNSS_LOW_POWER_ACQUISITION	0

#define	STGNSS_GNSS_GPS_ONLY			1
#define	STGNSS_GNSS_GLONASS_ONLY	2
#define	STGNSS_GNSS_COMBO					5

#define	STGNSS_ELEVATION_CNR_MASK_MODE	ELV_CNR_MASK_AUTO
#define	STGNSS_ELEVATION_MASK_DEGREE		5
#define	STGNSS_CNR_MASK	0

#define	STGNSS_DOP_MASK_DISABLE	0
#define	STGNSS_DOP_MASK_AUTO		1
#define	STGNSS_DOP_MASK_PDOP		2
#define	STGNSS_DOP_MASK_HDOP		3
#define	STGNSS_DOP_MASK_GDOP		4

#define	STGNSS_PDOP_MASK	(30.0)
#define	STGNSS_HDOP_MASK	(30.0)
#define	STGNSS_GDOP_MASK	(30.0)

#define	STGNSS_FIX_NONE				0
#define	STGNSS_FIX_PREDICTION	1
#define	STGNSS_FIX_2D					2
#define	STGNSS_FIX_3D					3
#define	STGNSS_FIX_DIFF				4
#define	STGNSS_N_FIX_MODE			5

#define	STGNSS_DATALOG_MODE_ON	1
#define	STGNSS_DATALOG_MODE_OFF	0

#define	STGNSS_PINNING_MODE_ON	1
#define	STGNSS_PINNING_MODE_OFF	0

#define	STGNSS_SAEE_MODE_ON		1
#define	STGNSS_SAEE_MODE_OFF	0

#define	STGNSS_PSE_LOW_POWER_ACQUISITION	0
#define	STGNSS_PSE_MID_POWER_ACQUISITION	1
#define	STGNSS_PSE_HIGH_POWER_ACQUISITION	2
#define	STGNSS_PSE_FULL_POWER_ACQUISITION	3

#define STGNSS_POSITION_UPDATE_RATE_1HZ   1
#define STGNSS_POSITION_UPDATE_RATE_2HZ   2
#define STGNSS_POSITION_UPDATE_RATE_4HZ   4
#define STGNSS_POSITION_UPDATE_RATE_5HZ   5
#define STGNSS_POSITION_UPDATE_RATE_8HZ   8
#define STGNSS_POSITION_UPDATE_RATE_10HZ  10

// defines for navigation modes
#define	STGNSS_NAV_MODE_AUTO				0
#define	STGNSS_NAV_MODE_PEDESTRIAN	1
#define	STGNSS_NAV_MODE_CAR					2
#define	STGNSS_NAV_MODE_MARINE			3
#define	STGNSS_NAV_MODE_BALLOON			4
#define	STGNSS_NAV_MODE_AIRBORNE		5

#define	STGNSS_BIT_CONSTELLATION_GPS			(0x01)
#define	STGNSS_BIT_CONSTELLATION_GLONASS	(0x02)
#define	STGNSS_BIT_CONSTELLATION_GALILEO	(0x04)
#define	STGNSS_BIT_CONSTELLATION_BEIDOU2	(0x08)

#define	STGNSS_UART_5BITS_WORD_LENGTH	5
#define	STGNSS_UART_6BITS_WORD_LENGTH	6
#define	STGNSS_UART_7BITS_WORD_LENGTH	7
#define	STGNSS_UART_8BITS_WORD_LENGTH	8

#define	STGNSS_UART_1STOP_BITS	0
#define	STGNSS_UART_2STOP_BITS	1

#define	STGNSS_UART_NOPARITY		0
#define	STGNSS_UART_ODDPARITY		1
#define	STGNSS_UART_EVENPARITY	2

//talker ID
//for NMEA talker id
#define STGNSS_TALKER_ID_MODE_GP   0
#define STGNSS_TALKER_ID_MODE_GN   1

#define	SWCFG_SD_CARD	0
#define	SWCFG_I2C			0

#if SWCFG_SD_CARD
	//#define	SD_MMC_DEBUG
	#define	SPI_CSMMC_ACTIVATE		0x02
	#define	SPI_CSMMC_DEACTIVATE	0x0
	#define	USED_INSERT_PIN		0
	#define	SD_INSERT_PIN			11
	#define	SD_WRITE_PROTECT	15
	//#define	SD_TEST_READ
	#define	SD_TEST_WRITE
	#if defined(SD_TEST_READ) && defined(SD_TEST_WRITE)
		echo "SD_NMEA_WRITE and SD_NMEA_WRITE can't exist at the same time";
	#endif
#endif

// ----------------------
// defines for structures
// ----------------------

typedef struct {
	D64	lat_deg; // latitude in degrees
	D64	lon_deg; // longitude in degrees
	F32	alt; // Altitude in meters
	F32	course; // Course over ground in degrees
	F32	speed; // Speed over ground in knots
	D64	ecef_px; // x-axis coordinate in ECEF coordinate
	D64	ecef_py; // y-axis coordinate in ECEF coordinate
	D64	ecef_pz; // z-axis coordinate in ECEF coordinate
	D64	ecef_vx; // Speed in x-axis in ECEF coordinate
	D64	ecef_vy; // Speed in y-axis in ECEF coordinate
	D64	ecef_vz; // Speed in z-axis in ECEF coordinate
	S16	fix_mode; // GG7 fix mode. Values of STGV8_FIX_NONE, STGV8_FIX_PREDICTION, STGV8_FIX_2D, STGV8_FIX_3D, STGV8_FIX_DIFF
	S16	nsv_used_gps; // Number of GPS satellites used for position fix
	S08	id_nums_gps[STGNSS_GPS_NCHAN]; // Array containing PRN number of GPS satellites used for position fix.
	S16	nsv_used_beidou2; // Number of BD2 satellites used for position fix
	S08	id_nums_beidou2[STGNSS_BD2_NCHAN]; // Array containing PRN number of BD2 satellites used for position fix.
	S16	nsv_used_glonass; // Number of GLONASS satellites used for position fix
	S08	id_nums_glonass[STGNSS_GLONASS_NCHAN]; // Array containing PRN number of GLONASS satellites used for position fix
	F32	pdop; // Position dilution of precision
	F32	hdop; // Horizontal dilution of precision
	F32	vdop; // Vertical dilution of precision
	F32	gdop; // Geometric dilution of precision
	F32	tdop; // Time dilution of precision
	S16	wn; // GPS week number
	D64	tow; // GPS time of week in second
} PVT_DATA_T;

typedef struct {
	S16	sv; // Satellite ID number 1~32 for GPS, 65~96 for GLONASS (64 plus slot numbers)
	S16	elv; // Elevation angle in degree, 0 ~ 90 degrees
	S16	azm; // Azimuth in degree, 000~359
	S16	cn0; // C/N0 ratio in dB/Hz
} SV_DATA_T;

typedef struct {
	S16				n_gps_sv_in_view; // Number of GPS satellites in view
	SV_DATA_T	gps_sv_data[STGNSS_GPS_NCHAN]; // Array with in-view GPS satellite information
	S16				n_beidou2_sv_in_view; // Number of BEIDOU2 satellites in view
	SV_DATA_T	beidou2_sv_data[STGNSS_BD2_NCHAN]; // Array with in-view BD2 satellite information
	S16				n_glonass_sv_in_view; // Number of GLONASS satellites in view
	SV_DATA_T	glonass_sv_data[STGNSS_GLONASS_NCHAN]; // Array with in-view GLONASS satellite information
} SV_INFO_T;

typedef struct {
	U16	year;	// UTC year, range 1980 onward
	U16	month; // UTC month, range 1 ~ 12
	U16	day; // UTC day, range 1 ~ 31
	U16	hour; // UTC hour, range 0 ~ 23
	U16	minute; // UTC minute, range 0 ~ 59
	U16	sec; // UTC second, range 0 ~ 59
} GNSS_UTC_IN_T;

typedef struct {
	U16	year;	// UTC year, range 1980 onward
	U16	month; // UTC month, range 1 ~ 12
	U16	day; // UTC day, range 1 ~ 31
	U16	hour; // UTC hour, range 0 ~ 23
	U16	minute; // UTC minute, range 0 ~ 59
	U16	sec; // UTC second, range 0 ~ 59
	U32 usec; // micro-seconds, range 0 ~ 999999
} GNSS_UTC_TIMESTAMP_T;

typedef struct {
	U08	mode;
	D64	lat_deg;	// latitude in degrees
	D64	lon_deg;	// longitude in degrees
	F32	alt;			// Altitude in meters
	GNSS_UTC_IN_T	utc;
} GNSS_RESTART_T;

typedef struct {
	U08	gga_t;
	U08	gll_t;
	U08	gsa_t;
	U08	gsv_t;
	U08	rmc_t;
	U08	vtg_t;
	U08	zda_t;
} NMEA_INERVAL_T;

typedef struct {
	U32	baud;
	U08	word_length;
	U08	stop_bits;
	U08	parity;
} UART_PARAM_T;

typedef struct {
	U08	power_mode;  // STGV8_POWER_SAVE_ON or STGV8_POWER_SAVE_OFF
	U08	acquisition_mode; // STGV8_ENHANCE_ACQUISITION, STGV8_LOW_POWER_ACQUISITION
	U08	position_update_rate; //STGNSS_POSITION_UPDATE_RATE_1HZ~10HZ
	U08	gnss_select; // STGV8_GNSS_GPS_ONLY, STGV8_GNSS_GLONASS_ONLY or STGV8_GNSS_COMBO
	U08	datalog_mode;
	U08	nav_mode;
	U08	pinning_mode;
	U08	saee_mode;
  U08 talker_id;
	UART_PARAM_T			uart1;
	UART_PARAM_T			uart2;
	NMEA_INERVAL_T		nmea_interval;
	ELV_CNR_MASK_T		elv_cnr_mask; // elv_cnr mask
	DOP_MASK_T				dop_mask; // dop mask
	SBAS_PARAMETER_T	sbas_para;
	QZSS_PARAMETER_T	qzss_para;
} GNSS_INIT_MODE_T;

typedef struct {
	U16	year;	// year (4 digit)
	U08	month;	// month 1 ~ 12
	U08	day;	// day 1 ~ 31
	U08	hour;	// UTC hour
	U08	minute;	// UTC minute
	F32 sec;	// UTC in second
} RTC_INFO_T;

typedef struct{
  S16 time_is_valid;
  U16 trigger_count;
  U16 wn_of_last_trigger_edge;
  U32 tow_of_last_trigger_edge_unit_is_ms;
  U32 ms_fraction_of_tow_of_last_trigger_edge_unit_is_ns;
} TIME_STAMPING_STATUS_T;

typedef struct {
  S16 year;
  S16 month;
  S16 day;
  S16 day_of_year;
  S16 hour;
  S16 minute;
  D64 sec; // bcz round error
} UTC_TIME_T;

// -------------------------------
// declarations for GNSS functions
// -------------------------------

#if defined(USE_UART1_FOR_NMEA) && (USE_UART1_FOR_NMEA==1)
void	gnss_init( const GNSS_INIT_MODE_T* init_mode_p );
#else
void	gnss_init( void );
#endif

void	gnss_close( void );
void	gnss_get_NMEA_interval(U08* gga_p, U08* gll_p, U08* gsa_p, U08* gsv_p, U08* rmc_p, U08* vtg_p, U08* zda_p );
void	gnss_uart_tx_send( U08 uart, U08 s );
void	gnss_uart_critical_section_enable( U08 uart );
void	gnss_uart_critical_section_disable( U08 uart );
void	gnss_system_reboot();
void	gnss_spi_master_onoff( U08 onoff );
void	gnss_process_1ms_in_isr();
void	gnss_system_version( U32 odm_version, U32 odm_revision );
void	gnss_system_restart( const GNSS_RESTART_T* init_p );
void	gnss_gpio_set_input( U08 gpio );
void	gnss_gpio_set_output( U08 gpio );
void	gnss_gpio_high( U08 gpio );
void	gnss_gpio_low( U08 gpio );
void	gnss_gpio_interrupt_enable(U08 gpio);
void	gnss_gpio_interrupt_disable(U08 gpio);
void	gnss_gpio_set_mux_mode(U08 gpio, U08 is_gpio);
void	gnss_gpio_interrupt_property(U08 polarity, U08 edge);
void	gnss_uart_resetRX( S16 uart );
void	gnss_get_lla_position( LLA_T* lla_p );
void	gnss_get_lat_lon_degree( D64* lat_p, D64* lon_p );
void	gnss_get_ecef_position( POS_T* pos_p );
void	gnss_uart_process();
void	gnss_disable_irq( S16 irq );
void	gnss_setup_isr( S16 irq, void *isr );
void	gnss_start_irq( S16 irq );
void	gnss_enable_irq( S16 irq );
void	gnss_watchdog_enable( U32 sec );
void	gnss_watchdog_disable( void );
void  gnss_convert_gps_to_utc_time( S16 wn, D64 tow, UTC_TIME_T *utc_time_p, S16 rounding );

S16	gnss_process( void );
S16	gnss_set_datum( const S16 datum_ndx );
S16	gnss_get_datum( S16* datum_ndx_p );
S16	gnss_get_software_version( S08* buf_p );
S16	gnss_get_power_mode( U08* power_mode );
S16	gnss_set_SBAS( const SBAS_PARAMETER_T* sbas_mode_p );
S16	gnss_get_SBAS( SBAS_PARAMETER_T* sbas_mode_p );
S16	gnss_set_QZSS( const QZSS_PARAMETER_T* qzss_mode_p );
S16	gnss_get_QZSS( QZSS_PARAMETER_T* qzss_mode_p );
S16	gnss_set_DOP_mask( const DOP_MASK_T* dop_mask_p );
S16	gnss_get_DOP_mask( DOP_MASK_T* dop_mask_p );
S16	gnss_set_ELV_SNR_mask( const ELV_CNR_MASK_T* elv_snr_mask_p );
S16	gnss_get_ELV_SNR_mask( ELV_CNR_MASK_T* elv_snr_mask_p );
S16 gnss_set_ELV_CNR_mask( const ELV_CNR_MASK_T* elv_cnr_mask_p );
S16 gnss_get_ELV_CNR_mask( ELV_CNR_MASK_T* elv_cnr_mask_p );
S16	gnss_get_pvt_data( PVT_DATA_T* pvt_data_p );
S16	gnss_get_sv_info( SV_INFO_T* sv_info_p );
S16	gnss_get_rtc( RTC_INFO_T* rtc_info_p );
S16	gnss_gpio_read_input( U08 gpio );
S16	gnss_get_fixmode();
S16	gnss_get_fix_svs();
S16	gnss_has_fixed();
S16 gnss_get_enu_speed( VEL_ENU_T *enu_speed_p );

U08 gnss_get_constellation_type_for_fix();
U08 gnss_get_constellation_capability();
U08	gnss_uart_tx_status( U08 uart );
U08	gnss_uart_rx_status( U08 uart );
U08	gnss_uart_rx_receive( U08 uart );
U08 gnss_uart_init( U08 com, U32 baudrate, U08 wordlength, U08 stopbits, U08 parity );
U08 gnss_uart_putline( S16 u_id, U08 *buf, S16 count );
U08 gnss_uart_loop_uart0( U08* bufin_intf, U32 uart_buf_indx );
U08 gnss_timer_init( U08 unit, U32 count, U08 enable_int, U08 auto_reload );
U08 gnss_timer_enable( U08 unit );
U08 gnss_timer_disable( U08 unit );
U08 gnss_timer_clear_interrupt( U08 unit );
U08 gnss_timer_interrupt_pending_status( U08 unit );
U08	gnss_datalog_set_poi();
S08 gnss_uart_getchar( U08 uart, U32 *uart_buf_indx, U08* buf, U32 size );
U08 gnss_trig_setup( U08 trigEnable, U08 trigMode, void *func );

U32	gnss_get_cpu_clock();
U32	gnss_get_peripheral_hw_clock();
U32 gnss_get_prescaler();
U32 gnss_sti_sprintf(char *buf, const char *fmt, ...);
U32	gnss_get_fix_count();
F32 gnss_get_speed();
F32 gnss_get_course();
F32 gnss_get_altitude();
F32 gnss_get_gdop();
F32 gnss_get_pdop();
F32 gnss_get_hdop();
F32 gnss_get_vdop();
F32 gnss_get_tdop();
F32 gnss_get_geo_separation();

void gnss_1ms_setup_callbacks( void *func );
void gnss_hw_peripheral_isr_setup_callbacks( void *func);

#ifdef __cplusplus
}	// extern "C"
#endif

#endif
