#ifndef ST_TYPE_INCLUDED
#define ST_TYPE_INCLUDED

#if defined(ST_CONST_SEL) && (ST_CONST_SEL==1)
#include "st_const_gps.h"
#endif
#if defined(ST_CONST_SEL) && (ST_CONST_SEL==2)
#include "st_const_gps_bd2.h"
#endif
#if defined(ST_CONST_SEL) && (ST_CONST_SEL==3)
#include "st_const_gps_gln.h"
#endif

typedef signed   char      S08;
typedef unsigned char      U08;
typedef signed   short int S16;
typedef unsigned short int U16;

typedef signed   long int         S32;
typedef unsigned long int         U32;
typedef float                     F32;
typedef double                    D64;
typedef signed   long long int    S64;
typedef unsigned long long int    U64;

typedef S08                      *PS08;
typedef U08                      *PU08;
typedef S16                      *PS16;
typedef U16                      *PU16;
typedef S32                      *PS32;
typedef U32                      *PU32;
typedef F32                      *PF32;
typedef D64                      *PD64;
typedef signed   long long int   *PS64;
typedef unsigned long long int   *PU64;

#ifndef FALSE
#define FALSE       0
#endif  // #ifndef FALSE
#ifndef TRUE
#define TRUE        (!FALSE)
#endif  // #ifndef TRUE

enum{
  ELV_CNR_MASK_DISABLE,
  ELV_CNR_MASK_AUTO, // check elv & cnr
  ELV_CNR_MASK_ELV_ONLY,
  ELV_CNR_MASK_CNR_ONLY };

typedef struct {
  D64 px;
  D64 py;
  D64 pz;
} POS_T;

typedef struct {
  F32 vx;
  F32 vy;
  F32 vz;
} VEL_T;

typedef struct {
  D64 pe;
  D64 pn;
  D64 pu;
} POS_ENU_T;

typedef struct {
  F32 ve;
  F32 vn;
  F32 vu;
} VEL_ENU_T;

typedef struct {
  F32 ax;
  F32 ay;
  F32 az;
} ACC_T;

typedef struct {
  D64 lat;
  D64 lon;
  F32 alt;
} LLA_T;

typedef struct {
  S16 prn;      // satellite id
  S16 elv;      // elevation in degrees, 90 maximum
  S16 azm;      // azimuth in degrees, 000 ~ 359
  S16 cn0;      // C/N0 ratio
} NMEA_SV_T;

typedef struct {
  S16 slot_n;   // glonass slot_n
  S16 elv;      // elevation in degrees, 90 maximum
  S16 azm;      // azimuth in degrees, 000 ~ 359
  S16 cn0;      // C/N0 ratio
} NMEA_SV_GLONASS_T;


typedef struct{
  U08 elv_cnr_select_mode; //0: disable, 1: input elv & cnr, 2: elv only, 3: cnr only
  S16 elv_mask;   //3~85 degree
  S16 cnr_mask;   //0~40 db
} ELV_CNR_MASK_T;


typedef struct{
  U08 dop_mask_mode; // 0: disable dop mask, 1: auto(3d for pdop; 2d for hdop) gdop, 2: pdop only, 3: hdop only, 4: gdop only,
  F32 pdop_mask;     //0.5~30
  F32 hdop_mask;     //0.5~30
  F32 gdop_mask;     //0.5~30
} DOP_MASK_T;

typedef struct{
  U08 sbas_enable_disable;
  U08 ranging_enable_disable;
  U08 ranging_ura_mask;
  U08 correction_enable_disable;
  U08 num_of_tracking_channels; // 0~3
  U08 subsystem_mask;
} SBAS_PARAMETER_T;

typedef struct{
  U08 qzss_enable_disable;
  U08 qzss_max_num;
} QZSS_PARAMETER_T;


// NMEA types
typedef struct {
  // position
  S16 is_east;  // TRUE: east, FALSE: west
  S16 is_north; // TRUE: north, FALSE, south
  S16 lat_d;    // Latitude in degrees +East
  S16 lon_d;    // Lontitude in degrees +North
  F32 lat_m;    // Latitude in minutes fraction
  F32 lon_m;    // Lontitude in minutes fraction
  F32 alt;      // Altitude in meter
  F32 course;   // Course Over Ground, degrees true
  F32 speed;    // Speed over ground, knots

  // time
  U08 day;      // day 1 ~ 31
  U08 month;    // month 1 ~ 12
  U16 year;     // year (4 digit)
  U08 hour;     // UTC hour
  U08 minute;   // UTC minute
  F32 sec;   // UTC in second

  // others
  S16 fix_mode; // define in st_const.h
  U16 diff_id;  // differential reference station ID, 0000-1023
  U16 aod;      // Age of differential GPS data
  S16 geoid;    // Geoidal separation in 1/10 m
  F32 mag_var;  // Magnetic variation in 1/10 degree, +East
  F32 pdop;     // PDOP
  F32 hdop;     // HDOP
  F32 vdop;     // VDOP

  // how many GNSS system enabled:
  //   1: single GNSS mode
  //   2: combo GNSS mode
  //   3: triple GNSS mode
  //   4. quad GNSS mode
  S16 n_gnss_system;
  S16 is_gps_enabled;
  S16 is_glonass_enabled;
  S16 is_galileo_enabled;
  S16 is_beidou2_enabled;

  // how many GNSS system used in navigations solutions
  //    1: single GNSS mode used in solution
  //    2: two GNSS mode combined used in solution
  //    3: two GNSS mode combined used in solution
  //    4: two GNSS mode combined used in solution
  S16 n_gnss_combined_used;

  S16 is_INS;   // the position come from INS or DR output
  S16 nsv_used_gps; // number of satellites in use, 00-12
  S16 id_nums_gps[ST_MAX_GPS_NCHAN]; // ID numbers of GPS satellites used in solution

  S16 nsv_used_glonass;                    // number of glonass satellites in use, 00-8
  S16 id_nums_glonass[ST_MAX_GLO_NCHAN]; // ID numbers of GLONASS satellites used in solution
  S16 nsv_used_beidou2;                    // number of BEIDOU2 satellites in use
  S16 id_nums_beidou2[ST_MAX_BD2_NCHAN]; // ID numbers of BEIDOU2 satellites used in solution
  S16 nsv_used_galileo;                    // number of BEIDOU2 satellites in use
  S16 id_nums_galileo[ST_MAX_GAL_NCHAN]; // ID numbers of BEIDOU2 satellites used in solution
  // start PRN number for each GNSS
  S16 start_prn_sbas;
  S16 start_prn_qzss;
  S16 start_prn_glonass;
  S16 start_prn_galileo;
  S16 start_prn_beidou2;
  
  // for DGPS (GGA)
  F32 dgps_age_of_data;
  S16 dgps_stid;

  S16 dgps_enable;
  S16 dgps_acrrier_smooth_enable;
  S16 dgps_health;
  S16 dgps_status;

  //binary message
  D64 lat_deg;    // Latitude in degrees +East
  D64 lon_deg;
  S16 wn;
  D64 tow;
  F32 gdop;     // GDOP
  F32 tdop;     // TDOP
  POS_T pos;
  VEL_T vel;

  //for GPGST output
#if SWCFG_POSITION_DOMAIN_ERROR_ANALYSIS
  F32 rms_range_inputs_std;
  F32 std_semi_major_axis;
  F32 std_semi_minor_axis;
  F32 orientation_semi_major_axis;
  F32 std_enu_error[3];
#endif
} NMEA_DATA_T;

#endif  // #ifndef ST_TYPE_INCLUDED

