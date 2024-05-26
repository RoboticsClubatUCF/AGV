#ifndef _ROS_gps_common_GPSFix_h
#define _ROS_gps_common_GPSFix_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "gps_common/GPSStatus.h"

namespace gps_common
{

  class GPSFix : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef gps_common::GPSStatus _status_type;
      _status_type status;
      typedef double _latitude_type;
      _latitude_type latitude;
      typedef double _longitude_type;
      _longitude_type longitude;
      typedef double _altitude_type;
      _altitude_type altitude;
      typedef double _track_type;
      _track_type track;
      typedef double _speed_type;
      _speed_type speed;
      typedef double _climb_type;
      _climb_type climb;
      typedef double _pitch_type;
      _pitch_type pitch;
      typedef double _roll_type;
      _roll_type roll;
      typedef double _dip_type;
      _dip_type dip;
      typedef double _time_type;
      _time_type time;
      typedef double _gdop_type;
      _gdop_type gdop;
      typedef double _pdop_type;
      _pdop_type pdop;
      typedef double _hdop_type;
      _hdop_type hdop;
      typedef double _vdop_type;
      _vdop_type vdop;
      typedef double _tdop_type;
      _tdop_type tdop;
      typedef double _err_type;
      _err_type err;
      typedef double _err_horz_type;
      _err_horz_type err_horz;
      typedef double _err_vert_type;
      _err_vert_type err_vert;
      typedef double _err_track_type;
      _err_track_type err_track;
      typedef double _err_speed_type;
      _err_speed_type err_speed;
      typedef double _err_climb_type;
      _err_climb_type err_climb;
      typedef double _err_time_type;
      _err_time_type err_time;
      typedef double _err_pitch_type;
      _err_pitch_type err_pitch;
      typedef double _err_roll_type;
      _err_roll_type err_roll;
      typedef double _err_dip_type;
      _err_dip_type err_dip;
      double position_covariance[9];
      typedef uint8_t _position_covariance_type_type;
      _position_covariance_type_type position_covariance_type;
      enum { COVARIANCE_TYPE_UNKNOWN =  0 };
      enum { COVARIANCE_TYPE_APPROXIMATED =  1 };
      enum { COVARIANCE_TYPE_DIAGONAL_KNOWN =  2 };
      enum { COVARIANCE_TYPE_KNOWN =  3 };

    GPSFix():
      header(),
      status(),
      latitude(0),
      longitude(0),
      altitude(0),
      track(0),
      speed(0),
      climb(0),
      pitch(0),
      roll(0),
      dip(0),
      time(0),
      gdop(0),
      pdop(0),
      hdop(0),
      vdop(0),
      tdop(0),
      err(0),
      err_horz(0),
      err_vert(0),
      err_track(0),
      err_speed(0),
      err_climb(0),
      err_time(0),
      err_pitch(0),
      err_roll(0),
      err_dip(0),
      position_covariance(),
      position_covariance_type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_latitude;
      u_latitude.real = this->latitude;
      *(outbuffer + offset + 0) = (u_latitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_latitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_latitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_latitude.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_latitude.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_latitude.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_latitude.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_latitude.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->latitude);
      union {
        double real;
        uint64_t base;
      } u_longitude;
      u_longitude.real = this->longitude;
      *(outbuffer + offset + 0) = (u_longitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_longitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_longitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_longitude.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_longitude.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_longitude.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_longitude.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_longitude.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->longitude);
      union {
        double real;
        uint64_t base;
      } u_altitude;
      u_altitude.real = this->altitude;
      *(outbuffer + offset + 0) = (u_altitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_altitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_altitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_altitude.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_altitude.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_altitude.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_altitude.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_altitude.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->altitude);
      union {
        double real;
        uint64_t base;
      } u_track;
      u_track.real = this->track;
      *(outbuffer + offset + 0) = (u_track.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_track.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_track.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_track.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_track.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_track.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_track.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_track.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->track);
      union {
        double real;
        uint64_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed);
      union {
        double real;
        uint64_t base;
      } u_climb;
      u_climb.real = this->climb;
      *(outbuffer + offset + 0) = (u_climb.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_climb.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_climb.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_climb.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_climb.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_climb.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_climb.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_climb.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->climb);
      union {
        double real;
        uint64_t base;
      } u_pitch;
      u_pitch.real = this->pitch;
      *(outbuffer + offset + 0) = (u_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pitch.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pitch.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pitch.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pitch.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pitch);
      union {
        double real;
        uint64_t base;
      } u_roll;
      u_roll.real = this->roll;
      *(outbuffer + offset + 0) = (u_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_roll.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_roll.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_roll.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_roll.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->roll);
      union {
        double real;
        uint64_t base;
      } u_dip;
      u_dip.real = this->dip;
      *(outbuffer + offset + 0) = (u_dip.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dip.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dip.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dip.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dip.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dip.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dip.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dip.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dip);
      union {
        double real;
        uint64_t base;
      } u_time;
      u_time.real = this->time;
      *(outbuffer + offset + 0) = (u_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->time);
      union {
        double real;
        uint64_t base;
      } u_gdop;
      u_gdop.real = this->gdop;
      *(outbuffer + offset + 0) = (u_gdop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gdop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gdop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gdop.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_gdop.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_gdop.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_gdop.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_gdop.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->gdop);
      union {
        double real;
        uint64_t base;
      } u_pdop;
      u_pdop.real = this->pdop;
      *(outbuffer + offset + 0) = (u_pdop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pdop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pdop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pdop.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pdop.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pdop.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pdop.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pdop.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pdop);
      union {
        double real;
        uint64_t base;
      } u_hdop;
      u_hdop.real = this->hdop;
      *(outbuffer + offset + 0) = (u_hdop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hdop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_hdop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_hdop.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_hdop.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_hdop.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_hdop.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_hdop.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->hdop);
      union {
        double real;
        uint64_t base;
      } u_vdop;
      u_vdop.real = this->vdop;
      *(outbuffer + offset + 0) = (u_vdop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vdop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vdop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vdop.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vdop.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vdop.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vdop.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vdop.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vdop);
      union {
        double real;
        uint64_t base;
      } u_tdop;
      u_tdop.real = this->tdop;
      *(outbuffer + offset + 0) = (u_tdop.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tdop.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tdop.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tdop.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tdop.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tdop.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tdop.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tdop.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tdop);
      union {
        double real;
        uint64_t base;
      } u_err;
      u_err.real = this->err;
      *(outbuffer + offset + 0) = (u_err.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_err.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_err.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_err.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_err.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->err);
      union {
        double real;
        uint64_t base;
      } u_err_horz;
      u_err_horz.real = this->err_horz;
      *(outbuffer + offset + 0) = (u_err_horz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_horz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_horz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_horz.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_err_horz.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_err_horz.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_err_horz.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_err_horz.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->err_horz);
      union {
        double real;
        uint64_t base;
      } u_err_vert;
      u_err_vert.real = this->err_vert;
      *(outbuffer + offset + 0) = (u_err_vert.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_vert.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_vert.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_vert.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_err_vert.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_err_vert.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_err_vert.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_err_vert.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->err_vert);
      union {
        double real;
        uint64_t base;
      } u_err_track;
      u_err_track.real = this->err_track;
      *(outbuffer + offset + 0) = (u_err_track.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_track.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_track.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_track.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_err_track.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_err_track.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_err_track.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_err_track.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->err_track);
      union {
        double real;
        uint64_t base;
      } u_err_speed;
      u_err_speed.real = this->err_speed;
      *(outbuffer + offset + 0) = (u_err_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_speed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_err_speed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_err_speed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_err_speed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_err_speed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->err_speed);
      union {
        double real;
        uint64_t base;
      } u_err_climb;
      u_err_climb.real = this->err_climb;
      *(outbuffer + offset + 0) = (u_err_climb.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_climb.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_climb.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_climb.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_err_climb.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_err_climb.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_err_climb.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_err_climb.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->err_climb);
      union {
        double real;
        uint64_t base;
      } u_err_time;
      u_err_time.real = this->err_time;
      *(outbuffer + offset + 0) = (u_err_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_err_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_err_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_err_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_err_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->err_time);
      union {
        double real;
        uint64_t base;
      } u_err_pitch;
      u_err_pitch.real = this->err_pitch;
      *(outbuffer + offset + 0) = (u_err_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_pitch.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_err_pitch.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_err_pitch.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_err_pitch.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_err_pitch.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->err_pitch);
      union {
        double real;
        uint64_t base;
      } u_err_roll;
      u_err_roll.real = this->err_roll;
      *(outbuffer + offset + 0) = (u_err_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_roll.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_err_roll.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_err_roll.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_err_roll.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_err_roll.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->err_roll);
      union {
        double real;
        uint64_t base;
      } u_err_dip;
      u_err_dip.real = this->err_dip;
      *(outbuffer + offset + 0) = (u_err_dip.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_dip.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_dip.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_dip.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_err_dip.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_err_dip.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_err_dip.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_err_dip.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->err_dip);
      for( uint32_t i = 0; i < 9; i++){
      union {
        double real;
        uint64_t base;
      } u_position_covariancei;
      u_position_covariancei.real = this->position_covariance[i];
      *(outbuffer + offset + 0) = (u_position_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_covariancei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_position_covariancei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_position_covariancei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_position_covariancei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_position_covariancei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->position_covariance[i]);
      }
      *(outbuffer + offset + 0) = (this->position_covariance_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position_covariance_type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_latitude;
      u_latitude.base = 0;
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_latitude.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->latitude = u_latitude.real;
      offset += sizeof(this->latitude);
      union {
        double real;
        uint64_t base;
      } u_longitude;
      u_longitude.base = 0;
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_longitude.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->longitude = u_longitude.real;
      offset += sizeof(this->longitude);
      union {
        double real;
        uint64_t base;
      } u_altitude;
      u_altitude.base = 0;
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_altitude.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->altitude = u_altitude.real;
      offset += sizeof(this->altitude);
      union {
        double real;
        uint64_t base;
      } u_track;
      u_track.base = 0;
      u_track.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_track.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_track.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_track.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_track.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_track.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_track.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_track.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->track = u_track.real;
      offset += sizeof(this->track);
      union {
        double real;
        uint64_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      union {
        double real;
        uint64_t base;
      } u_climb;
      u_climb.base = 0;
      u_climb.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_climb.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_climb.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_climb.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_climb.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_climb.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_climb.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_climb.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->climb = u_climb.real;
      offset += sizeof(this->climb);
      union {
        double real;
        uint64_t base;
      } u_pitch;
      u_pitch.base = 0;
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pitch.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pitch = u_pitch.real;
      offset += sizeof(this->pitch);
      union {
        double real;
        uint64_t base;
      } u_roll;
      u_roll.base = 0;
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_roll.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->roll = u_roll.real;
      offset += sizeof(this->roll);
      union {
        double real;
        uint64_t base;
      } u_dip;
      u_dip.base = 0;
      u_dip.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dip.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dip.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dip.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_dip.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_dip.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_dip.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_dip.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->dip = u_dip.real;
      offset += sizeof(this->dip);
      union {
        double real;
        uint64_t base;
      } u_time;
      u_time.base = 0;
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->time = u_time.real;
      offset += sizeof(this->time);
      union {
        double real;
        uint64_t base;
      } u_gdop;
      u_gdop.base = 0;
      u_gdop.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gdop.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gdop.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gdop.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_gdop.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_gdop.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_gdop.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_gdop.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->gdop = u_gdop.real;
      offset += sizeof(this->gdop);
      union {
        double real;
        uint64_t base;
      } u_pdop;
      u_pdop.base = 0;
      u_pdop.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pdop.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pdop.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pdop.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_pdop.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_pdop.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_pdop.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_pdop.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->pdop = u_pdop.real;
      offset += sizeof(this->pdop);
      union {
        double real;
        uint64_t base;
      } u_hdop;
      u_hdop.base = 0;
      u_hdop.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hdop.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_hdop.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_hdop.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_hdop.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_hdop.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_hdop.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_hdop.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->hdop = u_hdop.real;
      offset += sizeof(this->hdop);
      union {
        double real;
        uint64_t base;
      } u_vdop;
      u_vdop.base = 0;
      u_vdop.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vdop.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vdop.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vdop.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vdop.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vdop.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vdop.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vdop.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vdop = u_vdop.real;
      offset += sizeof(this->vdop);
      union {
        double real;
        uint64_t base;
      } u_tdop;
      u_tdop.base = 0;
      u_tdop.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tdop.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tdop.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tdop.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_tdop.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_tdop.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_tdop.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_tdop.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->tdop = u_tdop.real;
      offset += sizeof(this->tdop);
      union {
        double real;
        uint64_t base;
      } u_err;
      u_err.base = 0;
      u_err.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_err.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_err.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_err.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_err.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->err = u_err.real;
      offset += sizeof(this->err);
      union {
        double real;
        uint64_t base;
      } u_err_horz;
      u_err_horz.base = 0;
      u_err_horz.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_horz.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_horz.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_horz.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_err_horz.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_err_horz.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_err_horz.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_err_horz.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->err_horz = u_err_horz.real;
      offset += sizeof(this->err_horz);
      union {
        double real;
        uint64_t base;
      } u_err_vert;
      u_err_vert.base = 0;
      u_err_vert.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_vert.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_vert.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_vert.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_err_vert.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_err_vert.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_err_vert.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_err_vert.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->err_vert = u_err_vert.real;
      offset += sizeof(this->err_vert);
      union {
        double real;
        uint64_t base;
      } u_err_track;
      u_err_track.base = 0;
      u_err_track.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_track.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_track.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_track.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_err_track.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_err_track.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_err_track.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_err_track.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->err_track = u_err_track.real;
      offset += sizeof(this->err_track);
      union {
        double real;
        uint64_t base;
      } u_err_speed;
      u_err_speed.base = 0;
      u_err_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_err_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_err_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_err_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_err_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->err_speed = u_err_speed.real;
      offset += sizeof(this->err_speed);
      union {
        double real;
        uint64_t base;
      } u_err_climb;
      u_err_climb.base = 0;
      u_err_climb.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_climb.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_climb.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_climb.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_err_climb.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_err_climb.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_err_climb.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_err_climb.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->err_climb = u_err_climb.real;
      offset += sizeof(this->err_climb);
      union {
        double real;
        uint64_t base;
      } u_err_time;
      u_err_time.base = 0;
      u_err_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_err_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_err_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_err_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_err_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->err_time = u_err_time.real;
      offset += sizeof(this->err_time);
      union {
        double real;
        uint64_t base;
      } u_err_pitch;
      u_err_pitch.base = 0;
      u_err_pitch.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_pitch.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_pitch.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_pitch.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_err_pitch.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_err_pitch.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_err_pitch.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_err_pitch.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->err_pitch = u_err_pitch.real;
      offset += sizeof(this->err_pitch);
      union {
        double real;
        uint64_t base;
      } u_err_roll;
      u_err_roll.base = 0;
      u_err_roll.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_roll.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_roll.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_roll.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_err_roll.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_err_roll.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_err_roll.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_err_roll.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->err_roll = u_err_roll.real;
      offset += sizeof(this->err_roll);
      union {
        double real;
        uint64_t base;
      } u_err_dip;
      u_err_dip.base = 0;
      u_err_dip.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_dip.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_dip.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_dip.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_err_dip.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_err_dip.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_err_dip.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_err_dip.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->err_dip = u_err_dip.real;
      offset += sizeof(this->err_dip);
      for( uint32_t i = 0; i < 9; i++){
      union {
        double real;
        uint64_t base;
      } u_position_covariancei;
      u_position_covariancei.base = 0;
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_position_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->position_covariance[i] = u_position_covariancei.real;
      offset += sizeof(this->position_covariance[i]);
      }
      this->position_covariance_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->position_covariance_type);
     return offset;
    }

    virtual const char * getType() override { return "gps_common/GPSFix"; };
    virtual const char * getMD5() override { return "3db3d0a7bc53054c67c528af84710b70"; };

  };

}
#endif
