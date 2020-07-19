// Copyright(c) 2019 Yohei Matsumoto, All right reserved.
// f_radar_gxhd.hpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_radar_gxhd.hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_radar_gxhd.hpp.  If not, see <http://www.gnu.org/licenses/>.

// This program is based on radar_pi developed under GPLv2.
// See https://github.com/opencpn-radar-pi/radar_i

#ifndef F_RADAR_GXHD_HPP
#define F_RADAR_GXHD_HPP
#include "filter_base.hpp"
#include "ch_state.hpp"
#include "ch_radar.hpp"

#include "radar_pi/socketutil.h"
#include "radar_pi/garminxhd.h"
#include "radar_pi/GarminxHDControl.h"
#include "radar_pi/GarminxHDReceive.h"

struct receive_statistics
{
  unsigned long long packets;
  unsigned long long broken_packets;
  unsigned long long spokes;
  unsigned long long missing_spokes;
  long long tstart, tend;
  unsigned long long  hist[256];

  uint8_t vth;
  uint16_t dmin, dmax;
  uint8_t vmin, vmax, vavg;
  receive_statistics() : packets(0), broken_packets(0), spokes(0), missing_spokes(0),
                         tstart(0), tend(0), vth(0), dmin(UINT16_MAX), dmax(0),
                         vmin(0), vmax(0), vavg(0)
  {
    for(int i = 0; i < 256; i++)
      hist[i] = 0;
  }

  void accumulate(uint8_t *line)
  {
    for (int i = 0; i < GARMIN_XHD_MAX_SPOKE_LEN; i++)
    {
      if (line[i] > vth)
      {
        dmax = max((uint16_t)i, dmax);
        dmin = min((uint16_t)i, dmin);
      }
      hist[line[i]]++;
    }
  }

  void calc_stat()
  {
    double sum = 0.0;
    unsigned long long total = 0;
    vmin = 255; vmax=0;
    for (int i = 0; i < 256; i++)
    {
      total += hist[i];
      if(hist[i] > 0 && i < vmin)
        vmin = i;
      if(hist[i] > 0 && i > vmax)
        vmax = i;

      sum += (double)hist[i] * (double)i;
    }
    sum /= (double)total;
    vavg = (uint8_t) sum;
  }
};

class f_radar_sim : public f_base
{
protected:
  ch_state *state;
  ch_radar_image *radar_image;

  int range_meters;
  long long period;
  double miss_rate;
  enum e_sim_mode
  {
    SM_TEST,
    SM_NONE
  } mode;
  static const char *str_sim_mode[SM_NONE];
  int spoke_next;
  long long tprev_proc;
  long long tprev_image_update;
  long long tproc_period;
  int num_spokes_per_proc;

  unsigned char **spokes;
  int spoke_offset, range_offset;
  void generate_test_pattern();

public:
  f_radar_sim(const char *name) : f_base(name), mode(SM_TEST),
                                  miss_rate(0), period(2 * SEC), range_meters(1852), spoke_next(0), 
                                  tprev_proc(-1), tprev_image_update(-1)
  {
    register_fpar("state", (ch_base **)&state, typeid(ch_state).name(), "State channel");
    register_fpar("radar_image", (ch_base **)&radar_image, typeid(ch_radar_image).name(), "Radar image channel");
    register_fpar("range_meters", &range_meters, "Range in meter (default 1852)");
    register_fpar("period", &period, "Scanner rotating period in 100nsec. (default 2e7)");
    register_fpar("miss_rate", &miss_rate, "Spoke miss rate. (default 0.3)");
  }

  virtual ~f_radar_sim()
  {
  }

  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();
};

class f_radar_gxhd : public f_base
{
protected:
  GarminxHDControl control;
  GarminxHDReceive receive;
  NetworkAddress interface_address;

  ch_state *state;
  ch_radar_image *radar_image;
  ch_radar_state *radar_state;
  ch_radar_ctrl *radar_ctrl;

  static const int range_vals[16];

  // find an allowed and the nearest range on the rader to specified argument.
  const int find_nearest_range(const int range_meter)
  {
    int irange = 0;
    for (; irange < 16 && range_vals[irange] < range_meter; irange++)
      ;
    return range_vals[irange];
  }

  //  void write_radar_image(int val);
  bool verb_flag;

public:
  static const char *str_radar_command_id[RC_NONE];
  radar_command cmd;
  int cmd_state;

  long long m_boot_time;
  long long m_radar_timeout, m_data_timeout, m_stayalive_timeout;

  unsigned int max_log_size;
  bool enable_replay, enable_log;
  c_log logger, state_logger; // scan data logger, and radar state logger

  receive_statistics m_statistics;
  long long GetBootTime() { return m_boot_time; };

  void DetectedRadar(const NetworkAddress &radarAddress)
  {
    if (!control.Init("GarminxHDControl", interface_address, radarAddress))
    {
      return;
    }
    m_stayalive_timeout = 0;
  }

  void resetTimeout(long long now)
  {
    m_radar_timeout = now + WATCHDOG_TIMEOUT;
  }

  f_radar_gxhd(const char *name);
  virtual ~f_radar_gxhd();

  virtual bool init_run();

  virtual void destroy_run();

  virtual bool proc();
};
#endif
