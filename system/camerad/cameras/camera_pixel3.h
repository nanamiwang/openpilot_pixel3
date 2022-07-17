#pragma once

#include <cstdint>

#include <media/cam_req_mgr.h>

#include "system/camerad/cameras/camera_common.h"
#include "common/util.h"

#include <map>
#include <list>
#include <media/cam_sync.h>

#define FRAME_BUF_COUNT 4
//#define EXTRA_IO_CFGS 1

struct CameraRequestInfo {
public:
  CameraRequestInfo() {
  }
  CameraRequestInfo(int fbi, int so): frame_buf_idx(fbi), sync_obj(so) {}
  void set_sync_status(int status) { sync_status = status;}
  void set_sof_ready() {
    sof_ready = true;
  }
  bool finished() {
    return sync_status == CAM_SYNC_STATE_SIGNALED_SUCCESS && sof_ready;
  }
  bool sync_error() { return sync_status == CAM_SYNC_STATE_SIGNALED_ERROR;}
  int32_t sync_obj = 0;
  int frame_buf_idx = 0;
  int32_t sync_status = CAM_SYNC_STATE_INVALID;
  bool sof_ready = false;
};

class CameraState {
public:
  MultiCameraState *multi_cam_state;
  CameraInfo ci;

  std::mutex exp_lock;
  float digital_gain = 1.0;

  // exposure
  uint32_t pixel_clock, line_length_pclk;
  uint32_t frame_length;
  unsigned int max_gain;
  float cur_exposure_frac, cur_gain_frac;
  int cur_gain, cur_integ_lines;

  int exposure_time;
  bool dc_gain_enabled;
  float analog_gain_frac;

  float cur_ev[3];
  float min_ev, max_ev;

  float measured_grey_fraction;
  float target_grey_fraction;
  int gain_idx;

  unique_fd sensor_fd;
  unique_fd csiphy_fd;
  unique_fd actuator_fd;
  bool focus_set = false;

  int camera_num;

  void config_isp(int io_mem_handle, int fence, int request_id, int buf0_mem_handle, int buf0_offset);
  void enqueue_buffer();
  int clear_req_queue();
  void handle_camera_event(void *evdat);
  bool handle_camera_sync_event(struct cam_sync_ev_header *event_data);
  void handle_req_finished(int req_id);
  void set_camera_exposure(float grey_frac);
  void reset_all_maps();

  void sensors_start();
  void sensors_poke(int request_id);
  void sensors_i2c(struct i2c_random_wr_payload* dat, int len, int op_code);
  void actuator_i2c(struct i2c_random_wr_payload* dat, int len, int req_id);
  void sensors_init();

  void camera_open();
  void camera_init(MultiCameraState *multi_cam_state, VisionIpcServer * v, int camera_id, int camera_num, unsigned int fps, cl_device_id device_id, cl_context ctx, VisionStreamType rgb_type, VisionStreamType yuv_type);
  void camera_close();

  int32_t session_handle;
  int32_t sensor_dev_handle;
  int32_t isp_dev_handle;
  int32_t csiphy_dev_handle;
  int32_t actuator_dev_handle;

  int32_t link_handle;

  void *buf0_ptr;
  int buf0_handle;
  int meta_right_buf_handle;
  size_t meta_right_buf_size;
  size_t meta_right_buf_alloc_size;
  int meta_right_buf_handle0;
  size_t meta_right_buf_size0;
  size_t meta_right_buf_alloc_size0;
  int meta_left_buf_handle;
  size_t meta_left_buf_size;
  size_t meta_left_buf_alloc_size;
  int meta_left_buf_handle0;
  size_t meta_left_buf_size0;
  size_t meta_left_buf_alloc_size0;
  int meta_dual_buf_handle;
  size_t meta_dual_buf_size;
  size_t meta_dual_buf_alloc_size;
#ifdef EXTRA_IO_CFGS
  int extra_io_mem_handles[10];
  int extra_sync_objs[10];
#endif
  int buf_handle[FRAME_BUF_COUNT];
  int request_id_last = 1;
  CameraBuf buf;
  std::map<int, CameraRequestInfo> req_info_map;
  std::map<int, int> sync_obj_to_req_id;
  std::list<int> frame_buf_idx_list;
};

typedef struct MultiCameraState {
  unique_fd video0_fd;
  unique_fd video1_fd;
  unique_fd isp_fd;
  int device_iommu;
  int cdm_iommu;


  CameraState road_cam;
  CameraState driver_cam;
  SubMaster *sm;
  PubMaster *pm;
} MultiCameraState;
