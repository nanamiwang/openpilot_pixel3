#include "selfdrive/camerad/cameras/camera_pixel3.h"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "media/cam_defs.h"
#include "media/cam_isp.h"
#include "media/cam_isp_ife.h"
#include "media/cam_sensor.h"
#include "media/cam_sensor_cmn_header.h"
#include "media/cam_sync.h"
#include "common/swaglog.h"
#include "system/camerad/cameras/sensor_i2c_pixel3.h"
#include "isp_programs.h"

//#define DEBUG_CAM_SYNC_AND_REQ_MGR 1


extern ExitHandler do_exit;

#define USE_1080P

#ifndef USE_RAW10
// NV21
const uint32_t cam_format = CAM_FORMAT_NV21;
const uint32_t cam_format_out_port = CAM_FORMAT_NV12;
const uint32_t cam_res_type = CAM_ISP_IFE_OUT_RES_FULL;
const uint32_t cam_color_space = CAM_COLOR_SPACE_BT601_FULL;
const uint32_t cam_color_pattern = 0x0;
const uint32_t cam_bpp = 0x0;
const uint32_t cam_comp_grp_id = 0x1;
const uint64_t left_bw_bps_0 = 1112188520;
const uint64_t right_bw_bps_0 = 1111755325;
const uint64_t extra_bw_bps_0 = 545828880;
const bool need_debayer = false;
#ifdef USE_1080P
const uint32_t cam_frame_y_width = 1920;
const uint32_t cam_frame_y_height = 1080;
const uint32_t cam_frame_y_plane_stride = 1920;
const uint32_t cam_frame_y_slice_height = 1080;
const uint32_t cam_frame_uv_width = 1920;
const uint32_t cam_frame_uv_height = 540;
const uint32_t cam_frame_uv_plane_stride = 1920;
const uint32_t cam_frame_uv_slice_height = 540;
const size_t FRAME_WIDTH = 1920;
const size_t FRAME_HEIGHT = 1080;
const size_t FRAME_STRIDE = 1920;
const uint64_t left_bw_bps = 222437704;
const uint64_t right_bw_bps = 222351065;
const uint64_t extra_bw_bps = 109165776;
#else // USE_1080P
const uint32_t cam_frame_y_width = 2688;
const uint32_t cam_frame_y_height = 1512;
const uint32_t cam_frame_y_plane_stride = 2688;
const uint32_t cam_frame_y_slice_height = 1512;
const uint32_t cam_frame_uv_width = 2688;
const uint32_t cam_frame_uv_height = 756;
const uint32_t cam_frame_uv_plane_stride = 2688;
const uint32_t cam_frame_uv_slice_height = 756;
const size_t FRAME_WIDTH = 2688;
const size_t FRAME_HEIGHT = 1512;
const size_t FRAME_STRIDE = 2688;
const uint64_t left_bw_bps = 276760673;
const uint64_t right_bw_bps = 276674034;
const uint64_t extra_bw_bps = 109165776;
#endif // USE_1080P
#else // USE_RAW10
const uint32_t cam_frame_width = RAW_FRAME_WIDTH*2;
const uint32_t cam_frame_height = RAW_FRAME_HEIGHT;
#if RAW_FRAME_WIDTH == 2016
const uint32_t cam_frame_plane_stride = 2528;
#else
const uint32_t cam_frame_plane_stride = 5056;
#endif
const size_t FRAME_STRIDE = cam_frame_plane_stride;  // for 10 bit output.
const uint32_t cam_frame_slice_height = RAW_FRAME_HEIGHT;
const size_t FRAME_WIDTH = RAW_FRAME_WIDTH;
const size_t FRAME_HEIGHT = RAW_FRAME_HEIGHT;

const uint32_t cam_format = CAM_FORMAT_MIPI_RAW_10;
const uint32_t cam_format_out_port = CAM_FORMAT_MIPI_RAW_10;
const uint32_t cam_res_type = CAM_ISP_IFE_OUT_RES_RDI_2;
const uint32_t cam_color_space = CAM_COLOR_SPACE_BASE;
const uint32_t cam_color_pattern = 0x5;
const uint32_t cam_bpp = 0xA;
const uint64_t left_bw_bps = 1383803365;
const uint64_t right_bw_bps = 1383370170;
const uint64_t extra_bw_bps = 545828880;
const uint64_t left_bw_bps_0 = 1383803365;
const uint64_t right_bw_bps_0 = 1383370170;
const uint64_t extra_bw_bps_0 = 545828880;
const uint32_t cam_comp_grp_id = 0x0;
const bool need_debayer = true;
#endif // USE_RAW10

const size_t IN_PORT_FRAME_WIDTH = RAW_FRAME_WIDTH;
const size_t IN_PORT_FRAME_HEIGHT = RAW_FRAME_HEIGHT;

const int MIPI_SETTLE_CNT = 14;  // Calculated by camera_freqs.py

CameraInfo cameras_supported[CAMERA_ID_MAX] = {
  [CAMERA_ID_IMX363] = {
    .frame_width = FRAME_WIDTH,
    .frame_height = FRAME_HEIGHT,
    .frame_stride = FRAME_STRIDE,
    .frame_offset = 0,
    .bayer = need_debayer,
    .bayer_flip = 3,
#ifdef ENABLE_HDR
    .hdr = true
#else
    .hdr = false
#endif
  },
};

const float DC_GAIN = 2.5;
const float sensor_analog_gains[] = {
  1.0/8.0, 2.0/8.0, 2.0/7.0, 3.0/7.0, // 0, 1, 2, 3
  3.0/6.0, 4.0/6.0, 4.0/5.0, 5.0/5.0, // 4, 5, 6, 7
  5.0/4.0, 6.0/4.0, 6.0/3.0, 7.0/3.0, // 8, 9, 10, 11
  7.0/2.0, 8.0/2.0, 8.0/1.0};         // 12, 13, 14, 15 = bypass

const int ANALOG_GAIN_MIN_IDX = 0x1; // 0.25x
const int ANALOG_GAIN_REC_IDX = 0x6; // 0.8x
const int ANALOG_GAIN_MAX_IDX = 0xD; // 4.0x

const int EXPOSURE_TIME_MIN = 2; // with HDR, fastest ss
const int EXPOSURE_TIME_MAX = 1904; // with HDR, slowest ss

// ************** low level camera helpers ****************
int do_cam_control(int fd, int op_code, void *handle, int size) {
  struct cam_control camcontrol = {0};
  camcontrol.op_code = op_code;
  camcontrol.handle = (uint64_t)handle;
  if (size == 0) {
    camcontrol.size = 8;
    camcontrol.handle_type = CAM_HANDLE_MEM_HANDLE;
  } else {
    camcontrol.size = size;
    camcontrol.handle_type = CAM_HANDLE_USER_POINTER;
  }

  int ret = HANDLE_EINTR(ioctl(fd, VIDIOC_CAM_CONTROL, &camcontrol));
  if (ret == -1) {
    LOGE("VIDIOC_CAM_CONTROL error: fd: %d, op_code %d - errno %d", fd, op_code, errno);
  }
  return ret;
}

std::optional<int32_t> device_acquire(int fd, int32_t session_handle, void *data) {
  struct cam_acquire_dev_cmd cmd = {
      .session_handle = session_handle,
      .handle_type = CAM_HANDLE_USER_POINTER,
      .num_resources = (uint32_t)(data ? 1 : 0),
      .resource_hdl = (uint64_t)data,
  };
  int err = do_cam_control(fd, CAM_ACQUIRE_DEV, &cmd, sizeof(cmd));
  return err == 0 ? std::make_optional(cmd.dev_handle) : std::nullopt;
};

int device_config(int fd, int32_t session_handle, int32_t dev_handle, uint64_t packet_handle) {
  struct cam_config_dev_cmd cmd = {
      .session_handle = session_handle,
      .dev_handle = dev_handle,
      .packet_handle = packet_handle,
  };
  return do_cam_control(fd, CAM_CONFIG_DEV, &cmd, sizeof(cmd));
}

int device_control(int fd, int op_code, int session_handle, int dev_handle) {
  // start stop and release are all the same
  struct cam_start_stop_dev_cmd cmd { .session_handle = session_handle, .dev_handle = dev_handle };
  return do_cam_control(fd, op_code, &cmd, sizeof(cmd));
}

void *alloc_w_mmu_hdl(int video0_fd, int len, uint32_t *handle, int align = 8, int flags = CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE,
                      int mmu_hdl = 0, int mmu_hdl2 = 0) {
  struct cam_mem_mgr_alloc_cmd mem_mgr_alloc_cmd = {0};
  mem_mgr_alloc_cmd.len = len;
  mem_mgr_alloc_cmd.align = align;
  mem_mgr_alloc_cmd.flags = flags;
  mem_mgr_alloc_cmd.num_hdl = 0;
  if (mmu_hdl != 0) {
    mem_mgr_alloc_cmd.mmu_hdls[0] = mmu_hdl;
    mem_mgr_alloc_cmd.num_hdl++;
  }
  if (mmu_hdl2 != 0) {
    mem_mgr_alloc_cmd.mmu_hdls[1] = mmu_hdl2;
    mem_mgr_alloc_cmd.num_hdl++;
  }

  do_cam_control(video0_fd, CAM_REQ_MGR_ALLOC_BUF, &mem_mgr_alloc_cmd, sizeof(mem_mgr_alloc_cmd));
  *handle = mem_mgr_alloc_cmd.out.buf_handle;

  void *ptr = NULL;
  if (mem_mgr_alloc_cmd.out.fd > 0) {
    ptr = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, mem_mgr_alloc_cmd.out.fd, 0);
    assert(ptr != MAP_FAILED);
  }

  // LOGD("allocated: %x %d %llx mapped %p", mem_mgr_alloc_cmd.out.buf_handle, mem_mgr_alloc_cmd.out.fd, mem_mgr_alloc_cmd.out.vaddr, ptr);

  return ptr;
}

void release(int video0_fd, uint32_t handle) {
  int ret;
  struct cam_mem_mgr_release_cmd mem_mgr_release_cmd = {0};
  mem_mgr_release_cmd.buf_handle = handle;

  ret = do_cam_control(video0_fd, CAM_REQ_MGR_RELEASE_BUF, &mem_mgr_release_cmd, sizeof(mem_mgr_release_cmd));
  assert(ret == 0);
}

void release_fd(int video0_fd, uint32_t handle) {
  // handle to fd
  close(handle>>16);
  release(video0_fd, handle);
}

int CameraState::clear_req_queue() {
  struct cam_req_mgr_flush_info req_mgr_flush_request = {0};
  req_mgr_flush_request.session_hdl = session_handle;
  req_mgr_flush_request.link_hdl = link_handle;
  req_mgr_flush_request.flush_type = CAM_REQ_MGR_FLUSH_TYPE_ALL;
  int ret;
  ret = do_cam_control(multi_cam_state->video0_fd, CAM_REQ_MGR_FLUSH_REQ, &req_mgr_flush_request, sizeof(req_mgr_flush_request));
  // LOGD("flushed all req: %d", ret);
  return ret;
}

// ************** high level camera helpers ****************

void CameraState::sensors_start() {
  int start_reg_len = sizeof(start_reg_array_imx363) / sizeof(struct i2c_random_wr_payload);
  sensors_i2c(start_reg_array_imx363, start_reg_len, CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG);
}

void CameraState::sensors_poke(int request_id) {
  uint32_t cam_packet_handle = 0;
  int size = sizeof(struct cam_packet);
  struct cam_packet *pkt = (struct cam_packet *)alloc_w_mmu_hdl(multi_cam_state->video0_fd, size, &cam_packet_handle);
  pkt->num_cmd_buf = 0;
  pkt->kmd_cmd_buf_index = -1;
  pkt->header.size = size;
  pkt->header.op_code = 0x7f;
  pkt->header.request_id = request_id;

  int ret = device_config(sensor_fd, session_handle, sensor_dev_handle, cam_packet_handle);
  assert(ret == 0);

  munmap(pkt, size);
  release_fd(multi_cam_state->video0_fd, cam_packet_handle);
}

void CameraState::sensors_i2c(struct i2c_random_wr_payload* dat, int len, int op_code) {
  // LOGD("sensors_i2c: %d", len);
  uint32_t cam_packet_handle = 0;
  int size = sizeof(struct cam_packet)+sizeof(struct cam_cmd_buf_desc)*1;
  struct cam_packet *pkt = (struct cam_packet *)alloc_w_mmu_hdl(multi_cam_state->video0_fd, size, &cam_packet_handle);
  pkt->num_cmd_buf = 1;
  pkt->kmd_cmd_buf_index = -1;
  pkt->header.size = size;
  pkt->header.op_code = op_code;
  struct cam_cmd_buf_desc *buf_desc = (struct cam_cmd_buf_desc *)&pkt->payload;

  buf_desc[0].size = buf_desc[0].length = sizeof(struct i2c_rdwr_header) + len*sizeof(struct i2c_random_wr_payload);
  buf_desc[0].type = CAM_CMD_BUF_I2C;

  struct cam_cmd_i2c_random_wr *i2c_random_wr = (struct cam_cmd_i2c_random_wr *)alloc_w_mmu_hdl(multi_cam_state->video0_fd, buf_desc[0].size, (uint32_t*)&buf_desc[0].mem_handle);
  i2c_random_wr->header.count = len;
  i2c_random_wr->header.op_code = 1;
  i2c_random_wr->header.cmd_type = CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR;
  i2c_random_wr->header.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  i2c_random_wr->header.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  memcpy(i2c_random_wr->random_wr_payload, dat, len*sizeof(struct i2c_random_wr_payload));

  int ret = device_config(sensor_fd, session_handle, sensor_dev_handle, cam_packet_handle);
  assert(ret == 0);

  munmap(i2c_random_wr, buf_desc[0].size);
  release_fd(multi_cam_state->video0_fd, buf_desc[0].mem_handle);
  munmap(pkt, size);
  release_fd(multi_cam_state->video0_fd, cam_packet_handle);
}

void CameraState::actuator_i2c(struct i2c_random_wr_payload* dat, int len, int req_id) {
  uint32_t cam_packet_handle = 0;
  int size = sizeof(struct cam_packet)+sizeof(struct cam_cmd_buf_desc)*1;
  struct cam_packet *pkt = (struct cam_packet *)alloc_w_mmu_hdl(multi_cam_state->video0_fd, size, &cam_packet_handle);
  pkt->num_cmd_buf = 1;
  pkt->kmd_cmd_buf_index = -1;
  pkt->header.size = size;
  pkt->header.op_code = 0x2000002;
  pkt->header.request_id = req_id;
  struct cam_cmd_buf_desc *buf_desc = (struct cam_cmd_buf_desc *)&pkt->payload;

  buf_desc[0].size = buf_desc[0].length = sizeof(struct i2c_rdwr_header) + len*sizeof(struct i2c_random_wr_payload);
  buf_desc[0].type = CAM_CMD_BUF_I2C;

  struct cam_cmd_i2c_random_wr *i2c_random_wr = (struct cam_cmd_i2c_random_wr *)alloc_w_mmu_hdl(multi_cam_state->video0_fd, buf_desc[0].size, (uint32_t*)&buf_desc[0].mem_handle);
  i2c_random_wr->header.count = len;
  i2c_random_wr->header.op_code = 1;
  i2c_random_wr->header.cmd_type = CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR;
  i2c_random_wr->header.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  i2c_random_wr->header.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
  memcpy(i2c_random_wr->random_wr_payload, dat, len*sizeof(struct i2c_random_wr_payload));

  int ret = device_config(actuator_fd, session_handle, sensor_dev_handle, cam_packet_handle);
  assert(ret == 0);

  munmap(i2c_random_wr, buf_desc[0].size);
  release_fd(multi_cam_state->video0_fd, buf_desc[0].mem_handle);
  munmap(pkt, size);
  release_fd(multi_cam_state->video0_fd, cam_packet_handle);
}

static cam_cmd_power *power_set_wait(cam_cmd_power *power, int16_t delay_ms) {
  cam_cmd_unconditional_wait *unconditional_wait = (cam_cmd_unconditional_wait *)((char *)power + (sizeof(struct cam_cmd_power) + (power->count - 1) * sizeof(struct cam_power_settings)));
  unconditional_wait->cmd_type = CAMERA_SENSOR_CMD_TYPE_WAIT;
  unconditional_wait->delay = delay_ms;
  unconditional_wait->op_code = CAMERA_SENSOR_WAIT_OP_SW_UCND;
  return (struct cam_cmd_power *)(unconditional_wait + 1);
};

void CameraState::sensors_init() {
  int video0_fd = multi_cam_state->video0_fd;
  uint32_t cam_packet_handle = 0;
  int size = sizeof(struct cam_packet)+sizeof(struct cam_cmd_buf_desc)*2;
  struct cam_packet *pkt = (struct cam_packet *)alloc_w_mmu_hdl(video0_fd, size, &cam_packet_handle);
  pkt->num_cmd_buf = 2;
  pkt->kmd_cmd_buf_index = -1;
  pkt->header.op_code = 0x1000003;
  pkt->header.size = size;
  struct cam_cmd_buf_desc *buf_desc = (struct cam_cmd_buf_desc *)&pkt->payload;

  buf_desc[0].size = buf_desc[0].length = sizeof(struct cam_cmd_i2c_info) + sizeof(struct cam_cmd_probe);
  buf_desc[0].type = CAM_CMD_BUF_LEGACY;
  struct cam_cmd_i2c_info *i2c_info = (struct cam_cmd_i2c_info *)alloc_w_mmu_hdl(video0_fd, buf_desc[0].size, (uint32_t*)&buf_desc[0].mem_handle);
  auto probe = (struct cam_cmd_probe *)(i2c_info + 1);

  switch (camera_num) {
    case 0:
      // port 0
      i2c_info->slave_addr = 0x34;
      probe->camera_id = 0;
      probe->reg_addr = 0x0;
      probe->expected_data = 0x0;
      break;
    case 1:
      // port 1
      i2c_info->slave_addr = 0x20;
      probe->camera_id = 1;
      probe->reg_addr = 0x16;
      probe->expected_data = 0x355;
      break;
    case 2:
      // port 2
      i2c_info->slave_addr = 0x34;
      probe->camera_id = 2;
      probe->reg_addr = 0x16;
      probe->expected_data = 0x355;
      break;
  }

  // 0(I2C_STANDARD_MODE) = 100khz, 1(I2C_FAST_MODE) = 400khz
  //i2c_info->i2c_freq_mode = I2C_STANDARD_MODE;
  i2c_info->i2c_freq_mode = I2C_FAST_MODE;
  i2c_info->cmd_type = CAMERA_SENSOR_CMD_TYPE_I2C_INFO;

  probe->data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  probe->addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
  probe->op_code = 3;   // don't care?
  probe->cmd_type = CAMERA_SENSOR_CMD_TYPE_PROBE;
  probe->data_mask = 0;

  //buf_desc[1].size = buf_desc[1].length = 148;
  buf_desc[1].size = buf_desc[1].length = 196;
  buf_desc[1].type = CAM_CMD_BUF_I2C;
  struct cam_cmd_power *power_settings = (struct cam_cmd_power *)alloc_w_mmu_hdl(video0_fd, buf_desc[1].size, (uint32_t*)&buf_desc[1].mem_handle);
  memset(power_settings, 0, buf_desc[1].size);
  // 7750
  /*power->count = 2;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_UP;
  power->power_settings[0].power_seq_type = 2;
  power->power_settings[1].power_seq_type = 8;
  power = (void*)power + (sizeof(struct cam_cmd_power) + (power->count-1)*sizeof(struct cam_power_settings));*/

  // 885a
  struct cam_cmd_power *power = power_settings;
  power->count = 4;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_UP;
  power->power_settings[0].power_seq_type = 3; // clock??
  power->power_settings[1].power_seq_type = 1; // analog
  power->power_settings[2].power_seq_type = 2; // digital
  power->power_settings[3].power_seq_type = 8; // reset low
  power = power_set_wait(power, 5);

  // set clock
  power->count = 1;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_UP;
  power->power_settings[0].power_seq_type = 0;
  power->power_settings[0].config_val_low = 24000000; //TODO // 19200000; //Hz
  power = power_set_wait(power, 10);

  // 8,1 is this reset?
  power->count = 1;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_UP;
  power->power_settings[0].power_seq_type = 8;
  power->power_settings[0].config_val_low = 1;
  power = power_set_wait(power, 100);

  // probe happens here

  // disable clock
  power->count = 1;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_DOWN;
  power->power_settings[0].power_seq_type = 0;
  power->power_settings[0].config_val_low = 0;
  power = power_set_wait(power, 1);

  // reset high
  power->count = 1;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_DOWN;
  power->power_settings[0].power_seq_type = 8;
  power->power_settings[0].config_val_low = 1;
  power = power_set_wait(power, 1);

  // reset low
  power->count = 1;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_DOWN;
  power->power_settings[0].power_seq_type = 8;
  power->power_settings[0].config_val_low = 0;
  power = power_set_wait(power, 1);

  // 7750
  /*power->count = 1;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_DOWN;
  power->power_settings[0].power_seq_type = 2;
  power = (void*)power + (sizeof(struct cam_cmd_power) + (power->count-1)*sizeof(struct cam_power_settings));*/

  // 885a
  power->count = 3;
  power->cmd_type = CAMERA_SENSOR_CMD_TYPE_PWR_DOWN;
  power->power_settings[0].power_seq_type = 2;
  power->power_settings[1].power_seq_type = 1;
  power->power_settings[2].power_seq_type = 3;

  LOGD("probing the sensor");
  int ret = do_cam_control(sensor_fd, CAM_SENSOR_PROBE_CMD, (void *)(uintptr_t)cam_packet_handle, 0);
  assert(ret == 0);

  munmap(i2c_info, buf_desc[0].size);
  release_fd(video0_fd, buf_desc[0].mem_handle);
  munmap(power_settings, buf_desc[1].size);
  release_fd(video0_fd, buf_desc[1].mem_handle);
  munmap(pkt, size);
  release_fd(video0_fd, cam_packet_handle);
}

void CameraState::config_isp(int io_mem_handle, int fence, int request_id, int buf0_mem_handle, int buf0_offset) {
  uint32_t cam_packet_handle = 0;
  int size = sizeof(struct cam_packet)+sizeof(struct cam_cmd_buf_desc)*5;
  if (io_mem_handle != 0) {
#ifdef EXTRA_IO_CFGS
    size += (sizeof(struct cam_buf_io_cfg) * 11);
#else
    size += sizeof(struct cam_buf_io_cfg);
#endif
  }
  struct cam_packet *pkt = (struct cam_packet *)alloc_w_mmu_hdl(multi_cam_state->video0_fd, size, &cam_packet_handle);
  pkt->num_cmd_buf = 5;
  pkt->kmd_cmd_buf_index = 0;

  if (io_mem_handle != 0) {
    pkt->io_configs_offset = sizeof(struct cam_cmd_buf_desc)*5;
#ifdef EXTRA_IO_CFGS
    pkt->num_io_configs = 11;
#else
    pkt->num_io_configs = 1;
#endif
  }

  if (io_mem_handle != 0) {
    pkt->header.op_code = 0xf000001;
    pkt->header.request_id = request_id;
  } else {
    pkt->header.op_code = 0xf000000;
  }
  pkt->header.size = size;
  struct cam_cmd_buf_desc *buf_desc = (struct cam_cmd_buf_desc *)&pkt->payload;
  struct cam_buf_io_cfg *io_cfg = (struct cam_buf_io_cfg *)((char*)&pkt->payload + pkt->io_configs_offset);

  // TODO: support MMU
  buf_desc[0].size = 68332;
  buf_desc[0].length = 0;
  buf_desc[0].type = CAM_CMD_BUF_DIRECT;
  buf_desc[0].meta_data = 3;
  buf_desc[0].mem_handle = buf0_mem_handle;
  buf_desc[0].offset = buf0_offset;
#ifndef USE_RAW10
  unsigned char* isp_prog;
  if (buf0_offset == 0) {
    buf_desc[0].length = sizeof(isp_prog1)-1;
    isp_prog = isp_prog1;
  } else {
    buf_desc[0].length = sizeof(isp_prog2)-1;
    isp_prog = isp_prog2;
  }
  LOGD("isp program length %d\n", buf_desc[0].length);
  memcpy((unsigned char*)this->buf0_ptr + buf0_offset, isp_prog, buf_desc[0].length);
  pkt->kmd_cmd_buf_offset = buf_desc[0].length;
#endif

  // parsed by cam_isp_packet_generic_blob_handler
  struct isp_packet {
    uint32_t type_0;
    cam_isp_resource_hfr_config resource_hfr;

    uint32_t type_1;
    cam_isp_clock_config clock;
    uint64_t extra_rdi_hz[3];

    uint32_t type_2;
    cam_isp_bw_config bw;
    struct cam_isp_bw_vote extra_rdi_vote[3];
  } __attribute__((packed)) tmp;
  memset(&tmp, 0, sizeof(tmp));

  tmp.type_0 = CAM_ISP_GENERIC_BLOB_TYPE_HFR_CONFIG;
  tmp.type_0 |= sizeof(cam_isp_resource_hfr_config) << 8;
  static_assert(sizeof(cam_isp_resource_hfr_config) == 0x20);
  tmp.resource_hfr = {
    .num_ports = 1,  // 10 for YUV (but I don't think we need them)
    .port_hfr_config[0] = {
      .resource_type = cam_res_type,
      .subsample_pattern = 1,
      .subsample_period = 0,
      .framedrop_pattern = 1,
      .framedrop_period = 0,
    }};

  tmp.type_1 = CAM_ISP_GENERIC_BLOB_TYPE_CLOCK_CONFIG;
  tmp.type_1 |= (sizeof(cam_isp_clock_config) + sizeof(tmp.extra_rdi_hz)) << 8;
  static_assert((sizeof(cam_isp_clock_config) + sizeof(tmp.extra_rdi_hz)) == 0x38);
  tmp.clock = {
    .usage_type = 1, // dual mode
    .num_rdi = 4,
    .left_pix_hz = 404000000,
    .right_pix_hz = 404000000,
    .rdi_hz[0] = 404000000,
  };


  tmp.type_2 = CAM_ISP_GENERIC_BLOB_TYPE_BW_CONFIG;
  tmp.type_2 |= (sizeof(cam_isp_bw_config) + sizeof(tmp.extra_rdi_vote)) << 8;
  static_assert((sizeof(cam_isp_bw_config) + sizeof(tmp.extra_rdi_vote)) == 0x98);
  tmp.bw = {
    .usage_type = 1, // dual mode
    .num_rdi = 4,
    .left_pix_vote = {
      .resource_id = 0,
      .cam_bw_bps = pkt->header.op_code == 0xf000001 ? left_bw_bps : left_bw_bps_0,
      .ext_bw_bps = pkt->header.op_code == 0xf000001 ? left_bw_bps : left_bw_bps_0,
    },
    .right_pix_vote = {
      .resource_id = 0,
      .cam_bw_bps = pkt->header.op_code == 0xf000001 ? right_bw_bps : right_bw_bps_0,
      .ext_bw_bps = pkt->header.op_code == 0xf000001 ? right_bw_bps : right_bw_bps_0,
    },
    .rdi_vote[0] = {
      .resource_id = 0,
      .cam_bw_bps = pkt->header.op_code == 0xf000001 ? extra_bw_bps : extra_bw_bps_0,
      .ext_bw_bps = pkt->header.op_code == 0xf000001 ? extra_bw_bps : extra_bw_bps_0,
    },
  };


  static_assert(offsetof(struct isp_packet, type_2) == 0x60);

  buf_desc[1].size = sizeof(tmp);
  buf_desc[1].offset = io_mem_handle != 0 ? 0x60 : 0;
  buf_desc[1].length = buf_desc[1].size - buf_desc[1].offset;
  buf_desc[1].type = CAM_CMD_BUF_GENERIC;
  buf_desc[1].meta_data = CAM_ISP_PACKET_META_GENERIC_BLOB_COMMON;
  uint32_t *buf2 = (uint32_t *)alloc_w_mmu_hdl(multi_cam_state->video0_fd, buf_desc[1].size, (uint32_t*)&buf_desc[1].mem_handle, 0x20);
  memcpy(buf2, &tmp, sizeof(tmp));

  if(pkt->header.op_code == 0xf000001) {
    buf_desc[2].mem_handle = meta_right_buf_handle;
    buf_desc[2].size = meta_right_buf_size;
    buf_desc[2].length = sizeof(meta_right) - 1;
  } else {
    buf_desc[2].mem_handle = meta_right_buf_handle0;
    buf_desc[2].size = meta_right_buf_size0;
    buf_desc[2].length = sizeof(meta_right_0) - 1;
  }
  buf_desc[2].offset = 0;
  buf_desc[2].type = CAM_CMD_BUF_DIRECT;
  buf_desc[2].meta_data = CAM_ISP_PACKET_META_RIGHT;

  if(pkt->header.op_code == 0xf000001) {
    buf_desc[3].mem_handle = meta_left_buf_handle;
    buf_desc[3].length = sizeof(meta_left) - 1;
    buf_desc[3].size = meta_left_buf_size;
  } else {
    buf_desc[3].mem_handle = meta_left_buf_handle0;
    buf_desc[3].length = sizeof(meta_left_0) - 1;
    buf_desc[3].size = meta_left_buf_size0;
  }
  buf_desc[3].offset = 0;
  buf_desc[3].type = CAM_CMD_BUF_DIRECT;
  buf_desc[3].meta_data = CAM_ISP_PACKET_META_LEFT;

  buf_desc[4].mem_handle = meta_dual_buf_handle;
  buf_desc[4].size = meta_dual_buf_size;
  buf_desc[4].offset = 0;
  if(pkt->header.op_code == 0xf000001)
    buf_desc[4].length = sizeof(meta_dual) - 1;
  else
    buf_desc[4].length = 0;
  buf_desc[4].type = CAM_CMD_BUF_GENERIC;
  buf_desc[4].meta_data = CAM_ISP_PACKET_META_DUAL_CONFIG;

  if (io_mem_handle != 0) {
    io_cfg[0].mem_handle[0] = io_mem_handle;
#ifndef USE_RAW10
    io_cfg[0].mem_handle[1] = io_mem_handle;
#endif
	io_cfg[0].offsets[0] = 0;
	io_cfg[0].planes[0] = (struct cam_plane_cfg){
#ifndef USE_RAW10
	 .width = cam_frame_y_width,
	 .height = cam_frame_y_height,
	 .plane_stride = cam_frame_y_plane_stride,
	 .slice_height = cam_frame_y_slice_height,
#else
	 .width = cam_frame_width,
	 .height = cam_frame_height,
	 .plane_stride = cam_frame_plane_stride,
	 .slice_height = cam_frame_slice_height,
#endif
	 .meta_stride = 0x0,
	 .meta_size = 0x0,
	 .meta_offset = 0x0,
	 .packer_config = 0x0,  // 0xb for YUV
	 .mode_config = 0x0,    // 0x9ef for YUV
	 .tile_config = 0x0,
	 .h_init = 0x0,
	 .v_init = 0x0,
	};
#ifndef USE_RAW10
	io_cfg[0].offsets[1] = cam_frame_y_plane_stride * cam_frame_y_slice_height;
	io_cfg[0].planes[1] = (struct cam_plane_cfg){
	 .width = cam_frame_uv_width,
	 .height = cam_frame_uv_height,
	 .plane_stride = cam_frame_uv_plane_stride,
	 .slice_height = cam_frame_uv_slice_height,
	 .meta_stride = 0x0,
	 .meta_size = 0x0,
	 .meta_offset = 0x0,
	 .packer_config = 0x0,  // 0xb for YUV
	 .mode_config = 0x0,    // 0x9ef for YUV
	 .tile_config = 0x0,
	 .h_init = 0x0,
	 .v_init = 0x0,
	};
#endif
    io_cfg[0].format = cam_format;
    io_cfg[0].color_space = cam_color_space;
    io_cfg[0].color_pattern = cam_color_pattern;
    io_cfg[0].bpp = cam_bpp;
    io_cfg[0].resource_type = cam_res_type;
    io_cfg[0].fence = fence;
    io_cfg[0].direction = CAM_BUF_OUTPUT;
    io_cfg[0].subsample_pattern = 0x1;
    io_cfg[0].framedrop_pattern = 0x1;
  }

#ifdef EXTRA_IO_CFGS
io_cfg[1].format = 36; //UNKNOWN
io_cfg[1].color_space = 1;
io_cfg[1].color_pattern = 0;
io_cfg[1].bpp = 0;
io_cfg[1].resource_type = 0x3001; //CAM_ISP_IFE_OUT_RES_DS4
io_cfg[1].fence = extra_sync_objs[0];
io_cfg[1].direction = 2;
io_cfg[1].subsample_pattern = 1;
io_cfg[1].framedrop_pattern = 1;
io_cfg[1].mem_handle[0] = extra_io_mem_handles[0];
io_cfg[1].offsets[0] = 0;
io_cfg[1].planes[0] = (struct cam_plane_cfg){
  .width = 1920,
  .height = 136,
  .plane_stride = 1920,
  .slice_height = 136,
  .meta_stride = 0,
  .meta_size = 0,
  .meta_offset = 0,
  .packer_config = 0x0,
  .mode_config = 0x0,
  .tile_config = 0x0,
  .h_init = 0,
  .v_init = 0,
};
io_cfg[2].format = 36; //UNKNOWN
io_cfg[2].color_space = 1;
io_cfg[2].color_pattern = 0;
io_cfg[2].bpp = 0;
io_cfg[2].resource_type = 0x3002; //CAM_ISP_IFE_OUT_RES_DS16
io_cfg[2].fence = extra_sync_objs[1];
io_cfg[2].direction = 2;
io_cfg[2].subsample_pattern = 1;
io_cfg[2].framedrop_pattern = 1;
io_cfg[2].mem_handle[0] = extra_io_mem_handles[1];
io_cfg[2].offsets[0] = 0;
io_cfg[2].planes[0] = (struct cam_plane_cfg){
  .width = 480,
  .height = 34,
  .plane_stride = 576,
  .slice_height = 36,
  .meta_stride = 0,
  .meta_size = 0,
  .meta_offset = 0,
  .packer_config = 0x0,
  .mode_config = 0x0,
  .tile_config = 0x0,
  .h_init = 0,
  .v_init = 0,
};
io_cfg[3].format = 20; //UNKNOWN
io_cfg[3].color_space = 1;
io_cfg[3].color_pattern = 0;
io_cfg[3].bpp = 0;
io_cfg[3].resource_type = 0x300A; //CAM_ISP_IFE_OUT_RES_STATS_HDR_BE
io_cfg[3].fence = extra_sync_objs[2];
io_cfg[3].direction = 2;
io_cfg[3].subsample_pattern = 1;
io_cfg[3].framedrop_pattern = 1;
io_cfg[3].mem_handle[0] = extra_io_mem_handles[2];
io_cfg[3].offsets[0] = 0;
io_cfg[3].planes[0] = (struct cam_plane_cfg){
  .width = 294912,
  .height = 1,
  .plane_stride = 294912,
  .slice_height = 1,
  .meta_stride = 0,
  .meta_size = 0,
  .meta_offset = 0,
  .packer_config = 0x0,
  .mode_config = 0x0,
  .tile_config = 0x0,
  .h_init = 0,
  .v_init = 0,
};
io_cfg[4].format = 20; //UNKNOWN
io_cfg[4].color_space = 1;
io_cfg[4].color_pattern = 0;
io_cfg[4].bpp = 0;
io_cfg[4].resource_type = 0x300E; //CAM_ISP_IFE_OUT_RES_STATS_AWB_BG
io_cfg[4].fence = extra_sync_objs[3];
io_cfg[4].direction = 2;
io_cfg[4].subsample_pattern = 1;
io_cfg[4].framedrop_pattern = 1;
io_cfg[4].mem_handle[0] = extra_io_mem_handles[3];
io_cfg[4].offsets[0] = 0;
io_cfg[4].planes[0] = (struct cam_plane_cfg){
  .width = 1382400,
  .height = 1,
  .plane_stride = 1382400,
  .slice_height = 1,
  .meta_stride = 0,
  .meta_size = 0,
  .meta_offset = 0,
  .packer_config = 0x0,
  .mode_config = 0x0,
  .tile_config = 0x0,
  .h_init = 0,
  .v_init = 0,
};
io_cfg[5].format = 20; //UNKNOWN
io_cfg[5].color_space = 1;
io_cfg[5].color_pattern = 0;
io_cfg[5].bpp = 0;
io_cfg[5].resource_type = 0x300B; //CAM_ISP_IFE_OUT_RES_STATS_HDR_BHIST
io_cfg[5].fence = extra_sync_objs[4];
io_cfg[5].direction = 2;
io_cfg[5].subsample_pattern = 1;
io_cfg[5].framedrop_pattern = 1;
io_cfg[5].mem_handle[0] = extra_io_mem_handles[4];
io_cfg[5].offsets[0] = 0;
io_cfg[5].planes[0] = (struct cam_plane_cfg){
  .width = 6144,
  .height = 1,
  .plane_stride = 6144,
  .slice_height = 1,
  .meta_stride = 0,
  .meta_size = 0,
  .meta_offset = 0,
  .packer_config = 0x0,
  .mode_config = 0x0,
  .tile_config = 0x0,
  .h_init = 0,
  .v_init = 0,
};
io_cfg[6].format = 20; //UNKNOWN
io_cfg[6].color_space = 1;
io_cfg[6].color_pattern = 0;
io_cfg[6].bpp = 0;
io_cfg[6].resource_type = 0x300F; //CAM_ISP_IFE_OUT_RES_STATS_BHIST
io_cfg[6].fence = extra_sync_objs[5];
io_cfg[6].direction = 2;
io_cfg[6].subsample_pattern = 1;
io_cfg[6].framedrop_pattern = 1;
io_cfg[6].mem_handle[0] = extra_io_mem_handles[5];
io_cfg[6].offsets[0] = 0;
io_cfg[6].planes[0] = (struct cam_plane_cfg){
  .width = 32768,
  .height = 1,
  .plane_stride = 32768,
  .slice_height = 1,
  .meta_stride = 0,
  .meta_size = 0,
  .meta_offset = 0,
  .packer_config = 0x0,
  .mode_config = 0x0,
  .tile_config = 0x0,
  .h_init = 0,
  .v_init = 0,
};
io_cfg[7].format = 20; //UNKNOWN
io_cfg[7].color_space = 1;
io_cfg[7].color_pattern = 0;
io_cfg[7].bpp = 0;
io_cfg[7].resource_type = 0x300C; //CAM_ISP_IFE_OUT_RES_STATS_TL_BG
io_cfg[7].fence = extra_sync_objs[6];
io_cfg[7].direction = 2;
io_cfg[7].subsample_pattern = 1;
io_cfg[7].framedrop_pattern = 1;
io_cfg[7].mem_handle[0] = extra_io_mem_handles[6];
io_cfg[7].offsets[0] = 0;
io_cfg[7].planes[0] = (struct cam_plane_cfg){
  .width = 294912,
  .height = 1,
  .plane_stride = 294912,
  .slice_height = 1,
  .meta_stride = 0,
  .meta_size = 0,
  .meta_offset = 0,
  .packer_config = 0x0,
  .mode_config = 0x0,
  .tile_config = 0x0,
  .h_init = 0,
  .v_init = 0,
};
io_cfg[8].format = 20; //UNKNOWN
io_cfg[8].color_space = 1;
io_cfg[8].color_pattern = 0;
io_cfg[8].bpp = 0;
io_cfg[8].resource_type = 0x300D; //CAM_ISP_IFE_OUT_RES_STATS_BF
io_cfg[8].fence = extra_sync_objs[7];
io_cfg[8].direction = 2;
io_cfg[8].subsample_pattern = 1;
io_cfg[8].framedrop_pattern = 1;
io_cfg[8].mem_handle[0] = extra_io_mem_handles[7];
io_cfg[8].offsets[0] = 0;
io_cfg[8].planes[0] = (struct cam_plane_cfg){
  .width = 11536,
  .height = 1,
  .plane_stride = 11536,
  .slice_height = 1,
  .meta_stride = 0,
  .meta_size = 0,
  .meta_offset = 0,
  .packer_config = 0x0,
  .mode_config = 0x0,
  .tile_config = 0x0,
  .h_init = 0,
  .v_init = 0,
};
io_cfg[9].format = 20; //UNKNOWN
io_cfg[9].color_space = 1;
io_cfg[9].color_pattern = 0;
io_cfg[9].bpp = 0;
io_cfg[9].resource_type = 0x3010; //CAM_ISP_IFE_OUT_RES_STATS_RS
io_cfg[9].fence = extra_sync_objs[8];
io_cfg[9].direction = 2;
io_cfg[9].subsample_pattern = 1;
io_cfg[9].framedrop_pattern = 1;
io_cfg[9].mem_handle[0] = extra_io_mem_handles[8];
io_cfg[9].offsets[0] = 0;
io_cfg[9].planes[0] = (struct cam_plane_cfg){
  .width = 65536,
  .height = 1,
  .plane_stride = 65536,
  .slice_height = 1,
  .meta_stride = 0,
  .meta_size = 0,
  .meta_offset = 0,
  .packer_config = 0x0,
  .mode_config = 0x0,
  .tile_config = 0x0,
  .h_init = 0,
  .v_init = 0,
};

io_cfg[10].format = 31; //CAM_FORMAT_NV21
io_cfg[10].color_space = 1;
io_cfg[101].color_pattern = 0;
io_cfg[10].bpp = 0;
io_cfg[10].resource_type = 0x3004; //CAM_ISP_IFE_OUT_RES_FD
io_cfg[10].fence = extra_sync_objs[9];
io_cfg[10].direction = 2;
io_cfg[10].subsample_pattern = 1;
io_cfg[10].framedrop_pattern = 1;
io_cfg[10].mem_handle[0] = extra_io_mem_handles[9];
io_cfg[10].offsets[0] = 0;
io_cfg[10].mem_handle[1] = extra_io_mem_handles[9];
io_cfg[10].offsets[1] = 307200;
io_cfg[10].planes[0] = (struct cam_plane_cfg){
  .width = 640,
  .height = 480,
  .plane_stride = 640,
  .slice_height = 480,
  .meta_stride = 0,
  .meta_size = 0,
  .meta_offset = 0,
  .packer_config = 0x0,
  .mode_config = 0x0,
  .tile_config = 0x0,
  .h_init = 0,
  .v_init = 0,
};
io_cfg[10].planes[1] = (struct cam_plane_cfg){
  .width = 640,
  .height = 240,
  .plane_stride = 640,
  .slice_height = 240,
  .meta_stride = 0,
  .meta_size = 0,
  .meta_offset = 0,
  .packer_config = 0x0,
  .mode_config = 0x0,
  .tile_config = 0x0,
  .h_init = 0,
  .v_init = 0,
};
#endif

  int ret = device_config(multi_cam_state->isp_fd, session_handle, isp_dev_handle, cam_packet_handle);
  assert(ret == 0);
  if (ret != 0) {
    printf("ISP CONFIG FAILED\n");
  }

  munmap(buf2, buf_desc[1].size);
  release_fd(multi_cam_state->video0_fd, buf_desc[1].mem_handle);
  // release_fd(multi_cam_state->video0_fd, buf_desc[0].mem_handle);
  munmap(pkt, size);
  release_fd(multi_cam_state->video0_fd, cam_packet_handle);
}

void CameraState::enqueue_buffer() {
  int ret;
  int request_id = request_id_last++;
  struct cam_req_mgr_sched_request req_mgr_sched_request = {0};
  req_mgr_sched_request.session_hdl = session_handle;
  req_mgr_sched_request.link_hdl = link_handle;
  req_mgr_sched_request.req_id = request_id;
  ret = do_cam_control(multi_cam_state->video0_fd, CAM_REQ_MGR_SCHED_REQ, &req_mgr_sched_request, sizeof(req_mgr_sched_request));
  assert(ret == 0);
  // create output fence
  struct cam_sync_info sync_create = {0};
  strcpy(sync_create.name, "NodeOutputPortFence");
  ret = do_cam_control(multi_cam_state->video1_fd, CAM_SYNC_CREATE, &sync_create, sizeof(sync_create));
  assert(ret == 0);
  // Without payload registration, cam sync V4L2 event will not work.
  struct cam_sync_userpayload_info pli = {0};
  pli.sync_obj = sync_create.sync_obj;
  ret = do_cam_control(multi_cam_state->video1_fd, CAM_SYNC_REGISTER_PAYLOAD, &pli, sizeof(pli));
  //LOGD("Register payload: %d %d", ret, sync_objs[i]);

  // poke sensor
  sensors_poke(request_id);
  // LOGD("Poked sensor");
  if(!focus_set) {
    struct i2c_random_wr_payload wp[] = {{0x84, 0x0ed7}};
    actuator_i2c(&wp[0], 1, request_id);
  } else {
    uint32_t cam_packet_handle = 0;
    uint32_t size = sizeof(struct cam_packet);
    struct cam_packet* pkt = (struct cam_packet*)alloc_w_mmu_hdl(multi_cam_state->video0_fd, size, &cam_packet_handle);
    pkt->num_cmd_buf = 1;
    pkt->kmd_cmd_buf_index = -1;
    pkt->header.size = size;
    pkt->header.op_code = CAM_PKT_NOP_OPCODE;
    pkt->header.request_id = request_id;
    int ret_ = device_config(actuator_fd, session_handle, actuator_dev_handle, cam_packet_handle);
    assert(ret_ == 0);
    munmap(pkt, size);
    release_fd(multi_cam_state->video0_fd, cam_packet_handle);
  }

#ifdef DEBUG_CAM_SYNC_AND_REQ_MGR
  LOGD("Insert to map: req_id %d, sync_obj %d", request_id, sync_create.sync_obj);
#endif

  int frame_buf_idx = frame_buf_idx_list.front();
  frame_buf_idx_list.pop_front();
  req_info_map.insert({request_id, CameraRequestInfo(frame_buf_idx, sync_create.sync_obj)});
  sync_obj_to_req_id.insert({sync_create.sync_obj, request_id});
  // push the buffer
  config_isp(buf_handle[frame_buf_idx], sync_create.sync_obj, request_id, buf0_handle, 68332*(frame_buf_idx+1));
}

// ******************* camera *******************

void CameraState::camera_init(MultiCameraState *multi_cam_state_, VisionIpcServer * v, int camera_id, int camera_num_, unsigned int fps, cl_device_id device_id, cl_context ctx, VisionStreamType rgb_type, VisionStreamType yuv_type) {
  LOGD("camera init %d", camera_num);
  multi_cam_state = multi_cam_state_;
  assert(camera_id < std::size(cameras_supported));
  ci = cameras_supported[camera_id];
  assert(ci.frame_width != 0);

  camera_num = camera_num_;
  request_id_last = 1;
  for(int i = 0; i < FRAME_BUF_COUNT;i++)
    frame_buf_idx_list.push_back(i);

  min_ev = EXPOSURE_TIME_MIN * sensor_analog_gains[ANALOG_GAIN_MIN_IDX];
  max_ev = EXPOSURE_TIME_MAX * sensor_analog_gains[ANALOG_GAIN_MAX_IDX] * DC_GAIN;
  target_grey_fraction = 0.3;

  frame_length = 0xC38; // {0x340, 0xC},  {0x341, 0x38},
  dc_gain_enabled = false;
  gain_idx = ANALOG_GAIN_REC_IDX;
  exposure_time = 5;
  cur_ev[0] = cur_ev[1] = cur_ev[2] = (dc_gain_enabled ? DC_GAIN : 1) * sensor_analog_gains[gain_idx] * exposure_time;

  buf.init(device_id, ctx, this, v, FRAME_BUF_COUNT, rgb_type, yuv_type);
}

void CameraState::camera_open() {
  sensor_fd = open_v4l_by_name_and_index("cam-sensor-driver", camera_num);
  assert(sensor_fd >= 0);
  LOGD("opened sensor for %d", camera_num);

  // probe the sensor
  LOGD("-- Probing sensor %d", camera_num);
  sensors_init();

  // create session
  struct cam_req_mgr_session_info session_info = {};
  LOGD("CAM_REQ_MGR_CREATE_SESSION");
  int ret = do_cam_control(multi_cam_state->video0_fd, CAM_REQ_MGR_CREATE_SESSION, &session_info, sizeof(session_info));
  LOGD("get session: %d 0x%X", ret, session_info.session_hdl);
  session_handle = session_info.session_hdl;

  // access the sensor
  LOGD("-- Accessing sensor");
  auto sensor_dev_handle_ = device_acquire(sensor_fd, session_handle, nullptr);
  assert(sensor_dev_handle_);
  sensor_dev_handle = *sensor_dev_handle_;
  LOGD("acquire sensor dev");

  struct cam_isp_in_port_info in_port_info = {
      .res_type = (uint32_t[]){CAM_ISP_IFE_IN_RES_PHY_0, CAM_ISP_IFE_IN_RES_PHY_1, CAM_ISP_IFE_IN_RES_PHY_2}[camera_num],

      .lane_type = CAM_ISP_LANE_TYPE_DPHY,
      .lane_num = 4,
      .lane_cfg = 0x3210,

      .vc = 0x0,
      //.dt = 0x2C,  // CSI_RAW12
      .dt = 0x2B,  //CSI_RAW10
      .format = CAM_FORMAT_MIPI_RAW_10,

      .test_pattern = 0x2,  // 0x3?
#ifdef USE_RAW10
      .usage_type = 0x0,
#else
      .usage_type = 0x1,
#endif

      .left_start = 0,
      .left_stop = 2279,
      .left_width = 2280,

      .right_start = 1868,
      .right_stop = IN_PORT_FRAME_WIDTH - 1,
      .right_width = 2164,

      .line_start = 0,
      .line_stop = IN_PORT_FRAME_HEIGHT - 1,
      .height = IN_PORT_FRAME_HEIGHT,

      .pixel_clk = 0x0,
      .batch_size = 0x0,
      .dsp_mode = CAM_ISP_DSP_MODE_NONE,
      .hbi_cnt = 0x0,

      .num_out_res = 0x1,
      .data[0] = (struct cam_isp_out_port_info){
          .res_type = cam_res_type,
          .format = cam_format_out_port,
          .width = FRAME_WIDTH,
          .height = FRAME_HEIGHT,
          .comp_grp_id = cam_comp_grp_id,
		  .split_point = FRAME_WIDTH/2,
		  .secure_mode = 0x0,
      },
  };

#if false
  struct {
	struct cam_isp_in_port_info in_port_info;
	struct cam_isp_out_port_info    data[11];
  } tmp = {
	.in_port_info = in_port_info,
	.data[0] = (struct cam_isp_out_port_info){
	  .res_type = CAM_ISP_IFE_OUT_RES_DS4,
	  .format = 36,
	  .width = 672,
	  .height = 378,
	  .comp_grp_id = 4,
	  .split_point = 336,
	  .secure_mode = 0,
	},
	.data[1] = (struct cam_isp_out_port_info){
	  .res_type = CAM_ISP_IFE_OUT_RES_DS16,
	  .format = 36,
	  .width = 168,
	  .height = 96,
	  .comp_grp_id = 4,
	  .split_point = 84,
	  .secure_mode = 0,
	},
	.data[2] = (struct cam_isp_out_port_info){
	  .res_type = CAM_ISP_IFE_OUT_RES_STATS_HDR_BE,
	  .format = 19,
	  .width = 294912,
	  .height = 1,
	  .comp_grp_id = 2,
	  .split_point = 147456,
	  .secure_mode = 0,
	},
	.data[3] = (struct cam_isp_out_port_info){
	  .res_type = CAM_ISP_IFE_OUT_RES_STATS_AWB_BG,
	  .format = 19,
	  .width = 1382400,
	  .height = 1,
	  .comp_grp_id = 2,
	  .split_point = 691200,
	  .secure_mode = 0,
	},
	.data[4] = (struct cam_isp_out_port_info){
	  .res_type = CAM_ISP_IFE_OUT_RES_STATS_HDR_BHIST,
	  .format = 19,
	  .width = 6144,
	  .height = 1,
	  .comp_grp_id = 2,
	  .split_point = 3072,
	  .secure_mode = 0,
	},
	.data[5] = (struct cam_isp_out_port_info){
	  .res_type = CAM_ISP_IFE_OUT_RES_STATS_BHIST,
	  .format = 19,
	  .width = 32768,
	  .height = 1,
	  .comp_grp_id = 2,
	  .split_point = 16384,
	  .secure_mode = 0,
	},
	.data[6] = (struct cam_isp_out_port_info){
	  .res_type = CAM_ISP_IFE_OUT_RES_STATS_TL_BG,
	  .format = 19,
	  .width = 294912,
	  .height = 1,
	  .comp_grp_id = 2,
	  .split_point = 147456,
	  .secure_mode = 0,
	},
	.data[7] = (struct cam_isp_out_port_info){
	  .res_type = CAM_ISP_IFE_OUT_RES_STATS_BF,
	  .format = 19,
	  .width = 11536,
	  .height = 1,
	  .comp_grp_id = 3,
	  .split_point = 5768,
	  .secure_mode = 0,
	},
	.data[8] = (struct cam_isp_out_port_info){
	  .res_type = CAM_ISP_IFE_OUT_RES_STATS_RS,
	  .format = 17,
	  .width = 65536,
	  .height = 1,
	  .comp_grp_id = 2,
	  .split_point = 32768,
	  .secure_mode = 0,
	},
	.data[9] = (struct cam_isp_out_port_info){
	  .res_type = CAM_ISP_IFE_OUT_RES_FD,
	  .format = 32,
	  .width = 640,
	  .height = 360,
	  .comp_grp_id = 5,
	  .split_point = 320,
	  .secure_mode = 0,
	},
	.data[10] = (struct cam_isp_out_port_info){
	  .res_type = CAM_ISP_IFE_OUT_RES_RDI_2,
	  .format = 3,
	  .width = 4032,
	  .height = 3024,
	  .comp_grp_id = 0,
	  .split_point = 2016,
	  .secure_mode = 0,
	},
  };
  tmp.in_port_info.num_out_res = 12;
  struct cam_isp_resource isp_resource = {
      .resource_id = CAM_ISP_RES_ID_PORT,
      .handle_type = CAM_HANDLE_USER_POINTER,
      .res_hdl = (uint64_t)&tmp,
      .length = sizeof(tmp),
  };
#else
  struct cam_isp_resource isp_resource = {
      .resource_id = CAM_ISP_RES_ID_PORT,
      .handle_type = CAM_HANDLE_USER_POINTER,
      .res_hdl = (uint64_t)&in_port_info,
      .length = sizeof(in_port_info),
  };
#endif

  auto isp_dev_handle_ = device_acquire(multi_cam_state->isp_fd, session_handle, &isp_resource);
  assert(isp_dev_handle_);
  isp_dev_handle = *isp_dev_handle_;
  LOGD("acquire isp dev");

  csiphy_fd = open_v4l_by_name_and_index("cam-csiphy-driver", camera_num);
  assert(csiphy_fd >= 0);
  LOGD("opened csiphy for %d", camera_num);

  struct cam_csiphy_acquire_dev_info csiphy_acquire_dev_info = {.combo_mode = 0};
  auto csiphy_dev_handle_ = device_acquire(csiphy_fd, session_handle, &csiphy_acquire_dev_info);
  assert(csiphy_dev_handle_);
  csiphy_dev_handle = *csiphy_dev_handle_;
  LOGD("acquire csiphy dev");

  actuator_fd = open_v4l_by_name_and_index("cam-actuator-driver", camera_num);
  assert(actuator_fd >= 0);
  LOGD("opened actuator for %d", camera_num);

  struct cam_sensor_acquire_dev actuator_acquire_cmd = {.session_handle=(uint32_t)session_handle, .handle_type = CAM_HANDLE_USER_POINTER, .info_handle=0};
  ret = do_cam_control(actuator_fd, CAM_ACQUIRE_DEV, &actuator_acquire_cmd, sizeof(actuator_acquire_cmd));
  assert(ret == 0);
  actuator_dev_handle = actuator_acquire_cmd.device_handle;

  LOG("-- Config actuator");
  {
    uint32_t cam_packet_handle = 0;
    uint32_t size = sizeof(struct cam_packet) + sizeof(struct cam_cmd_buf_desc) * 3;
    struct cam_packet* pkt = (struct cam_packet*)alloc_w_mmu_hdl(multi_cam_state->video0_fd, size, &cam_packet_handle);
    pkt->num_cmd_buf = 3;
    pkt->kmd_cmd_buf_index = -1;
    pkt->header.size = size;
    pkt->header.op_code = 0x2000000;
    struct cam_cmd_buf_desc* buf_desc = (struct cam_cmd_buf_desc*)&pkt->payload;
    buf_desc[0].size = 4;
    buf_desc[0].length = 4;
    buf_desc[0].type = 9;
    buf_desc[0].offset = 0;
    buf_desc[0].meta_data = 0;
    uint8_t* ptr_0 = (uint8_t *)alloc_w_mmu_hdl(multi_cam_state->video0_fd, buf_desc[0].size, (uint32_t*)&buf_desc[0].mem_handle);
    static unsigned char actuator_cmd_0[] = "\xE4\x00\x03\x04";
    memcpy(ptr_0, actuator_cmd_0, 4);
    buf_desc[1].size = 36;
    buf_desc[1].length = 36;
    buf_desc[1].type = 9;
    buf_desc[1].offset = 0;
    buf_desc[1].meta_data = 0;
    uint8_t* ptr_1 = (uint8_t *)alloc_w_mmu_hdl(multi_cam_state->video0_fd, buf_desc[1].size, (uint32_t*)&buf_desc[1].mem_handle);
    static unsigned char actuator_cmd_1[] = "\x01\x00\x00\x02\x04\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x08\x00\x03\x09\x01\x00\x21\x03\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";
    memcpy(ptr_1, actuator_cmd_1, 36);
    buf_desc[2].size = 52;
    buf_desc[2].length = 52;
    buf_desc[2].type = 7;
    buf_desc[2].offset = 0;
    buf_desc[2].meta_data = 0;
    uint8_t* ptr_2 = (uint8_t *)alloc_w_mmu_hdl(multi_cam_state->video0_fd, buf_desc[2].size, (uint32_t*)&buf_desc[2].mem_handle);
    static unsigned char actuator_cmd_2[] = "\x01\x00\x01\x05\x01\x01\x00\x00\xE0\x00\x00\x00\x01\x00\x00\x00\x01\x01\x01\x09\x02\x00\x00\x00\xB3\x00\x00\x00\x00\x00\x00\x00\xFF\xFF\x00\x00\x01\x00\x01\x05\x02\x01\x00\x00\x84\x00\x00\x00\x00\x00\x00\x00";
    memcpy(ptr_2, actuator_cmd_2, 52);
    int ret_ = device_config(actuator_fd, session_handle, actuator_dev_handle, cam_packet_handle);
    assert(ret_ == 0);
    munmap(ptr_0, buf_desc[0].size);
    release_fd(multi_cam_state->video0_fd, buf_desc[0].mem_handle);
    munmap(ptr_1, buf_desc[1].size);
    release_fd(multi_cam_state->video0_fd, buf_desc[1].mem_handle);
    munmap(ptr_2, buf_desc[2].size);
    release_fd(multi_cam_state->video0_fd, buf_desc[2].mem_handle);
    munmap(pkt, size);
    release_fd(multi_cam_state->video0_fd, cam_packet_handle);
  }

  // config ISP
  this->buf0_ptr = alloc_w_mmu_hdl(multi_cam_state->video0_fd, 1025280, (uint32_t*)&buf0_handle, 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);

  LOGD("meta right len %u", sizeof(meta_right)-1);
  meta_right_buf_size = 68332;
  meta_right_buf_alloc_size = 1025280;
  void *meta_right_ptr = alloc_w_mmu_hdl(multi_cam_state->video0_fd, meta_right_buf_alloc_size, (uint32_t*)&meta_right_buf_handle, 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  memcpy((unsigned char*)meta_right_ptr, meta_right, sizeof(meta_right)-1);

  LOGD("meta right 0 len %u", sizeof(meta_right_0)-1);
  meta_right_buf_size0 = 68332;
  meta_right_buf_alloc_size0 = 1025280;
  void *meta_right_ptr0 = alloc_w_mmu_hdl(multi_cam_state->video0_fd, meta_right_buf_alloc_size0, (uint32_t*)&meta_right_buf_handle0, 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  memcpy((unsigned char*)meta_right_ptr0, meta_right_0, sizeof(meta_right_0)-1);

  LOGD("meta left len %u", sizeof(meta_left)-1);
  meta_left_buf_size = 68332;
  meta_left_buf_alloc_size = 1025280;
  void *meta_left_ptr = alloc_w_mmu_hdl(multi_cam_state->video0_fd, meta_left_buf_alloc_size, (uint32_t*)&meta_left_buf_handle, 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  memcpy((unsigned char*)meta_left_ptr, meta_left, sizeof(meta_left)-1);

  LOGD("meta left 0 len %u", sizeof(meta_left_0)-1);
  meta_left_buf_size0 = 68332;
  meta_left_buf_alloc_size0 = 1025280;
  void *meta_left_ptr0 = alloc_w_mmu_hdl(multi_cam_state->video0_fd, meta_left_buf_alloc_size0, (uint32_t*)&meta_left_buf_handle0, 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  memcpy((unsigned char*)meta_left_ptr0, meta_left_0, sizeof(meta_left_0)-1);

  LOGD("meta dual len %u", sizeof(meta_dual)-1);
  meta_dual_buf_size = 1848;
  meta_dual_buf_alloc_size = 27840;
  void *meta_dual_ptr = alloc_w_mmu_hdl(multi_cam_state->video0_fd, meta_dual_buf_alloc_size, (uint32_t*)&meta_dual_buf_handle, 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  memcpy((unsigned char*)meta_dual_ptr, meta_dual, sizeof(meta_dual)-1);

#ifdef EXTRA_IO_CFGS
  for(int i = 0;i < 10 ;i++) {
    extra_sync_objs[i] = 0;
  }
  alloc_w_mmu_hdl(multi_cam_state->video0_fd, 261120, (uint32_t*)&extra_io_mem_handles[0], 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  alloc_w_mmu_hdl(multi_cam_state->video0_fd, 20736, (uint32_t*)&extra_io_mem_handles[1], 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  alloc_w_mmu_hdl(multi_cam_state->video0_fd, 294912, (uint32_t*)&extra_io_mem_handles[2], 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  alloc_w_mmu_hdl(multi_cam_state->video0_fd, 1382400, (uint32_t*)&extra_io_mem_handles[3], 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  alloc_w_mmu_hdl(multi_cam_state->video0_fd, 6144, (uint32_t*)&extra_io_mem_handles[4], 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  alloc_w_mmu_hdl(multi_cam_state->video0_fd, 32768, (uint32_t*)&extra_io_mem_handles[5], 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  alloc_w_mmu_hdl(multi_cam_state->video0_fd, 294912, (uint32_t*)&extra_io_mem_handles[6], 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  alloc_w_mmu_hdl(multi_cam_state->video0_fd, 11536, (uint32_t*)&extra_io_mem_handles[7], 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  alloc_w_mmu_hdl(multi_cam_state->video0_fd, 65536, (uint32_t*)&extra_io_mem_handles[8], 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
  alloc_w_mmu_hdl(multi_cam_state->video0_fd, 6096384, (uint32_t*)&extra_io_mem_handles[9], 0x20, CAM_MEM_FLAG_HW_READ_WRITE | CAM_MEM_FLAG_KMD_ACCESS | CAM_MEM_FLAG_UMD_ACCESS | CAM_MEM_FLAG_CMD_BUF_TYPE, multi_cam_state->device_iommu, multi_cam_state->cdm_iommu);
#endif

  config_isp(0, 0, 1, buf0_handle, 0);

  LOG("-- Configuring sensor");
  sensors_i2c(init_array_imx363, std::size(init_array_imx363), CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG);
  //sensors_i2c(start_reg_array, std::size(start_reg_array), CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMON);
  //sensors_i2c(stop_reg_array, std::size(stop_reg_array), CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMOFF);


  // config csiphy
  LOG("-- Config CSI PHY");
  {
    uint32_t cam_packet_handle = 0;
    int size = sizeof(struct cam_packet)+sizeof(struct cam_cmd_buf_desc)*1;
    struct cam_packet *pkt = (struct cam_packet *)alloc_w_mmu_hdl(multi_cam_state->video0_fd, size, &cam_packet_handle);
    pkt->num_cmd_buf = 1;
    pkt->kmd_cmd_buf_index = -1;
    pkt->header.size = size;
    struct cam_cmd_buf_desc *buf_desc = (struct cam_cmd_buf_desc *)&pkt->payload;

    buf_desc[0].size = buf_desc[0].length = sizeof(struct cam_csiphy_info);
    buf_desc[0].type = CAM_CMD_BUF_GENERIC;

    struct cam_csiphy_info *csiphy_info = (struct cam_csiphy_info *)alloc_w_mmu_hdl(multi_cam_state->video0_fd, buf_desc[0].size, (uint32_t*)&buf_desc[0].mem_handle);
    csiphy_info->lane_mask = 0x1f;
    csiphy_info->lane_assign = 0x3210;// skip clk. How is this 16 bit for 5 channels??
    csiphy_info->csiphy_3phase = 0x0; // no 3 phase, only 2 conductors per lane
    csiphy_info->combo_mode = 0x0;
    csiphy_info->lane_cnt = 0x4;
    csiphy_info->secure_mode = 0x0;
    csiphy_info->settle_time = MIPI_SETTLE_CNT * 200000000ULL;
    csiphy_info->data_rate = 547200000; //48000000;  // Calculated by camera_freqs.py

    int ret_ = device_config(csiphy_fd, session_handle, csiphy_dev_handle, cam_packet_handle);
    assert(ret_ == 0);

    munmap(csiphy_info, buf_desc[0].size);
    release_fd(multi_cam_state->video0_fd, buf_desc[0].mem_handle);
    munmap(pkt, size);
    release_fd(multi_cam_state->video0_fd, cam_packet_handle);
  }

  // link devices
  LOG("-- Link devices");
  struct cam_req_mgr_link_info req_mgr_link_info = {0};
  req_mgr_link_info.session_hdl = session_handle;
  req_mgr_link_info.num_devices = 3;
  req_mgr_link_info.dev_hdls[0] = isp_dev_handle;
  req_mgr_link_info.dev_hdls[1] = sensor_dev_handle;
  req_mgr_link_info.dev_hdls[2] = actuator_dev_handle;
  ret = do_cam_control(multi_cam_state->video0_fd, CAM_REQ_MGR_LINK, &req_mgr_link_info, sizeof(req_mgr_link_info));
  link_handle = req_mgr_link_info.link_hdl;
  LOGD("link: %d hdl: 0x%X", ret, link_handle);

  struct cam_req_mgr_link_control req_mgr_link_control = {0};
  req_mgr_link_control.ops = CAM_REQ_MGR_LINK_ACTIVATE;
  req_mgr_link_control.session_hdl = session_handle;
  req_mgr_link_control.num_links = 1;
  req_mgr_link_control.link_hdls[0] = link_handle;
  ret = do_cam_control(multi_cam_state->video0_fd, CAM_REQ_MGR_LINK_CONTROL, &req_mgr_link_control, sizeof(req_mgr_link_control));
  LOGD("link control: %d", ret);

  ret = device_control(csiphy_fd, CAM_START_DEV, session_handle, csiphy_dev_handle);
  LOGD("start csiphy: %d", ret);
  ret = device_control(multi_cam_state->isp_fd, CAM_START_DEV, session_handle, isp_dev_handle);
  LOGD("start isp: %d", ret);
  ret = device_control(sensor_fd, CAM_START_DEV, session_handle, sensor_dev_handle);
  LOGD("start sensor: %d", ret);
  ret = device_control(actuator_fd, CAM_START_DEV, session_handle, actuator_dev_handle);
  LOGD("start actuator: %d", ret);

  for (int i = 0; i < FRAME_BUF_COUNT; i++) {
    // configure ISP to put the image in place
    struct cam_mem_mgr_map_cmd mem_mgr_map_cmd = {0};
    mem_mgr_map_cmd.mmu_hdls[0] = multi_cam_state->device_iommu;
    mem_mgr_map_cmd.num_hdl = 1;
    mem_mgr_map_cmd.flags = CAM_MEM_FLAG_HW_READ_WRITE;
    mem_mgr_map_cmd.fd = buf.camera_bufs[i].fd;
    ret = do_cam_control(multi_cam_state->video0_fd, CAM_REQ_MGR_MAP_BUF, &mem_mgr_map_cmd, sizeof(mem_mgr_map_cmd));
    LOGD("map buf req: (fd: %d) 0x%x %d", buf.camera_bufs[i].fd, mem_mgr_map_cmd.out.buf_handle, ret);
    buf_handle[i] = mem_mgr_map_cmd.out.buf_handle;
  }
  for(int i = 0; i < FRAME_BUF_COUNT;i++)
    enqueue_buffer();
}

void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx) {
#if false
  s->driver_cam.camera_init(s, v, CAMERA_ID_IMX363, 1, 20, device_id, ctx, VISION_STREAM_RGB_DRIVER, VISION_STREAM_DRIVER);
  printf("driver camera initted \n");
#endif
  s->road_cam.camera_init(s, v, CAMERA_ID_IMX363, 0, 20, device_id, ctx, VISION_STREAM_RGB_ROAD, VISION_STREAM_ROAD); // swap left/right
  printf("road camera initted \n");

  s->sm = new SubMaster({"driverState"});
  s->pm = new PubMaster({"roadCameraState", "driverCameraState", "wideRoadCameraState", "thumbnail"});
}

void cameras_open(MultiCameraState *s) {
  int ret;

  LOG("-- Opening devices");
  // :/ # cat /sys/class/video4linux/video0/name
  // cam-req-mgr
  // :/ # cat /sys/class/video4linux/video1/name
  // cam_sync

  // video0 is req_mgr, the target of many ioctls
  s->video0_fd = open_cam_dev_by_name("cam-req-mgr", O_RDWR | O_NONBLOCK);
  assert(s->video0_fd >= 0);
  LOGD("opened req_mgr fd: %d", s->video0_fd.fd_);

  // video1 is cam_sync, the target of some ioctls
  s->video1_fd = open_cam_dev_by_name("cam_sync", O_RDWR | O_NONBLOCK);
  assert(s->video1_fd >= 0);
  LOGD("opened cam_sync fd: %d", s->video1_fd.fd_);

  // looks like there's only one of these
  s->isp_fd = open_v4l_by_name_and_index("cam-isp");
  assert(s->isp_fd >= 0);
  LOGD("opened isp fd: %d", s->isp_fd.fd_);

  // query icp for MMU handles
  LOG("-- Query ICP for MMU handles");
  static struct cam_isp_query_cap_cmd isp_query_cap_cmd = {0};
  static struct cam_query_cap_cmd query_cap_cmd = {0};
  query_cap_cmd.handle_type = 1;
  query_cap_cmd.caps_handle = (uint64_t)&isp_query_cap_cmd;
  query_cap_cmd.size = sizeof(isp_query_cap_cmd);
  ret = do_cam_control(s->isp_fd, CAM_QUERY_CAP, &query_cap_cmd, sizeof(query_cap_cmd));
  assert(ret == 0);
  LOGD("using MMU handle: %x", isp_query_cap_cmd.device_iommu.non_secure);
  LOGD("using MMU handle: %x", isp_query_cap_cmd.cdm_iommu.non_secure);
  s->device_iommu = isp_query_cap_cmd.device_iommu.non_secure;
  s->cdm_iommu = isp_query_cap_cmd.cdm_iommu.non_secure;

  // subscribe
  LOG("-- Subscribing");
  static struct v4l2_event_subscription sub = {0};
  sub.type = V4L_EVENT_CAM_REQ_MGR_EVENT;
  sub.id = V4L_EVENT_CAM_REQ_MGR_SOF_BOOT_TS;
  ret = HANDLE_EINTR(ioctl(s->video0_fd, VIDIOC_SUBSCRIBE_EVENT, &sub));
  printf("req mgr subscribe: %d\n", ret);

  static struct v4l2_event_subscription sub2 = {0};
  sub2.type = CAM_SYNC_V4L_EVENT;
  sub2.id = CAM_SYNC_V4L_EVENT_ID_CB_TRIG;
  ret = HANDLE_EINTR(ioctl(s->video1_fd, VIDIOC_SUBSCRIBE_EVENT, &sub2));
  printf("cam sync subscribe: %d\n", ret);

#if false
  s->driver_cam.camera_open();
  printf("driver camera opened \n");
#endif
  s->road_cam.camera_open();
  printf("road camera opened \n");
}

void CameraState::camera_close() {
  int ret;

  // stop devices
  LOG("-- Stop devices");
  // ret = device_control(sensor_fd, CAM_STOP_DEV, session_handle, sensor_dev_handle);
  // LOGD("stop sensor: %d", ret);
  ret = device_control(multi_cam_state->isp_fd, CAM_STOP_DEV, session_handle, isp_dev_handle);
  LOGD("stop isp: %d", ret);
  ret = device_control(csiphy_fd, CAM_STOP_DEV, session_handle, csiphy_dev_handle);
  LOGD("stop csiphy: %d", ret);

  ret = device_control(actuator_fd, CAM_STOP_DEV, session_handle, actuator_dev_handle);
  LOGD("stop actuator: %d", ret);

  // link control stop
  LOG("-- Stop link control");
  static struct cam_req_mgr_link_control req_mgr_link_control = {0};
  req_mgr_link_control.ops = CAM_REQ_MGR_LINK_DEACTIVATE;
  req_mgr_link_control.session_hdl = session_handle;
  req_mgr_link_control.num_links = 1;
  req_mgr_link_control.link_hdls[0] = link_handle;
  ret = do_cam_control(multi_cam_state->video0_fd, CAM_REQ_MGR_LINK_CONTROL, &req_mgr_link_control, sizeof(req_mgr_link_control));
  LOGD("link control stop: %d", ret);

  // unlink
  LOG("-- Unlink");
  static struct cam_req_mgr_unlink_info req_mgr_unlink_info = {0};
  req_mgr_unlink_info.session_hdl = session_handle;
  req_mgr_unlink_info.link_hdl = link_handle;
  ret = do_cam_control(multi_cam_state->video0_fd, CAM_REQ_MGR_UNLINK, &req_mgr_unlink_info, sizeof(req_mgr_unlink_info));
  LOGD("unlink: %d", ret);

  // release devices
  LOGD("-- Release devices");
  ret = device_control(sensor_fd, CAM_RELEASE_DEV, session_handle, sensor_dev_handle);
  LOGD("release sensor: %d", ret);
  ret = device_control(multi_cam_state->isp_fd, CAM_RELEASE_DEV, session_handle, isp_dev_handle);
  LOGD("release isp: %d", ret);
  ret = device_control(csiphy_fd, CAM_RELEASE_DEV, session_handle, csiphy_dev_handle);
  LOGD("release csiphy: %d", ret);
  ret = device_control(actuator_fd, CAM_RELEASE_DEV, session_handle, actuator_dev_handle);
  LOGD("release actuator: %d", ret);

  for (int i = 0; i < FRAME_BUF_COUNT; i++) {
    release(multi_cam_state->video0_fd, buf_handle[i]);
  }
  release(multi_cam_state->video0_fd, meta_right_buf_handle);
  release(multi_cam_state->video0_fd, meta_left_buf_handle);
  release(multi_cam_state->video0_fd, meta_right_buf_handle0);
  release(multi_cam_state->video0_fd, meta_left_buf_handle0);
  release(multi_cam_state->video0_fd, meta_dual_buf_handle);
  // destroyed session
  struct cam_req_mgr_session_info session_info = {.session_hdl = session_handle};
  ret = do_cam_control(multi_cam_state->video0_fd, CAM_REQ_MGR_DESTROY_SESSION, &session_info, sizeof(session_info));
  LOGD("destroyed session: %d", ret);
}

void cameras_close(MultiCameraState *s) {
#if false
  s->driver_cam.camera_close();
#endif
  s->road_cam.camera_close();

  delete s->sm;
  delete s->pm;
}

void CameraState::reset_all_maps() {
  for (const auto& [sync_obj, req_id] : sync_obj_to_req_id) {
#ifdef DEBUG_CAM_SYNC_AND_REQ_MGR
  LOGD("%s: erase sync_obj: %u", __FUNCTION__, sync_obj);
#endif
#if false
   struct cam_sync_userpayload_info pli = {0};
    pli.sync_obj = sync_obj;
    int ret = do_cam_control(multi_cam_state->video1_fd, CAM_SYNC_DEREGISTER_PAYLOAD, &pli, sizeof(pli));

    struct cam_sync_info sync_destroy = {0};
    strcpy(sync_destroy.name, "NodeOutputPortFence");
    sync_destroy.sync_obj = sync_obj;
    ret = do_cam_control(multi_cam_state->video1_fd, CAM_SYNC_DESTROY, &sync_destroy, sizeof(sync_destroy));
#endif
    req_info_map.erase(req_id);
  }
  sync_obj_to_req_id.clear();
}

// Return true if sync_obj found in map, return false otherwise
bool CameraState::handle_camera_sync_event(struct cam_sync_ev_header *event_data) {
#ifdef DEBUG_CAM_SYNC_AND_REQ_MGR
  LOGD("%s: sync_obj: %u, status: %u", __FUNCTION__, event_data->sync_obj ,event_data->status);
#endif
  if(sync_obj_to_req_id.find(event_data->sync_obj) != sync_obj_to_req_id.end()) {
    int req_id = sync_obj_to_req_id[event_data->sync_obj];
    req_info_map[req_id].set_sync_status(event_data->status);
#ifdef DEBUG_CAM_SYNC_AND_REQ_MGR
  LOGD("Erase sync_obj: %d", event_data->sync_obj);
#endif
    sync_obj_to_req_id.erase(event_data->sync_obj);
    if(req_info_map[req_id].sync_error()) {
      LOGE("sync error for req_id %u, sync_obj: %u", req_id, event_data->sync_obj);
      return true;
    }
    if(req_info_map[req_id].finished())
      handle_req_finished(req_id);
    return true;
  }
  return false;
}

void CameraState::handle_req_finished(int req_id) {
#ifdef DEBUG_CAM_SYNC_AND_REQ_MGR
  LOGD("Erase req_id:  %d", req_id);
#endif
  // dispatch
  buf.camera_bufs_metadata[req_info_map[req_id].frame_buf_idx].timestamp_eof = (uint64_t)nanos_since_boot(); // set true eof
  buf.queue(req_info_map[req_id].frame_buf_idx);
  struct cam_sync_userpayload_info pli = {0};
  pli.sync_obj = req_info_map[req_id].sync_obj;
  int ret = do_cam_control(multi_cam_state->video1_fd, CAM_SYNC_DEREGISTER_PAYLOAD, &pli, sizeof(pli));
  //LOGD("Deregister payload: %d %d", ret, sync_objs[i]);

  // destroy old output fence
  struct cam_sync_info sync_destroy = {0};
  strcpy(sync_destroy.name, "NodeOutputPortFence");
  sync_destroy.sync_obj = req_info_map[req_id].sync_obj;
  ret = do_cam_control(multi_cam_state->video1_fd, CAM_SYNC_DESTROY, &sync_destroy, sizeof(sync_destroy));
#ifdef DEBUG_CAM_SYNC_AND_REQ_MGR
  LOGD("destroy sync_obj: %d %d", ret, sync_destroy.sync_obj);
#endif
  frame_buf_idx_list.push_back(req_info_map[req_id].frame_buf_idx);
  req_info_map.erase(req_id);
  // Schedule new req
  enqueue_buffer();
}

void CameraState::handle_camera_event(void *evdat) {
  struct cam_req_mgr_message *event_data = (struct cam_req_mgr_message *)evdat;
  assert(event_data->session_hdl == session_handle);
  assert(event_data->u.frame_msg.link_hdl == link_handle);

  uint64_t timestamp = event_data->u.frame_msg.timestamp;
  int main_id = event_data->u.frame_msg.frame_id;
  int real_id = event_data->u.frame_msg.request_id;

  if (real_id != 0) { // next ready
    if(req_info_map.find(real_id) != req_info_map.end()) {
      auto &meta_data = buf.camera_bufs_metadata[req_info_map[real_id].frame_buf_idx];
      meta_data.frame_id = main_id;
      meta_data.timestamp_sof = timestamp;
      exp_lock.lock();
      meta_data.gain = dc_gain_enabled ? analog_gain_frac * DC_GAIN : analog_gain_frac;
      meta_data.high_conversion_gain = dc_gain_enabled;
      meta_data.integ_lines = exposure_time;
      meta_data.measured_grey_fraction = measured_grey_fraction;
      meta_data.target_grey_fraction = target_grey_fraction;
      exp_lock.unlock();
      req_info_map[real_id].set_sof_ready();
    } else {
      LOGE("req_id %d not found in map", real_id);
      return;
    }
  } else {
    LOGD("request_id is zero???");
  }
}

static void imx363_apply_exposure(CameraState *s, int gain, int integ_lines, uint32_t frame_length) {
  int analog_gain = std::min(gain, 448);
  s->digital_gain = gain > 448 ? (512.0/(512-(gain))) / 8.0 : 1.0;
#if 0
  LOGD("%5d/%5d %5d %f\n", s->cur_integ_lines, s->frame_length, analog_gain, s->digital_gain);
#endif

  struct i2c_random_wr_payload exp_reg_array[] = {
    // REG_HOLD
    {0x104,0x1},
    {0x3002,0x0}, // long autoexposure off

    // FRM_LENGTH
    {0x340, (uint16_t)(frame_length >> 8)}, {0x341, (uint16_t)(frame_length & 0xff)},
    // INTEG_TIME aka coarse_int_time_addr aka shutter speed
    {0x202, (uint16_t)(integ_lines >> 8)}, {0x203, (uint16_t)(integ_lines & 0xff)},
    // global_gain_addr
    // if you assume 1x gain is 32, 448 is 14x gain, aka 2^14=16384
    {0x204, (uint16_t)(analog_gain >> 8)}, {0x205, (uint16_t)(analog_gain & 0xff)},

    // digital gain for colors: gain_greenR, gain_red, gain_blue, gain_greenB
    /*{0x20e, digital_gain_gr >> 8}, {0x20f,digital_gain_gr & 0xFF},
    {0x210, digital_gain_r >> 8}, {0x211,digital_gain_r & 0xFF},
    {0x212, digital_gain_b >> 8}, {0x213,digital_gain_b & 0xFF},
    {0x214, digital_gain_gb >> 8}, {0x215,digital_gain_gb & 0xFF},*/

    // REG_HOLD
    {0x104,0x0},
  };
  s->sensors_i2c(exp_reg_array, sizeof(exp_reg_array)/sizeof(struct i2c_random_wr_payload),
              CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG);
}

static void set_exposure(CameraState *s, float exposure_frac, float gain_frac) {
  uint32_t gain = s->cur_gain;
  uint32_t integ_lines = s->cur_integ_lines;

  if (exposure_frac >= 0) {
    exposure_frac = std::clamp(exposure_frac, 2.0f / s->frame_length, 1.0f);
    integ_lines = s->frame_length * exposure_frac;

    // See page 79 of the datasheet, this is the max allowed (-1 for phase adjust)
    integ_lines = std::min(integ_lines, s->frame_length - 11);
  }

  if (gain_frac >= 0) {
    // ISO200 is minimum gain
    gain_frac = std::clamp(gain_frac, 1.0f/64, 1.0f);

    // linearize gain response
    // TODO: will be wrong for driver camera
    // 0.125 -> 448
    // 0.25  -> 480
    // 0.5   -> 496
    // 1.0   -> 504
    // 512 - 512/(128*gain_frac)
    gain = (s->max_gain/510) * (512 - 512/(256*gain_frac));
  }

  if (gain != s->cur_gain || integ_lines != s->cur_integ_lines) {
    imx363_apply_exposure(s, gain, integ_lines, s->frame_length);
    std::lock_guard lk(s->exp_lock);
    s->cur_gain = gain;
    s->cur_integ_lines = integ_lines;
  }

  {
    s->cur_exposure_frac = exposure_frac;
    std::lock_guard lk(s->exp_lock);
    s->cur_gain_frac = gain_frac;
  }
#if 0
  LOGD("set exposure: %f %f - %d", exposure_frac, gain_frac);
#endif
}

static void do_autoexposure(CameraState *s, float grey_frac) {
  const float target_grey = 0.3;

  s->exp_lock.lock();
  s->measured_grey_fraction = grey_frac;
  s->target_grey_fraction = target_grey;
  s->exp_lock.unlock();

  float new_exposure = s->cur_exposure_frac;
  new_exposure *= pow(1.05, (target_grey - grey_frac) / 0.05 );
#if 0
  LOGD("diff %f: %f to %f", target_grey - grey_frac, s->cur_exposure_frac, new_exposure);
#endif

  float new_gain = s->cur_gain_frac;
  if (new_exposure < 0.10) {
    new_gain *= 0.95;
  } else if (new_exposure > 0.40) {
    new_gain *= 1.05;
  }
  set_exposure(s, new_exposure, new_gain);
}

void CameraState::set_camera_exposure(float grey_frac) {
  exp_lock.lock();
  measured_grey_fraction = grey_frac;
  exp_lock.unlock();

  // Processing a frame takes right about 50ms, so we need to wait a few ms
  // so we don't send i2c commands around the frame start.
  int ms = (nanos_since_boot() - buf.cur_frame_data.timestamp_sof) / 1000000;
  if (ms < 60) {
    util::sleep_for(60 - ms);
  }
  do_autoexposure(this, grey_frac);
}

void camera_autoexposure(CameraState *s, float grey_frac) {
  s->set_camera_exposure(grey_frac);
}

// called by processing_thread
void process_road_camera(MultiCameraState *s, CameraState *c, int cnt) {
  const CameraBuf *b = &c->buf;

  MessageBuilder msg;
  auto framed = c == &s->road_cam ? msg.initEvent().initRoadCameraState() : msg.initEvent().initWideRoadCameraState();
  fill_frame_data(framed, b->cur_frame_data);
  if (c == &s->road_cam && env_send_road) {
    framed.setImage(get_frame_image(b));
  }
  if (c == &s->road_cam) {
    framed.setTransform(b->yuv_transform.v);
  }
  s->pm->send("roadCameraState", msg);

  // TODO: adjust the box for IMX363
#ifndef USE_RAW10
  const auto [x, y, w, h] = std::tuple(96, 160, FRAME_WIDTH - 194, FRAME_HEIGHT - 94);
#else
  const auto [x, y, w, h] = std::tuple(96, 160, FRAME_WIDTH/2 - 194, FRAME_HEIGHT/2 - 94);
#endif
  const int skip = 2;
  camera_autoexposure(c, set_exposure_target(b, x, x + w, skip, y, y + h, skip));
}

void cameras_run(MultiCameraState *s) {
  LOG("-- Starting threads");
  std::vector<std::thread> threads;
#if false
  threads.push_back(start_process_thread(s, &s->driver_cam, common_process_driver_camera));
#endif
  threads.push_back(start_process_thread(s, &s->road_cam, process_road_camera));

  // start devices
  LOG("-- Starting devices");
#if false
  s->driver_cam.sensors_start();
#endif
  s->road_cam.sensors_start();

  // poll events
  LOG("-- Dequeueing Video events");
  while (!do_exit) {
    struct pollfd fds[2] = {{0}, {0}};

    fds[0].fd = s->video0_fd;
    fds[0].events = POLLPRI;
    fds[1].fd = s->video1_fd;
    fds[1].events = POLLPRI;

    int ret = poll(fds, std::size(fds), 1000);
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      LOGE("poll failed (%d - %d)", ret, errno);
      break;
    }

    if (fds[0].revents) {
      struct v4l2_event ev = {0};
      ret = HANDLE_EINTR(ioctl(fds[0].fd, VIDIOC_DQEVENT, &ev));
      if (ret == 0) {
        if (ev.type == V4L_EVENT_CAM_REQ_MGR_EVENT) {
          struct cam_req_mgr_message *event_data = (struct cam_req_mgr_message *)ev.u.data;
          if (env_debug_frames) {
            printf("sess_hdl 0x%X, link_hdl 0x%X, frame_id %lu, req_id %lu, timestamp 0x%lx, sof_status %d\n", event_data->session_hdl, event_data->u.frame_msg.link_hdl, event_data->u.frame_msg.frame_id, event_data->u.frame_msg.request_id, event_data->u.frame_msg.timestamp, event_data->u.frame_msg.sof_status);
          }

          if (event_data->session_hdl == s->road_cam.session_handle) {
            s->road_cam.handle_camera_event(event_data);
#if false
          } else if (event_data->session_hdl == s->driver_cam.session_handle) {
            s->driver_cam.handle_camera_event(event_data);
#endif
          } else {
            printf("Unknown vidioc event source\n");
            assert(false);
          }
        }
      } else {
        LOGE("VIDIOC_DQEVENT failed, errno=%d", errno);
      }
    }

 if (fds[1].revents) {
      struct v4l2_event ev = {0};
      ret = HANDLE_EINTR(ioctl(fds[1].fd, VIDIOC_DQEVENT, &ev));
      if (ret == 0) {
        if (ev.type == CAM_SYNC_V4L_EVENT) {
          struct cam_sync_ev_header *event_data = CAM_SYNC_GET_HEADER_PTR(ev);
          if(!s->road_cam.handle_camera_sync_event(event_data))
              s->driver_cam.handle_camera_sync_event(event_data);
        }
      } else {
        LOGE("VIDIOC_DQEVENT for cam sync failed, errno=%d", errno);
      }
    }
  }
  LOG(" ************** STOPPING **************");

  for (auto &t : threads) t.join();

  cameras_close(s);
}
