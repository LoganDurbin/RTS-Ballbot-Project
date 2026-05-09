#pragma once
#include <Arduino.h>
#include <lfs.h>
#include <hardware/flash.h>
#include <hardware/regs/addressmap.h>
#include "imu.h"

#define LOG_FILENAME "ballbot_data.csv"
#define BUFFER_SIZE 512

// LittleFS configuration for RP2040 flash
#define LFS_BLOCK_SIZE 4096
#define LFS_PROG_SIZE 256
#define LFS_READ_SIZE 16
#define LFS_LOOKAHEAD_SIZE 128

// Use last 256KB of flash for logging (RP2040 has 2MB total)
#define FLASH_LOG_START_OFFSET (PICO_FLASH_SIZE_BYTES - (256 * 1024))
#define FLASH_LOG_BLOCKS (256 * 1024 / LFS_BLOCK_SIZE)

// Global littlefs structures
static lfs_t lfs;
static lfs_file_t file;
static uint8_t lfs_read_buffer[LFS_READ_SIZE];
static uint8_t lfs_prog_buffer[LFS_PROG_SIZE];
static uint8_t lfs_lookahead_buffer[LFS_LOOKAHEAD_SIZE];

// RP2040 flash read callback for littlefs
static int pico_flash_read(const struct lfs_config *c, lfs_block_t block,
                           lfs_off_t off, void *buffer, lfs_size_t size) {
  uint32_t addr = FLASH_LOG_START_OFFSET + (block * LFS_BLOCK_SIZE) + off;
  memcpy(buffer, (void *)(XIP_BASE + addr), size);
  return LFS_ERR_OK;
}

// RP2040 flash program callback for littlefs
static int pico_flash_prog(const struct lfs_config *c, lfs_block_t block,
                           lfs_off_t off, const void *buffer, lfs_size_t size) {
  uint32_t addr = FLASH_LOG_START_OFFSET + (block * LFS_BLOCK_SIZE) + off;
  uint32_t flash_offset = addr - XIP_BASE;
  
  flash_range_program(flash_offset, (uint8_t *)buffer, size);
  
  return LFS_ERR_OK;
}

// RP2040 flash erase callback for littlefs
static int pico_flash_erase(const struct lfs_config *c, lfs_block_t block) {
  uint32_t addr = FLASH_LOG_START_OFFSET + (block * LFS_BLOCK_SIZE);
  uint32_t flash_offset = addr - XIP_BASE;
  
  flash_range_erase(flash_offset, LFS_BLOCK_SIZE);
  
  return LFS_ERR_OK;
}

// RP2040 flash sync callback (no-op)
static int pico_flash_sync(const struct lfs_config *c) {
  return LFS_ERR_OK;
}

class DataLogger {
private:
  char buffer[BUFFER_SIZE];
  unsigned long entry_count;
  bool is_initialized;
  struct lfs_config cfg;

public:
  DataLogger() : entry_count(0), is_initialized(false) {}

  bool begin() {
    // Configure LittleFS for RP2040 flash
    cfg.context = nullptr;
    cfg.read = pico_flash_read;
    cfg.prog = pico_flash_prog;
    cfg.erase = pico_flash_erase;
    cfg.sync = pico_flash_sync;
    cfg.read_size = LFS_READ_SIZE;
    cfg.prog_size = LFS_PROG_SIZE;
    cfg.block_size = LFS_BLOCK_SIZE;
    cfg.block_count = FLASH_LOG_BLOCKS;
    cfg.block_cycles = 500;
    cfg.cache_size = 256;
    cfg.lookahead_size = LFS_LOOKAHEAD_SIZE;
    cfg.read_buffer = lfs_read_buffer;
    cfg.prog_buffer = lfs_prog_buffer;
    cfg.lookahead_buffer = lfs_lookahead_buffer;

    // Mount or format the filesystem
    int err = lfs_mount(&lfs, &cfg);
    if (err != LFS_ERR_OK) {
      Serial.println("LittleFS mount failed, formatting...");
      lfs_format(&lfs, &cfg);
      err = lfs_mount(&lfs, &cfg);
      if (err != LFS_ERR_OK) {
        Serial.println("ERROR: LittleFS format/mount failed!");
        return false;
      }
    }

    // Open or create the log file
    err = lfs_file_open(&lfs, &file, LOG_FILENAME, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);
    if (err != LFS_ERR_OK) {
      Serial.println("ERROR: Could not open log file!");
      lfs_unmount(&lfs);
      return false;
    }

    // Write CSV header if this is a new file
    if (lfs_file_size(&lfs, &file) == 0) {
      write_header();
    }

    is_initialized = true;
    Serial.print("Data logger initialized. Logging to ");
    Serial.println(LOG_FILENAME);
    return true;
  }

  void write_header() {
    const char *header = "time_ms,motor_a,motor_b,motor_c,pitch_deg,roll_deg,tilt_error,ax_g,ay_g,az_g,gx_dps,gy_dps,loop_us,max_loop_us,missed_timing\n";
    lfs_file_write(&lfs, &file, header, strlen(header));
    lfs_file_sync(&lfs, &file);
  }

  void log_entry(unsigned long timestamp_ms,
                 int motor_a, int motor_b, int motor_c,
                 float pitch, float roll,
                 float tilt_error,
                 const IMUData &imu_data,
                 unsigned long loop_us,
                 unsigned long max_loop_us,
                 bool timing_miss) {
    
    if (!is_initialized) return;

    // Format: time_ms, motor_a, motor_b, motor_c, pitch, roll, tilt_error, ax, ay, az, gx, gy, loop_us, max_loop_us, missed_timing
    int len = snprintf(buffer, BUFFER_SIZE,
             "%lu,%d,%d,%d,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%lu,%lu,%d\n",
             timestamp_ms,
             motor_a, motor_b, motor_c,
             pitch, roll, tilt_error,
             imu_data.ax, imu_data.ay, imu_data.az,
             imu_data.gx, imu_data.gy,
             loop_us, max_loop_us,
             timing_miss ? 1 : 0);

    lfs_file_write(&lfs, &file, buffer, len);
    entry_count++;

    // Flush every 100 entries to avoid data loss
    if (entry_count % 100 == 0) {
      lfs_file_sync(&lfs, &file);
      Serial.print("Logged ");
      Serial.print(entry_count);
      Serial.println(" entries");
    }
  }

  void end() {
    if (is_initialized) {
      lfs_file_sync(&lfs, &file);
      lfs_file_close(&lfs, &file);
      lfs_unmount(&lfs);
      Serial.print("Logging complete. Total entries: ");
      Serial.println(entry_count);
      is_initialized = false;
    }
  }

  unsigned long get_entry_count() const {
    return entry_count;
  }

  bool is_ready() const {
    return is_initialized;
  }

  // Utility function to delete log file and start fresh
  static bool clear_logs() {
    lfs_remove(&lfs, LOG_FILENAME);
    Serial.println("Log file cleared.");
    return true;
  }
};
