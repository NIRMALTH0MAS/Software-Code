#include <Arduino.h>
#include "driver/twai.h"
#include <math.h>

// ==========================================================
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);

// ==========================================================
// CAN ID Builder
// ==========================================================
static inline uint32_t build_ext_id(uint8_t mode, uint16_t data16, uint8_t id8)
{
  return ((uint32_t)(mode & 0x1F) << 24) |
         ((uint32_t)data16 << 8) |
         (uint32_t)id8;
}

// ==========================================================
// Protocol
// ==========================================================
static const uint8_t COMM_GET_ID    = 0x00;
static const uint8_t COMM_RUN_EN     = 0x03;
static const uint8_t COMM_RUN_DISABLE= 0x04;
static const uint8_t COMM_READ_ONE   = 0x11;
static const uint8_t COMM_WRITE_ONE = 0x12;

static const uint8_t RESP_MARKER    = 0xFE;
static const uint8_t MASTER_ID      = 0xFD;

// ==========================================================
// Registers
// ==========================================================
static const uint16_t IDX_runmode   = 0x7005;
static const uint16_t IDX_loc_ref   = 0x7016;
static const uint16_t IDX_limit_spd = 0x7017;
static const uint16_t IDX_loc_actual= 0x7019; // Telemetry feedback for actual pos

// ==========================================================
// Runtime & State
// ==========================================================
static uint8_t connected_motors[128];
static uint8_t num_motors = 0;    
static const float MAX_SPEED_RADS = 4.0f; 

// Local Working Coordinate System tracker
static float user_zero_offsets[128] = {0.0f};

// ==========================================================
// Trajectory Engine Structure
// ==========================================================
struct TrajectoryProfile {
  bool active = false;
  bool is_enabled = false;
  float start_pos = 0.0f;
  float target_pos = 0.0f;
  
  float v_max = MAX_SPEED_RADS; 
  float a_max = 5.0f;     // Default acceleration limit (rad/s^2)
  
  uint32_t start_time_us = 0;
  
  // Profile shapes calculated internally
  float dir = 1.0f;
  float d_total = 0.0f;
  float t_a = 0.0f;
  float t_c = 0.0f;
  float d_a = 0.0f;
  float d_c = 0.0f;
  float v_peak = 0.0f;
  float total_time = 0.0f;
  
  void compute(float start, float target) {
      start_pos = start;
      target_pos = target;
      active = true;
      start_time_us = micros();
      
      d_total = target - start;
      dir = (d_total >= 0) ? 1.0f : -1.0f;
      d_total = abs(d_total);
      
      // Calculate stopping distance for a trapezoid
      d_a = (v_max * v_max) / (2.0f * a_max);
      
      if (2.0f * d_a > d_total) {
          // Triangular profile (accelerates then immediately decelerates)
          d_a = d_total / 2.0f;
          v_peak = sqrt(2.0f * a_max * d_a);
          t_a = v_peak / a_max;
          t_c = 0.0f;
          d_c = 0.0f;
          total_time = 2.0f * t_a;
      } else {
          // Trapezoidal profile (accelerates, hits cruise speed, configures brake)
          v_peak = v_max;
          t_a = v_max / a_max;
          d_c = d_total - 2.0f * d_a;
          t_c = d_c / v_max;
          total_time = 2.0f * t_a + t_c;
      }
  }
  
  float get_position(uint32_t current_us) {
      float t = (current_us - start_time_us) / 1000000.0f;
      float pos_offset = 0.0f;
      
      if (t >= total_time) {
          active = false;
          return target_pos; // Snap to final position and end trajectory.
      }
      
      if (t <= t_a) {
          // Acceleration phase
          pos_offset = 0.5f * a_max * t * t;
      } 
      else if (t <= (t_a + t_c)) {
          // Cruise phase
          float t_cruise = t - t_a;
          pos_offset = d_a + v_peak * t_cruise;
      } 
      else {
          // Deceleration phase
          float t_dec = t - (t_a + t_c);
          pos_offset = d_a + d_c + v_peak * t_dec - 0.5f * a_max * t_dec * t_dec;
      }
      
      return start_pos + (dir * pos_offset);
  }
};
static TrajectoryProfile motor_profiles[128];

// ==========================================================
// Auto-Tuner State Machine
// ==========================================================
enum TuneState { TUNE_IDLE, TUNE_MOVE_OUT, TUNE_WAIT_OUT, TUNE_MOVE_IN, TUNE_WAIT_IN, TUNE_FAILED };

struct AutoTuner {
  TuneState state = TUNE_IDLE;
  uint8_t target_id = 0;
  float test_accel = 2.0f; 
  float max_accel = 2.0f;
  float base_pos = 0.0f;
  float test_pos = 0.0f;
  float lag_threshold = 0.15f; // ~8.5 degrees of absolute lag allowed before failure
  uint32_t state_timer = 0;
};
static AutoTuner tuner;

// ==========================================================
static inline bool can_send(const twai_message_t &tx)
{
  return twai_transmit((twai_message_t*)&tx, pdMS_TO_TICKS(10)) == ESP_OK;
}

// ==========================================================
static inline void float_to_le(float v, uint8_t* b)
{
  uint32_t u;
  memcpy(&u, &v, 4);

  b[0] = u;
  b[1] = u >> 8;
  b[2] = u >> 16;
  b[3] = u >> 24;
}

static inline float le_to_float(const uint8_t* b)
{
  uint32_t u = ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | b[0];
  float v;
  memcpy(&v, &u, 4);
  return v;
}

// ==========================================================
static void clear_can_rx()
{
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {}
}

// ==========================================================
static bool send_enable(uint8_t id)
{
  twai_message_t tx = {};
  tx.identifier = build_ext_id(COMM_RUN_EN, ((uint16_t)MASTER_ID << 8), id);
  tx.flags = TWAI_MSG_FLAG_EXTD;
  tx.data_length_code = 8;
  memset(tx.data, 0, 8);
  return can_send(tx);
}

// ==========================================================
static bool send_disable(uint8_t id)
{
  twai_message_t tx = {};
  tx.identifier = build_ext_id(COMM_RUN_DISABLE, ((uint16_t)MASTER_ID << 8), id);
  tx.flags = TWAI_MSG_FLAG_EXTD;
  tx.data_length_code = 8;
  memset(tx.data, 0, 8);
  return can_send(tx);
}

// ==========================================================
static bool write_param_f(uint16_t index, float val, uint8_t id)
{
  twai_message_t tx = {};
  tx.identifier = build_ext_id(COMM_WRITE_ONE, ((uint16_t)MASTER_ID << 8), id);
  tx.flags = TWAI_MSG_FLAG_EXTD;
  tx.data_length_code = 8;

  tx.data[0] = lowByte(index);
  tx.data[1] = highByte(index);
  tx.data[2] = 0;
  tx.data[3] = 0;
  float_to_le(val, &tx.data[4]);

  return can_send(tx);
}

// ==========================================================
static bool write_param_u8(uint16_t index, uint8_t val, uint8_t id)
{
  twai_message_t tx = {};
  tx.identifier = build_ext_id(COMM_WRITE_ONE, ((uint16_t)MASTER_ID << 8), id);
  tx.flags = TWAI_MSG_FLAG_EXTD;
  tx.data_length_code = 8;

  tx.data[0] = lowByte(index);
  tx.data[1] = highByte(index);
  tx.data[2] = 0;
  tx.data[3] = 0;
  tx.data[4] = val;
  tx.data[5] = 0;
  tx.data[6] = 0;
  tx.data[7] = 0;

  return can_send(tx);
}

// ==========================================================
static float read_encoder_blocking(uint8_t id, bool* success) 
{
  *success = false;
  clear_can_rx();

  twai_message_t tx = {};
  tx.identifier = build_ext_id(COMM_READ_ONE, ((uint16_t)MASTER_ID << 8), id);
  tx.flags = TWAI_MSG_FLAG_EXTD;
  tx.data_length_code = 8;
  tx.data[0] = lowByte(IDX_loc_actual);
  tx.data[1] = highByte(IDX_loc_actual);
  memset(&tx.data[2], 0, 6);

  if (!can_send(tx)) return 0.0f;

  uint32_t t0 = millis();
  while (millis() - t0 < 50) {
    twai_message_t rx = {};
    if (twai_receive(&rx, pdMS_TO_TICKS(5)) == ESP_OK) {
      if (rx.flags & TWAI_MSG_FLAG_EXTD) {
        uint32_t extid = rx.identifier;
        uint8_t mode = (extid >> 24) & 0x1F;
        
        if (mode == COMM_READ_ONE) {
          uint16_t out_idx = rx.data[0] | (rx.data[1] << 8);
          if (out_idx == IDX_loc_actual) {
             float val_rad = le_to_float(&rx.data[4]);
             *success = true;
             return val_rad;
          }
        }
      }
    }
  }
  return 0.0f;
}

// ==========================================================
static void scan_ids(uint32_t wait_ms = 1000)
{
  num_motors = 0;
  clear_can_rx();

  Serial.println("Scanning bus for daisy-chained motors...");

  for (uint16_t id = 0; id <= 127; id++)
  {
    twai_message_t tx = {};
    tx.identifier = build_ext_id(COMM_GET_ID, ((uint16_t)MASTER_ID << 8), id);
    tx.flags = TWAI_MSG_FLAG_EXTD;
    tx.data_length_code = 8;
    memset(tx.data, 0, 8);
    can_send(tx);
    delayMicroseconds(700);
  }

  uint32_t t0 = millis();
  while (millis() - t0 < wait_ms)
  {
    twai_message_t rx = {};
    if (twai_receive(&rx, pdMS_TO_TICKS(10)) != ESP_OK) continue;
    if (!(rx.flags & TWAI_MSG_FLAG_EXTD)) continue;

    uint32_t extid = rx.identifier;
    uint8_t mode = (extid >> 24) & 0x1F;
    uint16_t data16 = (extid >> 8) & 0xFFFF;
    uint8_t id8 = extid & 0xFF;

    if (mode == COMM_GET_ID && id8 == RESP_MARKER)
    {
      uint8_t cur_id = (uint8_t)(data16 & 0xFF);
      connected_motors[num_motors++] = cur_id;
      Serial.printf("[FOUND] Motor ID = 0x%02X\n", cur_id);
    }
  }

  if (num_motors == 0) {
    Serial.println("No motors found.");
  } else {
    Serial.printf("Scan complete. Tracking %d motor(s).\n", num_motors);
  }
}

// ==========================================================
static void execute_command(char cmd_char, uint8_t id, float val) 
{
  if (cmd_char == 'P') {
    if (val < 0) val = 0;
    if (val > 100) val = 100;
    float limit_spd = (val / 100.0f) * MAX_SPEED_RADS;
    if (limit_spd < 0.1f) limit_spd = 0.1f;
    
    // Update internal software profile generator limit
    motor_profiles[id].v_max = limit_spd;
    // Set hardware hardware limit as backup fail-safe
    write_param_f(IDX_limit_spd, limit_spd, id);
    Serial.printf(">> Motor 0x%02X Set Cruise Speed = %.0f%% (%.2f rad/s)\n", id, val, limit_spd);
  } 
  else if (cmd_char == 'C') {
    if (val < 0.1f) val = 0.1f; 
    // val goes directly into internal acceleration (rad/s^2)
    motor_profiles[id].a_max = val; 
    Serial.printf(">> Motor 0x%02X Set Acceleration = %.2f rad/s^2\n", id, val);
  }
  else if (cmd_char == 'D') {
    motor_profiles[id].active = false;
    motor_profiles[id].is_enabled = false;
    send_disable(id);
    Serial.printf(">> Motor 0x%02X Disabled (Torque OFF). Safe for manual manipulation.\n", id);
  }
  else if (cmd_char == 'Z') {
    bool success;
    float current_rad = read_encoder_blocking(id, &success);
    if (!success) {
      Serial.printf("!-- System error: Cannot locate encoder for 0x%02X to set Zero.\n", id);
      return; 
    }
    user_zero_offsets[id] = current_rad;
    Serial.printf(">> Motor 0x%02X Software Zero LOCKED at %.2f deg.\n", id, current_rad * 180.0f / PI);
  }
  else if (cmd_char == 'A' || cmd_char == 'I') {
    float target_deg = val;
    float target_rad = val * PI / 180.0f;
    
    // If Incremental, add the offset to the target
    if (cmd_char == 'I') {
        target_rad += user_zero_offsets[id];
        Serial.printf(">> Motor 0x%02X [Incremental Move] %.2f deg relative to Zero Offset.\n", id, target_deg);
    } else {
        Serial.printf(">> Motor 0x%02X [Absolute Move] %.2f deg relative to Hardware Factory Zero.\n", id, target_deg);
    }
    
    // Grab actual mechanical position to prevent jump
    bool success;
    float current_rad = read_encoder_blocking(id, &success);
    if (!success) {
      Serial.printf("!-- System error: Cannot locate encoder starting point for %02X.\n", id);
      return; 
    }
    
    if (!motor_profiles[id].is_enabled) {
      write_param_u8(IDX_runmode, 5, id); 
      delay(2);
      send_enable(id);
      motor_profiles[id].is_enabled = true;
      delay(20); 
    }
    
    // Initiate Trajectory!
    motor_profiles[id].compute(current_rad, target_rad);
  }
  else if (cmd_char == 'T') {
    if (tuner.state != TUNE_IDLE) {
       Serial.println("Autotuner already running.");
       return;
    }
    
    bool success;
    float current_rad = read_encoder_blocking(id, &success);
    if (!success) {
      Serial.printf("!-- System error: Cannot locate encoder starting point for %02X.\n", id);
      return; 
    }
    
    if (!motor_profiles[id].is_enabled) {
      write_param_u8(IDX_runmode, 5, id); 
      delay(2);
      send_enable(id);
      motor_profiles[id].is_enabled = true;
      delay(20); 
    }
    
    tuner.target_id = id;
    tuner.base_pos = current_rad;
    tuner.test_pos = current_rad + (45.0f * PI / 180.0f); // sweep 45 degrees
    tuner.test_accel = 2.0f; 
    tuner.max_accel = 2.0f;  
    tuner.state = TUNE_MOVE_OUT;
    
    Serial.printf("\n============================================\n");
    Serial.printf("[AUTOTUNE] Calibrating Motor 0x%02X Payload Inertia...\n", id);
    Serial.printf("[AUTOTUNE] Sweeping 45 degrees limit. STAND BACK!\n");
    Serial.printf("============================================\n");
  }
}

void print_help();

// ==========================================================
static void parse_and_execute_motor_cmd(String cmd) 
{
  cmd.trim();
  if(cmd.length() == 0) return;

  char cmd_char = toupper(cmd[0]);
  if (cmd_char == 'S') {
    scan_ids();
    return;
  }

  int first_space = cmd.indexOf(' ');
  if (first_space < 0) {
    print_help();
    return;
  }

  String rest = cmd.substring(first_space + 1);
  rest.trim();
  int second_space = rest.indexOf(' ');

  if (cmd_char == 'R') {
     if (rest.equalsIgnoreCase("ALL")) {
        if(num_motors == 0) Serial.println("No active motors. Run 'S' first.");
        for(int i=0; i<num_motors; i++) {
           bool success;
           uint8_t id = connected_motors[i];
           float rad = read_encoder_blocking(id, &success);
           if (success) {
              float abs_deg = rad * 180.0f / PI;
              float rel_deg = (rad - user_zero_offsets[id]) * 180.0f / PI;
              Serial.printf("Motor 0x%02X | Absolute: %8.2f deg | Incremental: %8.2f deg\n", id, abs_deg, rel_deg);
           }
        }
     } else {
        uint8_t target_id = rest.toInt();
        bool success;
        float rad = read_encoder_blocking(target_id, &success);
        if (success) {
            float abs_deg = rad * 180.0f / PI;
            float rel_deg = (rad - user_zero_offsets[target_id]) * 180.0f / PI;
            Serial.printf("Motor 0x%02X | Absolute: %8.2f deg | Incremental: %8.2f deg\n", target_id, abs_deg, rel_deg);
        }
     }
     return;
  }
  
  // D, Z, and T commands don't strictly require a value, but we can pass 0
  if (cmd_char == 'D' || cmd_char == 'Z' || cmd_char == 'T') {
     String id_str = rest;
     id_str.trim();
     if (id_str.equalsIgnoreCase("ALL")) {
        if(num_motors == 0) Serial.println("No active motors. Run 'S' first.");
        for(int i=0; i<num_motors; i++) {
            execute_command(cmd_char, connected_motors[i], 0);
            delay(5);
        }
     } else {
        uint8_t target_id = id_str.toInt();
        execute_command(cmd_char, target_id, 0);
     }
     return;
  }

  if (second_space < 0) {
    print_help();
    return;
  }

  String id_str = rest.substring(0, second_space);
  id_str.trim();
  String val_str = rest.substring(second_space + 1);
  val_str.trim();

  float val = val_str.toFloat();

  if (id_str.equalsIgnoreCase("ALL")) {
    if(num_motors == 0) Serial.println("No active motors. Run 'S' first.");
    for(int i=0; i<num_motors; i++) {
      execute_command(cmd_char, connected_motors[i], val);
      delay(5);  // Stagger messages
    }
  } else {
    uint8_t target_id = id_str.toInt();
    execute_command(cmd_char, target_id, val);
  }
}

// ==========================================================
void print_help() {
  Serial.println("----- Serial Commands -----");
  Serial.println("S                  : Scan bus for motors");
  Serial.println("C <id|ALL> <val>   : Target Accel Rate (e.g. 5.0 rad/s^2)");
  Serial.println("P <id|ALL> <0-100> : Target Cruise Speed percentage");
  Serial.println("D <id|ALL>         : Disable Torque (Allows manual manipulation)");
  Serial.println("Z <id|ALL>         : Set current angle as the Local Zero Point");
  Serial.println("A <id|ALL> <deg>   : Move ABSOLUTE  (relative to factory origin)");
  Serial.println("I <id|ALL> <deg>   : Move INCREMENTAL (relative to local zero)");
  Serial.println("T <id>             : Auto-Tune payload acceleration");
  Serial.println("R <id|ALL>         : Read dual encoder coordinates");
  Serial.println("---------------------------");
}

// ==========================================================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("Robstride Ready");

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
  {
    Serial.println("CAN Install Failed");
    while (1);
  }
  if (twai_start() != ESP_OK)
  {
    Serial.println("CAN Start Failed");
    while (1);
  }
  Serial.println("CAN Bus online. 1Mbps.");
  
  scan_ids();
  print_help();
}

// ==========================================================
void loop()
{
  // 1. Process Serial Inputs
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    parse_and_execute_motor_cmd(cmd);
  }

  // 2. Execute Real-Time Trajectories (Non-Blocking ~1ms tick for absolute smoothness)
  static uint32_t last_traj_update = 0;
  if (micros() - last_traj_update >= 1000) { // 1000Hz update rate
     last_traj_update = micros();
     uint32_t now = micros();
     
     for(int i=0; i<num_motors; i++) {
        uint8_t id = connected_motors[i];
        
        if (motor_profiles[id].active) {
            float setpoint = motor_profiles[id].get_position(now);
            write_param_f(IDX_loc_ref, setpoint, id);
            
            if (!motor_profiles[id].active) {
               // Only print complete msg if not autotuning
               if (tuner.state == TUNE_IDLE) {
                   Serial.printf(">> Motor 0x%02X Interploation Complete.\n", id);
               }
            }
        }
     }
  }

  // 3. Auto-Tuner Machine logic
  if (tuner.state != TUNE_IDLE) {
     uint8_t id = tuner.target_id;
     
     // Periodically check following error
     static uint32_t last_tune_poll = 0;
     if (millis() - last_tune_poll > 100) {
         last_tune_poll = millis();
         
         bool success;
         float actual = read_encoder_blocking(id, &success);
         if (success && motor_profiles[id].active) {
             float expected = motor_profiles[id].get_position(micros());
             float lag = abs(actual - expected);
             
             if (lag > tuner.lag_threshold) {
                 Serial.printf("\n[AUTOTUNE] Motor 0x%02X payload broke at accel = %.2f rad/s^2. Following Error: %.2f deg.\n", id, tuner.test_accel, lag * 180.0f / PI);
                 Serial.printf("[AUTOTUNE] Setting absolute safe maximum acceleration to %.2f rad/s^2.\n", tuner.max_accel);
                 
                 tuner.state = TUNE_FAILED;
                 motor_profiles[id].active = false; 
                 write_param_f(IDX_loc_ref, actual, id); // snap to actual to stop cleanly
                 motor_profiles[id].a_max = tuner.max_accel;
                 tuner.state = TUNE_IDLE;
             }
         }
     }
     
     if (tuner.state == TUNE_MOVE_OUT) {
        motor_profiles[id].a_max = tuner.test_accel;
        motor_profiles[id].compute(motor_profiles[id].start_pos, tuner.test_pos); // force move
        tuner.state = TUNE_WAIT_OUT;
     } 
     else if (tuner.state == TUNE_WAIT_OUT) {
        if (!motor_profiles[id].active) {
            tuner.state = TUNE_MOVE_IN;
            delay(500); 
        }
     }
     else if (tuner.state == TUNE_MOVE_IN) {
        motor_profiles[id].a_max = tuner.test_accel;
        motor_profiles[id].compute(motor_profiles[id].start_pos, tuner.base_pos);
        tuner.state = TUNE_WAIT_IN;
     }
     else if (tuner.state == TUNE_WAIT_IN) {
        if (!motor_profiles[id].active) {
            tuner.max_accel = tuner.test_accel; 
            tuner.test_accel += 1.0f; // gently bump testing accel
            
            Serial.printf("[AUTOTUNE] Pass success. Bumping Accel to %.2f rad/s^2...\n", tuner.test_accel);
            
            if (tuner.test_accel > 15.0f) {
                Serial.println("[AUTOTUNE] Maximum hardware tuning limit reached without detecting load faults. Great!");
                motor_profiles[id].a_max = tuner.max_accel;
                tuner.state = TUNE_IDLE;
            } else {
                tuner.state = TUNE_MOVE_OUT;
                delay(500);
            }
        }
     }
  }
}
