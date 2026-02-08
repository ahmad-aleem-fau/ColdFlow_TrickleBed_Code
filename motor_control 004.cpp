#include <Arduino.h>
#include <TimeLib.h>

// motor_test implementation for Teensy 4.1
// Serial protocol (line-based):
// SET PPR <int>
// SET WD <float_cm>
// SET SPEED <float_cm_per_s>
// BUTTON UP
// BUTTON DOWN
// STOP
// STATUS
// Responses: ACK:, STATUS:, TICK:, DONE:MOVE, ERROR:

// Pin definitions (adjust if you wire differently)
const int LED_PIN = 13; // flash on command received
const int PWM_PIN = 10; // PWM pin for motor speed
const int DIR_PIN = 9;  // Direction pin
const int ENA_PIN = 8;  // Enable pin
const int UPPER_PIN = 2; // Upper end switch (TCRT5000)
const int LOWER_PIN = 3; // Lower end switch (TCRT5000)



// State
volatile long ppr = 2000;
volatile float wheel_diam_cm = 5.0;
volatile float speed_cm_s = 1.0;

// timekeeping: UNIX epoch ms base stored when host sets time
static uint64_t epoch_base_ms = 0;
static unsigned long epoch_base_set_micros = 0;

bool moving = false;
unsigned long move_start_us = 0;
unsigned long last_tick_us = 0;
unsigned long tick_interval_us = 100000UL; // send tick every 100ms
// run duration (us) for motor moves - default 1 second
volatile unsigned long run_duration_us = 1000000UL;
// experiment runner state
volatile bool experiment_running = false;
size_t experiment_index = 0;
uint64_t experiment_start_us = 0;
// track overall experiment start (across repetitions) and per-run start
uint64_t experiment_global_start_us = 0;
uint64_t experiment_run_start_us = 0;
float experiment_current_speed = 0.0;
// Fixed update/communication step in microseconds
static unsigned long time_step_us = 10000UL; // 10 ms default (can be updated by host)
static uint64_t last_step_us = 0;
// repetition control: number of experiment runs requested by host
volatile int reps_total = 0;
volatile int reps_left = 0;
// LED flash state
unsigned long ledFlashUntil = 0;
const unsigned long LED_FLASH_MS = 500;

// Return approximate free RAM in bytes on platforms where available.
static int free_memory() {
#if defined(__AVR__)
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
#else
  // For non-AVR platforms provide a best-effort 0 (unknown) to avoid compile errors.
  return 0;
#endif
}

// Interrupt flags (set in ISR, handled in loop)
volatile bool upperTriggered = false;
volatile bool lowerTriggered = false;
// ISR-to-mainloop notification flags (safe to set in ISR)
volatile bool upperMessagePending = false;
volatile bool lowerMessagePending = false;
unsigned long lastUpperProcessed = 0;
unsigned long lastLowerProcessed = 0;
const unsigned long DEBOUNCE_MS = 50;

// ISRs - keep very short
void isr_upper() {
  upperTriggered = true;
  // upperMessagePending = true;
}

void isr_lower() {
  lowerTriggered = true;
  // lowerMessagePending = true;
}

void stopMotor() {
  analogWrite(PWM_PIN, 0);
  digitalWrite(ENA_PIN, HIGH);
  moving = false;
}

void startMotor(double pwm_freq, bool dir_up) {
  // Teensy API: analogWriteFrequency(pin, frequency)
  float freq = (pwm_freq < 1.0) ? 1.0f : (float)pwm_freq;
  analogWriteFrequency(PWM_PIN, freq);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(DIR_PIN, dir_up ? HIGH : LOW);
  analogWrite(PWM_PIN, 128); // 50% duty (0-255)
  moving = true;
  move_start_us = micros();
  last_tick_us = move_start_us;
}

void sendLine(const String &s) {
  Serial.println(s);
}

// Movement V_TABLE: store uploaded (time_ms, speed_cm_s) rows
// 32-bit aligned entries for fast access
struct VEntry {
  uint32_t t_ms;
  float speed_cm_s;
} __attribute__((aligned(4)));

static VEntry *V_TABLE = nullptr;
static size_t V_TABLE_CAP = 0;
static size_t V_TABLE_LEN = 0;
static bool V_TABLE_READY = false;

static bool vtable_ensure_capacity(size_t need) {
  if (need <= V_TABLE_CAP) return true;
  size_t newcap = V_TABLE_CAP ? V_TABLE_CAP * 2 : 256;
  while (newcap < need) newcap *= 2;
  void *p = realloc(V_TABLE, newcap * sizeof(VEntry));
  if (!p) return false;
  V_TABLE = (VEntry*)p;
  V_TABLE_CAP = newcap;
  return true;
}

static void vtable_reset() {
  // prepare for new upload
  V_TABLE_LEN = 0;
  V_TABLE_READY = false;
}

static bool vtable_push(uint32_t t, float v) {
  if (!vtable_ensure_capacity(V_TABLE_LEN + 1)) return false;
  V_TABLE[V_TABLE_LEN].t_ms = t;
  V_TABLE[V_TABLE_LEN].speed_cm_s = v;
  V_TABLE_LEN++;
  return true;
}

static void vtable_finalize() {
  V_TABLE_READY = true;
}

static size_t vtable_len() { return V_TABLE_LEN; }

void handleButtonMove(bool up) {
  // compute PWM freq: PWM_FREQ = (SPEED / (PI()*WD) ) * PPR
  double pwm_freq = (speed_cm_s / (PI * wheel_diam_cm)) * (double)ppr;
  if (pwm_freq < 1.0) pwm_freq = 1.0;
  sendLine(String("ACK:MOVE:") + (up ? "UP" : "DOWN") + String(":FREQ=") + String(pwm_freq, 1));
  startMotor(pwm_freq, up);
}

void parseCommand(const String &cmdline) {
  if (cmdline.length() == 0) return;
  // flash LED briefly to indicate a command was received
  ledFlashUntil = millis() + LED_FLASH_MS;
  // tokenize
  String s = cmdline;
  s.trim();
  // --- Movement table commands handling ---
  // We accept the following lines from the PC:
  //   BEGIN_MOVEDATA      -- start a new upload
  //   MOVE <ms>,<speed>    -- append a row (time in ms, speed cm/s)
  //   END_MOVEDATA        -- finish upload (marks V_TABLE ready)

  if (s.startsWith("SET PPR")) {
    long v = s.substring(8).toInt();
    if (v > 0) { ppr = v; sendLine(String("ACK:SET:PPR:") + String(ppr)); }
  } else if (s.startsWith("SET TIME_STEP_US") || s.startsWith("SET TIME_STEP")) {
    int sp = s.indexOf(' ');
    if (sp >= 0) {
      // Accept either: SET TIME_STEP_US <us> or SET TIME_STEP <us>
      String rest = s.substring(sp+1);
      // find last space and parse value
      int sp2 = rest.lastIndexOf(' ');
      if (sp2 >= 0) {
        String vstr = rest.substring(sp2+1);
        unsigned long v = (unsigned long)vstr.toInt();
        if (v >= 10) {
          time_step_us = v;
          sendLine(String("ACK:SET:TIME_STEP_US:") + String(time_step_us));
        } else {
          sendLine(String("ERR:SET:TIME_STEP_US:VALUE"));
        }
      }
    }
  } else if (s.equals("Hello motor!") || s.equalsIgnoreCase("HELLO MOTOR!")) {
    // simple handshake from host
    sendLine(String("Brummm..."));
  } else if (s.startsWith("SET WD")) {
    float v = s.substring(7).toFloat();
    if (v > 0) { wheel_diam_cm = v; sendLine(String("ACK:SET:WD:") + String(wheel_diam_cm)); }
  } else if (s.startsWith("SET SPEED")) {
    float v = s.substring(10).toFloat();
    if (v >= 0) { speed_cm_s = v; sendLine(String("ACK:SET:SPEED:") + String(speed_cm_s)); }
  } else if (s.startsWith("SET DURATION")) {
    // SET DURATION <seconds>  (1-30)
    float sec = s.substring(12).toFloat();
    run_duration_us = (unsigned long)(sec * 1000000.0);
    sendLine(String("ACK:SET:DURATION_US:") + String((unsigned long)run_duration_us));
  } else if (s.equalsIgnoreCase("BUTTON UP")) {
    handleButtonMove(true);
  } else if (s.equalsIgnoreCase("BUTTON DOWN")) {
    handleButtonMove(false);
  } else if (s.equalsIgnoreCase("STOP")) {
    stopMotor(); sendLine("ACK:STOP");
  } else if (s.equalsIgnoreCase("STATUS")) {
    sendLine(String("STATUS:OK:PPR=") + String(ppr) + String(",WD=") + String(wheel_diam_cm) + String(",SPEED=") + String(speed_cm_s));
  } else if (s.equalsIgnoreCase("PING")) {
    sendLine("PONG");
  } else if (s.equalsIgnoreCase("START_EXPERIMENT") || s.equalsIgnoreCase("START_EX")) {
    if (!V_TABLE_READY || vtable_len() == 0) {
      sendLine("ERR:START_EXPERIMENT:NO_VTABLE");
    } else {
      // if host didn't set repetitions, default to 1 run
      if (reps_left == 0) {
        reps_left = 1;
        reps_total = 1;
      }
      // clear any pending end-switch ISR flags when starting an experiment
      upperTriggered = false;
      lowerTriggered = false;
      upperMessagePending = false;
      lowerMessagePending = false;
      // initialize last-processed times to now to avoid immediate debounce firing
      lastUpperProcessed = millis();
      lastLowerProcessed = millis();
      experiment_running = true;
      experiment_index = 0;
      // mark global start on first run, and start the per-run timer
      experiment_global_start_us = micros();
      experiment_run_start_us = experiment_global_start_us;
      experiment_current_speed = 0.0f;
      // initialize step timer so updates/communication happen on a regular grid
      last_step_us = experiment_run_start_us;
      // ensure motor stopped at start
      // stopMotor();
      sendLine("ACK:START_EXPERIMENT");
      // debug: report experiment start and table length
      sendLine(String("DBG:EXPERIMENT:START:LEN=") + String(vtable_len()));
      // report run settings for diagnostics: duration, reps, sample entries
      unsigned long run_dur_ms = (unsigned long)(run_duration_us / 1000ULL);
      int vtlen = (int)vtable_len();
      String info = String("DBG:EXPERIMENT:INFO:RUNDUR_MS=") + String(run_dur_ms) + String(",REPS_TOTAL=") + String(reps_total) + String(",REPS_LEFT=") + String(reps_left) + String(",VT_LEN=") + String(vtlen);
      if (vtlen > 0) {
        info += String(",FIRST_T=") + String(V_TABLE[0].t_ms) + String(",FIRST_S=") + String(V_TABLE[0].speed_cm_s,3);
        info += String(",LAST_T=") + String(V_TABLE[vtlen-1].t_ms) + String(",LAST_S=") + String(V_TABLE[vtlen-1].speed_cm_s,3);
      }
      sendLine(info);
      // send initial DBG update for IDX=0 so host sees starting target even if unchanged
      if (vtable_len() > 0) {
        float first_speed = V_TABLE[0].speed_cm_s;
        double pwm0 = (fabs(first_speed) / (PI * wheel_diam_cm)) * (double)ppr;
        if (pwm0 < 1.0) pwm0 = 1.0;
        bool dir0 = (first_speed >= 0.0f);
        // report initial update (elapsed 0) including timestamp in micros and global elapsed
        unsigned long nowus_init = micros();
        unsigned long global_us_init = 0; // just started
        sendLine(String("DBG:EXPERIMENT:UPDATE:IDX=0,ELAPSED=0,TGT=") + String(first_speed,3) + String(",PWM=") + String(pwm0,2) + String(",DIR=") + String(dir0 ? "UP" : "DN") + String(",FREE=") + String(free_memory()) + String(",TS_US=") + String(nowus_init) + String(",GLOBAL_US=") + String(global_us_init));
        // if initial speed is non-zero, start motor immediately
        if (fabs(first_speed - experiment_current_speed) > 1e-4 && fabs(first_speed) > 1e-6) {
          experiment_current_speed = first_speed;
          speed_cm_s = experiment_current_speed;
          startMotor(pwm0, dir0);
        }
      }
    }
  } else if (s.equalsIgnoreCase("STOP_EXPERIMENT") || s.equalsIgnoreCase("STOP_EX")) {
    if (experiment_running) {
      experiment_running = false;
      stopMotor();
      // cancel any remaining repetitions
      reps_left = 0;
      sendLine("ACK:STOP_EXPERIMENT");
      sendLine("DBG:EXPERIMENT:STOPPED_BY_HOST");
      sendLine("DONE:EXPERIMENT:REASON=STOP");
    } else {
      sendLine("ACK:STOP_EXPERIMENT:NOT_RUNNING");
    }
  } else if (s.startsWith("SET_TIME") || s.startsWith("SET TIME")) {
    // SET_TIME <unix_ms> or SET TIME <unix_ms>
    int sp = s.indexOf(' ');
    if (sp >= 0) {
      String rest = s.substring(sp+1);
      // parse as unsigned long long to support full ms epoch
      const char *c = rest.c_str();
      unsigned long long v = strtoull(c, NULL, 10);
      epoch_base_ms = (uint64_t)v;
      epoch_base_set_micros = micros();
      // set hardware RTC (TimeLib)
      time_t tt = (time_t)(epoch_base_ms / 1000ULL);
      setTime(tt);
      sendLine(String("ACK:SET:TIME:") + String((unsigned long)epoch_base_ms));
    }
  } else if (s.equalsIgnoreCase("GET_TIME") ) {
    // Prefer RTC (TimeLib::now) if it appears to be set; otherwise use fallback
    time_t rtc_now = now();
    if (rtc_now > 100000) {
      sendLine(String("TIME_S:") + String((unsigned long)rtc_now));
    } else if (epoch_base_set_micros != 0) {
      uint64_t now_ms = epoch_base_ms + (uint64_t)((micros() - epoch_base_set_micros) / 1000ULL);
      unsigned long now_s = (unsigned long)(now_ms / 1000ULL);
      sendLine(String("TIME_S:") + String(now_s));
    } else {
      sendLine(String("TIME_UPTIME_S:") + String((unsigned long)(millis() / 1000UL)));
    }
  } else if (s.equalsIgnoreCase("GET_VTABLE") || s.equalsIgnoreCase("HAS_VTABLE")) {
    // Reply with the number of rows currently stored in the V_TABLE
    sendLine(String("ACK:VTABLE:LEN=") + String((unsigned long)vtable_len()));
  } else if (s.equalsIgnoreCase("BEGIN_MOVEDATA")) {
    // reset/prepare V_TABLE for incoming move rows
    vtable_reset();
    sendLine("ACK:MOVEDATA:BEGIN");
    // signal host it may send the first row
    sendLine("READY_FOR_ROW");
  } else if (s.startsWith("MOVE ")) {
    // format: MOVE <ms>,<speed>
    int sp = s.indexOf(' ');
    if (sp >= 0) {
      String rest = s.substring(sp+1);
      int comma = rest.indexOf(',');
      if (comma >= 0) {
        String tstr = rest.substring(0, comma);
        String sstr = rest.substring(comma+1);
        uint32_t t = (uint32_t) tstr.toInt();
        float vv = sstr.toFloat();
        if (vtable_push(t, vv)) {
          // acknowledge internally by sending READY_FOR_ROW for the next row only
          // (suppress per-row ACK to reduce upload chatter)
          sendLine("READY_FOR_ROW");
        } else {
          sendLine(String("ERR:MOVEDATA:MEM"));
        }
      }
    }
  } else if (s.equalsIgnoreCase("END_MOVEDATA")) {
    vtable_finalize();
    sendLine(String("ACK:MOVEDATA:END:") + String(vtable_len()));
  } else if (s.startsWith("REPS ")) {
    // REPS <n> -- number of experiment repetitions to run (1..N). 0 cancels repeats.
    int sp = s.indexOf(' ');
    if (sp >= 0) {
      int n = s.substring(sp+1).toInt();
      if (n < 0) n = 0;
      reps_total = n;
      reps_left = n;
      sendLine(String("ACK:REPS:") + String(reps_total));
    }
  } else {
    sendLine(String("ERR:UNKNOWN:") + s);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(UPPER_PIN, INPUT_PULLUP);
  pinMode(LOWER_PIN, INPUT_PULLUP);
  // attach interrupts for end switches (active-low -> FALLING)
  attachInterrupt(digitalPinToInterrupt(UPPER_PIN), isr_upper, FALLING);
  attachInterrupt(digitalPinToInterrupt(LOWER_PIN), isr_lower, FALLING);
  analogWriteResolution(8); // 0-255
  sendLine("Aufzug Motor Test: ready");
}

void loop() {
  // Serial parsing
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    parseCommand(line);
  }

  // Forward ISR notifications to host terminal (safe: done in main loop, not in ISR)
  if (upperMessagePending) {
    upperMessagePending = false;
    sendLine("ISR:UPPER");
  }
  if (lowerMessagePending) {
    lowerMessagePending = false;
    sendLine("ISR:LOWER");
  }

  // If moving, check runtime and switches
  if (moving) {
    // handle interrupts triggered by end switches (debounced in software)
    unsigned long now_us = micros();
    unsigned long now_ms = millis();
        if (upperTriggered) {
          upperTriggered = false;
      if (now_ms - lastUpperProcessed > DEBOUNCE_MS) {
        lastUpperProcessed = now_ms;
        stopMotor();
            sendLine("ERROR:UPPER");
            if (experiment_running) {
                  experiment_running = false;
                  sendLine("DONE:EXPERIMENT:REASON=UPPER");
            } else {
              sendLine("DONE:MOVE");
            }
        return;
      }
    }
    if (lowerTriggered) {
      lowerTriggered = false;
      if (now_ms - lastLowerProcessed > DEBOUNCE_MS) {
        lastLowerProcessed = now_ms;
        stopMotor();
        sendLine("ERROR:LOWER");
        if (experiment_running) {
          experiment_running = false;
          sendLine("DONE:EXPERIMENT:REASON=LOWER");
        } else {
          sendLine("DONE:MOVE");
        }
        return;
      }
    }

    // send ticks every tick_interval_us
    if (now_us - last_tick_us >= (unsigned long)tick_interval_us) {
      unsigned long elapsed_ms = (unsigned long)((now_us - move_start_us) / 1000ULL);
      // report elapsed ms and requested speed
      String tick = String("TICK") + String(elapsed_ms) + String(",") + String(speed_cm_s, 1); // in ms and cm/s
      sendLine(tick);
      last_tick_us = now_us;
    }

    // stop after configured duration as requested
    if (now_us - move_start_us >= run_duration_us) {
      stopMotor();
      sendLine("DONE:MOVE");
    }
  }

  // Experiment runner: step-driven updates every TIME_STEP_US
  if (experiment_running && V_TABLE_READY && vtable_len() > 0) {
    uint64_t nowe_us = (uint64_t)micros();
    // catch up loop: run one or more step ticks if we fell behind
    // Use addition to avoid unsigned underflow when last_step_us > nowe_us
    while (last_step_us + time_step_us <= nowe_us) {
      last_step_us += time_step_us;
      // elapsed for interpolation should be per-run and based on the step time
      uint64_t elapsed_ex_us = last_step_us - experiment_run_start_us;

      // advance index while next entry time <= elapsed (convert ms->us)
      while (experiment_index + 1 < vtable_len() && (uint64_t)V_TABLE[experiment_index + 1].t_ms * 1000ULL <= elapsed_ex_us) {
        experiment_index++;
      }

      // determine interpolation between current and next entry
      if (experiment_index < vtable_len()) {
        float target_speed = V_TABLE[experiment_index].speed_cm_s;
        if (experiment_index + 1 < vtable_len()) {
          uint32_t t0 = V_TABLE[experiment_index].t_ms;
          uint32_t t1 = V_TABLE[experiment_index + 1].t_ms;
          float v0 = V_TABLE[experiment_index].speed_cm_s;
          float v1 = V_TABLE[experiment_index + 1].speed_cm_s;
          uint64_t t0_us = (uint64_t)t0 * 1000ULL;
          uint64_t t1_us = (uint64_t)t1 * 1000ULL;
          if (t1_us > t0_us) {
            float frac = (float)(elapsed_ex_us - t0_us) / (float)(t1_us - t0_us);
            if (frac < 0.0f) frac = 0.0f;
            if (frac > 1.0f) frac = 1.0f;
            target_speed = v0 + (v1 - v0) * frac;
          }
        }

        // apply interpolated speed every step (even if unchanged we still report)
        experiment_current_speed = target_speed;
        speed_cm_s = experiment_current_speed;
        // compute direction and pwm
        bool dir_up = (speed_cm_s >= 0.0f);
        double pwm_freq = (fabs(speed_cm_s) / (PI * wheel_diam_cm)) * (double)ppr;
        if (pwm_freq < 1.0) pwm_freq = 1.0;
        // debug: report update (elapsed in ms) and free memory, include step timestamp
        unsigned long elapsed_ex_ms = (unsigned long)(elapsed_ex_us / 1000ULL);
        unsigned long ts_us = (unsigned long)last_step_us;
        unsigned long total_elapsed_us = 0;
        if (experiment_global_start_us != 0) {
          total_elapsed_us = (unsigned long)((uint64_t)last_step_us - (uint64_t)experiment_global_start_us);
        }
        sendLine(String("DBG:EXPERIMENT:UPDATE:IDX=") + String(experiment_index) + String(",ELAPSED=") + String(elapsed_ex_ms) + String(",TGT=") + String(target_speed,3) + String(",PWM=") + String(pwm_freq,2) + String(",DIR=") + String(dir_up ? "UP" : "DN") + String(",TS_US=") + String(ts_us) + String(",GLOBAL_US=") + String(total_elapsed_us));
        startMotor(pwm_freq, dir_up);
      }

      // finish if we've passed last entry of this run
      if (experiment_index + 1 >= vtable_len() && elapsed_ex_us >= (uint64_t)V_TABLE[experiment_index].t_ms * 1000ULL) {
        // one run completed
        // stopMotor();
        sendLine("DBG:EXPERIMENT:FINISHED");
        // decrement remaining repetitions (if set)
        if (reps_left > 0) reps_left--;
        // decide whether this is the overall experiment end
        // Use per-run elapsed (elapsed_ex_us) to decide timeout for this run.
        // Previously we compared overall global elapsed which caused the
        // experiment to stop immediately on subsequent repetitions once
        // the global elapsed exceeded run_duration_us.
        bool will_stop = (reps_left == 0) || (elapsed_ex_us >= run_duration_us);
        if (will_stop) {
          stopMotor();  
          const char *reason = (reps_left == 0) ? "REPS" : "TIMEOUT";
          sendLine(String("DONE:EXPERIMENT:REASON=") + String(reason));
          experiment_running = false;
          // reset global start so subsequent START_EXPERIMENT restarts timing
          experiment_global_start_us = 0;
        } else {
          // restart next run without stopping the motor
          sendLine(String("DBG:EXPERIMENT:RESTART:LEFT=") + String(reps_left));
          experiment_running = true;
          experiment_index = 0;
          // schedule next run start relative to current time to avoid starting mid-step
          experiment_run_start_us = (uint64_t)nowe_us + (uint64_t)time_step_us;
          last_step_us = experiment_run_start_us;
          // keep experiment_current_speed as-is and DO NOT call stopMotor()
          // notify start of next run
          sendLine(String("DBG:EXPERIMENT:START:LEN=") + String(vtable_len()));
        }
        break; // exit catch-up loop after handling end-of-run
      }
    } // while catch-up
  }

  // drive LED flash (non-blocking)
  if (millis() < ledFlashUntil) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}