// WEB PID SLIDER -- ESP32-WROOM / ESP32 Dev Module (dual core)
// Hotspot: Fahrez | Password di WIFI_PASS | Buka IP yang tampil di Serial Monitor
// Pin motor/encoder follows the supplied board. Enables assumed active onboard.
// PID control runs in its own 50-ms task; web/serial commands go through a queue.
// USB Serial Monitor: 115200 baud, Newline.
// Disconnect from the browser for >5 s stops a motor started from the web.
// This is MANUAL slider tuning, with no automatic parameter search.
// Encoder PPR/gear ratio must be calibrated on actual hardware.

// ======================================================
// PID KECEPATAN MOTOR DC JGA25-370 12V
// ESP32 + BTS7960 + ENCODER + WEB SLIDER PID
//
// MOTOR DIPASANG PADA PORT MOTOR 1
//
// Motor 1 input A : GPIO 25
// Motor 1 input B : GPIO 26
// Encoder A       : GPIO 13
// Encoder B       : GPIO 14
//
// Motor 2 dan kicker tidak digunakan.
//
// Serial Monitor:
// Baud rate   = 115200
// Line ending = Newline
//
// PERINTAH:
// s100         -> Setpoint 100 RPM
// s-100        -> Putar balik
// s0           -> Berhenti
// p1.2         -> Atur Kp
// i4.0         -> Atur Ki
// d0.005       -> Atur Kd
// k1.2,4,0.005 -> Atur Kp, Ki, Kd sekaligus
// m40          -> Atur PWM minimum
// t            -> Uji step 100 RPM
// t150         -> Uji step 150 RPM
// w            -> Simpan parameter
// x            -> Reset parameter ke default
// ?            -> Tampilkan parameter
// ======================================================

#include <Arduino.h>
#include <Preferences.h>
#include <esp_arduino_version.h>
#include <math.h>
// ===== Web panel, log mirror and task handoff =====
// Variables below are defined before the original controller functions.
#include <WiFi.h>
#include <WebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <atomic>
#include <stdarg.h>
#include <esp_system.h>

const char* WIFI_SSID="Fahrez";
const char* WIFI_PASS="12345678";
WebServer server(80);
constexpr int LOG_COUNT=128;
struct WebLog {uint32_t seq=0,ms=0;char text[192]={0};};
struct WebState {
  float kp=1.2f,ki=4.f,kd=.005f,sp=0,rpm=0,over=0,sse=0,testTarget=0;
  int pwm=0,minimum=40,rise=-1,settle=-1,response=0;
  bool step=false,fault=false,metric=false;
};
struct WebCommand {char text[96];bool web;};
SemaphoreHandle_t webMutex;
QueueHandle_t commandQueue;
WebLog webLogs[LOG_COUNT];
WebState published;
uint32_t logSequence=0,bootId=0;
std::atomic<bool> stopRequest{false};
std::atomic<uint32_t> lastWebPoll{0};
bool webControlsMotor=false;

void controlTask(void*);
void startWebServer();
void publishState();
void executeCommand(String line);

void logLine(const char* text="") {
  char cleaned[192];snprintf(cleaned,sizeof(cleaned),"%s",text);
  size_t n=strlen(cleaned);
  while(n&&(cleaned[n-1]=='\n'||cleaned[n-1]=='\r'))cleaned[--n]=0;
  if(webMutex){
    xSemaphoreTake(webMutex,portMAX_DELAY);
    uint32_t seq=++logSequence;
    WebLog& item=webLogs[(seq-1)%LOG_COUNT];
    item.seq=seq;item.ms=millis();
    snprintf(item.text,sizeof(item.text),"%s",cleaned);
    xSemaphoreGive(webMutex);
  }
  // Never wait for a full USB/UART TX buffer in the control task.
  if(Serial.availableForWrite()>=(int)n+2)Serial.println(cleaned);
}
void logPrintf(const char* fmt,...) {
  char text[192];va_list args;va_start(args,fmt);
  vsnprintf(text,sizeof(text),fmt,args);va_end(args);logLine(text);
}
bool enqueueCommand(const String& line,bool web) {
  if(line.length()==0||line.length()>=96)return false;
  if(line=="s0"||line=="0"||line=="!"){
    stopRequest.store(true);return true;
  }
  WebCommand c={};line.toCharArray(c.text,sizeof(c.text));c.web=web;
  return xQueueSend(commandQueue,&c,0)==pdTRUE;
}
bool readNumber(const String& text,float& value){
  if(!text.length())return false;
  char* end=nullptr;value=strtof(text.c_str(),&end);
  return end!=text.c_str()&&*end=='\0'&&isfinite(value);
}
bool commandValid(String text){
  text.trim();if(text=="?"||text=="w"||text=="x"||text=="t"||text=="!")return true;
  if(!text.length()||text.length()>=96)return false;
  char c=text[0];String a=text.substring(1);a.trim();float v=0;
  if(c=='k'){
    int x=a.indexOf(','),y=a.indexOf(',',x+1);float p,i,d;
    return x>0&&y>x+1&&a.indexOf(',',y+1)<0&&
      readNumber(a.substring(0,x),p)&&readNumber(a.substring(x+1,y),i)&&
      readNumber(a.substring(y+1),d)&&p>=0&&p<=5&&i>=0&&i<=5&&d>=0&&d<=5;
  }
  if(isDigit(c)||c=='-')return readNumber(text,v)&&fabs(v)<=300;
  if(!readNumber(a,v))return false;
  if(c=='s')return fabs(v)<=300;
  if(c=='t')return v>=1&&v<=300;
  if(c=='p'||c=='i')return v>=0&&v<=5;
  if(c=='d')return v>=0&&v<=5;
  if(c=='m')return v>=0&&v<=120;
  return false;
}


// ================= PIN ESP32 =================

#define MOTOR_PWM_A  25
#define MOTOR_PWM_B  26

#define ENC_A        13
#define ENC_B        14

#define MOTOR2_A     32
#define MOTOR2_B     33
#define KICKER_PIN   27

// Ubah menjadi -1 jika arah pembacaan encoder terbalik.
const int ENCODER_SIGN = 1;

// ================= PWM =================

const int PWM_FREQ = 5000;
const int PWM_RES  = 8;
const int PWM_MAX  = 255;

// Channel untuk Arduino-ESP32 versi 2.x.
const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;

// ================= MOTOR =================

// Pertahankan dari kode sebelumnya.
// Sesuaikan jika spesifikasi encoder/gearbox berbeda.
const float PPR_MOTOR  = 11.0f;
const float GEAR_RATIO = 34.0f;

// Kedua channel dibaca pada CHANGE: decoding x4.
// Berlaku jika PPR_MOTOR adalah jumlah siklus
// satu channel per putaran poros motor.
const float PULSE_PER_REV =
    PPR_MOTOR * 4.0f * GEAR_RATIO;

// ================= PARAMETER PID =================

const float DEF_KP = 0.4f;
const float DEF_KI = 0.8f;
const float DEF_KD = 0.0f;

const int DEF_PWMMIN = 40;

float Kp = DEF_KP;
float Ki = DEF_KI;
float Kd = DEF_KD;

int pwmMin = DEF_PWMMIN;

float setpointRPM = 0.0f;

const float INTEGRAL_LIMIT = 150.0f;
const unsigned long SAMPLE_MS = 50;

const float MAX_RPM_LIMIT = 400.0f;
const unsigned long STALL_TIMEOUT_MS = 2000;

// ================= PENGUJIAN STEP =================

const float DEF_STEP_RPM = 100.0f;

const unsigned long STEP_ON_MS  = 6000;
const unsigned long STEP_OFF_MS = 3000;

// Pita settling time +/-5% dari target.
const float SETTLE_BAND = 0.05f;

// Rata-rata error pada 1 detik terakhir pengujian.
const unsigned long SS_WINDOW_MS = 1000;

bool stepRunning = false;
bool metricsDone = false;

unsigned long stepStart = 0;

float stepTarget = DEF_STEP_RPM;
float startRPM = 0.0f;
float peakRPM = 0.0f;

unsigned long tRise10 = 0;
unsigned long tRise90 = 0;
unsigned long tPeak   = 0;
unsigned long tSettle = 0;
unsigned long metricLastElapsed = 0;
// Extra statistics for practical response-shape estimation.
unsigned int responseSamples=0,overRun=0,maxOverRun=0;
unsigned int responseCrossings=0;
int responseLastSide=0;
// Project-specific heuristic, not a definition of critical damping.
const unsigned long RESPONSE_FAST_SETTLE_MS=2000;

bool got10 = false;
bool got90 = false;

bool inBandNow = false;
bool hasSettleCandidate = false;
bool hasMetricSample = false;

float ssSumErr = 0.0f;
float ssSumRpm = 0.0f;
int ssCount = 0;

// ================= VARIABEL SISTEM =================

Preferences prefs;

volatile long encoderCount = 0;
long lastCount = 0;

float integral = 0.0f;
float lastError = 0.0f;
float rpmFiltered = 0.0f;

int lastOutput = 0;

unsigned long lastTime = 0;
unsigned long stallStart = 0;

bool faultLatched = false;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ================= INTERRUPT ENCODER =================

void IRAM_ATTR isrA() {
  bool a = digitalRead(ENC_A);
  bool b = digitalRead(ENC_B);

  portENTER_CRITICAL_ISR(&mux);

  if (a == b) {
    encoderCount++;
  } else {
    encoderCount--;
  }

  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR isrB() {
  bool a = digitalRead(ENC_A);
  bool b = digitalRead(ENC_B);

  portENTER_CRITICAL_ISR(&mux);

  if (a != b) {
    encoderCount++;
  } else {
    encoderCount--;
  }

  portEXIT_CRITICAL_ISR(&mux);
}

// ================= PWM BTS7960 =================

void writePWM_A(int duty) {
  duty = constrain(duty, 0, PWM_MAX);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PWM_A, duty);
#else
  ledcWrite(PWM_CHANNEL_A, duty);
#endif
}

void writePWM_B(int duty) {
  duty = constrain(duty, 0, PWM_MAX);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PWM_B, duty);
#else
  ledcWrite(PWM_CHANNEL_B, duty);
#endif
}

bool setupPWM() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3

  bool okA = ledcAttach(
    MOTOR_PWM_A, PWM_FREQ, PWM_RES
  );

  bool okB = ledcAttach(
    MOTOR_PWM_B, PWM_FREQ, PWM_RES
  );

  return okA && okB;

#else

  double freqA = ledcSetup(
    PWM_CHANNEL_A, PWM_FREQ, PWM_RES
  );

  double freqB = ledcSetup(
    PWM_CHANNEL_B, PWM_FREQ, PWM_RES
  );

  ledcAttachPin(MOTOR_PWM_A, PWM_CHANNEL_A);
  ledcAttachPin(MOTOR_PWM_B, PWM_CHANNEL_B);

  return freqA > 0 && freqB > 0;

#endif
}

// ================= KONTROL MOTOR =================

// PWM positif : input A aktif, input B nol.
// PWM negatif : input B aktif, input A nol.
// PWM nol     : kedua input nol.

void setMotor(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  lastOutput = pwm;

  if (pwm > 0) {
    // Nolkan sisi lawan sebelum mengaktifkan sisi tujuan.
    writePWM_B(0);
    writePWM_A(pwm);

  } else if (pwm < 0) {
    writePWM_A(0);
    writePWM_B(-pwm);

  } else {
    writePWM_A(0);
    writePWM_B(0);
  }
}

// ================= RESET PID =================

void resetPID() {
  integral = 0.0f;
  lastError = 0.0f;
  stallStart = 0;
}

// ================= SIMPAN PARAMETER =================

void saveParams() {
  prefs.begin("pid", false);

  prefs.putFloat("kp", Kp);
  prefs.putFloat("ki", Ki);
  prefs.putFloat("kd", Kd);
  prefs.putInt("pwmmin", pwmMin);

  prefs.end();

  logLine("Parameter berhasil disimpan.");
}

void loadParams() {
  prefs.begin("pid", true);

  Kp = prefs.getFloat("kp", DEF_KP);
  Ki = prefs.getFloat("ki", DEF_KI);
  Kd = prefs.getFloat("kd", DEF_KD);
  pwmMin = prefs.getInt("pwmmin", DEF_PWMMIN);

  prefs.end();
  if(!isfinite(Kp)||Kp<0||Kp>1000)Kp=DEF_KP;
  if(!isfinite(Ki)||Ki<0||Ki>1000)Ki=DEF_KI;
  if(!isfinite(Kd)||Kd<0||Kd>100)Kd=DEF_KD;
  Kp=constrain(Kp,0.0f,5.0f);
  Ki=constrain(Ki,0.0f,5.0f);
  Kd=constrain(Kd,0.0f,5.0f);
  pwmMin=constrain(pwmMin,0,120);
}

// ================= TAMPILKAN PARAMETER =================

void printParams() {
  logLine();
  logLine("========== PARAMETER PID ==========");

  logPrintf("Setpoint : %.1f RPM\n", setpointRPM);
  logPrintf("Kp       : %.4f\n", Kp);
  logPrintf("Ki       : %.4f\n", Ki);
  logPrintf("Kd       : %.4f\n", Kd);
  logPrintf("PWM min  : %d\n", pwmMin);

  logLine("Motor 1  : GPIO 25 dan 26");
  logLine("Encoder  : GPIO 13 dan 14");
  logLine("Perintah : s, p, i, d, k, m, t, w, x, ?");
  logLine("===================================");
}

// ================= ATUR SETPOINT =================

void setSetpoint(float value) {
  setpointRPM = constrain(
    value, -MAX_RPM_LIMIT, MAX_RPM_LIMIT
  );

  faultLatched = false;
  resetPID();

  // Perintah berhenti langsung menolkan keluaran.
  if (setpointRPM == 0.0f) {
    setMotor(0);
  }
}

// ================= MULAI PENGUKURAN =================

void startMetrics(float target) {
  stepTarget = target;

  startRPM = rpmFiltered;
  peakRPM = -1.0e9f;

  metricLastElapsed = 0;
  responseSamples=overRun=maxOverRun=responseCrossings=0;
  responseLastSide=0;
  tRise10 = 0;
  tRise90 = 0;
  tPeak   = 0;
  tSettle = 0;

  got10 = false;
  got90 = false;

  inBandNow = false;
  hasSettleCandidate = false;
  hasMetricSample = false;

  ssSumErr = 0.0f;
  ssSumRpm = 0.0f;
  ssCount = 0;

  metricsDone = false;
}

// ================= UPDATE METRIK =================

// Pengujian t menggunakan target positif.
void updateMetrics(unsigned long elapsedMs, float rpm) {
  if (elapsedMs > STEP_ON_MS) {
    return;
  }

  hasMetricSample = true;
  metricLastElapsed = elapsedMs;
  ++responseSamples;
  // Three consecutive samples above +5% reject a single noisy peak.
  if(rpm>stepTarget*1.05f){
    ++overRun;if(overRun>maxOverRun)maxOverRun=overRun;
  }else overRun=0;
  // Crossings count only excursions outside the +/-5% band.
  int side=rpm>stepTarget*1.05f?1:(rpm<stepTarget*.95f?-1:0);
  if(side){
    if(responseLastSide && side!=responseLastSide)++responseCrossings;
    responseLastSide=side;
  }

  // 1. Rise time 10% sampai 90%.
  if (!got10 && rpm >= 0.10f * stepTarget) {
    got10 = true;
    tRise10 = elapsedMs;
  }

  if (!got90 && rpm >= 0.90f * stepTarget) {
    got90 = true;
    tRise90 = elapsedMs;
  }

  // 2. Nilai puncak.
  if (rpm > peakRPM) {
    peakRPM = rpm;
    tPeak = elapsedMs;
  }

  // 3. Settling time dengan pita +/-5%.
  float band = SETTLE_BAND * stepTarget;
  bool inBand = fabs(rpm - stepTarget) <= band;

  if (inBand) {
    if (!inBandNow) {
      tSettle = elapsedMs;
      hasSettleCandidate = true;
    }

    inBandNow = true;

  } else {
    inBandNow = false;
    hasSettleCandidate = false;
    tSettle = 0;
  }

  // 4. Rata-rata error pada 1 detik terakhir.
  if (elapsedMs >= STEP_ON_MS - SS_WINDOW_MS) {
    ssSumErr += stepTarget - rpm;
    ssSumRpm += rpm;
    ssCount++;
  }
}

// ================= ESTIMASI BENTUK RESPONS =================
// This is a descriptive heuristic for the 6-s experiment, NOT an identified
// damping ratio. PID zeros, load and filtering can change response shape.
// +/-5% settling is a performance tolerance, not a definition of zeta = 1.
// Codes: 0 insufficient, 1 under-like, 2 over-like, 3 near-critical-like,
//        4 unfinished, 5 steady error, 6 invalid starting condition.
int classifyResponse(){
  if(!hasMetricSample || responseSamples<80 || metricLastElapsed<5500 ||
     stepTarget<=0 || ssCount<10)return 0;
  if(fabs(startRPM)>5.f)return 6;
  float overshoot=fmaxf(0.f,(peakRPM-stepTarget)/stepTarget*100.f);
  if(maxOverRun>=3 || responseCrossings>=3)return 1;
  // A lone overshoot peak is inconclusive rather than critical/overdamped.
  if(overshoot>5.f)return 0;
  bool settled=inBandNow && hasSettleCandidate &&
    metricLastElapsed>=tSettle && metricLastElapsed-tSettle>=1000;
  if(!settled || !got90)return 4;
  if(fabs(ssSumErr/ssCount)>stepTarget*.05f)return 5;
  return tSettle<=RESPONSE_FAST_SETTLE_MS?3:2;
}
const char* responseLabel(int code){
  switch(code){
    case 1:return "Underdamped (indikasi overshoot/osilasi)";
    case 2:return "Cenderung overdamped (estimasi)";
    case 3:return "Mendekati critically damped (estimasi)";
    case 4:return "Belum dapat ditentukan: belum settle dalam 6 detik";
    case 5:return "Belum dapat ditentukan: error akhir masih besar";
    case 6:return "Ulangi pengujian: motor belum diam saat mulai";
    default:return "Belum dapat ditentukan: data tidak cukup atau puncak sesaat";
  }
}

// ================= CETAK HASIL PENGUJIAN =================

void printMetrics() {
  logLine();
  logLine("==============================================");
  logLine("         HASIL ANALISIS RESPONS STEP");
  logLine("==============================================");

  logPrintf("Target RPM              : %.2f RPM\n", stepTarget);
  logPrintf("Kp                      : %.4f\n", Kp);
  logPrintf("Ki                      : %.4f\n", Ki);
  logPrintf("Kd                      : %.4f\n", Kd);
  logPrintf("PWM Minimum             : %d\n", pwmMin);

  if (!hasMetricSample) {
    logLine("Data respons belum tersedia.");
    logLine("==============================================");
    return;
  }

  if (fabs(startRPM) > 5.0f) {
    logPrintf(
      "PERINGATAN: Motor belum diam saat mulai (%.2f RPM)\n",
      startRPM
    );
  }

  if (got10 && got90 && tRise90 >= tRise10) {
    logPrintf(
      "Rise Time (10-90%%)      : %lu ms\n",
      tRise90 - tRise10
    );

    logPrintf(
      "Waktu mencapai 10%%      : %lu ms\n",
      tRise10
    );

    logPrintf(
      "Waktu mencapai 90%%      : %lu ms\n",
      tRise90
    );
  } else {
    logLine("Rise Time               : Tidak tercapai");
  }

  float overshoot = 0.0f;

  if (stepTarget > 0.0f && peakRPM > stepTarget) {
    overshoot =
      ((peakRPM - stepTarget) / stepTarget) * 100.0f;
  }

  logPrintf(
    "Overshoot               : %.2f %%\n",
    overshoot
  );

  logPrintf(
    "RPM Puncak              : %.2f RPM\n",
    peakRPM
  );

  logPrintf(
    "Waktu RPM Puncak        : %lu ms\n",
    tPeak
  );

  if (inBandNow && hasSettleCandidate &&
      metricLastElapsed >= tSettle && metricLastElapsed - tSettle >= 1000) {
    logPrintf(
      "Settling Time (+/-5%%)   : %lu ms\n",
      tSettle
    );

    logLine(
      "                         Tetap di pita sampai akhir uji."
    );
  } else {
    logPrintf(
      "Settling Time (+/-5%%)   : Tidak settle dalam %lu ms\n",
      STEP_ON_MS
    );
  }

  if (ssCount > 0 && stepTarget > 0.0f) {
    float ssError = ssSumErr / ssCount;
    float meanRPM = ssSumRpm / ssCount;
    float ssErrorPct = (ssError / stepTarget) * 100.0f;

    logPrintf(
      "Steady-State Error      : %.2f RPM\n",
      ssError
    );

    logPrintf(
      "SSE Persentase          : %.2f %%\n",
      ssErrorPct
    );

    logPrintf(
      "RPM Rata-rata (1 s)     : %.2f RPM\n",
      meanRPM
    );
  } else {
    logLine(
      "Steady-State Error      : Data tidak cukup"
    );
  }

  logPrintf("Estimasi respons        : %s",responseLabel(classifyResponse()));
  logLine("Heuristik: overshoot >5% terkonfirmasi/osilasi = indikasi underdamped.");
  logLine("Overshoot <=5% + settle <=2 s = mendekati kritis; >2 s = cenderung overdamped.");
  logLine("Ini estimasi bentuk respons, bukan pengukuran rasio redaman zeta.");
  logLine("==============================================");
  logLine();
}

// ================= PERINTAH SERIAL =================

void executeCommand(String line) {
  line.trim();
  if(!line.length()) return;
  char cmd = line.charAt(0);

  String arg = line.substring(1);
  arg.trim();

  float val = arg.toFloat();

  switch (cmd) {
    case 's':
      stepRunning = false;
      setSetpoint(val);

      logPrintf(
        "Setpoint = %.1f RPM\n", setpointRPM
      );
      break;

    case 'p':
    case 'i':
    case 'd':
      if (val < 0 || !isfinite(val)) {
        logLine("Nilai PID harus valid dan tidak negatif.");
        break;
      }

      if (cmd == 'p') Kp = val;
      if (cmd == 'i') Ki = val;
      if (cmd == 'd') Kd = val;

      resetPID();
      printParams();
      break;

    case 'k': {
      int c1 = arg.indexOf(',');
      int c2 = arg.indexOf(',', c1 + 1);

      if (c1 < 0 || c2 < 0) {
        logLine("Format salah. Contoh: k1.2,4,0.005");
        break;
      }

      float nkp = arg.substring(0, c1).toFloat();
      float nki = arg.substring(c1 + 1, c2).toFloat();
      float nkd = arg.substring(c2 + 1).toFloat();

      if (!isfinite(nkp) || !isfinite(nki) ||
          !isfinite(nkd) ||
          nkp < 0 || nki < 0 || nkd < 0) {
        logLine("Kp, Ki, Kd harus valid dan tidak negatif.");
        break;
      }

      Kp = nkp;
      Ki = nki;
      Kd = nkd;

      resetPID();
      printParams();
      break;
    }

    case 'm':
      pwmMin = constrain((int)val, 0, 120);
      logPrintf("PWM min = %d\n", pwmMin);
      break;

    case 't': {
      float target = (val > 0)
        ? constrain(val, 1.0f, MAX_RPM_LIMIT)
        : DEF_STEP_RPM;

      startMetrics(target);

      stepRunning = true;
      stepStart = millis();

      setSetpoint(target);

      logLine();
      logLine("========== UJI STEP DIMULAI ==========");

      logPrintf(
        "Setpoint Step : 0 -> %.1f RPM\n", target
      );

      logPrintf(
        "Fase naik     : %lu ms\n", STEP_ON_MS
      );

      logPrintf(
        "Fase berhenti : %lu ms\n", STEP_OFF_MS
      );

      logLine("======================================");
      break;
    }

    case 'w':
      saveParams();
      break;

    case 'x':
      Kp = DEF_KP;
      Ki = DEF_KI;
      Kd = DEF_KD;
      pwmMin = DEF_PWMMIN;

      resetPID();

      logLine("Parameter di-reset ke default.");
      printParams();
      break;

    case '?':
      printParams();
      break;

    default:
      if (isDigit(cmd) || cmd == '-') {
        stepRunning = false;
        setSetpoint(line.toFloat());

        logPrintf(
          "Setpoint = %.1f RPM\n", setpointRPM
        );
      } else {
        logLine("Perintah tidak dikenal. Ketik ?");
      }
      break;
  }
}

const char INDEX_HTML[] PROGMEM = R"PIDWEB(<!doctype html>
<html lang="id"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 PID Motor</title><style>
:root{font-family:system-ui,sans-serif;color:#dce8f5;background:#0c1422}*{box-sizing:border-box}body{margin:0}main{max-width:1180px;margin:auto;padding:25px}header{display:flex;justify-content:space-between;align-items:center;gap:14px}h1{font-size:26px;margin:6px 0}h2{font-size:17px;margin:0 0 18px}p{color:#a5b6cc;line-height:1.6;font-size:14px}.badge{border:1px solid #34455e;border-radius:30px;padding:8px 14px;font-size:13px;white-space:nowrap}.grid{display:grid;grid-template-columns:330px 1fr;gap:18px;margin-top:20px}.panel{border:1px solid #27364b;background:#121f31;border-radius:16px;padding:22px}.slider{margin:25px 0}.slider label{display:flex;justify-content:space-between;font-weight:600}.slider output{color:#69e3b5;font-variant-numeric:tabular-nums}input[type=range]{width:100%;accent-color:#60dbab;margin:18px 0 0;height:22px;cursor:ew-resize}input[type=number]{background:#091525;color:#eef6ff;border:1px solid #394d68;border-radius:8px;padding:9px;width:100%;font:inherit}.control label{display:block;color:#abbdd2;font-size:13px;margin-bottom:7px}button{border:1px solid #38516d;background:#1b3049;color:#e5f0fc;border-radius:9px;padding:11px 12px;font:600 13px system-ui;cursor:pointer}button:hover{filter:brightness(1.16)}button:disabled{opacity:.45;cursor:default}.buttons{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:12px}.primary{background:#54d4a4;color:#092518;border-color:#54d4a4}.stop{background:#dc4c5a;border-color:#dc4c5a;color:white;grid-column:1/-1;font-size:16px}.numbers{display:grid;grid-template-columns:repeat(3,1fr);gap:12px}.stat{color:#99adc7;font-size:12px}.stat strong{display:block;font-size:27px;color:#eff7ff;margin-top:5px;font-variant-numeric:tabular-nums}.legend{font-size:12px;color:#acbed2;display:flex;gap:14px;flex-wrap:wrap;margin:18px 0 8px}.legend span:before{content:'●';color:var(--c);margin-right:6px}canvas{width:100%;height:275px;display:block}.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:9px;border-top:1px solid #2c3b50;padding-top:16px;margin-top:12px}.metrics strong{font-size:17px}.console{margin-top:18px}.console-head{display:flex;align-items:center;justify-content:space-between;gap:10px;flex-wrap:wrap}pre{background:#07101c;border:1px solid #24344b;border-radius:9px;height:235px;overflow:auto;padding:14px;white-space:pre-wrap;word-break:break-word;font:12px/1.6 monospace;color:#9ee4c2}.small{font-size:12px;color:#8fa4be}.row{display:flex;gap:8px;margin-top:10px}.row input{flex:1;min-width:0;background:#091525;color:#fff;border:1px solid #394d68;border-radius:8px;padding:10px;font:13px monospace}#notice{min-height:22px;color:#f6cd85;font-size:13px}#applied{font:12px/1.6 monospace;color:#9cb5cc}.full{grid-column:1/-1}.mini{display:flex;align-items:center;justify-content:space-between;gap:8px}.mini input{width:85px;padding:5px;font-size:12px}.check{font-size:12px;color:#aabed6}details{margin-top:18px;color:#9db4cc;font-size:12px}summary{cursor:pointer}details p{font-size:12px}footer{margin-top:12px;font-size:12px;color:#8398b3}@media(max-width:760px){main{padding:15px}.grid{grid-template-columns:1fr}h1{font-size:22px}.metrics{grid-template-columns:1fr 1fr}.panel{padding:17px}header{align-items:flex-start}canvas{height:240px}}
</style></head><body><main>
<header><div><div class="small">ESP32 · BTS7960 · ENCODER</div><h1>Kontrol PID Motor</h1><p>Geser parameter, amati respons motor secara langsung.</p></div><span class="badge" id="connection">Menghubungkan…</span></header>
<div class="grid"><section class="panel"><h2>Parameter PID</h2>
<label class="small" for="scale">Skala slider</label><select id="scale" style="width:100%;margin:8px 0;background:#091525;color:#eef6ff;padding:9px;border:1px solid #394d68;border-radius:8px"><option value="fine">Halus — fokus angka kecil</option><option value="wide">Luas — semua 0 sampai 5</option></select>
<div class="slider"><label for="kp">Kp <output id="vp">0.400</output></label><input id="kp" type="range" min="0" max="2" step="0.001" value="0.4"><input aria-label="Nilai Kp presisi" id="np" type="number" min="0" max="5" step="0.001" value="0.4"></div>
<div class="slider"><label for="ki">Ki <output id="vi">0.800</output></label><input id="ki" type="range" min="0" max="5" step="0.001" value="0.8"><input aria-label="Nilai Ki presisi" id="ni" type="number" min="0" max="5" step="0.001" value="0.8"></div>
<div class="slider"><label for="kd">Kd <output id="vd">0.00000</output></label><input id="kd" type="range" min="0" max="0.05" step="0.00001" value="0"><input aria-label="Nilai Kd presisi" id="nd" type="number" min="0" max="5" step="0.00001" value="0"></div>
<div id="applied">Menunggu parameter ESP32…</div>
<p class="small">Mode halus: Kp 0–2, Ki 0–5, Kd 0–0,05. Nilai presisi dapat diketik. Mode luas tetap tersedia sampai 5.</p>
<div class="buttons"><button id="trialA">A · Awal pelan</button><button id="trialB">B · Lebih agresif</button><button id="trialC">C · Dengan D</button></div>
<p class="small">Set awal percobaan, belum dikalibrasi pada motor ini: A = 0,4 / 0,8 / 0; B = 1,2 / 4 / 0; C = 0,8 / 1,6 / 0,01. Tombol hanya mengatur PID. Jalankan uji step untuk melihat hasil sebenarnya.</p>
<div class="control" style="margin-top:24px"><label for="target">Target RPM (−300 sampai 300)</label><input id="target" type="number" min="-300" max="300" value="100" step="1"></div>
<div class="buttons"><button class="primary" id="run">Jalankan motor</button><button id="step">Uji step 6 detik</button><button class="stop" id="stop">STOP MOTOR</button><button id="save">Simpan PID</button><button id="reset">PID default</button></div>
<p class="small">Slider mengubah PID di RAM. Tombol Simpan menulis ke memori ESP32. Jika parameter diubah saat uji step, uji dibatalkan agar hasil tidak mencampur dua set PID.</p><div id="notice" role="status"></div>
</section><section class="panel"><h2>Respons kecepatan</h2><div class="numbers"><div class="stat">TARGET<strong id="sp">0.0</strong>RPM</div><div class="stat">AKTUAL<strong id="rpm">0.0</strong>RPM</div><div class="stat">KELUARAN<strong id="pwm">0</strong>PWM</div></div>
<div class="legend"><span style="--c:#f7bf61">Target</span><span style="--c:#5bdfb2">RPM terfilter</span><span style="--c:#6f91b8">Pita ±5%</span></div><canvas id="chart" aria-label="Grafik target dan RPM, 20 detik terakhir"></canvas>
<div class="small" id="state">Motor berhenti · Grafik 20 detik terakhir</div>
<div class="metrics"><div class="stat">RISE TIME<strong id="rise">—</strong></div><div class="stat">OVERSHOOT<strong id="over">—</strong></div><div class="stat">SETTLING ±5%<strong id="settle">—</strong></div><div class="stat">ERROR AKHIR<strong id="sse">—</strong></div></div><p class="small" id="metricnote">Metrik muncul setelah uji step selesai. Dihitung dari RPM terfilter.</p><p class="small">ESTIMASI KARAKTER RESPONS<br><strong id="response" style="color:#e4f2ff">Menunggu uji step</strong></p><p class="small">Label merupakan heuristik bentuk respons, bukan identifikasi rasio redaman. Batas praktikum: overshoot 5%, settling cepat ≤2 detik; settling harus bertahan minimal 1 detik.</p>
</section><section class="panel console full"><div class="console-head"><h2 style="margin:0">Serial Monitor Web</h2><div><label class="check"><input id="scroll" type="checkbox" checked> Gulir otomatis</label> <button id="csv">Unduh CSV</button> <button id="clear">Bersihkan tampilan</button></div></div><pre id="terminal" aria-live="off">Menunggu data ESP32…</pre><div class="row"><input id="command" aria-label="Perintah serial" placeholder="Contoh: s100, k1.2,4,0.005, t100, m40, ?, w"><button id="send">Kirim</button></div><p class="small">Data yang sama dikirim ke USB Serial 115200 baud. CSV berisi sampel yang diterima browser; celah data ditandai di monitor.</p></section></div>
<footer>Motor 1: GPIO 25/26 · Encoder: GPIO 13/14 · Halaman bekerja tanpa internet. Motor dari web berhenti jika koneksi halaman terputus lebih dari 5 detik.</footer>
</main><script>
const el=id=>document.getElementById(id);let boot=null,since=0,initialized=false,rows=[],lines=[],connected=false,lastGood=0,pendingTimer=null,lastSend=0,editing=false,pidBusy=false,sendAgain=false,pidVersion=0;const sliders=['kp','ki','kd'];
const fineMax={kp:2,ki:5,kd:0.05};
function labels(){el('vp').value=(+el('kp').value).toFixed(3);el('vi').value=(+el('ki').value).toFixed(3);el('vd').value=(+el('kd').value).toFixed(5);el('np').value=el('kp').value;el('ni').value=el('ki').value;el('nd').value=el('kd').value}
function setGainsUI(values){
  const needWide=values.some((v,i)=>v>fineMax[sliders[i]]);
  if(needWide)el('scale').value='wide';
  for(let i=0;i<sliders.length;i++){const id=sliders[i];el(id).max=el('scale').value==='wide'?5:fineMax[id];el(id).value=Math.min(5,Math.max(0,values[i]))}
  labels();
}
function log(text){lines.push(text);if(lines.length>500)lines.splice(0,lines.length-500);el('terminal').textContent=lines.join('\n');if(el('scroll').checked)el('terminal').scrollTop=el('terminal').scrollHeight}
async function command(c){if(!connected)throw Error('ESP32 belum terhubung.');const r=await fetch('/command',{method:'POST',body:new URLSearchParams({cmd:c}),signal:AbortSignal.timeout(2500)});const text=await r.text();if(!r.ok)throw Error(text);el('notice').textContent=text;return text}
function message(e){el('notice').textContent=e.message||e}
async function sendPID(){clearTimeout(pendingTimer);pendingTimer=null;if(pidBusy){sendAgain=true;return}pidBusy=true;sendAgain=false;const version=pidVersion;try{await command('k'+sliders.map(id=>Number(el(id).value)).join(','));lastSend=Date.now()}catch(e){message(e)}finally{pidBusy=false;if(sendAgain||version!==pidVersion)pendingTimer=setTimeout(sendPID,120)}}
for(const id of sliders){el(id).addEventListener('input',()=>{editing=true;pidVersion++;labels();if(!pendingTimer)pendingTimer=setTimeout(sendPID,100)});el(id).addEventListener('change',()=>{editing=false;clearTimeout(pendingTimer);sendPID()})}
el('scale').onchange=()=>{
  const values=sliders.map(id=>Number(el(id).value));
  if(el('scale').value==='fine'&&values.some((v,i)=>v>fineMax[sliders[i]])){
    el('scale').value='wide';message('Nilai saat ini melewati rentang halus. Turunkan nilainya melalui kolom angka dahulu.');
  }
  setGainsUI(values);
};
for(const [number,id]of [['np','kp'],['ni','ki'],['nd','kd']]){
  el(number).addEventListener('focus',()=>{editing=true});
  el(number).addEventListener('blur',()=>{editing=false});
  el(number).addEventListener('change',()=>{
    const text=el(number).value,v=Number(text);
    if(text===''||!Number.isFinite(v)||v<0||v>5){message('Nilai PID harus 0 sampai 5.');labels();editing=false;return}
    const values=sliders.map(key=>key===id?v:Number(el(key).value));setGainsUI(values);
    editing=false;pidVersion++;sendPID();
  });
}
for(const [button,values]of [['trialA',[0.4,0.8,0]],['trialB',[1.2,4,0]],['trialC',[0.8,1.6,0.01]]]){
  el(button).onclick=()=>{editing=false;setGainsUI(values);pidVersion++;sendPID()};
}
el('run').onclick=()=>command('s'+el('target').value).catch(message);
el('step').onclick=()=>{const t=Number(el('target').value);if(!(t>=1&&t<=300)){message('Uji step memakai target positif 1–300 RPM.');return}command('t'+t).catch(message)};
el('stop').onclick=async()=>{clearTimeout(pendingTimer);pendingTimer=null;sendAgain=false;try{await command('s0')}catch(e){message(e)}};
el('save').onclick=()=>command('w').catch(message);el('reset').onclick=()=>command('x').then(()=>{initialized=false}).catch(message);
function sendText(){const c=el('command').value.trim();if(c)command(c).then(()=>{el('command').value='';if(/^[kpidmx]/.test(c))initialized=false}).catch(message)}el('send').onclick=sendText;el('command').onkeydown=e=>{if(e.key==='Enter')sendText()};
el('clear').onclick=()=>{rows=[];lines=[];log('Tampilan dan data CSV browser dibersihkan.');draw()};
el('csv').onclick=()=>{if(!rows.length){message('Belum ada sampel untuk diunduh.');return}const text='waktu_ms,setpoint_rpm,rpm_terfilter,pwm\n'+rows.map(r=>r.join(',')).join('\n');const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([text],{type:'text/csv;charset=utf-8'}));a.download='data_pid_'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)};
function draw(){const c=el('chart'),dpr=devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;c.width=w*dpr;c.height=h*dpr;const ctx=c.getContext('2d');ctx.scale(dpr,dpr);ctx.clearRect(0,0,w,h);const p={l:44,r:12,t:15,b:28},cw=w-p.l-p.r,ch=h-p.t-p.b;const end=rows.length?rows[rows.length-1][0]:20000,start=Math.max(0,end-20000);const visible=rows.filter(r=>r[0]>=start);let lo=0,hi=120;for(const r of visible){lo=Math.min(lo,r[1]*1.05,r[2]);hi=Math.max(hi,r[1]*1.05,r[2])}lo=Math.floor(lo/20)*20-5;hi=Math.ceil(hi/20)*20+5;const x=t=>p.l+(t-start)/20000*cw,y=v=>p.t+(hi-v)/(hi-lo)*ch;ctx.font='11px system-ui';for(let i=0;i<=4;i++){const v=lo+(hi-lo)*i/4,yy=y(v);ctx.strokeStyle='#28384d';ctx.beginPath();ctx.moveTo(p.l,yy);ctx.lineTo(w-p.r,yy);ctx.stroke();ctx.fillStyle='#91a7c1';ctx.fillText(v.toFixed(0),2,yy+4)}for(let i=0;i<=4;i++){ctx.fillStyle='#91a7c1';ctx.fillText(((start+i*5000)/1000).toFixed(0)+'s',p.l+i*cw/4-8,h-5)}ctx.save();ctx.beginPath();ctx.rect(p.l,p.t,cw,ch);ctx.clip();for(const [idx,color,dash,scale]of [[1,'#476184',[3,4],.95],[1,'#476184',[3,4],1.05],[1,'#f7bf61',[6,4],1],[2,'#5bdfb2',[],1]]){ctx.strokeStyle=color;ctx.setLineDash(dash);ctx.lineWidth=idx===2?2:1.4;ctx.beginPath();let previous=null;for(const r of visible){const xx=x(r[0]),yy=y(r[idx]*scale);if(!previous||r[0]-previous[0]>250)ctx.moveTo(xx,yy);else ctx.lineTo(xx,yy);previous=r}ctx.stroke()}ctx.restore()}
window.addEventListener('resize',draw);
async function poll(){try{const r=await fetch('/state?since='+since,{cache:'no-store',signal:AbortSignal.timeout(2500)});if(!r.ok)throw Error('HTTP '+r.status);const s=await r.json();if(boot!==s.boot){boot=s.boot;since=0;rows=[];lines=[];initialized=false;log('ESP32 terhubung / baru menyala.')}connected=true;lastGood=Date.now();el('connection').textContent='● Terhubung';el('connection').style.color='#70e2b6';el('sp').textContent=s.sp.toFixed(1);el('rpm').textContent=s.rpm.toFixed(1);el('pwm').textContent=s.pwm;el('state').textContent=(s.fault?'FAULT':s.step?'Uji step aktif':s.sp?'Motor aktif':'Motor berhenti')+' · Grafik 20 detik terakhir';el('applied').textContent=`Aktif di ESP32: Kp ${s.kp.toFixed(3)} · Ki ${s.ki.toFixed(3)} · Kd ${s.kd.toFixed(5)} · PWM min ${s.min}`;
if((!initialized||Date.now()-lastSend>700)&&!editing&&!pidBusy&&!pendingTimer){setGainsUI([s.kp,s.ki,s.kd]);initialized=true}
if(s.gap)log('[Ada celah data: sebagian log tidak diterima browser.]');for(const item of s.logs){if(item[0]<=since)continue;since=item[0];log(item[2]);const m=item[2].match(/^Setpoint:([-\d.]+),RPM:([-\d.]+),PWM:([-\d]+)/);if(m)rows.push([item[1],Number(m[1]),Number(m[2]),Number(m[3])])}if(rows.length>12000)rows.splice(0,rows.length-12000);
if(s.metric){el('response').textContent=s.responseLabel||'Belum dapat ditentukan';el('rise').textContent=s.rise<0?'Belum tercapai':s.rise+' ms';el('over').textContent=s.over.toFixed(2)+' %';el('settle').textContent=s.settle<0?'Belum settle':s.settle+' ms';el('sse').textContent=s.sse.toFixed(2)+' RPM';el('metricnote').textContent='Hasil uji terakhir pada target '+s.testTarget.toFixed(1)+' RPM. Settling harus bertahan di pita ±5% setidaknya 1 detik sampai akhir uji.'}else{el('response').textContent=s.step?'Merekam respons…':'Menunggu uji step';for(const id of ['rise','over','settle','sse'])el(id).textContent='—';el('metricnote').textContent=s.step?'Merekam respons step…':'Jalankan uji step untuk menghitung metrik.'}draw();
}catch(e){if(Date.now()-lastGood>1500){connected=false;el('connection').textContent='● Terputus';el('connection').style.color='#ff8c9a'}}finally{setTimeout(poll,200)}}labels();draw();poll();
</script></body></html>
)PIDWEB";

// ================= SETUP =================

void setup() {
  Serial.begin(115200);
  webMutex=xSemaphoreCreateMutex();
  commandQueue=xQueueCreate(8,sizeof(WebCommand));
  if(!webMutex||!commandQueue){
    Serial.println("Gagal membuat antrean/mutex. Reset ESP32.");
    while(true)delay(1000);
  }

  // Batasi waktu tunggu pembacaan baris serial.
  Serial.setTimeout(20);

  // Asumsi board memakai input aktif HIGH.
  // Motor 2 dan kicker ditahan LOW.
  digitalWrite(MOTOR2_A, LOW);
  digitalWrite(MOTOR2_B, LOW);
  digitalWrite(KICKER_PIN, LOW);

  pinMode(MOTOR2_A, OUTPUT);
  pinMode(MOTOR2_B, OUTPUT);
  pinMode(KICKER_PIN, OUTPUT);

  digitalWrite(MOTOR_PWM_A, LOW);
  digitalWrite(MOTOR_PWM_B, LOW);

  pinMode(MOTOR_PWM_A, OUTPUT);
  pinMode(MOTOR_PWM_B, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  if (!setupPWM()) {
    logLine("GAGAL: Inisialisasi PWM BTS7960.");

    // Tahan di sini jika PWM tidak berhasil dibuat.
    while (true) {
      delay(1000);
    }
  }

  setMotor(0);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A), isrA, CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(ENC_B), isrB, CHANGE
  );

  loadParams();
  printParams();

  logLine("Siap. Kirim s50 untuk cek putaran.");
  logLine("Kirim t100 untuk mengukur respons step.");

  publishState();
  startWebServer();
  lastTime = millis();
  if(xTaskCreatePinnedToCore(controlTask,"motorPID",8192,nullptr,2,nullptr,1)!=pdPASS){
    setMotor(0);logLine("Gagal membuat task kontrol. Reset ESP32.");
    while(true)delay(1000);
  }
}

// ================= LOOP UTAMA =================

void controlTick() {

  unsigned long now = millis();

  if (now - lastTime < SAMPLE_MS) {
    return;
  }

  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;
  if(dt > 0.15f && setpointRPM != 0){
    stepRunning=false;metricsDone=false;faultLatched=true;
    setpointRPM=0;setMotor(0);
    logLine("FAULT: sampling terlambat >150 ms. Motor dihentikan.");
  }

  // ---------- BACA ENCODER ----------

  long count;

  portENTER_CRITICAL(&mux);
  count = encoderCount;
  portEXIT_CRITICAL(&mux);

  long delta = count - lastCount;
  lastCount = count;

  float rpm =
    ENCODER_SIGN *
    (delta / PULSE_PER_REV) / dt * 60.0f;

  // ---------- FILTER RPM ----------

  rpmFiltered =
    0.7f * rpmFiltered + 0.3f * rpm;

  // ---------- PENGUKURAN RESPONS STEP ----------

  if (stepRunning && !metricsDone) {
    unsigned long elapsed = now - stepStart;

    if (elapsed <= STEP_ON_MS) {
      updateMetrics(elapsed, rpmFiltered);
    }

    if (elapsed >= STEP_ON_MS) {
      metricsDone = true;

      // Hentikan keluaran sebelum mencetak hasil.
      setSetpoint(0);
      printMetrics();

      logLine("Step: setpoint -> 0 RPM");
    }
  }

  if (stepRunning &&
      now - stepStart >= STEP_ON_MS + STEP_OFF_MS) {
    stepRunning = false;
    logLine("Uji step selesai.");
  }

  // ---------- KONTROL PID ----------

  if (setpointRPM == 0.0f || faultLatched) {
    resetPID();
    setMotor(0);

  } else {
    float error = setpointRPM - rpmFiltered;

    integral += error * dt;

    integral = constrain(
      integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT
    );

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output =
      Kp * error +
      Ki * integral +
      Kd * derivative;

    // Kompensasi deadband motor.
    if (output > 0.0f) {
      output += pwmMin;
    } else if (output < 0.0f) {
      output -= pwmMin;
    }

    output = constrain(
      output, -(float)PWM_MAX, (float)PWM_MAX
    );

    setMotor((int)output);

    // ---------- DETEKSI MACET / ARAH SALAH ----------

    bool pwmSaturated = abs(lastOutput) >= PWM_MAX;
    bool noMovement = fabs(rpmFiltered) < 5.0f;

    bool wrongDir =
      rpmFiltered * setpointRPM < 0 &&
      fabs(rpmFiltered) > 20.0f;

    if ((pwmSaturated && noMovement) || wrongDir) {
      if (stallStart == 0) {
        stallStart = now;
      }

      if (now - stallStart > STALL_TIMEOUT_MS) {
        faultLatched = true;
        setMotor(0);

        stepRunning = false;

        logLine(
          "!! FAULT: Motor macet / arah encoder terbalik."
        );

        logLine(
          "Cek wiring, lalu kirim setpoint baru."
        );
      }

    } else {
      stallStart = 0;
    }
  }

  // ---------- MONITORING REAL-TIME ----------

  logPrintf(
    "Setpoint:%.1f,RPM:%.1f,PWM:%d\n",
    setpointRPM, rpmFiltered, lastOutput
  );
}
// ===== Published state and web endpoints =====
void publishState(){
  WebState s;
  s.kp=Kp;s.ki=Ki;s.kd=Kd;s.minimum=pwmMin;
  s.sp=setpointRPM;s.rpm=rpmFiltered;s.pwm=lastOutput;
  s.step=stepRunning;s.fault=faultLatched;
  s.metric=metricsDone&&hasMetricSample;
  s.response=s.metric?classifyResponse():0;
  s.testTarget=stepTarget;
  s.rise=(got10&&got90&&tRise90>=tRise10)?(int)(tRise90-tRise10):-1;
  bool settled=inBandNow&&hasSettleCandidate&&
    metricLastElapsed>=tSettle&&metricLastElapsed-tSettle>=1000;
  s.settle=settled?(int)tSettle:-1;
  s.over=stepTarget>0?fmaxf(0,(peakRPM-stepTarget)/stepTarget*100.f):0;
  s.sse=ssCount?ssSumErr/ssCount:0;
  xSemaphoreTake(webMutex,portMAX_DELAY);published=s;xSemaphoreGive(webMutex);
}
String escapeJSON(const char* text){
  String out;
  for(const char* p=text;*p;++p){
    if(*p=='"'||*p=='\\'){out+='\\';out+=*p;}
    else if(*p=='\n')out+="\\n";
    else if(*p=='\r')out+="\\r";
    else if(*p=='\t')out+="\\t";
    else if((uint8_t)*p>=32)out+=*p;
  }return out;
}
void handleState(){
  lastWebPoll.store(millis());
  uint32_t since=(uint32_t)strtoul(server.arg("since").c_str(),nullptr,10);
  static WebLog copied[LOG_COUNT];WebState s;uint32_t newest,first;
  int count=0;bool gap=false;
  xSemaphoreTake(webMutex,portMAX_DELAY);
  s=published;newest=logSequence;
  first=newest>=LOG_COUNT?newest-LOG_COUNT+1:1;
  // A browser can reconnect after the ESP32 has rebooted.
  if(since>newest)since=0;
  gap=since>0&&since+1<first;
  uint32_t from=since+1>first?since+1:first;
  for(uint32_t n=from;n<=newest&&count<LOG_COUNT;n++)copied[count++]=webLogs[(n-1)%LOG_COUNT];
  xSemaphoreGive(webMutex);
  String j;j.reserve(4000+count*90);
  j="{\"boot\":"+String(bootId)+",\"kp\":"+String(s.kp,5)+",\"ki\":"+String(s.ki,5)+",\"kd\":"+String(s.kd,6);
  j+=",\"min\":"+String(s.minimum)+",\"sp\":"+String(s.sp,2)+",\"rpm\":"+String(s.rpm,2)+",\"pwm\":"+String(s.pwm);
  j+=",\"step\":"+String(s.step?"true":"false")+",\"fault\":"+String(s.fault?"true":"false");
  j+=",\"metric\":"+String(s.metric?"true":"false")+",\"rise\":"+String(s.rise)+",\"settle\":"+String(s.settle);
  j+=",\"over\":"+String(s.over,3)+",\"sse\":"+String(s.sse,3)+",\"testTarget\":"+String(s.testTarget,2);
  j+=",\"response\":"+String(s.response)+",\"responseLabel\":\"";
  j+=escapeJSON(responseLabel(s.response));j+="\"";
  j+=",\"gap\":"+String(gap?"true":"false")+",\"logs\":[";
  for(int i=0;i<count;i++){
    if(i)j+=',';
    j+='[';j+=String(copied[i].seq);j+=',';j+=String(copied[i].ms);j+=",\"";
    j+=escapeJSON(copied[i].text);j+="\"]";
  }j+="]}";
  server.sendHeader("Cache-Control","no-store");server.send(200,"application/json",j);
}
void startWebServer(){
  bootId=esp_random();
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID,WIFI_PASS);

  logPrintf("Menghubungkan ke hotspot: %s",WIFI_SSID);
  unsigned long connectStart=millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-connectStart<15000){
    delay(250);
  }
  if(WiFi.status()==WL_CONNECTED){
    logLine("Hotspot terhubung.");
    logPrintf("Buka halaman kontrol: http://%s",WiFi.localIP().toString().c_str());
  } else {
    logLine("Hotspot belum terhubung. Periksa hotspot; koneksi dicoba ulang tiap 10 detik.");
  }
  // Start the server even when the first WiFi attempt fails.
  // A later successful retry can then serve the same panel without reset.
  server.on("/",HTTP_GET,[]{server.send_P(200,"text/html; charset=utf-8",INDEX_HTML);});
  server.on("/state",HTTP_GET,handleState);
  server.on("/command",HTTP_POST,[]{
    String c=server.arg("cmd");c.trim();
    if(!commandValid(c)){server.send(400,"text/plain","Perintah/nilai tidak valid.");return;}
    if(enqueueCommand(c,true))server.send(202,"text/plain","Perintah diterima ESP32.");
    else server.send(503,"text/plain","Antrean penuh. Coba lagi.");
  });
  server.onNotFound([]{server.send(404,"text/plain","Tidak ditemukan.");});
  server.begin();
}
void controlTask(void*){
  TickType_t wake=xTaskGetTickCount();
  for(;;){
    bool stopped=stopRequest.exchange(false);
    if(stopped){
      xQueueReset(commandQueue);stepRunning=false;metricsDone=false;
      setSetpoint(0);webControlsMotor=false;
      logLine("STOP: motor dihentikan.");
    }
    WebCommand c;
    // Bound work per tick; do not starve the 50-ms speed loop.
    for(int n=0;n<2&&!stopped&&xQueueReceive(commandQueue,&c,0)==pdTRUE;n++){
      String text(c.text);char kind=text.charAt(0);
      bool parameter=kind=='k'||kind=='p'||kind=='i'||kind=='d'||kind=='m'||kind=='x';
      if(parameter&&stepRunning){
        stepRunning=false;metricsDone=false;setSetpoint(0);
        logLine("PID berubah: uji dibatalkan. Jalankan uji step baru.");
      }
      if(kind=='w'&&(setpointRPM!=0||fabs(rpmFiltered)>5)){
        logLine("STOP dan tunggu motor diam sebelum menyimpan PID.");continue;
      }
      if(kind=='t'&&fabs(rpmFiltered)>5){
        logLine("Motor belum diam. STOP, tunggu RPM mendekati 0, lalu uji step.");continue;
      }
      executeCommand(text);
      if(kind=='s'||kind=='t'||isDigit(kind)||kind=='-')webControlsMotor=c.web;
      else if(c.web&&parameter&&setpointRPM!=0)webControlsMotor=true;
    }
    if(webControlsMotor&&setpointRPM!=0&&millis()-lastWebPoll.load()>5000){
      stepRunning=false;metricsDone=false;setSetpoint(0);webControlsMotor=false;
      logLine("Koneksi halaman terputus >5 detik: motor dihentikan.");
    }
    controlTick();publishState();
    vTaskDelayUntil(&wake,pdMS_TO_TICKS(50));
  }
}
void serviceWiFi(){
  static bool wasConnected=false;
  static unsigned long lastRetry=0;
  bool connected=WiFi.status()==WL_CONNECTED;
  if(connected && !wasConnected){
    logPrintf("WiFi aktif. Buka http://%s",WiFi.localIP().toString().c_str());
  }
  if(!connected && wasConnected){
    logLine("Hotspot terputus. Menunggu koneksi kembali.");
  }
  if(!connected && millis()-lastRetry>=10000){
    lastRetry=millis();
    WiFi.reconnect();
  }
  wasConnected=connected;
}

void loop(){
  serviceWiFi();
  server.handleClient();
  static char serialLine[96];static size_t used=0;static bool overflow=false;
  while(Serial.available()){
    char c=(char)Serial.read();
    if(c=='\r')continue;
    if(c=='\n'){
      serialLine[used]=0;
      if(overflow)logLine("Perintah terlalu panjang.");
      else {
        String text(serialLine);text.trim();
        if(text.length()&&(!commandValid(text)||!enqueueCommand(text,false)))logLine("Perintah tidak valid atau antrean penuh.");
      }
      used=0;overflow=false;
    }else if(used<sizeof(serialLine)-1)serialLine[used++]=c;
    else overflow=true;
  }
  delay(2);
}
