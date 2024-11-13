// Pinout
const int ENC_A = 2;
const int ENC_B = 3;
const int STP_EN =   4;
const int STP_SDI =  5;
const int STP_SCK =  6;
const int STP_CS =   7;
const int STP_SDO =  8;
const int STP_STEP = 9;
const int STP_DIR = 10;
const int LIM = 11;

// Encoder
const int PPR = 5120;

// Motor
const bool STEALTHCHOP = false; // Not quiet but much more power
const bool FWD = false;
const bool REV = !FWD;
const unsigned long MICROSTEPS = 16;
const unsigned long STEPS_PER_REV = 200 * MICROSTEPS;
const unsigned long MAX_POS = 2*STEPS_PER_REV;
const float DIST_PER_REV = 0.12;
const float DIST_PER_STEP = DIST_PER_REV / STEPS_PER_REV;

// Serial debugging
const bool DEBUG = true;
const unsigned long BAUDRATE = 230400;
