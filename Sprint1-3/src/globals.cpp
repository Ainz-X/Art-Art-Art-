#include "globals.h"

Adafruit_NeoMatrix matrix(
  MW, MH, MATRIX_PIN,
  NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS,
  COLOR_ORDER + NEO_KHZ800
);

std::map<String, PeerInfo> peers;

uint8_t selfMac[6] = {0};
uint32_t seqNo = 0;
unsigned long lastPing = 0;

PlayerTeam selectedTeam = TEAM_RED;
GameState gameState = TEAM_SELECT;
PlayerTeam myTeam = TEAM_NEUTRAL;
unsigned long gameStartTime = 0;
unsigned long lastCaptureTime = 0;
bool canCapture = true;

bool isBeingCaptured = false;
unsigned long captureStartTime = 0;
const unsigned long CAPTURE_WINDOW = 3000;
String capturingPlayer = "";
PlayerTeam capturingTeam = TEAM_NEUTRAL;
bool canDefend = true;
unsigned long lastDefendTime = 0;
const unsigned long DEFEND_COOLDOWN = 30000;

unsigned long victoryTime = 0;
PlayerTeam winningTeam = TEAM_NEUTRAL;

bool lastButton1 = false;
bool lastButton2 = false;
unsigned long lastButton1Press = 0;
unsigned long lastButton2Press = 0;
const unsigned long DEBOUNCE_TIME = 50;

unsigned long button1PressStartTime = 0;
bool button1Pressing = false;
const unsigned long LONG_PRESS_TIME = 3000;
