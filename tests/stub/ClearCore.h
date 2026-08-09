// Minimal ClearCore shim so the firmware can be compiled and exercised on
// a desktop. It is NOT an emulator: motors accept commands and report a
// fixed position. What it faithfully reproduces is the ONE thing the
// tests assert on — the text the board sends back — by capturing
// Serial.println into OUT.
//
// A no-op println would make every saw() assertion vacuously true, which
// is worse than having no tests at all. That bug was real once.
#pragma once
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>

typedef unsigned char byte;

#define OUTPUT 1
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0

struct String {
  std::string s;
  String(){} String(const char*c):s(c){} String(const std::string&x):s(x){}
  // Arduino's String has char overloads; without them `buf += c` picks
  // String(int) and appends the character's DECIMAL CODE. That silently
  // turned every received PLC frame into a digit soup which parsed as a
  // malformed length — the firmware was fine, the stub was lying.
  String(char c){s=std::string(1,c);}
  String(int v){char b[32];snprintf(b,32,"%d",v);s=b;}
  String(long v){char b[32];snprintf(b,32,"%ld",v);s=b;}
  String(unsigned v){char b[32];snprintf(b,32,"%u",v);s=b;}
  String(unsigned long v){char b[32];snprintf(b,32,"%lu",v);s=b;}
  String(double v,int d=2){char b[64];snprintf(b,64,"%.*f",d,v);s=b;}
  String(float v,int d=2){char b[64];snprintf(b,64,"%.*f",d,(double)v);s=b;}
  const char* c_str()const{return s.c_str();}
  int length()const{return (int)s.size();}
  char charAt(int i)const{return s[i];}
  int indexOf(const String&x)const{auto p=s.find(x.s);return p==std::string::npos?-1:(int)p;}
  int indexOf(char c,int from=0)const{auto p=s.find(c,from);return p==std::string::npos?-1:(int)p;}
  int indexOf(const char*c)const{auto p=s.find(c);return p==std::string::npos?-1:(int)p;}
  String substring(int a)const{return String(s.substr(a));}
  String substring(int a,int b)const{return String(s.substr(a,b-a));}
  bool startsWith(const String&x)const{return s.rfind(x.s,0)==0;}
  bool endsWith(const String&x)const{return s.size()>=x.s.size()&&s.compare(s.size()-x.s.size(),x.s.size(),x.s)==0;}
  void trim(){size_t a=s.find_first_not_of(" \t\r\n");size_t b=s.find_last_not_of(" \t\r\n");
              s=(a==std::string::npos)?"":s.substr(a,b-a+1);}
  void toUpperCase(){for(auto&c:s)c=toupper(c);}
  void replace(const String&a,const String&b){size_t p=0;while((p=s.find(a.s,p))!=std::string::npos){s.replace(p,a.s.size(),b.s);p+=b.s.size();}}
  double toDouble()const{return atof(s.c_str());}
  float toFloat()const{return (float)atof(s.c_str());}
  int toInt()const{return atoi(s.c_str());}
  String operator+(const String&o)const{return String(s+o.s);}
  String& operator+=(const String&o){s+=o.s;return *this;}
  String& operator+=(char c){s+=c;return *this;}
  bool operator==(const String&o)const{return s==o.s;}
  bool operator!=(const String&o)const{return s!=o.s;}
};
inline String operator+(const char*a,const String&b){return String(std::string(a)+b.s);}

// Feedback capture — the harness reads this.
extern std::vector<std::string> OUT;
struct SerialC{
  void begin(int){} operator bool()const{return true;} int available(){return 0;}
  String readStringUntil(char){return String();}
  void println(const String&x){OUT.push_back(x.s);}
  void println(const char*x){OUT.push_back(x);}
  void flush(){}
};
extern SerialC Serial;

// millis() used to return a constant 0, which made every timeout,
// watchdog and poll interval in the firmware untestable: no amount of
// waiting could ever elapse. It is now a variable the harness advances.
extern unsigned long MOCK_MILLIS;
inline unsigned long millis(){return MOCK_MILLIS;}
inline void delay(unsigned long){}
// digitalWrite RECORDS the level, it is not a no-op. The HOME request is
// a wire now — IO-0 into the PLC's X0 — so "did the board actually assert
// the line?" is a real assertion the harness has to be able to make. A
// swallowing stub would let every HOME test pass while the terminal never
// moved, which is the same trap Serial.println was in before it captured
// into OUT.
extern int PIN_LEVEL[64];
inline void digitalWrite(int p,int v){ if(p>=0&&p<64) PIN_LEVEL[p]=v; }
inline int  digitalRead(int){return 1;}
inline void pinMode(int,int){}
template<class T> T constrain(T v,T lo,T hi){return v<lo?lo:(v>hi?hi:v);}

namespace StepGenerator{enum{MOVE_TARGET_ABSOLUTE=1};}
namespace Connector{enum{CPM_MODE_STEP_AND_DIR=1};}
struct MotorConn{
  void VelMax(int32_t){} void AccelMax(int32_t){} void EnableRequest(bool){}
  void Move(int32_t,int){} bool StepsComplete(){return true;}
  void MoveStopDecel(int32_t){} void MoveVelocity(int32_t){}
  int32_t PositionRefCommanded(){return 0;} void PositionRefSet(int32_t){}
};
extern MotorConn ConnectorM0,ConnectorM1,ConnectorM2,ConnectorM3;
struct MotorManager{enum{CLOCK_RATE_LOW=0,MOTOR_ALL=0};
  void MotorInputClocking(int){} void MotorModeSet(int,int){}};
extern MotorManager MotorMgr;
#define IO0 0
#define IO1 1
#define IO2 2
#define IO3 3
#define IO4 4
#define IO5 5

// Arduino core constants the sketch relies on.
#define LED_BUILTIN 13
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232
#endif
template<class T> T min(T a, T b) { return a < b ? a : b; }
template<class T> T max(T a, T b) { return a > b ? a : b; }


// ══════════════════════════════════════════════════════════════
// ETHERNET SHIM — a scriptable socket, not an emulator.
//
// It records every byte the firmware SENDS (ETH_TX) so the tests can
// assert on the exact MC protocol frame, and plays back bytes the test
// queues (ETH_RX) as if the PLC had answered. That is the whole contract
// the firmware depends on, and it is the part that is worth getting
// wrong-proof: a frame with a mis-counted length field looks perfectly
// fine in source and is rejected by the real PLC.
// ══════════════════════════════════════════════════════════════
extern std::vector<std::string> ETH_TX;   // frames the firmware sent
extern std::string ETH_RX;                // bytes waiting to be read
extern bool ETH_CONNECTED;                // whether connect() succeeds
extern int  ETH_LINK;                     // Ethernet.linkStatus()

#define LinkOFF 0
#define LinkON  1

struct IPAddress {
  int a,b,c,d;
  IPAddress(){a=b=c=d=0;}
  IPAddress(int w,int x,int y,int z):a(w),b(x),c(y),d(z){}
};

struct EthernetClientStub {
  bool open=false;
  bool connect(IPAddress, uint16_t){ open = ETH_CONNECTED; return open; }
  bool connected(){ return open && ETH_CONNECTED; }
  void stop(){ open=false; }
  void print(const String &x){ ETH_TX.push_back(x.s); }
  void println(const String &x){ ETH_TX.push_back(x.s); }
  void flush(){}
  int  available(){ return (int)ETH_RX.size(); }
  int  read(){ if(ETH_RX.empty()) return -1; char c=ETH_RX[0]; ETH_RX.erase(0,1); return c; }
};
typedef EthernetClientStub EthernetClient;

struct EthernetStub {
  void begin(unsigned char*, IPAddress){}
  int linkStatus(){ return ETH_LINK; }
};
extern EthernetStub Ethernet;
