// Second v2. Existing AD-Project Mega wiring; no hardware change.
// Serial: 115200; magic A5 5A, kind, len, payload, CRC16-CCITT LE.
// Command(kind 1,len 7): seq:u16, pwm:i16, legacy_steer:i16, enabled:u8.
// Feedback(kind 2,len 9): seq:u16,pot:u16,pwm:i16,steer:i16,status:u8.
#include <Arduino.h>
#include <string.h>

const uint8_t LEFT_A=6, LEFT_B=7, RIGHT_A=5, RIGHT_B=4;
const uint8_t STEER_A=3, STEER_B=2, POT=A2;
const int CENTER=483, LIMIT=67, DEAD_BAND=3;
const uint32_t WATCHDOG_MS=250, CONTROL_MS=10, FEEDBACK_MS=40;
// Starting values inherited from the working car; verify with wheels raised.
const float STEER_KP=6.0f;
const int STEER_MIN_PWM=130, STEER_MAX_PWM=200;
uint8_t rx[64];
uint8_t used=0;
uint16_t sequence=0;
bool haveSequence=false, enabled=false;
int targetPWM=0, actualPWM=0, targetSteer=0, potValue=CENTER;
uint32_t receivedAt=0, controlledAt=0, feedbackAt=0, directionPauseUntil=0;

uint16_t crc16(const uint8_t* bytes, uint8_t len) {
  uint16_t crc=0xffff;
  for (uint8_t i=0;i<len;++i) {
    crc ^= uint16_t(bytes[i])<<8;
    for (uint8_t b=0;b<8;++b) crc=(crc&0x8000) ? (crc<<1)^0x1021 : crc<<1;
  }
  return crc;
}
uint16_t read16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1])<<8); }
void put16(uint8_t* p, uint16_t v) { p[0]=v&255; p[1]=v>>8; }

void motor(uint8_t a, uint8_t b, int pwm) {
  if (pwm>0) { analogWrite(b,0); analogWrite(a,pwm); }
  else if (pwm<0) { analogWrite(a,0); analogWrite(b,-pwm); }
  else { analogWrite(a,0); analogWrite(b,0); }
}
void allOff() {
  actualPWM=0;
  motor(LEFT_A,LEFT_B,0); motor(RIGHT_A,RIGHT_B,0); motor(STEER_A,STEER_B,0);
}
int steeringTarget(int cmd) {
  int a=min(abs(cmd),120);
  int offset=(a<=100) ? a/2 : 50+(a-100)*(LIMIT-50)/20;
  // Existing physical steering polarity: negative PC command = left.
  return CENTER-(cmd>=0 ? offset : -offset);
}
void consume(uint8_t n) {
  memmove(rx,rx+n,used-n); used-=n;
}
void receiveCommands() {
  while (Serial.available()) {
    if (used==sizeof(rx)) consume(1);
    rx[used++]=Serial.read();
    while (used>=4) {
      if (rx[0]!=0xa5 || rx[1]!=0x5a) { consume(1); continue; }
      uint8_t len=rx[3];
      if (len>32) { consume(1); continue; }
      if (used<len+6) break;
      if (crc16(rx+2,len+2)!=read16(rx+4+len)) { consume(1); continue; }
      if (rx[2]==1 && len==7) {
        uint16_t seq=read16(rx+4);
        int pwm=int16_t(read16(rx+6));
        int steer=int16_t(read16(rx+8));
        uint16_t delta=seq-sequence;
        if ((!haveSequence || (delta>0 && delta<32768)) && abs(pwm)<=255 && abs(steer)<=120 && rx[10]<=1) {
          sequence=seq; haveSequence=true;
          targetPWM=pwm; targetSteer=steer; enabled=rx[10]; receivedAt=millis();
          if (!enabled || targetPWM==0) {
            actualPWM=0; motor(LEFT_A,LEFT_B,0); motor(RIGHT_A,RIGHT_B,0);
          }
          if (!enabled) allOff();
        }
      }
      consume(len+6);
    }
  }
}
void sendFeedback(uint8_t status) {
  uint8_t tx[15]={0xa5,0x5a,2,9};
  put16(tx+4,sequence); put16(tx+6,potValue); put16(tx+8,int16_t(actualPWM));
  put16(tx+10,int16_t(targetSteer)); tx[12]=status;
  put16(tx+13,crc16(tx+2,11));
  Serial.write(tx,sizeof(tx));
}
void setup() {
  const uint8_t pins[]={LEFT_A,LEFT_B,RIGHT_A,RIGHT_B,STEER_A,STEER_B};
  for (uint8_t i=0;i<6;++i) pinMode(pins[i],OUTPUT);
  allOff(); Serial.begin(115200);
}
void loop() {
  receiveCommands();
  uint32_t now=millis();
  potValue=analogRead(POT);
  bool expired=!haveSequence || uint32_t(now-receivedAt)>WATCHDOG_MS;
  bool potFault=potValue<400 || potValue>590;
  uint8_t status=(enabled ? 1:0) | (potFault ? 2:0) | (expired ? 4:0);
  if (!enabled || expired || potFault) {
    allOff();
    if (expired) haveSequence=false;
  } else if (uint32_t(now-controlledAt)>=CONTROL_MS) {
    controlledAt=now;
    int diff=steeringTarget(targetSteer)-potValue;
    int effort=0;
    if (abs(diff)>DEAD_BAND) effort=constrain(int(abs(diff)*STEER_KP),STEER_MIN_PWM,STEER_MAX_PWM)*(diff>0 ? 1:-1);
    motor(STEER_A,STEER_B,effort);
    // Identical commands on rear motors: no differential parking turn.
    if (actualPWM*targetPWM<0) {
      actualPWM=0; directionPauseUntil=now+150;
    }
    if (int32_t(now-directionPauseUntil)>=0) {
      if (actualPWM<targetPWM) actualPWM=min(targetPWM,actualPWM+5);
      if (actualPWM>targetPWM) actualPWM=max(targetPWM,actualPWM-5);
    }
    motor(LEFT_A,LEFT_B,actualPWM); motor(RIGHT_A,RIGHT_B,actualPWM);
  }
  if (uint32_t(now-feedbackAt)>=FEEDBACK_MS) { feedbackAt=now; sendFeedback(status); }
}
