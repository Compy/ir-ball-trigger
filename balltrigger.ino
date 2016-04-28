
#include <IRremote.h>
#include <Bounce.h>

#define IR_LED_PIN  3
#define IR_RECV_PIN 2
#define STATUS_PIN  13
#define SCORE_WAIT_INTERVAL 2000
#define LED_BLINK_RATE 60
#define HEARTBEAT_INTERVAL 2000

boolean helloPassed = false;
byte packet[5]; 
int bytesRead = 0;

// Our handshake packet
byte handshake[] = { 16, 129, 0, 1, 1 };
// Score packet
byte scorePacket[] = { 1, 0, 0, 0, 0 };
// Heartbeat packet
byte heartbeatPacket[] = { 0, 0, 0, 0, 0 };

// Status variables
unsigned long startTime;
boolean hasSentScore = false;
boolean sendingScore = false;
unsigned long lastScoreSent = 0;
boolean isLedOn = false;
unsigned long lastLedChange = 0;
unsigned long lastHeartbeat = 0;
unsigned long heartbeatsMissed = 0;
boolean gotHeartbeatResponse = true;

// Our reset function is at addr 0x0
void(* resetFunc) (void) = 0;

Bounce* irReceiver;

// IR Sender
IRsend irSend;

void setup()
{
  helloPassed = false;
  
  pinMode(STATUS_PIN,OUTPUT);
  pinMode(IR_RECV_PIN,INPUT);
  
  Serial.begin(9600);
  digitalWrite(STATUS_PIN, LOW);
  startTime = millis();
  hasSentScore = false;
  sendingScore = false;
  gotHeartbeatResponse = true;
  
  // Set up our IR communication on PIN 2
  irSend.enableIROut(38);
  
  irReceiver = new Bounce(IR_RECV_PIN,1);
}

void loop()
{
  irSend.space(200);
  irSend.mark(500);
  if (!helloPassed)
  {
    if (Serial.available() == 5)
    {
      for (int i = 0; i < 5; i++)
      {
        packet[i] = Serial.read();
      }
      
      if (packet[0] == 16 && packet[1] == 128 && packet[2] == 0 && packet[3] == 0 && packet[4] == 4)
      {
        helloPassed = true;
        Serial.write(handshake,5);
        Serial.flush();
      }
    }
    // If the handshake hasn't been issued yet, default back to alignment mode.
    // The IR collector is an active LOW, so we want to light the LED when things are aligned, so invert the value
    digitalWrite(STATUS_PIN, !digitalRead(IR_RECV_PIN));
    
    if (digitalRead(IR_RECV_PIN) == LOW)
    {
      //Serial.write("OP\n");
    }
    else
    {
      //sSerial.write("CL\n");
    }
    
    
    return;
  }
  if (Serial.available() == 5)
  {
    for (int i = 0; i < 5; i++)
    {
      packet[i] = Serial.read();
    }
    if (packet[0] == 0 && packet[1] == 0 && packet[2] == 0 && packet[3] == 0 && packet[4] == 1)
    {
      // Heartbeat received
      heartbeatsMissed = 0;
      gotHeartbeatResponse = true;
    }
  }
  
  
  irReceiver->update();
  // If our recv pin is low, then the beam is broken
  if (irReceiver->risingEdge() && !sendingScore)
  {
    sendingScore = true;
    lastScoreSent = millis();
    
    Serial.write(scorePacket,5);
    Serial.flush();
  }
  // If the IR beam has closed again and we've waited long enough, reset our state so we can accept another
  // trigger
  if (millis() - lastScoreSent > SCORE_WAIT_INTERVAL)
  {
    sendingScore = false;
    // Turn off our indicator LED
    digitalWrite(STATUS_PIN,LOW);
    isLedOn = false;
  }
  
  // If we're sending the score, blink the LED
  if (sendingScore)
  {
    if (millis() - lastLedChange >= LED_BLINK_RATE)
    {
      isLedOn = !isLedOn;
      digitalWrite(STATUS_PIN, isLedOn);
      lastLedChange = millis();
    }
  }
  
  // If our heartbeat interval has elapsed, send a heartbeat packet
  if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL)
  {
    // If we didn't get a response from our last heartbeat, reset
    if (!gotHeartbeatResponse) heartbeatsMissed++;
    if (heartbeatsMissed >= 5)
    {
      resetFunc();
      return;
    }
    gotHeartbeatResponse = false;
    lastHeartbeat = millis();
    Serial.write(heartbeatPacket,5);
    Serial.flush();
  }
}
