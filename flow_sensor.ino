#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <PubSubClient.h>
#include <Base64.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define AP_SSID "*ssid"
#define AP_PASSWORD "*ssidpassword"
#define PROGRAMMING_DELAY 5  
#define MAX_CONNECT_RETRIES 5
#define MAX_WIFI_BEGIN_RETRIES 30

// set to unique flow sensor id in the scope of the mqtt broker
#define SENSOR_ID 2

#define DEBOUNCE_TIME 1000         // ignore interrupts for 1000 microseconds after triggering (debounce)
#define GPIO_SENSOR_INPUT_PIN 14   // D5 on Wemos D1 mini - connected to a flow sensor's signal pin with a 4.7k resistor inline to drop the voltage to something safe for the esp8266
#define GPIO_OUTPUT_PIN 13         // D7 on Wemos D1 mini - connect to the flow sensor pin of an opensprinker (need to see if OS will see the 3.3V signal)
#define GPIO_LED 2                 // D4 on Wemos D1 mini with LED (blinks LED in line with rise and fall of output pin)
#define TRIGGER_INTERVAL_US 100000 // 100,000 microseconds per output trigger (10Hz max output) - (OS doesn't seem to be able to poll fast enough when i was assuming 50Hz signal)
#define TRIGGER_DIVISOR 100        // trigger output every n ticks on the sensor input (100 to 1 "shift register")
#define TEMPERATURE_CHECK 10       // check temperature every n timer cycles
#define ONE_WIRE_BUS 4             // D2 on Wemos D1 mini - An DS18B20 can be connected to this pin via a Wemos temperature board (set measureTemperature to 1 in main() to enable)

// periodically reset when inactive - stops volatile variables overflowing and the module reporting invalid data
#define RESET_THRESHOLD 1350      // reset after n inactive timer cycles (1350 is about 45 minutes)

#define MESSAGE_REPEAT_THRESHOLD 100 // resend last message after n no change cycles

//#define DEBUG_TO_SERIAL

// forward declaration of ISRs
// -----------------------------------
void ICACHE_RAM_ATTR pinChanged();
void ICACHE_RAM_ATTR timerCallback();
// -----------------------------------

const char* mqtt_server = "*mqttserver";
char msg[200];
char sensorName[16];

// variables used within ISRs, must be marked volatile to force the compiler to load / store the variables on access
// -----------------------------------
// timestamp (per micros()) of when the ISR was last triggered
volatile unsigned long last_micros;

// number of falling edges detected via interrupt pin
volatile long ticks;

// the number of timer interrupts that have occurred. used as a timestamp in the rest of the system.
volatile long timerTicks;

// the number of flow sensor ticks remaining to occur before we write a pulse to the opensprinkler
volatile int tickTrigger;

// indicates the end time (in micros) when the pulse to the opensprinkler should fall
volatile long clearTrigger;
// -----------------------------------

// variables used within the main loop
// -----------------------------------
long lastTimerTicks;
long lastTicks;
int outputState;
long outputTicks;
unsigned long loopStartMillis;

// number of main loops completed
long loopCount;

// the timestamp (in timer ticks) of when the system last send a temperature message to the mqtt broker
long lastTemperatureCheck;

// counts down the number of inactive main loop timer tick cycles that have taken place since the system saw a tick from the flow sensor.
// when this hits 0 the system resets.
int inactiveCount;

// the timestamp (in timer ticks) of when the system last sent a data messge to the mqtt broker
long lastSendTimerTicks;

// set to 1 to attempt temperature measuring with the DS18S20 sensor. if the system can't measure temp from the sensor it will auto disable temperature readings
int measureTemperature;
// -----------------------------------

#define TIMER1_TICKS_PER_US (APB_CLK_FREQ / 1000000L)
#define REFRESH_INTERVAL 5000000      // about 2 seconds

WiFiClient espClient;
PubSubClient client(espClient);

void printStartMessage() {
  Serial.println("Flow and Temperature v1.5 - Start");
  Serial.print("[info] Sensor Id: ");
  Serial.println(sensorName);
    
  Serial.print("[info] Chip Id: ");
  Serial.println(ESP.getChipId());
  Serial.print("[info] Flash Chip Id: ");
  Serial.println(ESP.getFlashChipId());
  Serial.print("[info] Last reset reason: ");
  Serial.println(ESP.getResetReason());
  Serial.print("[info] clockCyclesPerMicrosecond(): ");
  Serial.println(clockCyclesPerMicrosecond());
  Serial.print("[info] APB_CLK_FREQ:");
  Serial.println(APB_CLK_FREQ);
}

void setup() {
  Serial.begin(115200);
  snprintf(sensorName, 16, "flow-gauge-%d", SENSOR_ID);

  ticks = 0;
  timerTicks = 0;
  lastTimerTicks = 0;
  clearTrigger = 0;
  tickTrigger = TRIGGER_DIVISOR;
  outputTicks = 0;
  loopStartMillis = 0;
  loopCount = 0;
  lastTemperatureCheck = 0;
  inactiveCount = RESET_THRESHOLD;

  delay(1000);
  printStartMessage();

  // set up one wire bus (for temperature sensor)
  pinMode(ONE_WIRE_BUS, OUTPUT);

  // set up flow input pin with interrupts
  pinMode(GPIO_SENSOR_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GPIO_SENSOR_INPUT_PIN), pinChanged, FALLING);

  // set up flow output pin
  pinMode(GPIO_OUTPUT_PIN, OUTPUT);
  digitalWrite(GPIO_OUTPUT_PIN, HIGH);
  outputState = LOW;

  // set up LED pin
  pinMode(GPIO_LED, OUTPUT);
  digitalWrite(GPIO_LED, HIGH); // HIGH is 0V

  // set up timer
  timer1_isr_init();
  timer1_attachInterrupt(timerCallback);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  timer1_write(usToTicks(REFRESH_INTERVAL));

  Serial.print("[info] ");
  Serial.print(PROGRAMMING_DELAY);
  Serial.print(" second delay in case trying to reprogram board");
  for (int i = 0; i < PROGRAMMING_DELAY; i++)
  {
    Serial.print(".");    
    delay(1000);
  }
  Serial.println();

  // set up wifi
  wifiConnect();

  client.setServer(mqtt_server, 1883);
  client.setBufferSize(256);

  Serial.print("[info] MQTT server is: ");
  Serial.println(mqtt_server);

  while(!client.connect(sensorName))
  {
    Serial.print("[wifi] failed, rc=");
    Serial.println(client.state());

    delay(5000);
  
    // just reconnect on error
    wifiConnect();
  }

  client.loop();
  client.loop();

  // send hello message to mqtt
  snprintf (msg, 200, "{\"sensor\": %d, \"status\": \"start\"}", SENSOR_ID);
  Serial.print("[info] send to mqtt server ");
  Serial.print(mqtt_server);
  Serial.print(": ");
  Serial.println(msg);
  client.publish("home/retic/status", msg);
  client.loop();

  // override: don't measure temperature, takes up too many cycles and interrupts get missed
  measureTemperature = 0;

  loopStartMillis = millis();
  last_micros = micros();
  lastSendTimerTicks = timerTicks;
}

uint32_t usToTicks(uint32_t us) {
    return (TIMER1_TICKS_PER_US / 16 * us);     // converts microseconds to ticks
}

void setOutputPin() {
  // this function runs in the main loop
  // if the ISR worked out that we need to pull the output pin high then do so
  // otherwise pull the output pin low

  if(clearTrigger > micros()) {
    // pull output pin high if needed
    if (outputState != HIGH) {
      digitalWrite(GPIO_OUTPUT_PIN, HIGH);
      outputState = HIGH;
      #ifdef DEBUG_TO_SERIAL
      Serial.println("[info] Output pin high");
      #endif
      digitalWrite(GPIO_LED, LOW);    // switch on LED by writing LOW (Vcc) to the GPIO pin
      outputTicks++;
    }
  } else {
    // pull output pin low
    if (outputState != LOW) {
      digitalWrite(GPIO_OUTPUT_PIN, LOW);
      outputState = LOW;
      #ifdef DEBUG_TO_SERIAL
      Serial.println("[info] Output pin low");
      #endif
      digitalWrite(GPIO_LED, HIGH);   // switch off LED by writing HIGH (0V) to the GPIO pin
    }
  }
}

void sendMqttMessage() {
  long deltaTicks = ticks - lastTicks;
  long deltaSendTimerTicks = timerTicks - lastSendTimerTicks;
  lastTimerTicks = timerTicks;
  lastTicks = ticks;

  // measure main loop frequency (cycles per second)
  float freq = (loopCount * 1.0f) / (millis() - loopStartMillis) * 1000.0f;
  if (freq == INFINITY) { 
    freq = 0; 
  }
  
  #ifdef DEBUG_TO_SERIAL
  Serial.print("[info] timerTicks: ");
  Serial.print(lastTimerTicks);
  Serial.print(", Ticks: ");
  Serial.print(ticks);
  Serial.print(", deltaTicks: ");
  Serial.print(deltaTicks);
  Serial.print(", tickTrigger: ");
  Serial.print(tickTrigger);
  Serial.print(", outputTicks: ");
  Serial.print(outputTicks);
  Serial.print(", freq: ");
  Serial.print(freq);
  Serial.print(", inactiveCycles: ");
  Serial.print(inactiveCount);
  Serial.print(", deltaSendTimerTicks: ");
  Serial.print(deltaSendTimerTicks);
  Serial.println();
  #endif

  if ((deltaTicks != 0) || (deltaSendTimerTicks % MESSAGE_REPEAT_THRESHOLD == 0)) {
    snprintf (msg, 200, "{\"sensor\": %d, \"tticks\": %d, \"fticks\": %d, \"dft\": %d, \"ftcd\": %d, \"ot\": %d, \"freq\": %.3f, \"iac\": %d, \"ms\": %d}", 
                            SENSOR_ID,    timerTicks,     ticks,          deltaTicks,   tickTrigger,  outputTicks,    freq,      inactiveCount,  millis());
    Serial.print("[info] send to mqtt server ");
    Serial.print(mqtt_server);
    Serial.print(": ");
    Serial.println(msg);
    
    client.publish("home/retic/flow", msg);
    client.loop();
    
    lastSendTimerTicks = timerTicks;

    // only reset the inactive count if deltaTicks is non-zero
    if (deltaTicks != 0) {
      inactiveCount = RESET_THRESHOLD;
    }
  } else {
    inactiveCount--;
  }  
}

void resetDueToInactivity() {
  Serial.println("[warn] restarting due to inactivity");

  snprintf (msg, 200, "{\"sensor\": %d, \"status\": \"restarting\"}", SENSOR_ID);
  Serial.print("[info] send to mqtt server ");
  Serial.print(mqtt_server);
  Serial.print(": ");
  Serial.println(msg);
  client.publish("home/retic/status", msg);
  client.loop();
  yield();
  delay(5000);

  ESP.restart();
  delay(5000);
}

void checkAndSendTemperature() {
  if (timerTicks == lastTemperatureCheck) {
    // already sent
    return;
  }
  
  lastTemperatureCheck = timerTicks;

  digitalWrite(ONE_WIRE_BUS, LOW);
  OneWire oneWire1(ONE_WIRE_BUS);
  DallasTemperature DS18B20(&oneWire1);
  float temp1;
  int count = 0;

  for (count = 0; count < 50; count++) {
    DS18B20.requestTemperatures(); 
    temp1 = DS18B20.getTempCByIndex(0);
    Serial.printf("[info] temperature 1: %.3f", temp1);
    Serial.println();

    // 85.0 and -127.0 are misreads, try again if we get these values
    if (temp1 != 85.0 && temp1 != (-127.0)) {
      break;
    }
  }

  if (count >= 50) {
    // temp sensor is broken, give up and stop recording temperature until the system restarting
    Serial.println("[warn] stopping temperature measurement as sensor gave too many invalid readings");
    measureTemperature = 0;
  } else {
    snprintf (msg, 200, "{\"sensor\": %d, \"temperature\": %.3f}", SENSOR_ID, temp1);
    Serial.print("[info] send to mqtt server ");
    Serial.print(mqtt_server);
    Serial.print(": ");
    Serial.println(msg);
    client.publish("home/retic/temperature", msg);
    client.loop();
  }
}
    
void loop() {
  loopCount++;
  client.loop();

  setOutputPin();

  if (lastTimerTicks != timerTicks) {
    sendMqttMessage();
    ESP.wdtFeed();
  }

  if (inactiveCount <= 0) {
    resetDueToInactivity();
  }

  if ((timerTicks % TEMPERATURE_CHECK == 0)) {
    if (measureTemperature != 0) {
      checkAndSendTemperature();
    }
  }

  // don't need to call yield(), the arduino framework runs esp8266 scheduled functions and other background tasks after loop() completes.
}

void ICACHE_RAM_ATTR timerCallback() {
  timerTicks++;
}

void ICACHE_RAM_ATTR pinChanged() {
  unsigned long current_micros;
  long delta;
  
  current_micros = micros();
  delta = (long)(current_micros - last_micros);
  
  if(delta >= DEBOUNCE_TIME       // debounced ok
     
     // the condition below should never happen as we restart the system faster than micros() overflows
     // || delta < 0                      // last_micros was larger than micros(), which means micros() wrapped
    ) {
    ticks++;
    last_micros = current_micros;

    // software shift register - decrement the trigger variable until it hits zero.
    // once at zero, set the desired period where the output pin is held high for the opensprinkler to pick up as a flow tick
    tickTrigger--;

    if (tickTrigger <= 0) {
      clearTrigger = current_micros + TRIGGER_INTERVAL_US;
      tickTrigger = TRIGGER_DIVISOR;
    }
  }
}

void wifiConnect() {
  Serial.print("[wifi] Connecting to AP: ");
  Serial.println(AP_SSID);
  WiFi.begin(AP_SSID, AP_PASSWORD);

  while(WiFi.status() != WL_CONNECTED) {
    for (int i = 0; (i < MAX_WIFI_BEGIN_RETRIES) && (WiFi.status() != WL_CONNECTED); i++) {
      delay(1000);
      Serial.print("[loop ");
      Serial.print(i);
      Serial.print("] Status: ");
      Serial.println(WiFi.status());
    }
  
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("[loop] Cannot connect. yielding for 10 seconds before reset");
      for (int i = 0; i < 10; i++) {
        Serial.print(".");
        delay(1000);
      }
      Serial.println("[warn] Reset");
      ESP.restart();
      delay(5000);
    }
  }
  
  Serial.print("[wifi] connected, local IP: ");  
  Serial.println(WiFi.localIP());
}
