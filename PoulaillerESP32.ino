/***************************************************************
 * BIBLIOTHÈQUES NÉCESSAIRES (à installer avant compilation)
 *
 * ESP RainMaker (version 1.4.0 ou plus récent)
 *    Nom : ESP RainMaker
 *    Lien : https://github.com/espressif/esp-rainmaker
 *
 * ESP32 Servo (version 0.11.0 recommandé)
 *    Nom : ESP32Servo
 *    Lien : https://github.com/madhephaestus/ESP32Servo
 *
 * Carte ESP32 (URL du gestionnaire de carte à ajouter dans l’IDE Arduino) :
 *    https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 *    Nom de la carte : ESP32 Dev Module (ou WROOM32, selon ton modèle)
 ***************************************************************/
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <ESP32Servo.h>

#define DEVICE_NAME       "PoulaillerESP"
#define SERVICE_NAME      "Poulailler_Device"
#define POP               "12345678"
#define POMPE_PIN         14
#define SERVO_PIN         13

Servo monServo;

// 1️⃣ Déclaration des Devices
static Switch pompe("Pompe", NULL);  // Device 1
static Device servoMotor("ServoMoteur", "esp.device.motor", NULL); // Device 2
static Param servoAngle("Angle", "esp.param.slider", value(90), PROP_FLAG_READ | PROP_FLAG_WRITE);

void write_callback(Device *device, Param *param, param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx) {
  const char* devName = device->getDeviceName();
  const char* paramName = param->getParamName();

  if (strcmp(devName, "Pompe") == 0 && strcmp(paramName, "Power") == 0) {
    digitalWrite(POMPE_PIN, val.val.b ? HIGH : LOW);
    param->updateAndReport(val);
  }

  if (strcmp(devName, "ServoMoteur") == 0 && strcmp(paramName, "Angle") == 0) {
    int angle = constrain(val.val.i, 0, 180);
    monServo.write(angle);
    Serial.printf("🎚️ Servo moved to %d°\n", angle);
    param->updateAndReport(val);
  }
}

void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
      Serial.printf("Provisioning started. Connect to \"%s\" via BLE\nPOP: %s\n", SERVICE_NAME, POP);
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("✅ Wi-Fi provisioning successful.");
      break;
    case ARDUINO_EVENT_PROV_CRED_FAIL:
      Serial.println("❌ Wi-Fi provisioning failed.");
      break;
    case ARDUINO_EVENT_PROV_END:
      Serial.println("✅ Provisioning finished.");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(POMPE_PIN, OUTPUT);
  digitalWrite(POMPE_PIN, LOW);

  monServo.attach(SERVO_PIN);
  monServo.write(90);

  Node node = RMaker.initNode(DEVICE_NAME);

  // 2️⃣ Pompe
  pompe.addCb(write_callback);
  node.addDevice(pompe);

  // 3️⃣ Servo avec slider
  servoAngle.addBounds(value(0), value(180), value(1));
  servoAngle.addUIType(ESP_RMAKER_UI_SLIDER);
  servoMotor.addParam(servoAngle);
  servoMotor.addCb(write_callback);
  node.addDevice(servoMotor);

  WiFi.onEvent(sysProvEvent);
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, POP, SERVICE_NAME);
  WiFiProv.printQR(SERVICE_NAME, POP, "ble");

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  RMaker.setTimeZone("CET-1CEST,M3.5.0/2,M10.5.0/3");
  RMaker.start();
}

void loop() {}
