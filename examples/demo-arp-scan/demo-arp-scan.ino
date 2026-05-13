/*
 * demo-arp-scan.ino
 *
 * Active ARP scan example for modem-freeRTOS (WiFi mode only).
 *
 * The sketch connects to a WiFi network and then performs an ARP sweep of the
 * entire local subnet.  Every host that replies is printed to Serial together
 * with its MAC address.
 *
 * Hardware: any ESP32 board.
 *
 * Wiring: none – uses the on-board WiFi radio.
 *
 * Compile-time note:
 *   Make sure editable_macros.h does NOT define ENABLE_LTE (WiFi mode).
 */

#include "modem-freeRTOS.hpp"

// ── WiFi credentials ─────────────────────────────────────────────────────────
#define WIFI_SSID  "Inloc"
#define WIFI_PASS  "inlocAPpwd"
// ─────────────────────────────────────────────────────────────────────────────

// How many hosts to store at most (must be <= ARP_SCAN_MAX_HOSTS = 32 by default)
#define MAX_RESULTS 32

// Milliseconds to wait for ARP replies after the sweep
#define ARP_TIMEOUT_MS 3000

MODEMfreeRTOS mRTOS;

// EspMQTTClient (included by modem-freeRTOS in WiFi mode) requires this
// global callback to exist at link time, even when MQTT is not used.
void onConnectionEstablished() {}

// ── helpers ──────────────────────────────────────────────────────────────────

static void printMAC(const uint8_t mac[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print(buf);
}

// ── setup ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== modem-freeRTOS  active ARP scan demo ===");

  // Initialise the library in WiFi mode
  mRTOS.init(WIFI_SSID, WIFI_PASS);

  // Wait until the WiFi association is established
  Serial.print("Connecting to WiFi");
  uint32_t t = millis();
  while (!MODEMfreeRTOS::isWifiConnected()) {
    mRTOS.loop();
    if (millis() - t > 30000) {
      Serial.println("\nFailed to connect – halting.");
      while (true) delay(1000);
    }
    delay(500);
    Serial.print('.');
  }
  Serial.println(" OK");
  Serial.printf("Local IP : %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("Subnet   : %s\n", WiFi.subnetMask().toString().c_str());
}

// ── loop ─────────────────────────────────────────────────────────────────────

void loop() {

  mRTOS.loop(); // keep the library state-machine running

  static uint32_t lastScan = 0;
  const  uint32_t SCAN_INTERVAL_MS = 30000; // repeat every 30 s

  if (millis() - lastScan >= SCAN_INTERVAL_MS || lastScan == 0) {
    lastScan = millis();

    if (!MODEMfreeRTOS::isWifiConnected()) {
      Serial.println("[demo] WiFi not connected, skipping scan.");
      return;
    }
    
    // ── run IP scan ─────────────────────────────────────────────────────
    Serial.println("\n--- Starting IP ARP scan ---");
    ARP_HOST host[1];
    bool res = mRTOS.arp_scan_ip(IPAddress(192, 168, 1, 3), host, ARP_TIMEOUT_MS); // example: resolve gateway IP
    if(res){
      Serial.printf("Gateway %s is at ", host[0].ip.toString().c_str());
      printMAC(host[0].mac);
      Serial.println();
    } else {
      Serial.println("Failed to resolve gateway IP");
    
    }
    // ── run the scan ─────────────────────────────────────────────────────
    Serial.println("\n--- Starting ARP scan ---");
    ARP_HOST results[MAX_RESULTS];
    uint8_t found = mRTOS.arp_scan(results, MAX_RESULTS, ARP_TIMEOUT_MS);

    // ── print results ────────────────────────────────────────────────────
    Serial.printf("--- Scan complete: %u live host(s) ---\n", found);
    for (uint8_t i = 0; i < found; i++) {
      Serial.printf("  [%2u]  %-15s  ", i + 1, results[i].ip.toString().c_str());
      printMAC(results[i].mac);
      Serial.println();
    }
    if (found == 0) {
      Serial.println("  (no hosts responded – check subnet / timeout)");
    }
    Serial.println("---------------------------------------");
  }
}
