/*
 * demo-arp-reverse.ino
 *
 * ARP scan with reverse DNS example for modem-freeRTOS (WiFi mode only).
 *
 * The sketch connects to a WiFi network and then performs an ARP sweep of
 * the entire local subnet. For every live host it sends a reverse DNS (PTR)
 * query to the router's DNS server to resolve the device hostname.
 *
 * This is the most effective way to map all devices on the network –
 * including phones, TVs, and IoT devices – because:
 *   - ARP works at layer 2: every device must respond regardless of software.
 *   - Reverse DNS uses the router's DHCP/DNS records, which usually contain
 *     device names as registered during IP assignment.
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
#define WIFI_SSID  "Inloc-5G"
#define WIFI_PASS  "inlocAPpwd"
// ─────────────────────────────────────────────────────────────────────────────

// Milliseconds to wait for ARP replies after the sweep
#define ARP_TIMEOUT_MS   3000
// Milliseconds to wait for each reverse DNS (PTR) reply
#define DNS_TIMEOUT_MS   1000
// Repeat scan every N milliseconds
#define SCAN_INTERVAL_MS 30000

MODEMfreeRTOS mRTOS;

// EspMQTTClient (included by modem-freeRTOS in WiFi mode) requires this
// global callback to exist at link time, even when MQTT is not used.
void onConnectionEstablished() {}

// ── setup ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== modem-freeRTOS  ARP + reverse DNS demo ===");

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
  Serial.printf("DNS      : %s\n", WiFi.dnsIP().toString().c_str());
}

// ── loop ─────────────────────────────────────────────────────────────────────

void loop() {

  mRTOS.loop(); // keep the library state-machine running

  static uint32_t lastScan = 0;

  if (millis() - lastScan >= SCAN_INTERVAL_MS || lastScan == 0) {
    lastScan = millis();

    if (!MODEMfreeRTOS::isWifiConnected()) {
      Serial.println("[demo] WiFi not connected, skipping scan.");
      return;
    }

    Serial.println("\n--- ARP + reverse DNS scan ---");

    NETWORK_HOST devices[ARP_SCAN_MAX_HOSTS];
    uint8_t found = mRTOS.arp_scan_with_names(devices, ARP_SCAN_MAX_HOSTS,
                                              ARP_TIMEOUT_MS,
                                              DNS_TIMEOUT_MS);

    Serial.printf("--- Scan complete: %u device(s) ---\n", found);
    for (uint8_t i = 0; i < found; i++) {
      char mac[18];
      snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
               devices[i].mac[0], devices[i].mac[1], devices[i].mac[2],
               devices[i].mac[3], devices[i].mac[4], devices[i].mac[5]);
      Serial.printf("  [%2u]  %-15s  %s  %s\n",
                    i + 1,
                    devices[i].ip.toString().c_str(),
                    mac,
                    devices[i].hostname[0] ? devices[i].hostname : "(unresolved)");
    }
    if (found == 0)
      Serial.println("  (no devices found – check subnet / timeout)");
    Serial.println("---------------------------------------");
  }
}
