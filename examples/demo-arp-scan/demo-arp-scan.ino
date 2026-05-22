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
 * Logging:
 *   On ESP32-C5 the built-in USB Serial/JTAG peripheral is exposed as
 *   the standard `Serial` object when the Arduino board option
 *   "USB CDC On Boot" is set to "Enabled" (default for most ESP32-C5 boards).
 *
 *   Just connect the USB-C cable and open /dev/cu.usbmodem* (macOS) or
 *   COMx (Windows) at any baud rate – no USB-to-UART adapter needed.
 *
 *   If you also want UART0 output on the TX pin, set
 *   "USB CDC On Boot" to "Disabled" and use an external adapter at 115200.
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

// ── Logging ───────────────────────────────────────────────────────────────────
// On ESP32-C5 with "USB CDC On Boot: Enabled", Serial goes directly to the
// built-in USB Serial/JTAG port – no adapter needed.
#define LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)

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
  LOG("\n=== modem-freeRTOS  active ARP scan demo ===\n");

  // Initialise the library in WiFi mode
  mRTOS.init(WIFI_SSID, WIFI_PASS);

  // Wait until the WiFi association is established
  LOG("Connecting to WiFi");
  uint32_t t = millis();
  while (!MODEMfreeRTOS::isWifiConnected()) {
    mRTOS.loop();
    if (millis() - t > 30000) {
      LOG("\nFailed to connect - halting.\n");
      while (true) delay(1000);
    }
    delay(500);
    LOG(".");
  }
  LOG(" OK\n");
  LOG("Local IP : %s\n", WiFi.localIP().toString().c_str());
  LOG("Subnet   : %s\n", WiFi.subnetMask().toString().c_str());
}

// ── loop ─────────────────────────────────────────────────────────────────────

void loop() {

  mRTOS.loop(); // keep the library state-machine running

  static uint32_t lastScan = 0;
  const  uint32_t SCAN_INTERVAL_MS = 30000; // repeat every 30 s

  if (millis() - lastScan >= SCAN_INTERVAL_MS || lastScan == 0) {
    lastScan = millis();

    if (!MODEMfreeRTOS::isWifiConnected()) {
      LOG("[demo] WiFi not connected, skipping scan.\n");
      return;
    }

    // ── run IP scan ─────────────────────────────────────────────────────
    LOG("\n--- Starting IP ARP scan ---\n");
    ARP_HOST host[1];
    bool res = mRTOS.arp_scan_ip(IPAddress(192, 168, 1, 3), host, ARP_TIMEOUT_MS);
    if (res) {
      LOG("Gateway %s is at ", host[0].ip.toString().c_str());
      printMAC(host[0].mac);
      LOG("\n");
    } else {
      LOG("Failed to resolve gateway IP\n");
    }

    // ── run the scan ─────────────────────────────────────────────────────
    LOG("\n--- Starting ARP scan ---\n");
    ARP_HOST results[MAX_RESULTS];
    uint8_t found = mRTOS.arp_scan(results, MAX_RESULTS, ARP_TIMEOUT_MS);

    // ── print results ────────────────────────────────────────────────────
    LOG("--- Scan complete: %u live host(s) ---\n", found);
    for (uint8_t i = 0; i < found; i++) {
      LOG("  [%2u]  %-15s  ", i + 1, results[i].ip.toString().c_str());
      printMAC(results[i].mac);
      LOG("\n");
    }
    if (found == 0) {
      LOG("  (no hosts responded - check subnet / timeout)\n");
    }
    LOG("---------------------------------------\n");
  }

  delay(500);
}
