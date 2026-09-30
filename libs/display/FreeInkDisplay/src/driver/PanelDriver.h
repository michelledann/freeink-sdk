#pragma once

// FreeInk SDK — panel driver interface.
//
// One PanelDriver implementation exists per display controller (SSD1677,
// UC8253-X3, ED2208-M5, UC8253-Murphy). The FreeInkDisplay facade owns the
// framebuffer and selects a driver at begin(); the driver owns all
// controller-specific register sequences, LUTs, timing, and cross-call state.
//
// The facade does all framebuffer composition (clear/draw) itself and passes
// raw buffer pointers in here — drivers only touch hardware. `prev` is the
// previous frame in dual-buffer mode, or nullptr in single-buffer mode (the
// controller's own RAM holds the previous frame).

#include <Arduino.h>

#include "../bus/EpdBus.h"

namespace freeink {

enum class RefreshMode : uint8_t { Full, Half, Fast };
enum class GrayPlane : uint8_t { Lsb, Msb };

struct PanelGeometry {
  uint16_t width;
  uint16_t height;
  uint16_t widthBytes;
  uint32_t bufferSize;
};

class PanelDriver {
 public:
  virtual ~PanelDriver() = default;

  // --- bus configuration (consumed by the facade before begin()) ---
  virtual uint32_t spiHz() const = 0;
  virtual BusyPolarity busyPolarity() const = 0;
  virtual PanelGeometry geometry() const = 0;
  virtual int8_t spiMiso() const { return -1; }  // SSD1677 uses none; M5 shares MISO
  virtual int8_t coCs() const { return -1; }      // co-resident SPI CS to hold high (M5 SD)

  // True for drivers backed by an external library that manages its own SPI /
  // display hardware (e.g. M5GFX, EPD_Painter). When true the facade does NOT
  // bring up its EpdBus — the driver owns the panel end to end.
  virtual bool usesExternalBus() const { return false; }

  // --- lifecycle ---
  virtual void begin(EpdBus& bus) = 0;
  virtual void deepSleep(EpdBus& bus) = 0;

  // --- core paint path (load RAM + refresh) ---
  virtual void display(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, RefreshMode mode, bool turnOff) = 0;
  virtual void displayWindow(EpdBus& bus, const uint8_t* fb, const uint8_t* prev, uint16_t x, uint16_t y, uint16_t w,
                             uint16_t h, bool turnOff) {
    display(bus, fb, prev, RefreshMode::Fast, turnOff);
  }

  // --- grayscale (dual-plane LSB/MSB) ---
  virtual bool supportsStripGrayscale() const { return false; }
  // Display `fb` as the base frame for a grayscale overlay that follows.
  // X3 runs the OEM pipeline (the "AA-pre-BW(mid)" bank as a differential
  // base update with calibrated drives); panels without a dedicated base
  // waveform fall back to a plain display() with `fallback` mode, preserving
  // their previous behavior.
  virtual void displayGrayscaleBase(EpdBus& bus, const uint8_t* fb, RefreshMode fallback, bool turnOff) {
    display(bus, fb, nullptr, fallback, turnOff);
  }

  // Grayscale preconditioning settle pass (OEM X3 "AA-pre-BW(mid)"), windowed
  // to the panel rect [x, x+w) x [y, y+h) like the OEM's PTL usage; fire after
  // the BW base frame is displayed, before grayscale planes are written.
  // Default no-op for panels whose grayscale needs no conditioning.
  virtual void preconditionGrayscale(EpdBus& bus, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    (void)bus; (void)x; (void)y; (void)w; (void)h;
  }
  virtual void copyGrayscaleLsb(EpdBus& bus, const uint8_t* lsb) { (void)bus; (void)lsb; }
  virtual void copyGrayscaleMsb(EpdBus& bus, const uint8_t* msb) { (void)bus; (void)msb; }
  virtual void writeGrayscalePlaneStrip(EpdBus& bus, GrayPlane plane, const uint8_t* rows, uint16_t yStart,
                                        uint16_t numRows) {
    (void)bus; (void)plane; (void)rows; (void)yStart; (void)numRows;
  }
  virtual void displayGray(EpdBus& bus, const uint8_t* fb, bool turnOff, const unsigned char* lut, bool factoryMode) {
    (void)lut;
    (void)factoryMode;
    display(bus, fb, nullptr, RefreshMode::Fast, turnOff);
  }
  virtual void cleanupGrayscaleBuffers(EpdBus& bus, const uint8_t* bw) { (void)bus; (void)bw; }

  // --- optional, controller-specific hooks (no-op by default) ---
  virtual void requestResync(uint8_t settlePasses) { (void)settlePasses; }
  virtual void skipInitialResync() {}
  virtual void requestCompleteWaveformNextRefresh() {}
  // Caller-confirmed hint: this begin() is a warm wake (e.g. an ESP32
  // deep-sleep timer wakeup) where the panel stayed powered the whole time,
  // not a genuine cold boot. Controllers whose init sequence would otherwise
  // unconditionally clear/reset RAM or force a full refresh can use this to
  // skip that and preserve the panel's real prior content for a differential
  // (fast) refresh. Call before begin(). No-op by default.
  virtual void setWarmWake(bool warm) { (void)warm; }
  // Caller-confirmed hint, with setWarmWake(true): the panel's power stayed on
  // through the sleep too, so registers and RAM are intact and the resets can
  // be skipped. False: the panel lost power -- reset and re-init in full (RAM
  // is undefined; the caller supplies the previous frame to display()).
  virtual void setPanelPowerHeld(bool held) { (void)held; }
  // Interrupted-refresh cutoff tuning (ED2208: where the gate scan freezes).
  virtual void setFastRefreshCutoffMs(uint16_t ms) { (void)ms; }
  virtual uint16_t fastRefreshCutoffMs() const { return 0; }
  virtual void grayscaleRevert(EpdBus& bus, const uint8_t* fb) { (void)bus; (void)fb; }
  virtual void setCustomLut(EpdBus& bus, bool enabled, const unsigned char* data) { (void)bus; (void)enabled; (void)data; }
};

}  // namespace freeink
