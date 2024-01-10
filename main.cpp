#include "mbed.h"

#include "ACAN2517FD.h"
#include "CANSerialBridge.hpp"
#include "MbedHardwareSPI.h"

#include "mdc_client/MDCClient.hpp"

using namespace acan2517fd;

#define SPI_MOSI PA_7
#define SPI_MISO PA_6
#define SPI_SCLK PB_3
#define SPI_CS PA_4
#define SPI_INT PA_3
#define ACKNOWLEDGE PA_1

Timer timer;

uint32_t getMillisecond() {
  return (uint32_t)duration_cast<std::chrono::milliseconds>(
             timer.elapsed_time())
      .count();
}

SPI spi(SPI_MOSI, SPI_MISO, SPI_SCLK);
DigitalOut acknowledge(ACKNOWLEDGE);
MbedHardwareSPI dev_spi(spi, SPI_CS);
ACAN2517FD dev_can(dev_spi, getMillisecond);
CANSerialBridge serial(&dev_can);

MDCClient mdc_client(&serial, 0);

DigitalIn canfd_int(SPI_INT);

static uint32_t gUpdateDate = 0;
static uint32_t gSentDate = 0;

int main() {
  timer.start();

  //  set up
  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_4MHz, 500UL * 1000UL,
                              DataBitRateFactor::x2);

  settings.mRequestedMode = ACAN2517FDSettings::NormalFD;

  settings.mDriverTransmitFIFOSize = 4;
  settings.mDriverReceiveFIFOSize = 3;

  const uint32_t errorCode0 = dev_can.begin(settings);
  if (errorCode0 == 0) {
    printf("Initializing process completed.\n\r");
  } else {
    printf("Configuration error 0x%x\n\r", errorCode0);
  }

  setting_struct_t mdc_settings = {OperatorMode::MD_OPERATOR,
                                   EncoderType::VELOCITY,
                                   1.0,
                                   false,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0};

  for (int i = 0; i < 4; i++) {
    mdc_client.update_setting(i, mdc_settings);
    mdc_client.set_target(i, 0.4);
    wait_us(1000 * 500);
  }

  while (true) {
    if (!canfd_int) {
      dev_can.isr_poll_core();
    }

    if (getMillisecond() - gUpdateDate > 40) {
      //   dev_can.poll();
      serial.update();
      if (mdc_client.update()) {
        //  toggle led
        acknowledge = !acknowledge;
      }

      gUpdateDate = getMillisecond();
    }

    if (getMillisecond() - gSentDate > 100) {
      //  send target values
      for (int i = 0; i < 4; i++) {
        mdc_client.set_target(0, 0.0);
      }
      mdc_client.send_target();

      gSentDate = getMillisecond();
    }
  }
}
