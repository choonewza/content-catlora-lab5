#include "CatLoRaS76S.h"
#include "LedModule.h"

#define RED_LED_PIN 13
#define GREEN_LED_PIN 12

#define LORA_ABP_CLASS "C"
#define LORA_DEV_EUI "1296D01174802100"
#define LORA_DEV_ADDR "74802100"
#define LORA_NWKS_KEY "28AED22B7E1516A609CFABF715884F3C"
#define LORA_APPS_KEY "1628AE2B7E15D2A6ABF7CF4F3C158809"

LedModule redLed(RED_LED_PIN);
LedModule greenLed(GREEN_LED_PIN);

CatLoRaS76S lora;

int sampleRate = 1000;

void setup() {
  Serial.begin(115200);
  delay(2000);

  redLed.begin();
  greenLed.begin();

  tcConfigure(1000);
  tcStartCounter();

  Serial.println("-> Lora Setting...");
  lora.begin(115200);

  Serial.println("-> Lora ABP Join...");
  lora.joinABP(String(LORA_ABP_CLASS), String(LORA_DEV_EUI), String(LORA_DEV_ADDR), String(LORA_NWKS_KEY), String(LORA_APPS_KEY));

  tcDisable();
  tcReset();

  greenLed.on();
  Serial.println("->Ready Go.");
}

void loop() {
  //Your Code
}

void TC5_Handler (void) {
  //----this code will run every second----
  redLed.toggle();
  //--------------------------------------
  -
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //Writing a 1 to INTFLAG.bit.MC0 clears the interrupt so that it will run again
}
void tcConfigure(int sampleRate)
{
  // Enable GCLK for TCC2 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);

  tcReset(); //reset TC5

  // Set Timer counter Mode to 16 bits
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  // Set TC5 mode as match frequency
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  //set prescaler and enable TC5
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE; //you can use different prescaler divisons here like TC_CTRLA_PRESCALER_DIV1 to get different ranges of frequencies
  //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
  TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
  while (tcIsSyncing());

  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 0);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (tcIsSyncing()); //wait until TC5 is done syncing
}

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}
