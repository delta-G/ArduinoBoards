#include "tusb.h"

#define SER Serial1

uint32_t heartbeatDelay = 2000;
const uint8_t heartPin = 13;

const uint8_t usbSwitch = 21;

const uint8_t startInput = 7;

//See hardware/renesas_uno/1.0.2/variants/UNOWIFIR4/variant.cpp for the source code.
__attribute__((weak)) void configure_usb_mux() {}

// void __maybe_start_usb() {
//     __USBStart();
// }

static const ioport_pin_cfg_t board_pin_cfg[] = {
  // USB FS D+, D-, VBus
  { .pin_cfg = IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_USB_FS, .pin = BSP_IO_PORT_04_PIN_07 },
  { .pin_cfg = IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_USB_FS, .pin = BSP_IO_PORT_09_PIN_14 },
  { .pin_cfg = IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_USB_FS, .pin = BSP_IO_PORT_09_PIN_15 },
};
static const ioport_cfg_t family_pin_cfg = {
  .number_of_pins = sizeof(board_pin_cfg) / sizeof(ioport_pin_cfg_t),
  .p_pin_cfg_data = board_pin_cfg,
};
static ioport_instance_ctrl_t port_ctrl;

void setup() {
  SER.begin(115200);
  delay(500);
  SER.println();
  SER.println("UNO R4 TinyUSB Host Experiment.");
  SER.println();
  pinMode(heartPin, OUTPUT);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(startInput, INPUT_PULLUP);

  SER.println("Holding for Pin 7 to Ground.");
  //  Hold here until we ground pin 7
  while (digitalRead(startInput)) heartbeat();


  heartbeatDelay = 500;
  SER.println("Starting Host Controller");

  // Switch the mux to the RA4M1 chip
  // configure_usb_mux();
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);

  // Set up our pins:
  R_IOPORT_Open(&port_ctrl, &family_pin_cfg);

  // Unlock Clock system registers
  R_SYSTEM->PRCR |= R_SYSTEM_PRCR_PRC1_Msk;

  // First un-Stop the USBFS system
  R_MSTP->MSTPCRB &= ~(R_MSTP_MSTPCRB_MSTPB11_Msk | R_MSTP_MSTPCRB_MSTPB12_Msk);

  // Setup SYSCFG

  R_USB_FS0->SYSCFG |= 0;  // Start in a known state.  Changing other stuff requires 0 in some bits.
  // Clock select while SYSCFG.SCKE is 0
  R_SYSTEM->USBCKCR = 0;  // Always 0 for host mode.
  //  First enable the clock.
  R_USB_FS0->SYSCFG |= R_USB_FS0_SYSCFG_SCKE_Msk;
  //  Host Mode
  R_USB_FS0->SYSCFG |= R_USB_FS0_SYSCFG_DCFM_Msk;
  //  Pull-Downs
  R_USB_FS0->SYSCFG |= R_USB_FS0_SYSCFG_DRPD_Msk;
  // Enable USB
  R_USB_FS0->SYSCFG |= R_USB_FS0_SYSCFG_USBE_Msk;

  // Enable the Bus
  R_USB_FS0->DVSTCTR0 |= R_USB_FS0_DVSTCTR0_UACT_Msk;

  // Lock Clock system registers
  R_SYSTEM->PRCR &= ~R_SYSTEM_PRCR_PRC1_Msk;




  tuh_init(0);

  // Turn on the regulator
  R_USB_FS0->USBMC |= R_USB_FS0_USBMC_VDCEN_Msk;

  printSomeData();


  SER.println("End of Setup.");
}

void loop() {
  if (digitalRead(8) == LOW) {
    // R_USB_FS0->INTSTS1 &= ~R_USB_FS0_INTSTS1_ATTCH_Msk;
    R_USB_FS0->INTSTS1 = 0;
  }
  int pin9State = digitalRead(9);
  static int oldPin9State = 0;
  if (pin9State != oldPin9State) {
    if (pin9State == LOW) {
      // reset USB
      SER.println("Resetting USB");
      R_USB_FS0->DVSTCTR0 = 0x0200;
      R_USB_FS0->DVSTCTR0 |= R_USB_FS0_DVSTCTR0_USBRST_Msk;
      delay(15);
      R_USB_FS0->DVSTCTR0 = 0x0210;
    }
    oldPin9State = pin9State;
  }

  printSomeData();
  heartbeat();
  tuh_task();
  // static unsigned long pm = millis();
  // unsigned long cm = millis();
  // if (cm - pm >= 200) {
  //   pm = cm;
  // }
}


void heartbeat() {
  static boolean heartState = false;
  static unsigned long pm = millis();
  unsigned long cm = millis();

  if (cm - pm >= heartbeatDelay) {
    heartState = !heartState;
    digitalWrite(heartPin, heartState);
    pm = cm;
  }
}

void printSomeData() {
  volatile uint16_t dvstctr0 = R_USB_FS0->DVSTCTR0;
  static uint16_t old_dvstctr0 = 0;
  if (dvstctr0 != old_dvstctr0) {
    SER.print("OLD DVSTCTR0 : ");
    SER.println(old_dvstctr0, HEX);
    SER.print("NEW DVSTCTR0 : ");
    SER.println(dvstctr0, HEX);
    old_dvstctr0 = dvstctr0;
  }
  volatile uint16_t syssts0 = R_USB_FS0->SYSSTS0;
  static uint16_t old_syssts0 = 0;
  if (syssts0 != old_syssts0) {
    SER.print("OLD SYSSTS0 : ");
    SER.println(old_syssts0, HEX);
    SER.print("NEW SYSSTS0 : ");
    SER.println(syssts0, HEX);
    old_syssts0 = syssts0;
  }
  volatile uint16_t syscfg = R_USB_FS0->SYSCFG;
  static uint16_t old_syscfg = 0;
  if (syscfg != old_syscfg) {
    SER.print("OLD SYSCFG : ");
    SER.println(old_syscfg, HEX);
    SER.print("NEW SYSCFG : ");
    SER.println(syscfg, HEX);
    old_syscfg = syscfg;
  }
  volatile uint16_t intsts0 = R_USB_FS0->INTSTS0;
  static uint16_t old_intsts0 = 0;
  if (intsts0 != old_intsts0) {
    SER.print("OLD INTSTS0 : ");
    SER.println(old_intsts0, HEX);
    SER.print("NEW INTSTS0 : ");
    SER.println(intsts0, HEX);
    old_intsts0 = intsts0;
  }
  volatile uint16_t intsts1 = R_USB_FS0->INTSTS1;
  static uint16_t old_intsts1 = 0;
  if (intsts1 != old_intsts1) {
    SER.print("OLD INTSTS1 : ");
    SER.println(old_intsts1, HEX);
    SER.print("NEW INTSTS1 : ");
    SER.println(intsts1, HEX);
    old_intsts1 = intsts1;
  }
  volatile uint16_t intenb0 = R_USB_FS0->INTENB0;
  static uint16_t old_intenb0 = 0;
  if (intenb0 != old_intenb0) {
    SER.print("OLD INTENB0 : ");
    SER.println(old_intenb0, HEX);
    SER.print("NEW INTENB0 : ");
    SER.println(intenb0, HEX);
    old_intenb0 = intenb0;
  }
  volatile uint16_t intenb1 = R_USB_FS0->INTENB1;
  static uint16_t old_intenb1 = 0;
  if (intenb1 != old_intenb1) {
    SER.print("OLD INTENB1 : ");
    SER.println(old_intenb1, HEX);
    SER.print("NEW INTENB1 : ");
    SER.println(intenb1, HEX);
    old_intenb1 = intenb1;
  }
  volatile uint16_t sofcfg = R_USB_FS0->SOFCFG;
  static uint16_t old_sofcfg = 0;
  if (sofcfg != old_sofcfg) {
    SER.print("OLD SOFCFG : ");
    SER.println(old_sofcfg, HEX);
    SER.print("NEW SOFCFG : ");
    SER.println(sofcfg, HEX);
    old_sofcfg = sofcfg;
  }
  volatile uint16_t dcpcfg = R_USB_FS0->DCPCFG;
  static uint16_t old_dcpcfg = 0;
  if (dcpcfg != old_dcpcfg) {
    SER.print("OLD DCPCFG : ");
    SER.println(old_dcpcfg, HEX);
    SER.print("NEW DCPCFG : ");
    SER.println(dcpcfg, HEX);
    old_dcpcfg = dcpcfg;
  }
  volatile uint16_t dcpctr = R_USB_FS0->DCPCTR;
  static uint16_t old_dcpctr = 0;
  if (dcpctr != old_dcpctr) {
    SER.print("OLD DCPCTR : ");
    SER.println(old_dcpctr, HEX);
    SER.print("NEW DCPCTR : ");
    SER.println(dcpctr, HEX);
    old_dcpctr = dcpctr;
  }
}

/*****************************************
************TINYUSB CALLBACKS*************
*****************************************/

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
  heartbeatDelay = 100;
  uint16_t vid;
  uint16_t pid;

  tuh_vid_pid_get(dev_addr, &vid, &pid);

  SER.print("New Device address = ");
  SER.print(dev_addr);
  SER.print("  Instance= ");
  SER.print(instance);
  SER.println("  Mounted");
  SER.print("VID = ");
  SER.print(vid);
  SER.print("  PID = ");
  SER.println(pid);

  if (!tuh_hid_receive_report(dev_addr, instance)) {
    SER.println("ERROR:  Cannot request initial report...");
  }
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  heartbeatDelay = 2000;
  SER.print("Device ");
  SER.print(dev_addr);
  SER.print("  Instance ");
  SER.print(instance);
  SER.println("  is disconnected.");
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
  SER.println("Received Report.");

  for (int i = 0; i < len; i++) {
    SER.println(report[i], HEX);
  }


  if (!tuh_hid_receive_report(dev_addr, instance)) {
    SER.println("ERROR:  Cannot request continuing report...");
  }
}

