/** @file
  B2LUsbHost - switch the Exynos 9820 DWC3 controller to host mode and
  report what the xHCI root port sees.

  Stage 1 of USB OTG host support (keyboard/mouse).  This is a standalone
  UEFI application, not a DXE driver: it is copied to the Windows ESP and
  started from startup.nsh, so the iOS firmware path is untouched.

  What it does, in order:
    1. brings the USB2 HS PHY up with the vendor sequence already proven by
       B2LUsbTest R12/R16 (PMU isolation, HSP tune, SIDDQ, POR release),
    2. switches DWC3 GCTL.PrtCapDir from device to host and waits for
       GSTS.CURMOD to follow,
    3. resets xHCI and dumps CAPLENGTH / HCSPARAMS / PORTSC,
    4. powers the root port and reports connect status and speed.

  It does not enumerate the device yet and does not touch VBUS/OTG; use a
  self-powered hub or a USB-C OTG adapter with its own power supply for the
  first measurement.  No register outside the DT-declared windows is touched.

  Address contract (merged Stock beyond2lte DT):
    DWC3/xHCI       0x10C00000 + 0x10000
    USB DRD PHY     0x10B00000 + 0x00200
    PMU syscon      0x15860000; USB PHY control at 0x72C
**/

#include <Uefi.h>

#include <Library/IoLib.h>
#include <Library/UefiApplicationEntryPoint.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

#define B2L_DWC3_BASE       0x10C00000ULL
#define B2L_USB_PHY_BASE    0x10B00000ULL
#define B2L_PMU_BASE        0x15860000ULL
#define B2L_PMU_USB_PHY     0x072C

/* USBCON registers, relative to the DRD PHY window. */
#define PHY_CTRL_VER        0x00
#define PHY_LINK_CTRL       0x04
#define PHY_LINK_PORT       0x08
#define PHY_CLKRST          0x20
#define PHY_UTMI            0x50
#define PHY_HSP             0x54
#define PHY_HSP_TUNE        0x58
#define PHY_HSP_TEST        0x5C

#define LINKCTRL_DIS_QACT_LINKGATE  BIT12
#define LINKCTRL_DIS_QACT_ID0       BIT11
#define LINKCTRL_DIS_QACT_VBUS_VALID BIT10
#define LINKCTRL_DIS_QACT_BVALID    BIT9
#define LINKCTRL_FORCE_QACT         BIT8
#define LINKCTRL_BUS_FILTER_BYPASS  (0xFU << 4)

#define LINKPORT_HUB_PORT_SEL_OCD_U3 BIT3
#define LINKPORT_HUB_PORT_SEL_OCD_U2 BIT2

#define CLKRST_PHY20_SW_RST     BIT13
#define CLKRST_PHY20_RST_SEL    BIT12
#define CLKRST_PHY30_SW_RST     BIT3
#define CLKRST_PHY30_RST_SEL    BIT2
#define CLKRST_PORT_RST         BIT1

#define UTMI_FORCE_VBUSVALID    BIT5
#define UTMI_FORCE_BVALID       BIT4
#define UTMI_DP_PULLDOWN        BIT3
#define UTMI_DM_PULLDOWN        BIT2
#define UTMI_FORCE_SUSPEND      BIT1
#define UTMI_FORCE_SLEEP        BIT0

#define HSP_VBUSVLDEXTSEL       BIT13
#define HSP_VBUSVLDEXT          BIT12
#define HSP_EN_UTMISUSPEND      BIT9
#define HSP_COMMONONN           BIT8

#define HSP_TUNE_TXVREF_MASK    (0xFU << 28)
#define HSP_TUNE_TXRES_MASK     (0x3U << 21)
#define HSP_TUNE_TXPREEMPA_MASK (0x3U << 18)
#define HSP_TUNE_SQRX_MASK      (0x7U << 8)
#define HSP_TUNE_COMPDIS_MASK   (0x7U << 0)
#define HSP_TUNE_DEVICE_VALUE   ((0xCU << 28) | (0x3U << 21) | \
                                 (0x3U << 18) | (0x3U << 8) | 0x7U)

#define HSP_TEST_SIDDQ          BIT24

#define PMU_PHY_ENABLE          BIT0

/* DWC3 global registers. */
#define DWC3_GCTL               0xC110
#define DWC3_GSTS               0xC118
#define DWC3_GSNPSID            0xC120
#define DWC3_GUSB2PHYCFG0       0xC200
#define DWC3_GUSB3PIPECTL0      0xC2C0

#define DWC3_GCTL_PRTCAP_SHIFT  12
#define DWC3_GCTL_PRTCAP_MASK   3U
#define DWC3_GCTL_PRTCAP_HOST   1U
#define DWC3_GCTL_PRTCAP_DEVICE 2U
#define DWC3_GCTL_PRTCAP_OTG    3U
#define DWC3_GCTL_CORESOFTRESET BIT11
#define DWC3_GCTL_SCALEDOWN_MASK (0x3U << 4)
#define DWC3_GCTL_DISSCRAMBLE   BIT3
#define DWC3_GCTL_DSBLCLKGTNG   BIT0

#define DWC3_GSTS_CURMOD_MASK   3U
#define DWC3_GSTS_CURMOD_HOST   1U

#define DWC3_GUSB2PHYCFG_SOFTRST BIT31
#define DWC3_GUSB2PHYCFG_SUSPHY  BIT6
#define DWC3_GUSB3PIPECTL_SUSPHY BIT17

#define DWC3_GSNPSID_MASK       0xFFFF0000U
#define DWC3_IP_DWC31           0x33310000U

/* xHCI, relative to B2L_DWC3_BASE. */
#define XHCI_CAPLENGTH          0x00
#define XHCI_HCSPARAMS1         0x04
#define XHCI_HCSPARAMS2         0x08
#define XHCI_HCCPARAMS1         0x10
#define XHCI_USBCMD             0x00
#define XHCI_USBSTS             0x04
#define XHCI_PAGESIZE           0x08
#define XHCI_PORTSC             0x400
#define XHCI_PORTSC_STRIDE      0x10

#define XHCI_CMD_RUN            BIT0
#define XHCI_CMD_HCRST          BIT1
#define XHCI_STS_HCH            BIT0
#define XHCI_STS_CNR            BIT11
#define XHCI_PORTSC_CCS         BIT0
#define XHCI_PORTSC_PED         BIT1
#define XHCI_PORTSC_PP          BIT9
#define XHCI_PORTSC_PLS_SHIFT   5
#define XHCI_PORTSC_PLS_MASK    (0xFU << XHCI_PORTSC_PLS_SHIFT)
#define XHCI_PORTSC_SPEED_SHIFT 10
#define XHCI_PORTSC_SPEED_MASK  (0xFU << XHCI_PORTSC_SPEED_SHIFT)
#define XHCI_PORTSC_CSC         BIT17
#define XHCI_PORTSC_PRC         BIT21

STATIC
UINT32
Dwc3Read32 (
  IN UINT32  Offset
  )
{
  return MmioRead32 ((UINTN)(B2L_DWC3_BASE + Offset));
}

STATIC
VOID
Dwc3Write32 (
  IN UINT32  Offset,
  IN UINT32  Value
  )
{
  MmioWrite32 ((UINTN)(B2L_DWC3_BASE + Offset), Value);
}

STATIC
UINT32
PhyRead32 (
  IN UINT32  Offset
  )
{
  return MmioRead32 ((UINTN)(B2L_USB_PHY_BASE + Offset));
}

STATIC
VOID
PhyWrite32 (
  IN UINT32  Offset,
  IN UINT32  Value
  )
{
  MmioWrite32 ((UINTN)(B2L_USB_PHY_BASE + Offset), Value);
}

STATIC
UINT32
PmuRead32 (
  IN UINT32  Offset
  )
{
  return MmioRead32 ((UINTN)(B2L_PMU_BASE + Offset));
}

STATIC
VOID
PmuWrite32 (
  IN UINT32  Offset,
  IN UINT32  Value
  )
{
  MmioWrite32 ((UINTN)(B2L_PMU_BASE + Offset), Value);
}

STATIC
EFI_STATUS
EnableHsPhy (
  VOID
  )
{
  UINT32  Value;
  UINTN   Poll;

  Value = PmuRead32 (B2L_PMU_USB_PHY);
  PmuWrite32 (B2L_PMU_USB_PHY, Value | PMU_PHY_ENABLE);
  for (Poll = 0; Poll < 20; Poll++) {
    if ((PmuRead32 (B2L_PMU_USB_PHY) & PMU_PHY_ENABLE) != 0) {
      break;
    }
    gBS->Stall (100);
  }
  if ((PmuRead32 (B2L_PMU_USB_PHY) & PMU_PHY_ENABLE) == 0) {
    Print (L"B2LHOST: PMU USB PHY isolation did not release\n");
    return EFI_DEVICE_ERROR;
  }
  gBS->Stall (1000);

  Value  = PhyRead32 (PHY_LINK_CTRL);
  Value |= LINKCTRL_DIS_QACT_LINKGATE | LINKCTRL_DIS_QACT_ID0 |
           LINKCTRL_DIS_QACT_VBUS_VALID | LINKCTRL_DIS_QACT_BVALID;
  Value &= ~LINKCTRL_FORCE_QACT;
  PhyWrite32 (PHY_LINK_CTRL, Value);
  gBS->Stall (500);
  PhyWrite32 (PHY_LINK_CTRL, PhyRead32 (PHY_LINK_CTRL) | LINKCTRL_FORCE_QACT);
  gBS->Stall (500);

  PhyWrite32 (PHY_CLKRST, PhyRead32 (PHY_CLKRST) |
              CLKRST_PHY20_SW_RST | CLKRST_PHY20_RST_SEL |
              CLKRST_PHY30_SW_RST | CLKRST_PHY30_RST_SEL);

  Value  = PhyRead32 (PHY_UTMI);
  Value &= ~(UTMI_FORCE_SUSPEND | UTMI_FORCE_SLEEP |
             UTMI_DP_PULLDOWN | UTMI_DM_PULLDOWN);
  PhyWrite32 (PHY_UTMI, Value);

  PhyWrite32 (PHY_HSP, PhyRead32 (PHY_HSP) |
              HSP_EN_UTMISUSPEND | HSP_COMMONONN);
  gBS->Stall (100);

  PhyWrite32 (PHY_LINK_CTRL, PhyRead32 (PHY_LINK_CTRL) |
              LINKCTRL_BUS_FILTER_BYPASS);

  /* Host mode drives VBUS itself.  Forcing BVALID/VBUSVALID like the device
   * path does would make the core believe a host is attached to it. */
  Value  = PhyRead32 (PHY_UTMI);
  Value &= ~(UTMI_FORCE_BVALID | UTMI_FORCE_VBUSVALID);
  PhyWrite32 (PHY_UTMI, Value);
  Value  = PhyRead32 (PHY_HSP);
  Value &= ~(HSP_VBUSVLDEXTSEL | HSP_VBUSVLDEXT);
  PhyWrite32 (PHY_HSP, Value);

  Value  = PhyRead32 (PHY_HSP_TUNE);
  Value &= ~(HSP_TUNE_TXVREF_MASK | HSP_TUNE_TXRES_MASK |
             HSP_TUNE_TXPREEMPA_MASK | HSP_TUNE_SQRX_MASK |
             HSP_TUNE_COMPDIS_MASK);
  Value |= HSP_TUNE_DEVICE_VALUE;
  PhyWrite32 (PHY_HSP_TUNE, Value);

  PhyWrite32 (PHY_HSP_TEST, PhyRead32 (PHY_HSP_TEST) & ~HSP_TEST_SIDDQ);
  gBS->Stall (10);

  Value  = PhyRead32 (PHY_CLKRST);
  Value |= CLKRST_PHY20_RST_SEL;
  Value &= ~(CLKRST_PHY20_SW_RST | CLKRST_PHY30_SW_RST | CLKRST_PORT_RST);
  PhyWrite32 (PHY_CLKRST, Value);
  gBS->Stall (75);

  PhyWrite32 (PHY_LINK_PORT, PhyRead32 (PHY_LINK_PORT) |
              LINKPORT_HUB_PORT_SEL_OCD_U3 | LINKPORT_HUB_PORT_SEL_OCD_U2);

  if ((PmuRead32 (B2L_PMU_USB_PHY) & PMU_PHY_ENABLE) == 0 ||
      (PhyRead32 (PHY_HSP_TEST) & HSP_TEST_SIDDQ) != 0) {
    Print (L"B2LHOST: PHY did not stay enabled PMU=%08x HSP_TEST=%08x\n",
           PmuRead32 (B2L_PMU_USB_PHY), PhyRead32 (PHY_HSP_TEST));
    return EFI_DEVICE_ERROR;
  }
  Print (L"B2LHOST: HS PHY up PMU=%08x HSP_TEST=%08x\n",
         PmuRead32 (B2L_PMU_USB_PHY), PhyRead32 (PHY_HSP_TEST));
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
SwitchToHost (
  VOID
  )
{
  UINT32  Gctl;
  UINT32  Gsts;
  UINTN   Poll;

  Gctl  = Dwc3Read32 (DWC3_GCTL);
  Gctl &= ~(DWC3_GCTL_SCALEDOWN_MASK | DWC3_GCTL_DISSCRAMBLE |
            DWC3_GCTL_DSBLCLKGTNG | (DWC3_GCTL_PRTCAP_MASK <<
                                     DWC3_GCTL_PRTCAP_SHIFT));
  Gctl |= (DWC3_GCTL_PRTCAP_HOST << DWC3_GCTL_PRTCAP_SHIFT);
  Dwc3Write32 (DWC3_GCTL, Gctl);
  gBS->Stall (1000);

  for (Poll = 0; Poll < 50; Poll++) {
    Gsts = Dwc3Read32 (DWC3_GSTS);
    if ((Gsts & DWC3_GSTS_CURMOD_MASK) == DWC3_GSTS_CURMOD_HOST) {
      break;
    }
    gBS->Stall (1000);
  }
  Gsts = Dwc3Read32 (DWC3_GSTS);
  Print (L"B2LHOST: GCTL=%08x GSTS=%08x CURMOD=%u\n",
         Dwc3Read32 (DWC3_GCTL), Gsts, Gsts & DWC3_GSTS_CURMOD_MASK);
  if ((Gsts & DWC3_GSTS_CURMOD_MASK) != DWC3_GSTS_CURMOD_HOST) {
    return EFI_DEVICE_ERROR;
  }

  /* Keep the USB2 PHY awake: the host controller must be able to reset a
   * newly attached device at any time. */
  Dwc3Write32 (DWC3_GUSB2PHYCFG0,
               Dwc3Read32 (DWC3_GUSB2PHYCFG0) &
               ~(DWC3_GUSB2PHYCFG_SUSPHY | DWC3_GUSB2PHYCFG_SOFTRST));

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
ResetXhci (
  OUT UINT32  *Caplength,
  OUT UINT32  *Hcsparams1
  )
{
  UINT32  Cap;
  UINT32  OpBase;
  UINT32  Value;
  UINTN   Poll;

  Cap = MmioRead32 ((UINTN)(B2L_DWC3_BASE + XHCI_CAPLENGTH));
  *Caplength  = Cap & 0xFF;
  *Hcsparams1 = MmioRead32 ((UINTN)(B2L_DWC3_BASE + XHCI_HCSPARAMS1));
  Print (L"B2LHOST: xHCI CAPLENGTH=%u HCIVERSION=%04x HCSPARAMS1=%08x HCSPARAMS2=%08x\n",
         *Caplength, (Cap >> 16) & 0xFFFF, *Hcsparams1,
         MmioRead32 ((UINTN)(B2L_DWC3_BASE + XHCI_HCSPARAMS2)));
  if (*Caplength == 0 || *Caplength == 0xFF) {
    Print (L"B2LHOST: no xHCI capability block at the DWC3 base\n");
    return EFI_DEVICE_ERROR;
  }

  OpBase = (UINT32)B2L_DWC3_BASE + *Caplength;
  MmioWrite32 (OpBase + XHCI_USBCMD, 0);
  for (Poll = 0; Poll < 100; Poll++) {
    if ((MmioRead32 (OpBase + XHCI_USBSTS) & XHCI_STS_HCH) != 0) {
      break;
    }
    gBS->Stall (1000);
  }

  MmioWrite32 (OpBase + XHCI_USBCMD, XHCI_CMD_HCRST);
  for (Poll = 0; Poll < 200; Poll++) {
    Value = MmioRead32 (OpBase + XHCI_USBCMD);
    if ((Value & XHCI_CMD_HCRST) == 0 &&
        (MmioRead32 (OpBase + XHCI_USBSTS) & XHCI_STS_CNR) == 0) {
      break;
    }
    gBS->Stall (1000);
  }
  Print (L"B2LHOST: xHCI reset USBCMD=%08x USBSTS=%08x after %u ms\n",
         MmioRead32 (OpBase + XHCI_USBCMD),
         MmioRead32 (OpBase + XHCI_USBSTS), (UINT32)(Poll + 1));

  MmioWrite32 (OpBase + XHCI_USBCMD, XHCI_CMD_RUN);
  gBS->Stall (2000);
  Print (L"B2LHOST: xHCI run USBCMD=%08x USBSTS=%08x\n",
         MmioRead32 (OpBase + XHCI_USBCMD),
         MmioRead32 (OpBase + XHCI_USBSTS));
  return EFI_SUCCESS;
}

STATIC
VOID
ProbePorts (
  IN UINT32  Caplength,
  IN UINT32  Hcsparams1
  )
{
  UINT32  OpBase;
  UINT32  Ports;
  UINT32  Port;
  UINT32  Sc;
  UINTN   Poll;

  OpBase = (UINT32)B2L_DWC3_BASE + Caplength;
  Ports  = (Hcsparams1 >> 24) & 0xFF;
  Print (L"B2LHOST: root ports=%u\n", Ports);
  if (Ports == 0 || Ports > 16) {
    return;
  }

  for (Port = 1; Port <= Ports; Port++) {
    Sc = MmioRead32 (OpBase + XHCI_PORTSC +
                     (Port - 1) * XHCI_PORTSC_STRIDE);
    Print (L"B2LHOST: port%u before PORTSC=%08x CCS=%u PP=%u PLS=%u speed=%u\n",
           Port, Sc, (Sc & XHCI_PORTSC_CCS) != 0,
           (Sc & XHCI_PORTSC_PP) != 0,
           (Sc & XHCI_PORTSC_PLS_MASK) >> XHCI_PORTSC_PLS_SHIFT,
           (Sc & XHCI_PORTSC_SPEED_MASK) >> XHCI_PORTSC_SPEED_SHIFT);

    MmioWrite32 (OpBase + XHCI_PORTSC + (Port - 1) * XHCI_PORTSC_STRIDE,
                 Sc | XHCI_PORTSC_PP);
    gBS->Stall (20000);

    for (Poll = 0; Poll < 100; Poll++) {
      Sc = MmioRead32 (OpBase + XHCI_PORTSC +
                       (Port - 1) * XHCI_PORTSC_STRIDE);
      if ((Sc & XHCI_PORTSC_CCS) != 0) {
        break;
      }
      gBS->Stall (10000);
    }

    Print (L"B2LHOST: port%u after  PORTSC=%08x CCS=%u PED=%u PP=%u PLS=%u speed=%u CSC=%u PRC=%u\n",
           Port, Sc, (Sc & XHCI_PORTSC_CCS) != 0,
           (Sc & XHCI_PORTSC_PED) != 0, (Sc & XHCI_PORTSC_PP) != 0,
           (Sc & XHCI_PORTSC_PLS_MASK) >> XHCI_PORTSC_PLS_SHIFT,
           (Sc & XHCI_PORTSC_SPEED_MASK) >> XHCI_PORTSC_SPEED_SHIFT,
           (Sc & XHCI_PORTSC_CSC) != 0, (Sc & XHCI_PORTSC_PRC) != 0);

    if ((Sc & XHCI_PORTSC_CCS) != 0) {
      MmioWrite32 (OpBase + XHCI_PORTSC +
                   (Port - 1) * XHCI_PORTSC_STRIDE,
                   (Sc & ~(XHCI_PORTSC_CSC | XHCI_PORTSC_PRC)) |
                   XHCI_PORTSC_PED | XHCI_PORTSC_PP);
      gBS->Stall (20000);
      Print (L"B2LHOST: port%u reset PORTSC=%08x\n", Port,
             MmioRead32 (OpBase + XHCI_PORTSC +
                         (Port - 1) * XHCI_PORTSC_STRIDE));
    }
  }
}

/**
  Entry point: host-mode probe.  Nothing is written unless the controller
  identifies itself as DWC_usb31, so a wrong base cannot program another
  block by accident.
**/
EFI_STATUS
EFIAPI
B2LUsbHostEntry (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS  Status;
  UINT32      SnpsId;
  UINT32      Caplength;
  UINT32      Hcsparams1;

  Print (L"B2LHOST: Exynos 9820 DWC3 host-mode probe\n");
  SnpsId = Dwc3Read32 (DWC3_GSNPSID);
  Print (L"B2LHOST: GSNPSID=%08x GCTL=%08x GSTS=%08x GUSB2PHYCFG0=%08x\n",
         SnpsId, Dwc3Read32 (DWC3_GCTL), Dwc3Read32 (DWC3_GSTS),
         Dwc3Read32 (DWC3_GUSB2PHYCFG0));
  if ((SnpsId & DWC3_GSNPSID_MASK) != DWC3_IP_DWC31) {
    Print (L"B2LHOST: unexpected core id, refusing to program\n");
    return EFI_UNSUPPORTED;
  }

  Status = EnableHsPhy ();
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = SwitchToHost ();
  if (EFI_ERROR (Status)) {
    Print (L"B2LHOST: core stayed in device/OTG mode\n");
    return Status;
  }

  Status = ResetXhci (&Caplength, &Hcsparams1);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  ProbePorts (Caplength, Hcsparams1);

  Print (L"B2LHOST: done; core left in host mode with xHCI running\n");
  return EFI_SUCCESS;
}
