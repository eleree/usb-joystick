
/********************************** (C) COPYRIGHT *******************************
 * File Name          :CompositeKM.C
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2018/02/28
 * Description        : CH559??USB????,??,?????,????
 *******************************************************************************/

#include "./Public/CH554.H"
#include "./Public/Debug.H"
#include <string.h>
#include <stdio.h>

#define NUM_GAMEPADS 2
#define NUM_BUTTONS 8
#define NUM_AXES 2
#define POLL_INTERVAL 2

//#define Fullspeed
#define THIS_ENDP0_SIZE DEFAULT_ENDP0_SIZE
#define ENDP1_IN_SIZE 8
#define ENDP2_IN_SIZE 4

// UINT8X Ep0Buffer[MIN(64, THIS_ENDP0_SIZE + 2)] _at_ 0x0000;                                                   //??0 OUT&IN???,??????
// UINT8X Ep1Buffer[MIN(64, ENDP1_IN_SIZE + 2)] _at_ MIN(64, THIS_ENDP0_SIZE + 2);                               //??1 IN???,??????
// UINT8X Ep2Buffer[MIN(64, ENDP2_IN_SIZE + 2)] _at_(MIN(64, THIS_ENDP0_SIZE + 2) + MIN(64, ENDP1_IN_SIZE + 2)); //??2 IN???,??????
UINT8X Ep0Buffer[64] _at_(0x0000);
UINT8X Ep4Buffer[64] _at_(0x0040);
UINT8X Ep1Buffer[64] _at_(0x0080);
UINT8X Ep2Buffer[64] _at_(0x00C0);
UINT8X Ep3Buffer[64] _at_(0x0100);
UINT8 SetupReq, SetupLen, Ready, Count, FLAG, UsbConfig, SentFlag;
PUINT8 pDescr;             // USB????
USB_SETUP_REQ SetupReqBuf; //??Setup?
sbit Ep2InKey = P1 ^ 5;
#define UsbSetupBuf ((PUSB_SETUP_REQ)Ep0Buffer)
#define DEBUG 0
#pragma NOAREGS

UINT8C ProductName[] = {
    20, 0x03, // Length = 20 bytes, String Descriptor (0x03)
    '0' + NUM_GAMEPADS, 0,
    'N', 0,
    'E', 0,
    'S', 0,
    '0' + NUM_GAMEPADS, 0,
    'S', 0,
    'N', 0,
    'E', 0,
    'S', 0};

// String Descriptors:
UINT8C DevName1[] = {
    34, 0x03, // Length = 34 bytes, String Descriptor (0x03)
    'N', 0,
    'E', 0,
    'S', 0,
    '/', 0,
    'S', 0,
    'N', 0,
    'E', 0,
    'S', 0,
    ' ', 0,
    'G', 0,
    'a', 0,
    'm', 0,
    'e', 0,
    'p', 0,
    'a', 0,
    'd', 0};

UINT8C ManuName[] = {
    16, 0x03, // Length = 30 bytes, String Descriptor (0x03)
    'R', 0,
    'a', 0,
    'p', 0,
    'h', 0,
    'n', 0,
    'e', 0,
    't', 0};

/*?????*/
UINT8C DevDesc[18] = {
    0x12,            // length (18)
    0x01,            // Descriptor type
    0x10, 0x01,      // USB version (1.1)
    0x00,            // class
    0x00,            // subclass
    0x00,            // protocol
    THIS_ENDP0_SIZE, // Max packet size
    // 0x3d, 0x41,      // VID 0x1D50 (OpenMoko)
    // 0x07, 0x21,      // PID 0x602D (5nes5snes (4x12))
    0x50, 0x1D, // VID 0x1D50 (OpenMoko)
    0x2D, 0x60, // PID 0x602D (5nes5snes (4x12))
    0x00, 0x00, // Device version
    0x00,       // manufacturer string index
    0x00,       // product string index
    0x00,       // S/N string index
    0x01        // # of configurations
};

#if 0
UINT8C CfgDesc[59] =
    {
        0x09, 0x02, 0x3b, 0x00, 0x02, 0x01, 0x00, 0xA0, 0x32, //?????
        0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x01, 0x01, 0x00, //?????,??
        0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x3e, 0x00, // HID????
        0x07, 0x05, 0x81, 0x03, ENDP1_IN_SIZE, 0x00, 0x0a,    //?????
        0x09, 0x04, 0x01, 0x00, 0x01, 0x03, 0x01, 0x02, 0x00, //?????,??
        0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x34, 0x00, // HID????
        0x07, 0x05, 0x82, 0x03, ENDP2_IN_SIZE, 0x00, 0x0a     //?????
};
#endif

#if (NUM_BUTTONS % 8) // Need to add byte alignment
#define BUTTON_BYTE_ALIGNMENT 0x95, (8 - (NUM_BUTTONS % 8)), 0x81, 0x03,
#define REP_DESC_LEN 49
#else
#define BUTTON_BYTE_ALIGNMENT
#define REP_DESC_LEN 45
#endif
#define GAMEPAD_REPORT_DESCRIPTOR(a) 0x05, 0x01, 0x09, 0x04, 0xA1, 0x01,                                                       \
                                     0x09, 0x01, 0xA1, 0x00,                                                                   \
                                     0x85, a,                                                                                  \
                                     0x09, 0x30, 0x09, 0x31, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x02, 0x81, 0x02, \
                                     0x05, 0x09, 0x19, 0x01, 0x29, NUM_BUTTONS, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,            \
                                     0x95, NUM_BUTTONS, 0x81, 0x02, BUTTON_BYTE_ALIGNMENT 0xC0,                                \
                                     0xC0

#define CFG_DESC_LEN (9 + ((9 + 9 + 7) * NUM_GAMEPADS))
#define CFG_INTERFACE_DESCR(a) 0x09, 0x04, a, 0x00, 0x01, 0x03, 0x00, 0x00, 0x01
#define CFG_HID_DESCR 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, REP_DESC_LEN, 0x00
#define CFG_EP_DESCR(a) 0x07, 0x05, a, 0x03, 0x10, 0x00, POLL_INTERVAL

UINT8C CfgDesc[CFG_DESC_LEN] =
    {
        9,    /* sizeof(usbDescriptorConfiguration): length of descriptor in bytes */
        0x02, /* descriptor type */
        CFG_DESC_LEN,
        0,            /* total length of data returned (including inlined descriptors) */
        NUM_GAMEPADS, /* number of interfaces in this configuration */
        1,            /* index of this configuration */
        0,            /* configuration name string index */
        0xA0,         // USBATTR_BUSPOWER + USBATTR_REMOTEWAKE
        100 / 2,      /* max USB current in 2mA units */
                      /* interface descriptor follows inline: */

        CFG_INTERFACE_DESCR(0),
        CFG_HID_DESCR,
        CFG_EP_DESCR(0x81),
#if NUM_GAMEPADS > 1
        CFG_INTERFACE_DESCR(1),
        CFG_HID_DESCR,
        CFG_EP_DESCR(0x82),
#endif
#if NUM_GAMEPADS > 2
        CFG_INTERFACE_DESCR(2),
        CFG_HID_DESCR,
        CFG_EP_DESCR(0x83),
#endif
#if NUM_GAMEPADS > 3
        CFG_INTERFACE_DESCR(3),
        CFG_HID_DESCR,
        CFG_EP_DESCR(0x84)
#endif
};

UINT8C ControllerRepDesc[NUM_GAMEPADS][REP_DESC_LEN] = {
    {GAMEPAD_REPORT_DESCRIPTOR(1)},
#if NUM_GAMEPADS > 1
    {GAMEPAD_REPORT_DESCRIPTOR(2)},
#endif
#if NUM_GAMEPADS > 2
    {GAMEPAD_REPORT_DESCRIPTOR(3)},
#endif
#if NUM_GAMEPADS > 3
    {GAMEPAD_REPORT_DESCRIPTOR(4)}
#endif
};

/*??????*/
// Language Descriptor
UINT8C LangDesc[] = {
    4, 0x03,   // Length = 4 bytes, String Descriptor (0x03)
    0x09, 0x04 // 0x0409 English - United States
};

#define USB_STRINGDESC_COUNT 4
const unsigned char *StringDescs[USB_STRINGDESC_COUNT] = {
    LangDesc,    // 0 (If you want to support string descriptors, you must have this!)
    DevName1,    // 1
    ProductName, // 2
    ManuName     // 3
};

/*HID??????*/
UINT8C KeyRepDesc[62] =
    {
        0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07,
        0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
        0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01,
        0x75, 0x08, 0x81, 0x01, 0x95, 0x03, 0x75, 0x01,
        0x05, 0x08, 0x19, 0x01, 0x29, 0x03, 0x91, 0x02,
        0x95, 0x05, 0x75, 0x01, 0x91, 0x01, 0x95, 0x06,
        0x75, 0x08, 0x26, 0xff, 0x00, 0x05, 0x07, 0x19,
        0x00, 0x29, 0x91, 0x81, 0x00, 0xC0};
UINT8C MouseRepDesc[52] =
    {
        0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01,
        0xA1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
        0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x03,
        0x81, 0x02, 0x75, 0x05, 0x95, 0x01, 0x81, 0x01,
        0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
        0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x03,
        0x81, 0x06, 0xC0, 0xC0};
/*????*/
UINT8 HIDMouse[4] = {0x0, 0x0, 0x0, 0x0};
/*????*/
UINT8 HIDKey[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

#define GAMEPAD_XMIT_DATA_LEN (1 + NUM_AXES + ((NUM_BUTTONS + 7) / 8))

UINT8 HIDCtrl[NUM_GAMEPADS][GAMEPAD_XMIT_DATA_LEN];

/*******************************************************************************
 * Function Name  : CH554SoftReset()
 * Description    : CH554???
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH554SoftReset()
{
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;
    GLOBAL_CFG |= bSW_RESET;
}

/*******************************************************************************
 * Function Name  : CH554USBDevWakeup()
 * Description    : CH554????????,??K??
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH554USBDevWakeup()
{
#ifdef Fullspeed
    UDEV_CTRL |= bUD_LOW_SPEED;
    mDelaymS(2);
    UDEV_CTRL &= ~bUD_LOW_SPEED;
#else
    UDEV_CTRL &= ~bUD_LOW_SPEED;
    mDelaymS(2);
    UDEV_CTRL |= bUD_LOW_SPEED;
#endif
}

/*******************************************************************************
 * Function Name  : USBDeviceInit()
 * Description    : USB??????,??????,??????,????
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USBDeviceInit()
{
    IE_USB = 0;
    USB_CTRL = 0x00;                           // USB device mode
    UEP2_DMA = Ep2Buffer;                      // Set endpoint 2's DMA buffer to Ep2Buffer
    UEP3_DMA = Ep3Buffer;                      // Set endpoint 3's DMA buffer to Ep3Buffer
    UEP2_3_MOD = bUEP3_TX_EN | bUEP2_TX_EN;    // Set both endpoints 2 and 3 as IN endpoints
    UEP2_CTRL = UEP_T_RES_NAK;                 // Tell host we don't have anything (yet)
    UEP3_CTRL = UEP_T_RES_NAK;                 // Tell host we don't have anything (yet)
    UEP0_DMA = Ep0Buffer;                      // Set endpoint 0's DMA buffer to Ep0Buffer
                                               // (WARNING: Ep4Buffer is always set to Ep0Buffer+0x40)
    UEP1_DMA = Ep1Buffer;                      // Set endpoint 1's DMA buffer to Ep3Buffer
    UEP4_1_MOD = bUEP1_TX_EN | bUEP4_TX_EN;    // Set both endpoints 1 and 4 as IN endpoints
    UEP1_CTRL = UEP_T_RES_NAK;                 // Tell host we don't have anything (yet)
    UEP4_CTRL = UEP_T_RES_NAK;                 // Tell host we don't have anything (yet)
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; // Tell host we don't have anything (yet) to transmit, but ready to receive

    USB_DEV_AD = 0x00;
    UDEV_CTRL = bUD_PD_DIS;                                   // Disable DP / DM pull-down resistor
    USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;     // Start USB device and DMA,
                                                              // and automatically respond with NAK before interrupt flag is not cleared during interrupt
    UDEV_CTRL |= bUD_PORT_EN;                                 // Enable USB port
    USB_INT_FG = 0xFF;                                        // Clear all interrupt flags?
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST; // Enable suspend, transfer and bus reset interrupts
    IE_USB = 1;
}

void Enp1IntIn()
{
    memcpy(Ep1Buffer, HIDCtrl[0], GAMEPAD_XMIT_DATA_LEN); // Copy the last generated data to the endpoint
    if (SentFlag & 1)                                     // if ((SentFlag & 0x11) == 0x11)
    {
        UEP1_T_LEN = GAMEPAD_XMIT_DATA_LEN;                      // Let the Host know we have this many bytes to send
        UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // Enable acknowledgements
        SentFlag ^= 1;
    }
}

#if NUM_GAMEPADS > 1
void Enp2IntIn()
{
    memcpy(Ep2Buffer, HIDCtrl[1], GAMEPAD_XMIT_DATA_LEN);
    if (SentFlag & 1) // if ((SentFlag & 0x21) == 0x21)
    {
        UEP2_T_LEN = GAMEPAD_XMIT_DATA_LEN;
        UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK;
        SentFlag ^= 1;
    }
}
#endif

/*******************************************************************************
 * Function Name  : DeviceInterrupt()
 * Description    : CH559USB??????
 *******************************************************************************/
void DeviceInterrupt(void) interrupt INT_NO_USB using 1 // USB??????,??????1
{
    UINT8 len = 0;
    while (UIF_TRANSFER) // USB transfer complete flag?
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 4: // Endpoint 4 interrupt endpoint upload
#if NUM_GAMEPADS > 3
            SentFlag |= 0x81; // Let the code know that no other interrupt endpoints are sending
#endif
            UEP4_T_LEN = 0;                                          // Clear the length (don't send data)
            UEP4_CTRL = UEP4_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // NACK requests
            UEP4_CTRL ^= bUEP_T_TOG;                                 // Toggle DATA0/DATA1 so that the next message we send is known to be fresh
                                                                     // NOTE: Endpoint 4 on CH55x MUST toggle the DATA0/DATA1 -- it doesn't support auto-toggle.
            break;
        case UIS_TOKEN_IN | 3: // Endpoint 3 interrupt endpoint upload
#if NUM_GAMEPADS > 2
            SentFlag |= 0x41; // Let the code know that no other interrupt endpoints are sending
#endif
            UEP3_T_LEN = 0;
            UEP3_CTRL = UEP3_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK;
            UEP3_CTRL ^= bUEP_T_TOG; // Don't need to actually do this on Endpoints 1-3,
                                     // but for consistency, I turned off auto-toggle.
            break;
        case UIS_TOKEN_IN | 2: // Endpoint 2 interrupt endpoint upload
#if NUM_GAMEPADS > 1
            SentFlag |= 0x21; // Let the code know that no other interrupt endpoints are sending
#endif
            UEP2_T_LEN = 0;
            UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK;
            UEP2_CTRL ^= bUEP_T_TOG;
            break;
        case UIS_TOKEN_IN | 1: // Endpoint 1 interrupt endpoint upload
            SentFlag |= 0x11;  // Let the code know that no other interrupt endpoints are sending
            UEP1_T_LEN = 0;
            UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK;
            UEP1_CTRL ^= bUEP_T_TOG;
            break;
        case UIS_TOKEN_SETUP | 0: // Setup transaction
            len = USB_RX_LEN;
            if (len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if (UsbSetupBuf->wLengthH || SetupLen > 0x7F)
                {
                    SetupLen = 0x7F; // Limit the total length to the maximum
                }
                len = 0; // Default is successful with 0 length
                SetupReq = UsbSetupBuf->bRequest;
                if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) // HID class command
                {
                    switch (SetupReq)
                    {
                    case 0x01: // GetReport
                        break;
                    case 0x02: // GetIdle
                        break;
                    case 0x03: // GetProtocol
                        break;
                    case 0x09: // SetReport
                        break;
                    case 0x0A: // SetIdle
                        break;
                    case 0x0B: // SetProtocol
                        break;
                    default:
                        len = 0xFF; // command not supported
                        break;
                    }
                }
                else
                {                     // Standard request
                    switch (SetupReq) // Request code
                    {
                    case USB_GET_DESCRIPTOR:
                        switch (UsbSetupBuf->wValueH)
                        {
                        case 1:               // Device Descriptor
                            pDescr = DevDesc; // Premade buffer to be sent
                            len = sizeof(DevDesc);
                            break;
                        case 2:               // Configuration Descriptor
                            pDescr = CfgDesc; // Premade buffer to be sent
                            len = sizeof(CfgDesc);
                            break;
                        case 3:                         // String Descriptor
                            len = UsbSetupBuf->wValueL; // Index
                            if (len < USB_STRINGDESC_COUNT)
                            {
                                pDescr = (UINT8 *)(StringDescs[len]);
                                len = pDescr[0];
                            }
                            else
                            {
                                len = 0xFF; // Not supported
                            }
                            break;
                        case 0x22: // Report Descriptor
#if 0
                            pDescr = ControllerRepDesc[0];
                            len = REP_DESC_LEN * NUM_GAMEPADS;
                            Ready = 1;
#else
                            if (UsbSetupBuf->wIndexL < NUM_GAMEPADS)
                            {
                                pDescr = ControllerRepDesc[UsbSetupBuf->wIndexL]; // Premade buffer to be sent
                                len = REP_DESC_LEN;
                                if (UsbSetupBuf->wIndexL == (NUM_GAMEPADS - 1))
                                {
																		//printf("READY\n");
                                    Ready = 1;
                                }
                            }
                            else
                            {
                                len = 0xff; // Normally wouldn't execute: Host should only ask for reports for 4 endpoints
                            }
#endif
                            break;
                        default:
                            len = 0xff; // Unsupported command or error
                            break;
                        }
                        if (SetupLen > len)
                        {
                            SetupLen = len; // Limit the total length
                        }
                        len = SetupLen >= 8 ? 8 : SetupLen; // Set current transmission length
                        memcpy(Ep0Buffer, pDescr, len);     // Set the data to upload
                        SetupLen -= len;                    // SetupLen = remaining bytes to send
                        pDescr += len;                      // Increment to get ready to send the next chunk (if exists)
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL; // Set temporary USB device address
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if (SetupLen >= 1)
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) // Endpoint
                        {
                            switch (UsbSetupBuf->wIndexL)
                            {
                            case 0x84:
                                UEP4_CTRL = UEP4_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                // SentFlag |= (0x80);
                                break;
                            case 0x83:
                                UEP3_CTRL = UEP3_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                // SentFlag |= (0x40);
                                break;
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                // SentFlag |= (0x20);
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                // SentFlag |= (0x10);
                                break;
                            case 0x01:
                                UEP1_CTRL = UEP1_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF; // Unsupported endpoint
                                break;
                            }
                        }
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
                        {
                            break;
                        }
                        else
                        {
                            len = 0xFF; // Unsupported
                        }
                        break;
                    case USB_SET_FEATURE:
                        if ((UsbSetupBuf->bRequestType & 0x1F) == 0x00) // Setting up the device
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01)
                            {
                                if (CfgDesc[7] & 0x20)
                                {
                                    // Setting up wake up enable
                                }
                                else
                                {
                                    len = 0xFF; // Operation failed
                                }
                            }
                            else
                            {
                                len = 0xFF; // Operation failed
                            }
                        }
                        else if ((UsbSetupBuf->bRequestType & 0x1F) == 0x02) // Set endpoint
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00)
                            {
                                switch (((UINT16)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL)
                                {
                                case 0x84:
                                    UEP4_CTRL = UEP4_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; // EP4 IN STALL
                                                                                             // SentFlag &= ~(0x80);
                                    break;
                                case 0x83:
                                    UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; // EP3 IN STALL
                                                                                             // SentFlag &= ~(0x40);
                                    break;
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; // EP2 IN STALL
                                                                                             // SentFlag &= ~(0x20);
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; // EP1 IN STALL
                                                                                             // SentFlag &= ~(0x10);
                                    break;
                                default:
                                    len = 0xFF; // Operation failed
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF; // Operation failed
                            }
                        }
                        else
                        {
                            len = 0xFF; // Operation failed
                        }
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if (SetupLen >= 2)
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff; // Operation failed
                        break;
                    }
                }
            }
            else
            {
                len = 0xff; // Packet length error
            }
            if (len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; // STALL
            }
            else if (len) // Uploading data or status phase returns 0 length packets
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // Default is DATA1 + ACK
            }
            else
            {
                // Because it has not yet reached the status phase, it is preset to upload 0-length packets in advance
                // to prevent the host from entering the status phase too early.
                UEP0_T_LEN = 0;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // Default is DATA1 + ACK
            }
            break;
        case UIS_TOKEN_IN | 0: // Endpoint0 IN
            switch (SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= 8 ? 8 : SetupLen; // Current transmission length
                memcpy(Ep0Buffer, pDescr, len);     // Data to send to host
                SetupLen -= len;                    // SetupLen holds the remaining byte count
                pDescr += len;                      // Increment pDescr to get ready for the next transaction
                UEP0_T_LEN = len;                   // Tell the host that "len" bytes are available
                UEP0_CTRL ^= bUEP_T_TOG;            // Flip the sync flag
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0; // Interruption of status phase completion or forced upload of
                                // 0-length data packets to end control transmission
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0: // endpoint0 OUT
            len = USB_RX_LEN;
            UEP0_CTRL ^= bUEP_R_TOG; // Flip the sync flag
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0; // Write 0 to clear interrupt
    }
    if (UIF_BUS_RST) // Device mode USB bus reset interrupt
    {
				Ready = 0;
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = UEP_T_RES_NAK;
        UEP2_CTRL = UEP_T_RES_NAK;
        UEP3_CTRL = UEP_T_RES_NAK;
        UEP4_CTRL = UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0; // Write 0 to clear interrupt
    }
    if (UIF_SUSPEND) // USB bus suspend / wake up completed
    {
        UIF_SUSPEND = 0;
				Ready = 0;
				//printf("NREADY\n");
				SentFlag = 1;
				UEP1_T_LEN = 0;
				UEP2_T_LEN = 0;
				UEP3_T_LEN = 0;
				UEP4_T_LEN = 0;
        if (USB_MIS_ST & bUMS_SUSPEND) // ?
        {
        }
    }
    else
    {                      // Unexpected interrupt, should never happen
        USB_INT_FG = 0xFF; // Clear all the interrupts and just blow out of the ISR.
    }
}

void HIDValueHandle(int gamepadID)
{
    // Copy the freshest data to the endpoints and alert the Host.
    switch (gamepadID)
    {
    case 0:
        Enp1IntIn();
        break;
#if NUM_GAMEPADS > 1
    case 1:
        Enp2IntIn();
        break;
#endif
#if NUM_GAMEPADS > 2
    case 2:
        Enp3IntIn();
        break;
#endif
#if NUM_GAMEPADS > 3
    case 3:
        Enp4IntIn();
        break;
#endif
    }
}

sbit JOYSTICK_LATCH = P1^6; 
sbit JOYSTICK_CLK = P1^7;
sbit JOYSTICK_DATA0 = P1^5;
sbit JOYSTICK_DATA1 = P1^4;

void P1_IN(UINT8 pins)
{
	P1_MOD_OC &= ~pins;
	P1_DIR_PU &= ~pins;
}

void P1_OUT(UINT8 pins)
{
	P1_MOD_OC &= ~pins;
	P1_DIR_PU |= pins;
}

void JoystickInit(void)
{
	P1_IN(4);
	P1_IN(5);
}

void JoystickRead(void)
{
	UINT8 i = 0;
	UINT16 key1 = 0;
	UINT16 key2 = 0;
	UINT8 pad1 = 0;
	UINT8 pad2 = 0;
	JOYSTICK_LATCH = 1;
	mDelayuS(12);
	JOYSTICK_LATCH = 0;
	mDelayuS(12);
	if(JOYSTICK_DATA0 == 1){
		key1 |= 1;
	}
	if(JOYSTICK_DATA1 == 1){
		key2 |= 1;
	}
	JOYSTICK_CLK  = 0;
	for (i = 0; i < 8; i++)
	{
		key1 <<= 1;
    key2 <<= 1;
		JOYSTICK_CLK  = 1;
		mDelayuS(6);
		if(JOYSTICK_DATA0 == 1){
			key1 |= 1;
		}
		if(JOYSTICK_DATA1 == 1){
			key2 |= 1;
		}
		JOYSTICK_CLK  = 0;
		mDelayuS(6);
	}
	JOYSTICK_CLK  = 0;
	key1 >>= 1;
  key2 >>= 1;
	pad1 = key1;
	pad2 = key2;
	//printf("%bu,%bu\r\n",~pad1, ~pad2);
	HIDCtrl[0][1] = 128;
  HIDCtrl[0][2] = 128;        
  HIDCtrl[1][1] = 128;
  HIDCtrl[1][2] = 128;
	HIDCtrl[0][3] = ~pad1;
	HIDCtrl[1][3] = ~pad2;
}
void JoystickReadTest(void)
{
    UINT8 i;
    static UINT8 k = 0;
    k++;
    i = _getkey();
    printf("%c", (UINT8)i);
    printf("%c", (UINT8)k);
    switch (i)
    {
    case 'L':
        HIDCtrl[0][1] = k;
        HIDCtrl[0][2] = k;
        HIDCtrl[0][3] = 'L';
        HIDCtrl[1][1] = k;
        HIDCtrl[1][2] = k;
        HIDCtrl[1][3] = 'L';
        break;
    case 'R':
        HIDCtrl[0][1] = 0;
        HIDCtrl[0][2] = 0;
        HIDCtrl[0][3] = 'R';
        HIDCtrl[1][1] = 0;
        HIDCtrl[1][2] = 0;
        HIDCtrl[1][3] = 'R';
        break;
    default:
        HIDCtrl[0][1] = i;
        HIDCtrl[0][2] = i;
        HIDCtrl[0][3] = i;
        HIDCtrl[1][1] = i;
        HIDCtrl[1][2] = i;
        HIDCtrl[1][3] = i;
        break;
    }
}

void usbMain(void)
{
    int i;
		UINT8 k = 0;
    SentFlag = 1;
    CfgFsys();    // CH559??????
    mDelaymS(5);  //????????????,??
    mInitSTDIO(); //??0???
#ifdef DE_PRINTF
    printf("start ...\n");
#endif

#ifdef DE_PRINTF //??????ID?
    printf("ID0 = %02x %02x \n", (UINT16) * (PUINT8C)(0x3FFA), (UINT16) * (PUINT8C)(0x3FFB));
    printf("ID1 = %02x %02x \n", (UINT16) * (PUINT8C)(0x3FFC), (UINT16) * (PUINT8C)(0x3FFD));
    printf("ID2 = %02x %02x \n", (UINT16) * (PUINT8C)(0x3FFE), (UINT16) * (PUINT8C)(0x3FFF));
#endif

    for (i = 0; i < NUM_GAMEPADS; i++)
    {
        HIDCtrl[i][0] = i + 1;
    }
		JoystickInit();
    USBDeviceInit(); // USB???????
    EA = 1;          //???????
    UEP1_T_LEN = 0;
    UEP2_T_LEN = 0;
    UEP3_T_LEN = 0;
    UEP4_T_LEN = 0;
    FLAG = 0;
    Ready = 0;
    while (1)
    {
			JoystickRead();

        for (i = 0; i < NUM_GAMEPADS; i++)
        {
            if (Ready)
            {
                HIDValueHandle(i);
            }
        }
    }
}
