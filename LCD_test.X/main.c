#include <xc.h>

// PRAGMA
#ifdef _18F8722
// CONFIG1H
#pragma config OSC = INTIO7     
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Two-Speed Start-up disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3L
#pragma config MODE = MC        // Processor Data Memory Mode Select bits (Microcontroller mode)
#pragma config ADDRBW = ADDR20BIT// Address Bus Width Select bits (20-bit Address Bus)
#pragma config DATABW = DATA16BIT// Data Bus Width Select bit (16-bit External Bus mode)
#pragma config WAIT = OFF       // External Bus Data Wait Enable bit (Wait selections are unavailable for table reads and table writes)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (ECCP2 input/output is multiplexed with RC1)
#pragma config ECCPMX = PORTE   // ECCP MUX bit (ECCP1/3 (P1B/P1C/P3B/P3C) are multiplexed onto RE6, RE5, RE4 and RE3 respectively)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RG5 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size Select bits (1K word (2 Kbytes) Boot Block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit Block 3 (Block 3 (00C000-00FFFFh) not code-protected)
#pragma config CP4 = OFF        // Code Protection bit Block 4 (Block 4 (010000-013FFFh) not code-protected)
#pragma config CP5 = OFF        // Code Protection bit Block 5 (Block 5 (014000-017FFFh) not code-protected)
#pragma config CP6 = OFF        // Code Protection bit Block 6 (Block 6 (01BFFF-018000h) not code-protected)
#pragma config CP7 = OFF        // Code Protection bit Block 7 (Block 7 (01C000-01FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit Block 3 (Block 3 (00C000-00FFFFh) not write-protected)
#pragma config WRT4 = OFF       // Write Protection bit Block 4 (Block 4 (010000-013FFFh) not write-protected)
#pragma config WRT5 = OFF       // Write Protection bit Block 5 (Block 5 (014000-017FFFh) not write-protected)
#pragma config WRT6 = OFF       // Write Protection bit Block 6 (Block 6 (01BFFF-018000h) not write-protected)
#pragma config WRT7 = OFF       // Write Protection bit Block 7 (Block 7 (01C000-01FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR4 = OFF      // Table Read Protection bit Block 4 (Block 4 (010000-013FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR5 = OFF      // Table Read Protection bit Block 5 (Block 5 (014000-017FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR6 = OFF      // Table Read Protection bit Block 6 (Block 6 (018000-01BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR7 = OFF      // Table Read Protection bit Block 7 (Block 7 (01C000-01FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not protected from table reads executed in other blocks)

#endif

#define S1 PORTBbits.RB0 // RBO with port B
#define S2 PORTAbits.RA5 // RA5 with port A

// set Leds outputs
#define LED0 PORTDbits.RD0
#define LED1 PORTDbits.RD1
#define LED2 PORTDbits.RD2
#define LED3 PORTDbits.RD3
#define LED4 PORTDbits.RD4
#define LED5 PORTDbits.RD5
#define LED6 PORTDbits.RD6
#define LED7 PORTDbits.RD7


#define LCD_delay           5                                // ~5mS
#define LCD_Startup         15                               // ~15mS

// Command set for LCD display controller
#define LCD_CLEAR           0x01
#define LCD_HOME            0x02
#define LCD_CURSOR_BACK     0x10
#define LCD_CURSOR_FWD      0x14
#define LCD_PAN_LEFT        0x18
#define LCD_PAN_RIGHT       0x1C
#define LCD_CURSOR_OFF      0x0C
#define LCD_CURSOR_ON       0x0E
#define LCD_CURSOR_BLINK    0x0F
#define LCD_CURSOR_LINE1    0x80
#define LCD_CURSOR_LINE2    0xC0

// Macros recommended for new labs
#define mOPEN_LCD           LCDInit()
#define mLCD_PAN_RIGHT      LCDPutInst(LCD_PAN_RIGHT)
#define mLCD_PAN_LEFT       LCDPutInst(LCD_PAN_LEFT)
#define mLCD_CURSOR_BLINK   LCDPutInst(LCD_CURSOR_BLINK)
#define mLCD_HOME           LCDPutInst(LCD_HOME)
#define mLCD_CLEAR          LCDPutInst(LCD_CLEAR)

#define mCURSOR_ON          LCDPutInst(LCD_CURSOR_ON)
#define mCURSOR_OFF         LCDPutInst(LCD_CURSOR_OFF)
#define mCURSOR_FWD         LCDPutInst(LCD_CURSOR_FWD)
#define mCURSOR_BACK        LCDPutInst(LCD_CURSOR_BACK)
#define mCURSOR_LINE1       LCDPutInst(LCD_CURSOR_LINE1)
#define mCURSOR_LINE2       LCDPutInst(LCD_CURSOR_LINE2)

// Macro definitions specific to XC8 (not recommended for new code)
#define text_display        LCDPutStr                       // legacy support
#define LCDLine1()          LCDPutInst(LCD_HOME)            // legacy support
#define LCDLine2()          LCDPutInst(LCD_CURSOR_LINE2)    // legacy support
#define shift_cursor()      LCDPutInst(LCD_CURSOR_FWD)      // legacy support
#define cursor_on()         LCDPutInst(LCD_CURSOR_ON)       // legacy support
#define DisplayClr()        LCDPutInst(LCD_CLEAR)           // Legacy support

// Function prototypes

/**
 * @FUNCTION     LCDInit()
 * @INPUTS       None
 * @OUTPUTS      None
 * @DESCRIPTION  Initialize the LCD
 */
#define _XTAL_FREQ  10000000

void LCDInit(void);

/**
 * @FUNCTION     InitBBSPI()
 * @INPUTS       None
 * @OUTPUTS      None
 * @DESCRIPTION  Initialize I/O Ports for Bit Bang SPI
 */

void InitBBSPI(void);

/**
 * @FUNCTION     SendByteBBSPI
 * @INPUTS       output = byte to be transmitted
 * @OUTPUTS      None
 * @DESCRIPTION  Outputs a byte through the set DOUT pin and receives dummy data through the DIN pin
 */

void SendByteBBSPI(unsigned char output);

/**
 * @FUNCTION     Port_BBSPIInit()
 * @INPUTS       port_dir = address of IODIRx
 * @OUTPUTS      None
 * @DESCRIPTION  Initialize MCP923S17 Portx as output
 */

void Port_BBSPIInit(unsigned char port_dir);

/**
 * @FUNCTION     WritePort_BBSPI()
 * @INPUTS       port_add = address of GPIOx;
 *               a = instruction register or data register
 * @OUTPUTS      None
 * @DESCRIPTION  Writes to PORTx and selects between data and instruction register
 */

void WritePort_BBSPI(unsigned char port_add, unsigned char a);

/**
 * @FUNCTION     LCDPutChar()
 * @INPUTS       ch = character to send
 * @OUTPUTS      None
 * @DESCRIPTION  Writes character to LCD at current cursor position
 */

void LCDPutChar(unsigned char);

/**
 * @FUNCTION     LCDPutInst()
 * @INPUTS       ch = character to send
 * @OUTPUTS      None
 * @DESCRIPTION  Writes character to LCD instruction register
 */

void LCDPutInst(unsigned char);

/**
 * @FUNCTION     LCDPutStr()
 * @INPUTS       *ptr = Pointer to string to send
 * @OUTPUTS      None
 * @DESCRIPTION  Writes string to LCD at current cursor position
 */

void LCDPutStr(const char *);

void LCDMoveCursor(unsigned char line, unsigned char pos);

void LCDPrint(unsigned char line, unsigned char pos, const char *ptr);

// Common legacy macros
#define LCDHome()           LCDPutInst(LCD_HOME)            // legacy support
#define LCDClr()            LCDPutInst(LCD_CLEAR)           // legacy support
#define LCDPutChar          LCDPutChar                      // legacy support

#define PORTA_DIR               0x00        // I/O Direction register of PORTA
#define PORTB_DIR               0x01        // I/O Direction register of PORTB
#define PORTA_ADD               0x12        // address of General Purpose I/O of PORTA
#define PORTB_ADD               0x13        // address of General Purpose I/O of PORTB

// Display controller setup commands
#define FUNCTION_SET        0x3C                         // 8 bit interface, 2 lines, 5x8 font
#define ENTRY_MODE          0x06                         // increment mode
#define DISPLAY_SETUP       0x0C                         // display on, cursor off, blink offd

//----------------------------------------------------------------------
// Definitions specific to the PICDEM PIC18 Explorer
//----------------------------------------------------------------------

// selecting between instruction register or data register
#define instr        0x00
#define data         0x80

// to send instruction or data to LCD
#define send_instr   0x40
#define send_data    0xC0

// These #defines create the pin connections to the LCD in case they are changed on a future demo board
#define LCD_CS          LATAbits.LATA2          //LCD chip select
#define LCD_CS_TRIS     TRISAbits.TRISA2        //LCD chip select
#define LCD_RST         LATFbits.LATF6          //LCD hardware reset
#define LCD_RST_TRIS    TRISFbits.TRISF6        //LCD hardware reset
#define LCD_DOUT        LATCbits.LATC5          //Serial Data Output pin
#define LCD_DOUT_TRIS   TRISCbits.TRISC5        //Serial Data Output pin
#define LCD_DIN         LATCbits.LATC4          //Serial Data Input pin
#define LCD_DIN_TRIS    TRISCbits.TRISC4        //Serial Data Input pin
#define LCD_SCLK        LATCbits.LATC3          //Serial Clock pin
#define LCD_SCLK_TRIS   TRISCbits.TRISC3        //Serial Clock pin

/*******************************************************************
 * FUNCTION:     LCDInit ()
 * INPUTS:       None
 * OUTPUTS:      None
 * DESCRIPTION:  Initialize the LCD
 ********************************************************************/

void LCDInit(void) {
    InitBBSPI(); // initialize SPI
    LCD_RST_TRIS = 0; // set RF6 as output to control RESET pin
    LCD_RST = 0; // reset MCP23S17
    __delay_ms(LCD_delay);
    LCD_RST = 1;
    Port_BBSPIInit(PORTA_DIR); // initialize MCP23S17 PORTA
    Port_BBSPIInit(PORTB_DIR); // initialize MCP23S17 PORTB
    WritePort_BBSPI(PORTA_ADD, 0);
    __delay_ms(LCD_Startup); // required by display controller to allow power to stabilize
    LCDPutInst(0x32); // required by display initialization
    LCDPutInst(FUNCTION_SET); // set interface size, # of lines and font
    LCDPutInst(DISPLAY_SETUP); // turn on display and sets up cursor
    mLCD_CLEAR; // clear the display
    LCDPutInst(ENTRY_MODE); // set cursor movement direction
}

/*******************************************************************
 * FUNCTION:     InitBBSPI ()
 * INPUTS:       None
 * OUTPUTS:      None
 * DESCRIPTION:  Initialize I/O Ports for Bit Bang SPI
 ********************************************************************/

void InitBBSPI(void) {
    LCD_CS_TRIS = 0; // make the CS pin an output
    LCD_DIN_TRIS = 1; // make the DIN pin an input
    LCD_DOUT_TRIS = 0; // make the DOUT pin an output
    LCD_SCLK_TRIS = 0; // make the SCLK pin an output

    LCD_CS = 1; // raise the CS pin
    LCD_DIN = 1; // set the DIN pin
    LCD_DOUT = 0; // clear the DOUT pin
    LCD_SCLK = 0; // clear the SCLK pin
}

/*******************************************************************
 * FUNCTION:     SendByteBBSPI ()
 * INPUTS:       output = byte to be transmitted
 * OUTPUTS:      None
 * DESCRIPTION:  Outputs a byte through the set DOUT pin
 *               and receives dummy data through the DIN pin
 ********************************************************************/

void SendByteBBSPI(unsigned char output) {
    unsigned char bitcount;
    unsigned char input = output;

    for (bitcount = 0; bitcount < 8; bitcount++) {
        // transmit data MSB
        if (output & 0x80) // condition if transmit byte MSB is 1
            LCD_DOUT = 1; // make SDO pin output high
        else // condition if byte MSB is 0
            LCD_DOUT = 0; // make SDO pin output low
        // receive dummy data
        if (LCD_DIN) // condition if receive byte MSB is 1
            input = (input << 1) | 0x1; // shift input 1 bit to the left and move carry bit to LSB
        else // condition if receive bit is 0
            input = input << 1; // shift input 1 bit to the left
        LCD_SCLK = 1; // set the SCLK pin
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP(); // produces ~50% duty cycle clock
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        LCD_SCLK = 0; // clear the SCLK pin
        output <<= 1; // shift output 1 bit to the left
    }
}

/*******************************************************************
 * FUNCTION:     Port_BBSPIInit ()
 * INPUTS:       port_dir = address of IODIRx
 * OUTPUTS:      None
 * DESCRIPTION:  Initialize MCP923S17 Portx as output
 ********************************************************************/

void Port_BBSPIInit(unsigned char port_dir) {
    // IOCON.BANK POR/RST Value = 0
    // Sequence = Device Opcode + Register Address + 1 Data Byte

    LCD_CS = 0; // lower CS to initialize SPI write operation on MCP923S17
    SendByteBBSPI(0x40); // transmit device opcode (slave address and write enable)
    SendByteBBSPI(port_dir); // point to address of IODIRx(I/O Direction register of PORTx)
    SendByteBBSPI(0x00); // set all PORTx pins as output
    LCD_CS = 1; // end sequence
}

/*******************************************************************
 * FUNCTION:     WritePort_BBSPI ()
 * INPUTS:       port_add = address of GPIOx
 *               a = instruction register or data register
 * OUTPUTS:      None
 * DESCRIPTION:  Writes to PORTx and selects between data and instruction
 *               register
 ********************************************************************/

void WritePort_BBSPI(unsigned char port_add, unsigned char a) {
    LCD_CS = 0; // lower CS to initialize SPI write operation on MCP923S17
    SendByteBBSPI(0x40); // transmit device opcode (slave address and write enable)
    SendByteBBSPI(port_add); // point to address of GPIOx (General Purpose I/O of PORTx)
    SendByteBBSPI(a); // write value to GPIOx
    LCD_CS = 1; // end sequence
}

/*******************************************************************
 * FUNCTION:     LCDPutChar ()
 * INPUTS:       ch = character to send
 * OUTPUTS:      None
 * DESCRIPTION:  Writes character to LCD at current cursor position
 ********************************************************************/

void LCDPutChar(unsigned char ch) {
    __delay_ms(LCD_delay);
    WritePort_BBSPI(PORTA_ADD, data); // prepare to send data to LCD
    __delay_ms(1);
    WritePort_BBSPI(PORTB_ADD, ch); // write the character to be displayed
    __delay_ms(1);
    WritePort_BBSPI(PORTA_ADD, send_data); // send data to LCD
    __delay_ms(1);
    WritePort_BBSPI(PORTA_ADD, 0x00); // stop sending data to LCD
}

/*******************************************************************
 * FUNCTION:     LCDPutInst ()
 * INPUTS:       ch = character to send
 * OUTPUTS:      None
 * DESCRIPTION:  Writes character to LCD instruction register
 *******************************************************************/

void LCDPutInst(unsigned char ch) {
    __delay_ms(LCD_delay);
    WritePort_BBSPI(PORTA_ADD, instr); // prepare to send instruction to LCD
    __delay_ms(1);
    WritePort_BBSPI(PORTB_ADD, ch); // write the instruction to be sent to LCD
    __delay_ms(1);
    WritePort_BBSPI(PORTA_ADD, send_instr); // send instruction to LCD
    __delay_ms(1);
    WritePort_BBSPI(PORTA_ADD, 0x00); // stop sending instruction to LCD
}

/*******************************************************************
 * FUNCTION:     LCDPutStr ()
 * INPUTS:       *ptr = Pointer to string to send
 * OUTPUTS:      None
 * DESCRIPTION:  Writes string to LCD at current cursor position
 *******************************************************************/

void LCDPutStr(const char *ptr) {
    while (*ptr) LCDPutChar(*(ptr++));
}

void LCDMoveCursor(unsigned char line, unsigned char pos) {
    unsigned char position;
    if (line == 0) {
        position = (128 + pos); //LCD start address (HEX) at line0 : 0x80 
    } else
        position = (192 + pos);// //LCD start address (HEX) at line0 : 0xC0

    LCDPutInst(position);
}

void LCDPrint(unsigned char line, unsigned char pos, const char *ptr) {
    if (line < 0 || line > 2 || pos < 0 || pos > 15)
        return;
    LCDMoveCursor(line, pos);
    LCDPutStr(ptr);
}

void OSCILLATOR_Initialize(void) {
    //OSCCON = 0b11100111; // 4 MHz
    OSCCON = 0b11110111; //8MHZ
    OSCTUNE = 0b00001111; // ????
}

void setup_BUTTON_LEDS() {
    TRISBbits.TRISB0 = 1; // Configure RB0 as input
    TRISAbits.TRISA5 = 1; // Configure SA5 as input ; SA5 belong to PORTA

    ADCON1bits.PCFG0 = 1; // Set RA<3:0> as inputs (follow sheet)
    ADCON1bits.PCFG1 = 1;
    ADCON1bits.PCFG2 = 1;
    ADCON1bits.PCFG3 = 1;

    TRISDbits.TRISD0 = 0; // Configure PORTD as outputs
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD3 = 0;
    TRISDbits.TRISD4 = 0;
    TRISDbits.TRISD5 = 0;
    TRISDbits.TRISD6 = 0;
    TRISDbits.TRISD7 = 0;

    LATD = 0b00000000; // output set value

    MEMCONbits.EBDIS = 1; // Enable PORTD I/O functions (where is the lights are)

}

void main(void) {
    OSCILLATOR_Initialize();
    setup_BUTTON_LEDS();
    mOPEN_LCD;


    while (1) {
        if (S1 == 0) {
            LCDClr();
            LCDPrint(0, 0, "Hello");
            mLCD_CURSOR_BLINK;
            LCDMoveCursor(0, 1);

        } else if (S2 == 0) {
            LCDClr();
            LCDPrint(0, 0, "salut");
            mCURSOR_FWD;
        }

    }

    return;
}