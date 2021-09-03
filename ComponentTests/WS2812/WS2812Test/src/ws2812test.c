/*
 * test a string of ws2812 leds on pin D2
 * output signal timing needs inline assembly, and disabled interrupts during signal generation
 * tried different approaches : 
 * 
 * showPixels : 
 * code similar to Adafruit_NeoPixel AVR implementation
 * output signal timing is not completely according to spec, but it seems to work
 * advantage is that code works for any port & pin
 * 
 * showPixels2 : 
 * -> improvement over showPixels with timing according to spec
 * -> still 8-bit numBytes
 * 
 * showPixels3 : 
 * -> 16-bit numBytes
 * -> bit0 timing is a bit stretched, but still working
 * -> used as base for neopixel stm8 library
 * -> more flexible than showPixelsBCCM, because there port & pin have to be hardcoded (see below)
 * 
 * showPixelsBCCM :
 * based on approach in fastled-STM8, using SLA (shift left) and the special STM8 instruction BCCM
 * -> correct signal timing
 * -> but bccm, bset, bres instructions need immediate values for port & pin (fixed port & pin)
 * -> code here uses fixed PD2
 * -> fastled-STM8 solves this with C++ templates, but compiles for IAR, not possible with sdcc
 * sdcc solution?
 * -> can't use lookup, because we need execution from flash (from sram = slow because of 8-bit fetching vs 32-bit)
 * -> fixed pin at compile time? not implemented here
 * -> to check if #defines can be used within inline assembly (eg. PORT_ODR, PIN)
 */
#include <Arduino.h>

#define MAX_NUM_PIXELS  16
volatile uint8_t *port;
uint8_t pinMask;
//uint16_t numBytes, numLeds;
uint8_t numBytes, numLeds; // TEMP, 16-bit nog op te lossen in asm
uint8_t thePixels[MAX_NUM_PIXELS][3];
uint8_t *pixels = (uint8_t*) thePixels; // adafruit gebruikt malloc
uint16_t numBytes16 = 3*15;

// STM8 assembler kan enkel met globals interfacen
volatile uint8_t
  hi,             // PORT w/output bit set high
  lo,             // PORT w/output bit set low
  next;           // next output bits

static void setPin(uint16_t p) {
  pinMode(p,OUTPUT);
  port    = portOutputRegister(digitalPinToPort(p));
  pinMask = digitalPinToBitMask(p);
}

// for test
static void initPixels() {
  // WS2812 is GRB order
  for (uint8_t i=0;i<MAX_NUM_PIXELS;i++) {
    thePixels[i][0] = 10; // green
    thePixels[i][1] = 0; // red
    thePixels[i][2] = 0; // blue
  }
  numLeds = 15;
  numBytes = 3*numLeds;
} // initPixels

void showPixels () {
  hi   = *port |  pinMask;
  lo   = *port & ~pinMask;
  next = lo;

  disableInterrupts();
  __asm
      ; mov _port, _next  ; // -> da werkt nie omdat port een ptr is !!

      ldw y, _pixels      ;  // ptr naar pixels in Y index register
      ldw x, _port        ; // ptr naar IOmem port, om ld [_port], a te vermijden, is allicht te traag

      push _numBytes      ; // TEMP, werkt enkel met numBytes uint8_t !! -> (0x03,sp)
      push #8             ; // bit counter -> (0x02,sp)
      push #0x80          ; // bitfield om te bit compare met (Y) -> (0x01,sp)
      ;nop                ; // een test of dit de fetching verbetert

    0020$:                ; // head20 (avr ref)
      ld a, _hi
      ld (x), a           ; // PORT=hi
      ld a, (0x01,sp)     ; // bitfield in register A
      bcp a, (y)          ; // bit test op bit 8, als de bit gezet is in (y),
      jreq 0021$          ; // bcp is niet nul, dus bit is gezet, dan moeten we hier port hi houden voor een T1H
      mov _next, _hi

    0021$:                ; // bcp is 0, dus bit niet gezet
      ld a, _next
      ld (x), a           ; // PORT=next
      mov _next, _lo
      rcf                 ; // reset carry bit, zodat we enkel 0 binnentrekken met de rshift
      rrc (0x01,sp)       ; //bitfield>>1, rechtstreeks via sp indexing
      ld a, _lo           ; // voor de jreq gezet ivm timing
      ld (x), a           ; // PORT = lo
      dec (0x02,sp)       ; // bit--
      jreq 0022$          ; // if (bit==0) from dec above, 0022$ = nextbyte20
      ;ld a, #8           ; // om de enkele bit timing net iets langer te maken (was 1208/1168ns) -> hoort bij 0022$
      jra 0020$           ; // relative jump always -> head (volgende bit)

    0022$:                ; //nextbyte20
      ld a, #8
      ld (0x02,sp), a     ; // bit = 8
      incw y              ; // ptr++
      ld a, #0x80
      ld (0x01,sp),a      ; // reset bitfield
      dec (0x03,sp)       ; // numBytes--
      jrne 0020$          ; // if (i!=0) -> next byte

      addw sp, #3         ; // 3 bytes terug van de stack halen, hoeft niet met afzonderlijke pop instructions

  __endasm;

  enableInterrupts();

// avr reference
/* 
  asm volatile(
   "head20:"                   "\n\t" // Clk  Pseudocode    (T =  0)
    "st   %a[port],  %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
    "sbrc %[byte],  7"         "\n\t" // 1-2  if(b & 128)
     "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  4)
    "dec  %[bit]"              "\n\t" // 1    bit--         (T =  5)
    "st   %a[port],  %[next]"  "\n\t" // 2    PORT = next   (T =  7)
    "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T =  8)
    "breq nextbyte20"          "\n\t" // 1-2  if(bit == 0) (from dec above)
    "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 10)
    "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 12)
    "nop"                      "\n\t" // 1    nop           (T = 13)
    "st   %a[port],  %[lo]"    "\n\t" // 2    PORT = lo     (T = 15)
    "nop"                      "\n\t" // 1    nop           (T = 16)
    "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 18)
    "rjmp head20"              "\n\t" // 2    -> head20 (next bit out)
   "nextbyte20:"               "\n\t" //                    (T = 10)
    "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 11)
    "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 13)
    "st   %a[port], %[lo]"     "\n\t" // 2    PORT = lo     (T = 15)
    "nop"                      "\n\t" // 1    nop           (T = 16)
    "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 18)
     "brne head20"             "\n"   // 2    if(i != 0) -> (next byte)
    : [port]  "+e" (port),
      [byte]  "+r" (b),
      [bit]   "+r" (bit),
      [next]  "+r" (next),
      [count] "+w" (i)
    : [ptr]    "e" (ptr),
      [hi]     "r" (hi),
      [lo]     "r" (lo));
*/
} // showPixels

// bit timing within spec
/* 
 * timing without nops : (measured with 42ns resolution = 24MHz)
 * 0 : 292ns high, 583ns low, bit time 875ns = 14cy, 5cy TOH, 9cy T0L
 * 1 : 583ns high, 333ns low, bit time 937.5ns = 15cy, 9cy T1H, 6cy T1L
 * bit0 time : 1417ns (because of extra setup before each new byte)
 * 0-bit is 1cy shorter than 1-bit because of jrnc instruction (1/2 cycle)
 * nops needed to extend timings
 * bit time : 20 cycles (19 cycles for 0-bit is OK too)
 * objective : T0H = 6cy (375ns),  T0L = 13cy (812ns)
 *             T1H = 13cy (812ns), T1L = 7cy (437ns)
 * -> 1 nop to extend T0H (&T1H), 5-> cy
 * -> 3 nops to extend T1H (4 in total), 9->13 cy
 * -> 1 nop to extend bit time (5 in total), 15->20 cy
 * 
 */
void showPixels2 () {
  hi   = *port |  pinMask;
  lo   = *port & ~pinMask;
  //next = lo;

  disableInterrupts();
  __asm
      ldw y, _pixels      ; // ptr naar pixels in Y index register
      ldw x, _port        ; // ptr naar IOmem port

      ld a, (y)
      push a              ; // current pixel byte -> (4,sp), will be left shifting bits out to carry bit for test
      push _numBytes      ; // TEMP, werkt enkel met numBytes uint8_t !! -> (3,sp)
      push #8             ; // bit counter -> (2,sp)
      push _lo            ; // _next op stack zetten -> (1,sp)

    0020$:                ; // head20 (avr ref)
      ld a, _hi
      ld (x), a           ; // PORT=hi
      nop                 ; // increase T0H to 6cy = 375ns
      sla (4,sp)          ; // carry = bit to test; 0 -> TO signal, 1 -> T1 signal
      jrnc 0021$          ; // if (bit!=1)
      ld (1,sp), a        ; // next=hi

    0021$:                ; 
      ld a, (1,sp)
      ld (x), a           ; // PORT=next
      nop                 ; // increase T1H to 13cy
      nop
      nop
      ld a, _lo           ; 
      ld (1,sp), a        ; // next=lo
      ld (x), a           ; // PORT = lo
      nop                 ; // increase T0L/T1L for 19cy/20cy bit time
      dec (0x02,sp)       ; // bit--
      jrne 0020$          ; // if (bit!=0) from dec above, 0020$ = next bit

    0022$:                ; //nextbyte20
      ld a, #8
      ld (2,sp), a        ; // bit = 8
      incw y              ; // ptr++
      ld a, (y)
      ld (4,sp), a        ; // current pixel byte in SP+4
      dec (3,sp)          ; // numBytes--
      jrne 0020$          ; // if (i!=0) -> next byte

      addw sp, #4         ; // 4 bytes terug van de stack halen, hoeft niet met afzonderlijke pop instructions

  __endasm;

  enableInterrupts();


} // showPixels2

// bit timing almost within spec, numBytes16
// based on showPixels2
/* 0 & 1 bit timings are within spec (same as showPixels2)
 * but bit0 time too long (2100ns, 34cy) 
 * because more instructions needed for extra setup before each new byte 
 * (numByte16--)
 * according to spec it bit time should be max 1250+-600, but this code works anyway
 * alternatively shorten 0/1 timings (eliminate nops), that works too
 */
void showPixels3 () {
  hi   = *port |  pinMask;
  lo   = *port & ~pinMask;
  //next = lo;

  disableInterrupts();
  __asm
      ldw y, _pixels      ; // ptr naar pixels in Y index register
      ld a, (y)
      push a              ; // current pixel byte -> (5,sp), will be left shifting bits out to carry bit for test
      ;push _numBytes      ; // TEMP, werkt enkel met numBytes uint8_t !! -> (3,sp)
      ldw x, _numBytes16  ; 
      pushw x             ; // -> (3,sp) - 2 bytes!
      push #8             ; // bit counter -> (2,sp)
      push _lo            ; // _next op stack zetten -> (1,sp)

    0020$:                ; // head20 (avr ref)
      ldw x, _port        ; // ptr naar IOmem port
      ld a, _hi
      ld (x), a           ; // PORT=hi
      nop                 ; // increase T0H to 6cy = 375ns
      sla (5,sp)          ; // carry = bit to test; 0 -> TO signal, 1 -> T1 signal
      jrnc 0021$          ; // if (bit!=1)
      ld (1,sp), a        ; // next=hi

    0021$:                ; 
      ld a, (1,sp)
      ld (x), a           ; // PORT=next
      nop                 ; // increase T1H to 13cy
      nop
      nop
      ld a, _lo           ; 
      ld (1,sp), a        ; // next=lo
      ld (x), a           ; // PORT = lo
      dec (2,sp)       ; // bit--
      jrne 0020$          ; // if (bit!=0) from dec above, 0020$ = next bit

    0022$:                ; //nextbyte20
      ld a, #8
      ld (2,sp), a        ; // bit = 8
      incw y              ; // ptr++
      ld a, (y)
      ld (5,sp), a        ; // current pixel byte in SP+4
      ;dec (3,sp)          ; // numBytes--
      ldw x, (3,sp)
      decw x              ; // numBytes16-- ;16-bit dec only on X,Y
      ldw (3,sp), x
      jrne 0020$          ; // if (i!=0) -> next byte
      addw sp, #5         ; // 4 bytes terug van de stack halen, hoeft niet met afzonderlijke pop instructions

  __endasm;

  enableInterrupts();


} // showPixels3

// doe 10 toggles
// waveform 125/167ns high, 292/333ns low; period 417ns/458ns ~7cycles (beperking sampleF 24MHz)
void fastToggleTest() {
  hi   = *port |  pinMask;
  lo   = *port & ~pinMask;
  __asm
      ldw x, _port        ; // ptr naar IOmem port, om ld [_port], a te vermijden, is allicht te traag
      push #10            ; // cnt

    0001$:
      ld a, _hi           ; 1cy
      ld (x), a           ; 1cy
      ld a, _lo           ; 1cy
      ld (x), a           ; 1cy
      dec (1,sp)          ; 1cy
      jrne 0001$          ; 2cy

      pop a
  __endasm;
}

// a test using BCCM instruction
/*
 * timing without nops : (measured with 42ns resolution = 24MHz)
 * 0 : 167ns high, 333ns low, bit0 917/958ns (3cy high, 5cy low)
 * 1 : 250ns high, 250ns low, bit0 917/958ns (4cy high, 4cy low)
 * period : 500ns
 * --> lots of spare cycles
 * --> lots of nops needed
 * 4 nops to extend the T0H/T1H (T0H total 7cy high = 437.5ns, for 400+-150ns spec)
 * 3 nops to extend the T0L/T1H (T1H total 11cy high = 687.5ns, for 800+-150ns spec)
 * 6 nops to extend H+L to 1250ns nominal  (-> 21cy per bit = 1312,5ns)
 * -> thinking error? because saleae measures 1250ns period (<> 500+812.5???)
 * -> bit0 increases to +- 1700ns = within spec 1250+-600ns
 */
void showPixelsBCCM() {
  // not used here, using fixed pin PD2 with bset,bres, bccm
  //hi   = *port |  pinMask;
  //lo   = *port & ~pinMask;
  //next = lo;

  disableInterrupts();
  // todo : is push/pop onderweg niet makkelijker dan (0x01,sp) gebruiken?
  __asm
      ldw y, _pixels      ; // ptr naar pixels in Y index register
      ldw x, _port        ; // ptr naar IOmem port, om ld [_port], a te vermijden, is allicht te traag

      push _numBytes      ; // iByte counter, TEMP, werkt enkel met numBytes uint8_t !! -> (0x02,sp)
      push #8             ; // bit counter -> (0x01,sp)
      ld a, (Y)           ; // a = *(pixels+iByte)

    0020$:                ; // bit loop
      bset 0x500F, #2    ; // PD2 HIGH
      sla a               ; // A<<1
      ; // nops here for timing
      nop
      nop
      nop
      nop
      bccm 0x500F, #2    ; // avoid branch instruction, PD2 low/high if corresponding bit 0/1
      dec (1,sp)         ; // bit--
      ; // nops here for timing
      nop
      nop
      nop
      bres 0x500F, #2    ; // PD2 low
      nop
      nop
      nop
      nop
      nop
      nop
      jrne 0020$         ; // if (bit!=0)

      ; // bit=0, prepare new byte
      ; // deze instructies vragen cycles en daardoor is de T0L/T1L timing afwijkend voor bit0
      ; // fastled-stm8 spreidt de instructies over verschillende bits, ten koste van meer & complexere code
      ld a, #8
      ld (1,sp), a        ; // reload bit counter
      incw y              ; // pixels ptr++
      ld a, (y)           ; // *pixels_ptr
      dec (2,sp)          ; // iByte--
      jrne 0020$          ; // if (iByte!=0) -> next byte
      ; // cleanup
      addw sp, #2         ; // 2 bytes terug van de stack halen, hoeft niet met afzonderlijke pop instructions

  __endasm;

  enableInterrupts();
}

void setup() {
  setPin(PD7); // 2 voor AVR, PD2 voor STM8
  digitalWrite(PD7,LOW);
  initPixels();
  pinMode(LED_BUILTIN,OUTPUT); // setup blinkie
} // setup

uint32_t blinkMillis;
void loop() {

  if ((millis() - blinkMillis) > 500) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    //showPixels();
    //fastToggleTest();
    //showPixelsBCCM();
    //showPixels2();
    showPixels3();
    blinkMillis = millis();
  }
}