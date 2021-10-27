/*  a test to copy flash contents to RAM
 *  see if this works under ROP
*/

#include "Arduino.h"

#define FLASH_START     0x8000
#define FLASH_SIZE      8192
#define FLASH_END       (FLASH_START + FLASH_SIZE)
#define RAM_BLOCK_SIZE  256 // copy per block of this size, because ram is limited

char ramBlock[RAM_BLOCK_SIZE];
bool triggerBlockCopy; // a block copy is triggered from the debugger
uint32_t flashStartAddress;

/*
uint8_t flashDownload[] = {
  // ramBlock           @ 0x01 (256 bytes)
  // triggerBlockCopy   @ 0x101 (1byte)
  // flashStartAddress  @ 0x102 (4bytes)

  // src/mymain.c: 21: flashStartAddress = FLASH_START;
  0xAE, 0x80, 0x00, //        [ 2]   99 	ldw	x, #0x8000
  0xCF, 0x01, 0x04, //        [ 2]  100 	ldw	_flashStartAddress+2, x
  0x5F,             //        [ 1]  101 	clrw	x
  0xCF, 0x01, 0x02, //        [ 2]  102 	ldw	_flashStartAddress+0, x
  // src/mymain.c: 22: triggerBlockCopy = true;
  //TEST 0x35, 0x01, 0x01, 0x01, //      [ 1]  105 	mov	_triggerBlockCopy+0, #0x01
  // src/mymain.c: 24: while (flashStartAddress < FLASH_END) {
  // 00104$:
  0xCE, 0x01, 0x04, //         [ 2]  111 	ldw	x, _flashStartAddress+2
  0xA3, 0xA0, 0x00, //         [ 2]  112 	cpw	x, #0xa000
  0xC6, 0x01,0x03,  //        [ 1]  113 	ld	a, _flashStartAddress+1
  0xA2, 0x00,       //           [ 1]  114 	sbc	a, #0x00
  0xC6, 0x01,0x02,  //         [ 1]  115 	ld	a, _flashStartAddress+0
  0xA2, 0x00,       //            [ 1]  116 	sbc	a, #0x00
  0x25, 0x03,       //            [ 1]  117 	jrc	00138$
  // 0xCC, 0x81, 0x8A, //         [ 2]  118 	jp	00110$ TODO AANPASSEN NAAR RAM ADDRESS
  // jump absolute vervangen door relative
  0x20, 0x42, 0x9D, // JRA 0x42, NOP ; sds

  // 00138$:
  // src/mymain.c: 25: if (triggerBlockCopy) {
  0x72, 0x00, 0x01, 0x01, 0x02, //   [ 2]  123 	btjt	_triggerBlockCopy+0, #0, 00139$
  0x20, 0xE4,       //            [ 2]  124 	jra	00104$
  // 00139$:
  // src/mymain.c: 26: for (uint16_t i=0;i<RAM_BLOCK_SIZE;i++) {
  0x5F, //               [ 1]  129 	clrw	x
  0x1F, 0x01, //         [ 2]  130 	ldw	(0x01, sp), x
  // 00108$:
  0x1E, 0x01, //            [ 2]  133 	ldw	x, (0x01, sp)
  0xA3, 0x01, 0x00, //         [ 2]  134 	cpw	x, #0x0100
  0x24, 0x14, //            [ 1]  135 	jrnc	00101$
  // src/mymain.c: 28: flashAddress = (uint8_t*) (flashStartAddress + i);
  0xCE, 0x01, 0x04, //         [ 2]  139 	ldw	x, _flashStartAddress+2
  0x72, 0xFB, 0x01, //         [ 2]  140 	addw	x, (0x01, sp)
  // src/mymain.c: 29: ramBlock[i] = *flashAddress;
  0x16, 0x01, //            [ 2]  143 	ldw	y, (0x01, sp)
  0xF6, //               [ 1]  144 	ld	a, (x)
  0x90, 0xD7, 0x00, 0x01, //    [ 1]  145 	ld	((_ramBlock + 0), y), a
  // src/mymain.c: 26: for (uint16_t i=0;i<RAM_BLOCK_SIZE;i++) {
  0x1E, 0x01, //            [ 2]  148 	ldw	x, (0x01, sp)
  0x5C, //               [ 1]  149 	incw	x
  0x1F, 0x01, //            [ 2]  150 	ldw	(0x01, sp), x
  0x20, 0xE5, //            [ 2]  151 	jra	00108$
  // 00101$:
  // src/mymain.c: 31: triggerBlockCopy = false;
  0x72, 0x5F, 0x01, 0x01, //      [ 1]  156 	clr	_triggerBlockCopy+0
  // src/mymain.c: 32: flashStartAddress += RAM_BLOCK_SIZE;
  0xCE, 0x01, 0x04, //         [ 2]  159 	ldw	x, _flashStartAddress+2
  0x1C, 0x01, 0x00, //         [ 2]  160 	addw	x, #0x0100
  0x90, 0xCE, 0x01, 0x02, //      [ 2]  161 	ldw	y, _flashStartAddress+0
  0x24, 0x02, //            [ 1]  162 	jrnc	00142$
  0x90, 0x5C, //            [ 1]  163 	incw	y
  // 00142$:
  0xCF, 0x01, 0x04, //         [ 2]  165 	ldw	_flashStartAddress+2, x
  0x90, 0xCF, 0x01, 0x02, //      [ 2]  166 	ldw	_flashStartAddress+0, y
  // 0xCC, 0x81, 0x34 //         [ 2]  168 	jp	00104$ TODO AANPASSEN NAAR RAM
  // jr -56h
  0x20, 0xAA
};
*/

void setup() {
  pinMode (LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  for (uint16_t i=0;i<256;i++)
    ramBlock[i] = 0;

  flashStartAddress = FLASH_START;
  triggerBlockCopy = false;
  /*
    __asm
      jp 0x140 ; // daar staat proefondervindelijk de flashDownload[]
    __endasm;
  */

  // lijkt te werken
  // ie. er wordt 256 bytes tegelijk uit flash naar ram gecopieerd
  // bytes met ramBlock in debugger komen overeen met readout met stm8flash
  /*
  while (flashStartAddress < FLASH_END) {
    if (triggerBlockCopy) {
      for (uint16_t i=0;i<RAM_BLOCK_SIZE;i++) {
        uint8_t *flashAddress;
        flashAddress = (uint8_t*) (flashStartAddress + i);
        ramBlock[i] = *flashAddress;
      }
      triggerBlockCopy = false;
      flashStartAddress += RAM_BLOCK_SIZE;
    }
    // na elke block copy moet debugger de vlag triggerBlockCopy terug true zetten
  }
  */
  
} // setup

uint32_t blinkMillis;
bool doRamJump = false;
void loop() {
  // simple blinkie
  if ((millis() - blinkMillis) > 500) {
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
    blinkMillis = millis();
  }
  /*
  if (doRamJump){
    __asm
      jp 0x140 ; // daar staat proefondervindelijk de flashDownload[]
    __endasm;

  }
  */
}

