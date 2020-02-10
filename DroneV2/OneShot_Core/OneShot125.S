#define __SFR_OFFSET 0
 
#include "avr/io.h"

.global start
.global TransmitThree
.global TransmitThreeWO
.global TransmitThreeWT

start:
  sbi   DDRB,0  ; Set PB0 as output
  sbi   DDRB,1  ; Set PB1 as output
  sbi   DDRB,2  ; Set PB2 as output
  sbi   DDRB,3  ; Set PB3 as output 
  ret


TransmitThree:
  push r29
  push r28
  push r18
  push r16 

  lds r28,(th) 
  lds r29,(th)+1 
  lds r18,(chan) ; set channel  1: Bit 0      2: Bit 1     4: Bit 2    8: Bit 4
  ldi r16,0 ; set low

  out   PORTB,r18 ; Set  HIGH

  DelayThree:
  sbiw r28,1
  brne DelayThree

  out   PORTB,r16 ; Set  LOW
  rjmp LoopEnd 

TransmitThreeWO:
  push r29
  push r28
  push r18
  push r16

  lds r28,(th) 
  lds r29,(th)+1 

  lds r18,(chan) ; set channel  1: Bit 0      2: Bit 1     4: Bit 2    8: Bit 4
  ldi r16,0 ; set low

  out   PORTB,r18 ; Set  HIGH

  DelayWO:
  sbiw r28, 1
  brne DelayWO
  nop

  out   PORTB,r16 ; Set  LOW
  rjmp LoopEnd 

TransmitThreeWT:
  push r29
  push r28
  push r18
  push r16

  lds r28,(th) 
  lds r29,(th)+1 
  lds r18,(chan) ; set channel  1: Bit 0      2: Bit 1     4: Bit 2    8: Bit 4
  ldi r16,0 ; set low

  out   PORTB,r18 ; Set  HIGH

  DelayWT:
  sbiw r28,1
  brne DelayWT
  nop
  nop

  out   PORTB,r16 ; Set  LOW
  rjmp LoopEnd 



LoopEnd:
  pop r16
  pop r18
  pop r28
  pop r29
  ret

