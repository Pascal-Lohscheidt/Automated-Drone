#define __SFR_OFFSET 0
 
#include "avr/io.h"

.global start
.global highblink
.global lowblink
.global Transmit

start:
  sbi   DDRB,0  ; Set PB0 as output
  sbi   DDRB,1  ; Set PB1 as output
  sbi   DDRB,2  ; Set PB2 as output
  sbi   DDRB,3  ; Set PB3 as output 
  ret

Transmit: 
  push r26
  push r27
  push r17
  push r18
  push r16
  push r20 
  lds r26,(th) 
  lds r27,(th)+1 
  ldi r17,16 
  lds r18,(chan) ; set channel  1: Bit 0      2: Bit 1     4: Bit 2    8: Bit 4
  ldi r16,0 ; set low

TransmissionFallBack:
nop
nop
nop
  rol r26
  rol r27

  brcs highblink
  out   PORTB,r18 ; Set  HIGH ; lowblink
  
  ldi r20,10
  call delaylp
  out   PORTB,r16 ; Set  LOW
  
  ldi r20,16 ; 17
  call delaylp

LoopEnd:
  dec r17 ; decrement counter
  brne TransmissionFallBack ; go through loop again
  pop r20
  pop r16
  pop r18
  pop r17
  pop r27
  pop r26
  ret

highblink:
  out   PORTB,r18 ; Set  HIGH
  
  ldi r20,23 ; 20 
  call delaylp
  out   PORTB,r16 ; Set  LOW
  
  ldi r20,3 ; 7
  call delaylp
  rjmp LoopEnd 

delaylp:
  subi    r20, 1
  brne    delaylp
  ret
