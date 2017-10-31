; * Author of BatProtFirmware is Pavel Palonen
; *
; * BatProtFirmware is free software: you can redistribute it and/or modify
; * it under the terms of the GNU General Public License as published by
; * the Free Software Foundation, either version 3 of the License, or
; * (at your option) any later version.
; *
; * BatProtFirmware is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
; * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; * GNU General Public License for more details.
; * this text shall be included in all
; * copies or substantial portions of the Software.
; *
; * See <http://www.gnu.org/licenses/>.

.include "tn13Adef.inc"


;---- END of configurable defines ----

; Predefined configurable parameters
.EQU	LOW_BAT_VOLTAGE		= 150	; means 15.0 volts
.EQU	BAT_CORRECTION		= 0		; Signed value for correction voltage readings (for debug use 1)

; END OF predefined configurable parameters


; If you did not changed hardware, then you don't need to change this...
.EQU	VOLT_DIV_CONST		= 141	; To get this number use formula (for 23v max): 
										; 4095/(Vmax*10)*8, where Vmax=(R1+R2)*Vref/R2
										; Vref=5v 
										; and resistor values is from divider (12K/3.3K)
										; Vmax=(12+3.3)*5/3.3=23.18
										; 4095/(23.18*10)*8=141
										
.EQU	ENLDO_PIN	= PB0	; Pin for device Configuration
.EQU	LED_PIN		= PB2	; Vertical sync pin
.EQU	CNTRL_PIN	= PB3	; Mosfet Control pin
.EQU	VBAT_PIN	= PB4	; Resistor divider (10K/3K) for voltage measurement (21v max)

.def	z0			=	r0
.def	z1			=	r1
.def	r_sreg		=	r2	; Store SREG register in interrupts
.def	tmp			=	r16
.def	tmp1		=	r17
.def	tmp2		=	r3
.def	itmp		=	r18	; variables to use in interrupts
.def	itmp1		=	r19	; variables to use in interrupts
.def	itmp2		=	r4	; variables to use in interrupts
.def	voltage		=	r20	; voltage in volts * 10
.def	lowbat_cntr	=	r21	; counter for blinking voltage when it gets low
.def	adc_cntr	=	r23	; counter for ADC readings
; Variables XL:XH, YL:YH, ZL:ZH are used in interrupts, so only use them in main code when interrupts are disabled
.def	adc_sumL	=	r8	; accumulated readings of ADC (sum of 64 values)
.def	adc_sumH	=	r9	; accumulated readings of ADC (sum of 64 values)

.DSEG
.ORG 0x60

.CSEG
.ORG 0
		rjmp RESET ; Reset Handler
		reti	;rjmp EXT_INT0 ; IRQ0 Handler
		reti	;rjmp PCINT_int ; PCINT0 Handler
		reti	;rjmp TIM0_OVF ; Timer0 Overflow Handler
		reti	;rjmp EE_RDY ; EEPROM Ready Handler
		reti	;rjmp ANA_COMP ; Analog Comparator Handler
		reti	;rjmp TIM0_COMPA ; Timer0 CompareA Handler
		reti	;rjmp TIM0_COMPB ; Timer0 CompareB Handler
		reti	;rjmp WATCHDOG
		reti	;rjmp ADC ; ADC Conversion Handler

.include "adc.inc"

RESET:
		ldi tmp, low(RAMEND); Main program start
		out SPL,tmp ; Set Stack Pointer to top of RAM
		
		;init variables
		clr z0
		clr z1
		inc z1
		;clr adc_cntr		; couter for ADC readings. No need to initialize. Anyway we give some time for ADC to initialize all variables and states
		
		; change speed (ensure 9.6 mhz ossc)
;		ldi tmp, 1<<CLKPCE
;		ldi tmp1, 1<<CLKPS2	; 16 divisor
;		out CLKPR, tmp		; enable clock change
;		out CLKPR, tmp1		; prescaler 16
		
		; Configure Video pin as OUTPUT (LOW)
		sbi	PORTB, ENLDO_PIN
		sbi	DDRB, ENLDO_PIN
		sbi	DDRB, LED_PIN
		sbi	DDRB, CNTRL_PIN
		; Enable pullup on Configure Pin. We will enter configure mode if this pin will go LOW (by PCINT interrupt)
		sbi	PORTB, LED_PIN ; LED ON indicating, that power to the MCU is ok.
		cbi	PORTB, CNTRL_PIN ; MOSFET OFF
		
		; turn off digital circuity in analog pin
		sbi DIDR0, VBAT_PIN
		
		; Configure ADC
		; 5Vref, ADC channel, 10bit ADC result
		ldi tmp, 0<<REFS0 | 0<<MUX0 | 1<<MUX1
		out ADMUX, tmp
		; normal mode (single conversion mode), 16 prescaler (about 75khz at 9.6/8mhz ossc).
		ldi tmp, 1<<ADEN | 1<<ADSC | 1<<ADPS2 | 0<<ADPS1 | 0<<ADPS0
		out ADCSRA, tmp


		; Wait for voltage stabilizing and ADC warmup
		ldi lowbat_cntr, 254	; We want to start this counter to make a delay for voltage stabilizing
strt_wt:rcall ReadVoltage
		dec lowbat_cntr
		brne strt_wt	; read ADC 255 times
		; now our voltage and voltage_min is messed. Lets reset at least voltage_min.
		
		ldi tmp, 1<<SM1 | 0<<SM0 | 1<<SE	; power down sleep mode enable
		out MCUCR, tmp
		
		rcall WDT_Start	; start timer
		
		sei ; Enable interrupts
		
		sbi	PORTB, CNTRL_PIN	; turn on mosfet

		cbi	PORTB, LED_PIN	; turn off LED
		
main_loop:
		sleep
		; read ADSC bit to see if conversion finished
		sbi	PORTB, LED_PIN	; turn off LED
		rcall ReadVoltage
		cbi	PORTB, LED_PIN	; turn off LED
		;compare voltage to minimum voltage
		cpi voltage, LOW_BAT_VOLTAGE
		brsh main_loop	; voltage is ok
		; voltage is critical
		; we need to have critical voltage for at least two seconds.
		sbi	PORTB, LED_PIN	; indicate that critical issue is started
		sleep	; wait for some time
		cbi	PORTB, LED_PIN	; indicate that critical issue is started
		; read voltage agian
		rcall ReadVoltage
		cpi voltage, LOW_BAT_VOLTAGE
		brsh main_loop	; voltage is good
		; low bat is confirmed
		cbi	PORTB, CNTRL_PIN	; turn off mosfet
		; indicate that battery is disconnected by flashing the LED with interval 0.5 sec
		rcall WDT_Start05s	; watchdog interval 0.5sec
		ldi lowbat_cntr, 40	; We want to start this counter to make a delay for voltage stabilizing
lowbatwarn:
		cbi	PORTB, LED_PIN
		sleep
		sbi	PORTB, LED_PIN
		sleep
		dec lowbat_cntr
		brne lowbatwarn
	
		; turn off everything
		;cbi	PORTB, ENLDO_PIN	; turn of MCU (kill me)
		cli
		;cbi	PORTB, LED_PIN
		; freeze
L1:		;sleep
		rjmp L1
		
WDT_Start05s:
		ldi   tmp1, (0<<WDE) | (1<<WDTIE) | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0)		; 0.5s
		rjmp WDT_Start1
WDT_Start:	; with interrupt behaviour
		ldi   tmp1, (0<<WDE) | (1<<WDTIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0)		; 2s
WDT_Start1:
		wdr		; Reset Watchdog Timer
		; Start timed sequence
		in    tmp, WDTCR
		ori   tmp, (1<<WDCE) | (1<<WDE)
		out   WDTCR, tmp
		; --  Got four cycles to set the new values from here -
		out   WDTCR, tmp1
		
		ret