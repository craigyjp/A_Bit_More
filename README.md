## Based on the the original Crumar Bit series synthesizers I have created an RP2040 based DCO synthesizer. 

## Unfortunately the project did not work out too well as the DCO's were noisy and the PWM and triangle sound atrocious.

## So a new synth is born with the same controller, new displays and the same filters, but now each RP2040 autotunes a pair of AS3340 VCO's

This is an 8 voice bi-timbral polyphonic dual VCO based synthesizer that uses an RP2040 to autotune and control a pair of AS3340 VCO chips.

I have also added portamento, octave switching, oscillator sync etc.

Filters are based on the Matrix 12 pole switching filters with 16 filter types available, dual LFO's are vailable for modulation and PWM, plus a white/pink noise source.
